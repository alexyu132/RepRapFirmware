# Kalico-style reciprocal pressure advance for RRF 3.6.4 (STM32F4)

Branch `kalico-reciprocal-pa-3.6.4` on `alexyu132/RepRapFirmware`, based on
gloomyandy v3.6.4. **Not yet tested on a printer** — see "First test" below.
An adversarial safety review is required before flashing anything from this
branch (per the user's requirement).

## Model

The pressure advance distance `D` varies with filament speed `v` as

```
D(v) = kLin * v + offset * v / (v + vLin)
```

- `kLin` — linear PA constant, seconds (the old `S` value).
- `offset` — reciprocal offset, mm of filament. This is the extra advance
  that saturates in at low speed; it is what dominates on Bowden setups.
- `vLin` — linearization velocity, mm/s of filament. Sets where the
  reciprocal term rolls off.

Effective PA `K_eff(v) = D(v)/v = kLin + offset/(v + vLin)` falls smoothly
from `kLin + offset/vLin` at low speed toward `kLin` at high speed. No knee,
no slope discontinuity. With `offset = 0` this reduces exactly to the old
linear behaviour.

Example `M572 D0 S0.03 O0.5 V2.5`:

| filament speed v | K_eff(v) | D(v) |
| --- | --- | --- |
| 1 mm/s | 0.173 s | 0.17 mm |
| 10 mm/s | 0.070 s | 0.70 mm |
| 25 mm/s | 0.048 s | 1.20 mm |

## M572 syntax

```
M572 D0 S0.05               linear PA only, exactly as stock 3.6.4
M572 D0 S0.03 O0.5 V2.5     reciprocal PA
M572                         report current values for all extruders
```

- `S` = kLin in seconds, `O` = offset in mm, `V` = vLin in mm/s.
- `O` and `V` must be given together; either alone is an error.
- Bare `M572` reports linear PA as a single number (`0.050`) or reciprocal
  PA as `(kLin offset vLin)`, e.g. `(0.030 0.500 2.50)`.
- **S-only clears reciprocal:** `M572 D0 S<k>` with no `O`/`V` resets the
  reciprocal parameters — a previously set offset/vLin is silently cleared
  and the extruder returns to linear-only PA. This is by design (S-only
  means "linear PA with this kLin"), but it is silent, so be aware when
  switching between linear and reciprocal tuning: re-issue the full
  `S... O... V...` line to restore reciprocal PA.
- `M572` still waits for standstill before applying a new value.
  Reciprocal PA removes the need for frequent M572 commands; it does not
  make M572 live.
- Remote CAN toolboards still use the old single-float protocol, so they get
  kLin only; the full reciprocal parameters apply on the main board.

### Validation limits (enforced, not just documented)

| parameter | allowed range |
| --- | --- |
| S (kLin) | 0 to 2.0 s |
| O (offset) | 0 to 5.0 mm |
| V (vLin) | 0.1 to 100 mm/s (when given) |
| low-speed slope S + O/V | at most 5.0 s |

A rejected `M572` is an error and leaves the previous known-good values in
place — nothing is partially applied. NaN, inf, negatives, zero `V`, and
absurd magnitudes are all rejected. The same checks run inside
`SetParameters`/`SetParametersSimple`, so the CAN single-float path cannot
smuggle a bad value in either.

## Flow conversion (1.75 mm filament)

Filament cross-section is pi*(1.75/2)^2 = 2.405 mm^2, so volumetric flow
`Q = v * 2.405 mm^3/s` and `v = Q / 2.405`. The reciprocal term is centred
around `vLin`: at `v = vLin`, half the offset has rolled in. With the Bowden
suggestion `V2.5`, that centre is `Q ≈ 6.0 mm^3/s`.

## Tuning (Bowden, Bambu-type hotend)

- Start with `V2.5` (Kalico suggests linearization_velocity 2–3 for Bowden;
  lower V = stronger low-speed effect but harder on the extruder; higher V =
  gentler). Sweep one parameter per tower, in this order:
  1. **Offset tower:** fix `S0.02 V2.5`, sweep `O` from 0 to ~1.0 mm.
     Pick the smallest offset that cleans up low-speed corners/seams
     without blobs.
  2. **Linear tower:** fix the offset from step 1, sweep `S` at high speed.
  3. **Re-check offset**, then optionally sweep `V` (1.5–5) for extruder
     smoothness — lower V makes the extruder work harder at speed changes.
- OrcaSlicer adaptive-PA calibration gives you K at several flow rates:
  fit `K_eff(v) = kLin + offset/(v + vLin)` through those points (3+ points:
  low/mid/high speed). A curve-fitting tool for this is planned as a
  follow-up; for now, eyeball the fit or solve it in a spreadsheet.
- If the extruder clicks/skips at speed changes, raise `V` before touching
  anything else.

## What this does NOT do (follow-ups, not in v1)

- No `time_offset`, no `smooth_time`, no acceleration term — reciprocal core
  only. The chord-average machinery is model-agnostic, so these can be added
  later without touching the motion pipeline again.

## Safety notes (for the adversarial review)

- **Validation:** see the limits table above. Every new division is guarded
  (vLin = 0 with offset = 0 is the linear path and returns kLin directly;
  speeds are clamped to >= 0; unconfigured/invalid steps/mm falls back to
  kLin; every computed advance is checked finite and clamped to the analytic
  range [kLin, kLin + offset/vLin] before it reaches a DDA segment).
- **Accel cap:** the extruder acceleration cap in `DDA.cpp` uses the chord
  slope of `D(v)` over the move's actual speed range
  (`ExtruderShaper::GetChordSlopeToFilamentSpeed`, via
  `Move::GetPressureAdvanceChordClocksForLogicalDrive`) — the slope the
  advance itself uses — not the zero-speed maximum. The maximum slope
  `kLin + offset/vLin` is the steepest point of the reciprocal curve and
  only applies to infinitesimal speed intervals near zero; using it for
  every move capped XY acceleration to ~700 mm/s² for a typical Bowden tune
  (fixed 2026-11-18: the cap was the only place using the max slope, the
  advance was already correct). The max slope remains the analytic upper
  bound the chord slope is clamped to, and the fallback when the move's
  speed range cannot be established. The cap is applied in two stages.
  At `Init` it uses the chord over `[0, feed rate]` (the move accelerating
  from rest to the requested speed) and is applied after the kinematics
  limits, with the no-PA limits stored (`maxAccelerationNoPA`,
  `maxDecelerationNoPA`). The feed rate is in mm/step-clock
  (`RawMove.h`), so it is converted to mm/s with
  `InverseConvertSpeedToMmPerSec` before the chord is computed — passing
  mm/step-clock to a chord that expects mm/s would understate the speed
  750,000x and return the max slope, reproducing the ~708 mm/s² cap
  (caught by audit 2026-11-18, fixed the same day). Once the lookahead has set the move's actual
  start/end speeds and `RecalculateMove` its top speed,
  `DDA::RefinePressureAdvanceCap` (called from `DoLookahead`) recomputes
  the cap from the chord over `[min(start, end), top]`
  (`GetChordSlopeFilamentRange`) — a real move usually starts and ends at
  junction speeds, so this chord is lower and the cap can be raised (for a
  typical Bowden tune the 100 mm/s cap goes from ~5,200 to ~8,200 mm/s²) —
  and raises `maxAcceleration`/`maxDeceleration` up to the no-PA limits,
  recomputing the profile (bounded to 3 passes; long moves converge on the
  first). The refinement only runs when `DoLookahead` runs (a following
  move arrives while this move is still provisional and melding is
  allowed); unrefined moves keep the Init cap, which is correct on its
  own. The refinement only ever raises the cap, never lowers it, so it
  cannot make an end speed the lookahead already set infeasible. Caveat: a
  move may not reach the requested feed rate (short move, lookahead), so
  the Init-time cap is optimistic for very short low-speed moves; the
  consequence is bounded (a possible extruder skip on such a move, not a
  crash), and the DDA merges tiny speed changes so infinitesimal intervals
  never form an accel phase. The Init cap itself is covered by the host
  harness: `ExtruderShaper::GetPressureAdvanceCapXY` is the exact function
  the DDA calls, and the CAPXY suite drives it with feed rates in
  mm/step-clock (as `RawMove` stores them) and checks the resulting cap
  against an independent Python reference, including Alex's tune at
  50/100/150/200/300 mm/s (3,320/5,167/6,542/7,606/9,144 mm/s²) and
  inverse-time moves. The suite was proven to catch the B1 unit error: with
  the mm/s conversion removed it fails 1,231 checks. The refinement cap is
  covered the same way: `ExtruderShaper::GetPressureAdvanceCapXYRange` is
  the exact function `RefinePressureAdvanceCap` calls, and the REFINECAP
  suite drives it with DDA speeds in mm/step-clock, verifying Alex's
  corner-to-100 (~8,200 mm/s²) and corner-to-200 (~10,657 mm/s²) caps; it
  was proven to catch a degenerate refinement (using the [0, top] chord
  instead of [low, top]) with 1,227 failures. What is still NOT covered is
  the `DoLookahead` wiring and the raise loop itself (when refinement runs,
  the 3-pass iteration, the `RecalculateMove` interaction) — that part is
  verified only by a clean build and on-printer testing.
- **Thermal isolation:** the diff touches only
  `src/Movement/ExtruderShaper.{h,cpp}`, `Move.{h,cpp}`, `Move2.cpp`,
  `DDA.{h,cpp}`. No heater, thermistor, fan, or thermal-protection code is
  touched. Pressure advance drives extruder motion only.
- **Memory:** `ExtruderShaper` is 4 floats = 16 bytes (stock: 1 float =
  4 bytes), one per `DriveMovement`. The F4 build has 29 DriveMovements
  (`MaxAxesPlusExtruders` 15 + `NumDirectDrivers` 14), so +12 bytes x 29 =
  **+348 bytes RAM** vs stock, against 18,484 bytes never-used RAM at idle
  on this board. No heap use, no per-move allocation; `PrepParams` gains
  three speeds but is a stack struct.
- **Safe defaults:** with no M572 set, the shaper is `kLin = 0, offset = 0`
  (PA off, as stock). Plain `M572 D0 S<k>` behaves identically to stock
  3.6.4 linear PA — the new model is opt-in only, via `O`+`V`.

## First test (do not flash unattended)

1. Flash, then `M122` and confirm the board boots and reports normally.
   `M572` with no arguments must report `0.000` (PA off).
2. First print with linear only: `M572 D0 S0.05`. Confirm print quality
   matches stock behaviour — this proves the motion-pipeline changes are
   neutral before the new model is engaged.
3. Then try reciprocal, conservative: `M572 D0 S0.03 O0.3 V2.5` on a small
   calibration print (corners + seams + a speed change). Watch the extruder:
   clicking/skipping/grinding, blobs or zits at low speed, or runaway
   extrusion (excess ooze/purge) all mean stop the print and revert.
4. Only then tune upward (offset tower first, per above).
5. **Revert:** `M572 D0 S0` turns PA off entirely, or `M572 D0 S<old k>`
   restores your previous linear value. To remove the firmware completely,
   reflash the stock 3.6.4 `firmware_e3pro3_f4.bin` from the official
   gloomyandy release.
