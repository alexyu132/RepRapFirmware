/*
 * PressureAdvanceShaper.h
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_EXTRUDERSHAPER_H_
#define SRC_MOVEMENT_EXTRUDERSHAPER_H_

#include <RepRapFirmware.h>

// Kalico-style reciprocal (nonlinear) pressure advance, adapted for
// RepRapFirmware 3.6.4 (STM32F4). See Kalico docs/Nonlinear_Pressure_Advance.md
// (Prunt3D/kalico_for_prunt_board_3) and Duet3D forum topic 37604.
//
// Model: the pressure advance distance D varies with filament speed v as
//		D(v) = kLin * v + offset * v/(v + vLin)
// where v is filament speed, kLin is the linear PA constant (seconds), offset
// is a distance in mm of filament, and vLin is the linearization velocity
// (mm/s of filament). This is a smooth saturating curve: the effective PA
// K_eff(v) = D(v)/v falls smoothly from kLin + offset/vLin at low speed
// toward kLin at high speed, with no knee and no discontinuity in slope.
// That low-speed offset is what dominates on Bowden setups, where the long
// tube and melt-zone nonlinearity make linear PA under-compensate at low
// speeds and over-compensate at high speeds. With offset = 0 this reduces
// exactly to the old linear behaviour.
// The chord-average architecture (GetAverageAdvanceClocks over a phase's
// speed interval) is model-agnostic: the chord slope of D(v) is computed
// analytically below, one division per evaluation, no transcendental
// functions and no libm dependency, so it costs nothing in the step ISR
// (it runs once per extruder move at prepare time, not per step).
// M572 still waits for standstill before applying a new value; reciprocal PA
// removes the need for frequent M572 commands, it does not make M572 live.
//
// SAFETY: parameter limits below are enforced on every path that can set
// parameters (M572 and the CAN single-float path) and re-checked inside
// SetParameters/SetParametersSimple, so an invalid value can never reach the
// motion pipeline; the previous known-good values stay in place. All new
// divisions are guarded against zero/negative/non-finite inputs, and every
// value handed to the DDA segment builder is checked finite first. The
// extruder acceleration cap uses the chord slope of D(v) over the move's
// actual speed range [0, feed rate] (GetChordSlopeToFilamentSpeed), not the
// zero-speed maximum: the maximum slope only applies to infinitesimal
// intervals near zero speed, and using it for every move capped XY accel
// to ~700 mm/s^2 on a typical Bowden tune. The zero-speed slope
// kLin + offset/vLin remains the analytic upper bound that the chord slope
// is clamped to, and the fallback when the speed range is unknown.

// Pressure advance parameters as set by M572. kLin is in seconds, offset in
// mm of filament, vLin in mm/s of filament.
struct PressureAdvanceParameters
{
	float kLin;					// linear pressure advance constant in seconds
	float offset;					// reciprocal offset in mm of filament (0 = linear PA)
	float vLin;						// linearization velocity in mm/s of filament
};

// This class holds pressure advance parameters for an extruder.
class ExtruderShaper
{
public:
	// Safety limits (user units) for M572 parameters. Enforced by M572 and
	// re-checked inside SetParameters/SetParametersSimple. The effective
	// low-speed slope kLin + offset/vLin is capped at MaxEffectiveSeconds so
	// the extruder acceleration cap (which divides by that slope) always
	// stays meaningful. All limits are generous vs any plausible tune
	// (Bowden kLin rarely exceeds 1 s, Kalico offset rarely exceeds 2 mm,
	// suggested vLin is 2-3 mm/s).
	static constexpr float MaxKlinSeconds = 2.0;
	static constexpr float MaxOffsetMm = 5.0;
	static constexpr float MinVlinMmPerSec = 0.1;
	static constexpr float MaxVlinMmPerSec = 100.0;
	static constexpr float MaxEffectiveSeconds = 5.0;	// max kLin + offset/vLin

	ExtruderShaper() noexcept
	{
		SetParametersSimple(0.0);
	}

	// Validate a parameter set (finite, in range, sane effective slope).
	// Used by M572 (which reports the failure) and re-checked by SetParameters.
	static bool ParametersAreValid(const PressureAdvanceParameters& params) noexcept;

	// Get the maximum pressure advance slope (at zero speed) in step clocks.
	// This is kLin + offset/vLin, the steepest point of the reciprocal curve.
	// It is the analytic upper bound the chord slope is clamped to (see
	// GetAverageAdvanceClocks) and the fallback for the acceleration cap
	// when the move's speed range is unknown. It is NOT the slope the cap
	// uses for a real move: see GetChordSlopeToFilamentSpeed.
	float GetMaxAdvanceClocks() const noexcept;
	float GetMaxAdvanceSeconds() const noexcept { return (float)GetMaxAdvanceClocks() * StepClocksToSeconds; }

	// Get the chord slope (in step clocks) of D(v) between filament speeds 0
	// and highFilamentSpeedMmPerSec (mm/s of filament). This is the slope the
	// DDA acceleration cap must use for a move that accelerates from rest to
	// that filament speed: the chord slope over the move's actual speed range.
	// With offset = 0 this is just kLin. Unknown/invalid speeds fall back to
	// the zero-speed maximum (safe but conservative).
	float GetChordSlopeToFilamentSpeed(float highFilamentSpeedMmPerSec) const noexcept;

	// Get the chord slope (in step clocks) of D(v) between two filament speeds
	// (mm/s of filament). This is the slope the lookahead refinement of the
	// DDA acceleration cap uses once the move's actual start/end/top speeds
	// are known: the chord over [low, high], not over [0, high]. With
	// offset = 0 this is just kLin. Unknown/invalid speeds, or low > high,
	// fall back to the zero-speed maximum (safe but conservative).
	float GetChordSlopeFilamentRange(float lowFilamentSpeedMmPerSec, float highFilamentSpeedMmPerSec) const noexcept;

	// Get the XY acceleration cap (mm/step-clock^2) imposed by pressure
	// advance for a move with the given feed rate. This is the DDA Init cap,
	// factored out so the host harness can test the exact code the firmware
	// runs. feedRateStepClocks is the requested feed rate in mm/step-clock
	// (as stored in RawMove.feedRate); for inverse-time moves it is the move
	// duration in step clocks. linearDistanceMm is the move's linear (XYZ)
	// distance in mm; r is the extrusion ratio (|movement|/linearDistance,
	// >= 0); maxInstantDv is the E jerk in mm/step-clock. The feed rate is
	// converted to mm/s here — it is stored in step clocks, NOT mm/s, and
	// passing mm/s would understate the speed 750,000x and return the
	// max-slope cap (~708 mm/s^2 for a typical Bowden tune). Returns the cap,
	// or the maximum float (no cap) when the move has no extrusion or no jerk
	// limit; an unknown/invalid speed falls back to the max-slope cap (safe
	// but conservative), matching the DDA's previous fallback.
	float GetPressureAdvanceCapXY(float feedRateStepClocks, bool inverseTimeMode, float linearDistanceMm, float r, float maxInstantDv) const noexcept;

	// Get the refined XY acceleration cap (mm/step-clock^2) imposed by
	// pressure advance for a move with the given actual speeds. This is the
	// lookahead refinement cap, factored out so the host harness can test the
	// exact code the firmware runs. startSpeedStepClocks, endSpeedStepClocks
	// and topSpeedStepClocks are the move's speeds in mm/step-clock (as
	// stored in the DDA); r is the extrusion ratio (directionVector[drive],
	// >= 0); maxInstantDv is the E jerk in mm/step-clock. The speeds are
	// converted to filament mm/s here (they are stored in step clocks, NOT
	// mm/s). The chord is taken over [min(start, end), top] — a real move
	// usually starts and ends at junction speeds, so this chord is lower
	// than the Init cap's [0, feed rate] chord and the cap can be raised.
	// Returns the cap, or the maximum float (no cap) when the move has no
	// extrusion, no jerk limit, or no top speed; an unknown/invalid range
	// falls back to the max-slope cap (safe but conservative).
	float GetPressureAdvanceCapXYRange(float startSpeedStepClocks, float endSpeedStepClocks, float topSpeedStepClocks, float r, float maxInstantDv) const noexcept;

	// Get the configured linear PA constant in step clocks (kLin only, not
	// the max slope). Used by the object model, which reports the configured
	// PA value; the max slope stays internal to the acceleration cap.
	float GetKlinClocks() const noexcept { return (float)kLin; }

	// Set reciprocal parameters. params.kLin is in seconds, params.offset in
	// mm of filament, params.vLin in mm/s of filament. Invalid parameters
	// are ignored, leaving the previous values in place.
	void SetParameters(const PressureAdvanceParameters& params) noexcept;
	// Set single-slope (linear) pressure advance, value in seconds. This is what
	// remote CAN toolboards using the old single-float protocol also use.
	// Invalid values (non-finite, negative, absurd) are ignored.
	// NOTE: upstream 3.7's SetParametersSimple stores its argument raw as step clocks
	// (and its V1 CAN handler passes it seconds, which looks like an upstream bug);
	// this port takes seconds and scales by StepClockRate, matching 3.6 SetKseconds.
	void SetParametersSimple(float seconds) noexcept;
	void UpdateStepsPerMm(float stepsPerMm) noexcept;

	bool IsActive() const noexcept { return kLin != (motioncalc_t)0.0 || offset != (motioncalc_t)0.0; }
	motioncalc_t GetPressureAdvanceDistance(motioncalc_t speed) const noexcept;
	motioncalc_t GetAverageAdvanceClocks(motioncalc_t lowSpeed, motioncalc_t highSpeed, motioncalc_t stepsPerMmMovement) const noexcept
		pre(highSpeed >= lowSpeed);

	void AppendParameters(const StringRef& reply) const noexcept;

private:
	// Specified parameters
	motioncalc_t kLin;					// the linear pressure advance constant in step clocks
	motioncalc_t offset;				// the reciprocal offset in mm of filament
	motioncalc_t vLin;					// the linearization velocity in mm per step clock

	motioncalc_t mmPerStep = (motioncalc_t)0.0;		// the reciprocal of steps/mm for this extruder; 0 until configured, so an unconfigured shaper falls back to linear PA instead of reading an indeterminate value
};

#endif /* SRC_MOVEMENT_EXTRUDERSHAPER_H_ */
