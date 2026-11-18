/*
 * PressureAdvanceShaper.cpp
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#include "ExtruderShaper.h"
#include <cmath>

// Validate a parameter set: all values finite and in range, and the effective
// low-speed slope kLin + offset/vLin within the cap. vLin == 0 is allowed
// only when offset == 0 (pure linear PA, as set by SetParametersSimple).
bool ExtruderShaper::ParametersAreValid(const PressureAdvanceParameters& params) noexcept
{
	if (!std::isfinite(params.kLin) || params.kLin < 0.0f || params.kLin > MaxKlinSeconds)
	{
		return false;
	}
	if (!std::isfinite(params.offset) || params.offset < 0.0f || params.offset > MaxOffsetMm)
	{
		return false;
	}
	if (!std::isfinite(params.vLin) || params.vLin < 0.0f || params.vLin > MaxVlinMmPerSec)
	{
		return false;
	}
	if (params.offset > 0.0f)
	{
		if (params.vLin < MinVlinMmPerSec)
		{
			return false;
		}
		if (params.kLin + params.offset/params.vLin > MaxEffectiveSeconds)
		{
			return false;
		}
	}
	return true;
}

// Get the maximum pressure advance slope (at zero speed) in step clocks:
// K_eff(0) = kLin + offset/vLin. With offset = 0 this is just kLin.
float ExtruderShaper::GetMaxAdvanceClocks() const noexcept
{
	const float result = (offset == (motioncalc_t)0.0 || vLin == (motioncalc_t)0.0)
							? (float)kLin
							: (float)(kLin + offset/vLin);
	return std::isfinite(result) ? result : (float)kLin;	// defence in depth: never hand a non-finite slope to the accel cap
}

// Get the chord slope of D(v) between filament speeds 0 and
// highFilamentSpeedMmPerSec (mm/s of filament), in step clocks.
// (D(vh) - D(0))/vh = D(vh)/vh = kLin + offset/(vh + vLin), because
// D(vh) = kLin*vh + offset*vh/(vh + vLin). As vh -> 0 this tends to the
// zero-speed maximum kLin + offset/vLin; as vh -> infinity it tends to kLin.
// This is the slope the DDA acceleration cap must use for a move that
// accelerates from rest to that filament speed. It needs no steps/mm
// conversion because the speed is already a filament speed.
// Caveat (documented in DDA.cpp): the move may not reach the requested feed
// rate (short move, lookahead), in which case the true chord slope over its
// narrower range is higher than the one returned here, so the cap is
// optimistic for very short low-speed moves. The consequence is bounded
// (a possible extruder skip on such a move, not a crash), and the DDA
// merges tiny speed changes so infinitesimal intervals never form an
// accel phase.
float ExtruderShaper::GetChordSlopeToFilamentSpeed(float highFilamentSpeedMmPerSec) const noexcept
{
	if (offset == (motioncalc_t)0.0 || vLin == (motioncalc_t)0.0)
	{
		return (float)kLin;
	}
	if (!std::isfinite(highFilamentSpeedMmPerSec) || highFilamentSpeedMmPerSec < 0.0f)
	{
		return GetMaxAdvanceClocks();		// unknown speed: fall back to the safe maximum
	}
	// high speed in mm per step clock, to match the stored vLin.
	const motioncalc_t vh = (motioncalc_t)(highFilamentSpeedMmPerSec / StepClockRate);
	const motioncalc_t result = kLin + offset/(vh + vLin);
	if (!std::isfinite((float)result))
	{
		return GetMaxAdvanceClocks();
	}
	// Clamp to the analytic range [kLin, max slope]: kLin + offset/vLin.
	const motioncalc_t maxSlope = GetMaxAdvanceClocks();
	return (float)min<motioncalc_t>(max<motioncalc_t>(result, kLin), maxSlope);
}

// Get the chord slope of D(v) between two filament speeds (mm/s of filament),
// in step clocks. (D(vh) - D(vl))/(vh - vl) = kLin + offset*vLin/((vh + vLin)
// * (vl + vLin)), the same analytic chord GetAverageAdvanceClocks computes,
// but taking filament speeds directly so no steps/mm conversion is needed.
// This is the slope the lookahead refinement of the DDA acceleration cap uses
// once the move's actual start/end/top speeds are known. As vh -> vl this
// tends to the derivative D'(v); as vl -> 0 it reduces to
// GetChordSlopeToFilamentSpeed(vh).
float ExtruderShaper::GetChordSlopeFilamentRange(float lowFilamentSpeedMmPerSec, float highFilamentSpeedMmPerSec) const noexcept
{
	if (offset == (motioncalc_t)0.0 || vLin == (motioncalc_t)0.0)
	{
		return (float)kLin;
	}
	if (!std::isfinite(lowFilamentSpeedMmPerSec) || !std::isfinite(highFilamentSpeedMmPerSec)
		|| lowFilamentSpeedMmPerSec < 0.0f || highFilamentSpeedMmPerSec < 0.0f
		|| lowFilamentSpeedMmPerSec > highFilamentSpeedMmPerSec)
	{
		return GetMaxAdvanceClocks();		// unknown or inverted range: fall back to the safe maximum
	}
	// Speeds in mm per step clock, to match the stored vLin.
	const motioncalc_t vl = (motioncalc_t)(lowFilamentSpeedMmPerSec / StepClockRate);
	const motioncalc_t vh = (motioncalc_t)(highFilamentSpeedMmPerSec / StepClockRate);
	const motioncalc_t result = kLin + offset * vLin/((vh + vLin) * (vl + vLin));
	if (!std::isfinite((float)result))
	{
		return GetMaxAdvanceClocks();
	}
	// Clamp to the analytic range [kLin, max slope]: kLin + offset/vLin.
	const motioncalc_t maxSlope = GetMaxAdvanceClocks();
	return (float)min<motioncalc_t>(max<motioncalc_t>(result, kLin), maxSlope);
}

// Get the XY acceleration cap (mm/step-clock^2) imposed by pressure advance
// for a move with the given feed rate. This is the DDA Init cap, factored
// out so the host harness can test the exact code the firmware runs.
// feedRateStepClocks is the requested feed rate in mm/step-clock (as stored
// in RawMove.feedRate); for inverse-time moves it is the move duration in
// step clocks. The conversion to mm/s happens here, and the parameter name
// says step clocks, so a caller cannot silently pass mm/s (which would
// understate the speed 750,000x and return the max-slope cap).
float ExtruderShaper::GetPressureAdvanceCapXY(float feedRateStepClocks, bool inverseTimeMode, float linearDistanceMm, float r, float maxInstantDv) const noexcept
{
	if (linearDistanceMm <= 0.0f || r <= 0.0f || maxInstantDv <= 0.0f)
	{
		return std::numeric_limits<float>::max();		// no extrusion or no jerk limit: no cap
	}
	// feedRate is in mm/step-clock (RawMove.h); convert to mm/s. For
	// inverse-time moves feedRate is a duration in step clocks, so
	// linearDistance/feedRate is mm/step-clock; convert that too.
	const float reqSpeedMmPerSec = (inverseTimeMode)
		? ((feedRateStepClocks > 0.0f) ? (linearDistanceMm/feedRateStepClocks) * (float)StepClockRate : 0.0f)
		: feedRateStepClocks * (float)StepClockRate;
	float compensationClocks = GetMaxAdvanceClocks();	// fallback: max slope (safe but conservative)
	if (std::isfinite(reqSpeedMmPerSec) && reqSpeedMmPerSec > 0.0f)
	{
		compensationClocks = GetChordSlopeToFilamentSpeed(reqSpeedMmPerSec * r);
	}
	if (!(compensationClocks > 0.0f))
	{
		return std::numeric_limits<float>::max();		// no PA: no cap
	}
	const float cap = maxInstantDv/(compensationClocks * r);
	return std::isfinite(cap) ? cap : std::numeric_limits<float>::max();
}

// Get the refined XY acceleration cap (mm/step-clock^2) imposed by pressure
// advance for a move with the given actual speeds. This is the lookahead
// refinement cap, factored out so the host harness can test the exact code
// the firmware runs. The DDA speeds are in mm/step-clock; they are converted
// to filament mm/s here (the parameter names say step clocks, so a caller
// cannot silently pass mm/s). The chord is over [min(start, end), top].
float ExtruderShaper::GetPressureAdvanceCapXYRange(float startSpeedStepClocks, float endSpeedStepClocks, float topSpeedStepClocks, float r, float maxInstantDv) const noexcept
{
	if (r <= 0.0f || maxInstantDv <= 0.0f)
	{
		return std::numeric_limits<float>::max();		// no extrusion or no jerk limit: no cap
	}
	// DDA speeds are in mm/step-clock; convert to filament mm/s.
	const float startFil = startSpeedStepClocks * (float)StepClockRate * r;
	const float endFil = endSpeedStepClocks * (float)StepClockRate * r;
	const float topFil = topSpeedStepClocks * (float)StepClockRate * r;
	if (topFil <= 0.0f)
	{
		return std::numeric_limits<float>::max();		// no top speed: no range to refine over
	}
	// Worst chord over the two phases: from the lower of the two junction
	// speeds up to the top speed. GetChordSlopeFilamentRange falls back to
	// the max slope for an unknown/invalid range (safe but conservative).
	const float lowFil = min<float>(startFil, endFil);
	const float chord = GetChordSlopeFilamentRange(lowFil, topFil);
	if (!(chord > 0.0f))
	{
		return std::numeric_limits<float>::max();		// no PA: no cap
	}
	const float cap = maxInstantDv/(chord * r);
	return std::isfinite(cap) ? cap : std::numeric_limits<float>::max();
}

// Set the pressure advance parameters. Invalid parameters are ignored,
// leaving the previous known-good values in place.
void ExtruderShaper::SetParameters(const PressureAdvanceParameters& params) noexcept
{
	if (!ParametersAreValid(params))
	{
		return;
	}
	// The three fields must be written atomically: the Move task reads them
	// in Prepare (via GetAverageAdvanceClocks) and the CAN task can call
	// SetParametersSimple concurrently, so a torn write would mix old and new
	// parameters mid-move. AtomicCriticalSectionLocker only saves/disables
	// interrupts (PRIMASK), so it is safe here, including from the
	// constructor during static initialisation.
	AtomicCriticalSectionLocker lock;
	kLin = (motioncalc_t)(params.kLin * StepClockRate);
	offset = (motioncalc_t)params.offset;
	// vLin arrives in mm/s of filament; store it in mm per step clock to match
	// the speed units used by GetAverageAdvanceClocks.
	vLin = (motioncalc_t)(params.vLin / StepClockRate);
}

// Set single-slope pressure advance, value in seconds. Invalid values are ignored.
void ExtruderShaper::SetParametersSimple(float seconds) noexcept
{
	if (!std::isfinite(seconds) || seconds < 0.0f || seconds > MaxKlinSeconds)
	{
		return;
	}
	AtomicCriticalSectionLocker lock;		// see SetParameters: the write must be atomic vs Prepare
	kLin = (motioncalc_t)(seconds * StepClockRate);
	offset = (motioncalc_t)0.0;
	vLin = (motioncalc_t)0.0;
}

void ExtruderShaper::UpdateStepsPerMm(float stepsPerMm) noexcept
{
	// Guard against zero/invalid steps/mm (e.g. M92 E0): mmPerStep = 0 makes
	// GetAverageAdvanceClocks fall back to the linear value kLin instead of
	// producing inf/NaN speeds.
	mmPerStep = (std::isfinite(stepsPerMm) && stepsPerMm > 0.0f) ? (motioncalc_t)1.0/(motioncalc_t)stepsPerMm : (motioncalc_t)0.0;
}

// Get the pressure advance distance for a given extrusion speed. This is not currently used.
motioncalc_t ExtruderShaper::GetPressureAdvanceDistance(motioncalc_t speed) const noexcept
{
	if (!std::isfinite((float)speed) || speed < (motioncalc_t)0.0)
	{
		return (motioncalc_t)0.0;
	}
	if (offset == (motioncalc_t)0.0 || vLin == (motioncalc_t)0.0)
	{
		return kLin * speed;
	}
	const motioncalc_t result = kLin * speed + offset * speed/(speed + vLin);
	return std::isfinite((float)result) ? result : (motioncalc_t)0.0;
}

// Get the average number of pressure advance clocks for a move segment that changes speed. Must have lowSpeed <= highSpeed.
// This is the exact chord slope of D(v) between lowSpeed and highSpeed:
//		(D(vh) - D(vl))/(vh - vl) = kLin + offset*vLin/((vh + vLin)*(vl + vLin))
// because vh/(vh + vLin) - vl/(vl + vLin) = vLin*(vh - vl)/((vh + vLin)*(vl + vLin)).
// The (vh - vl) cancels analytically, so this is also exact when vh == vl,
// where it equals the derivative D'(v) = kLin + offset*vLin/(v + vLin)^2.
// One division per evaluation, no transcendental functions.
// The chord slope always lies in [kLin, kLin + offset/vLin] for non-negative
// speeds, so the result is clamped to that range: a misconfiguration can never
// demand an absurd extruder speed or acceleration from this function.
motioncalc_t ExtruderShaper::GetAverageAdvanceClocks(motioncalc_t lowSpeed, motioncalc_t highSpeed, motioncalc_t stepsPerMmMovement) const noexcept
{
	// Fall back to the linear value if the speed conversion is unusable
	// (unconfigured or invalid steps/mm) instead of propagating inf/NaN.
	if (!std::isfinite((float)mmPerStep) || mmPerStep <= (motioncalc_t)0.0
		|| !std::isfinite((float)stepsPerMmMovement) || stepsPerMmMovement < (motioncalc_t)0.0)
	{
		return kLin;
	}
	if (offset == (motioncalc_t)0.0 || vLin == (motioncalc_t)0.0)
	{
		return kLin;
	}

	// Speeds are magnitudes; clamp defensively so (v + vLin) can never be zero.
	const motioncalc_t actualLowSpeed = max<motioncalc_t>(lowSpeed * stepsPerMmMovement * mmPerStep, (motioncalc_t)0.0);
	const motioncalc_t actualHighSpeed = max<motioncalc_t>(highSpeed * stepsPerMmMovement * mmPerStep, (motioncalc_t)0.0);

	const motioncalc_t result = kLin + offset * vLin/((actualHighSpeed + vLin) * (actualLowSpeed + vLin));
	if (!std::isfinite((float)result))
	{
		return kLin;
	}
	// Clamp to the analytic range [kLin, max slope]: kLin + offset/vLin.
	const motioncalc_t maxSlope = GetMaxAdvanceClocks();
	return min<motioncalc_t>(max<motioncalc_t>(result, kLin), maxSlope);
}

// Append the pressure advance parameters to a string. Linear PA reports as a
// single value (as before); reciprocal PA reports (kLin offset vLin) in user units.
void ExtruderShaper::AppendParameters(const StringRef& reply) const noexcept
{
	if (offset == (motioncalc_t)0.0)
	{
		reply.catf("%.3f", (double)(kLin * StepClocksToSeconds));
	}
	else
	{
		reply.catf("(%.3f %.3f %.2f)", (double)(kLin * StepClocksToSeconds), (double)offset, (double)(vLin * StepClockRate));
	}
}

// End
