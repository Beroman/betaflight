/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * QuickTune-inspired PID autotune for Betaflight.
 *
 * Algorithm summary:
 *  1. Snapshot the current PID gains as the baseline.
 *  2. For each axis (roll, pitch, optionally yaw):
 *     a. D-gain ramp: exponentially increase D while monitoring the PID
 *        output slew-rate.  When the slew-rate exceeds the oscillation
 *        threshold, scale D back by the configured gain margin.
 *     b. P-gain ramp: same procedure for P.
 *  3. Derive I from P using the configured P:I ratio.
 *  4. Write the new gains to the active PID profile and optionally save
 *     to EEPROM.
 *
 * Designed to run while the pilot holds the quad in Angle mode.  No special
 * excitation signal is injected — the normal stick input and angle-mode
 * corrections are sufficient because the gain ramp itself will eventually
 * push the loop into oscillation.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_AUTOTUNE

#include "build/debug.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "config/config.h"

#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/autotune.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "io/beeper.h"

// ---------------------------------------------------------------------------
// Compile-time defaults for the tuneable parameters (overridden by pidProfile)
// ---------------------------------------------------------------------------

// Exponential gain ramp: gain is multiplied by gainRampRate every second.
// 1.07 means the gain doubles in ~10 seconds.
#define AUTOTUNE_DEFAULT_GAIN_RAMP_RATE       1.07f

// After oscillation is detected, keep this fraction of the oscillation gain.
// 0.60 = 60% → 40% safety margin.
#define AUTOTUNE_DEFAULT_GAIN_MARGIN          0.60f

// PID output slew-rate threshold for oscillation detection (deg/s/s, after
// low-pass filtering).  This is compared against a first-derivative metric of
// the PID output; when the filtered derivative exceeds this value the axis is
// considered oscillating.
#define AUTOTUNE_DEFAULT_OSC_SLEW_THRESHOLD   25.0f

// I is derived from P:  I = P * piRatio.
#define AUTOTUNE_DEFAULT_PI_RATIO             0.80f

// Maximum allowed gain multiplier above the original gain.
#define AUTOTUNE_DEFAULT_MAX_GAIN_MULT        4.0f

// Settle time (ms) after changing a gain before we start measuring.
#define AUTOTUNE_DEFAULT_SETTLE_TIME_MS       500

// Maximum time (ms) allowed per axis per tuning phase.
#define AUTOTUNE_DEFAULT_TUNE_TIMEOUT_MS      30000

// Low-pass coefficient for the slew-rate metric.  Larger = more smoothing.
// Applied as an exponential moving average: metric = alpha*new + (1-alpha)*old.
// We want roughly a 50 Hz cutoff at 4 kHz PID rate → alpha ≈ 0.075.
#define SLEW_RATE_FILTER_ALPHA                0.075f

// Minimum gain value (raw uint8 scale) to avoid setting gains to zero.
#define MIN_GAIN_VALUE                        5

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

static autotuneState_t autotuneState;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint32_t currentTimeMs(void)
{
    return millis();
}

// Convert a float gain back to Betaflight's uint8_t (0-250) representation.
static uint8_t floatToGainU8(float gain, float scale)
{
    float raw = gain / scale;
    raw = constrainf(raw, MIN_GAIN_VALUE, PID_GAIN_MAX);
    return (uint8_t)lrintf(raw);
}

// Read the current raw PID gain for an axis and convert to float.
static float readGainFloat(int axis, int term)
{
    const pidProfile_t *pp = currentPidProfile;
    switch (term) {
    case 0: return pp->pid[axis].P * PTERM_SCALE;
    case 1: return pp->pid[axis].I * ITERM_SCALE;
    case 2: return pp->pid[axis].D * DTERM_SCALE;
    case 3: return pp->pid[axis].F * FEEDFORWARD_SCALE * 0.01f;
    default: return 0.0f;
    }
}

// Write a float gain back to the live PID profile and reinitialise.
static void writeGainU8(int axis, int term, float value, float scale)
{
    pidProfile_t *pp = pidProfilesMutable(systemConfig()->pidProfileIndex);
    uint8_t u8val = floatToGainU8(value, scale);
    switch (term) {
    case 0: pp->pid[axis].P = u8val; break;
    case 1: pp->pid[axis].I = u8val; break;
    case 2: pp->pid[axis].D = u8val; break;
    default: break;
    }
    pidInitConfig(pp);
}

// ---------------------------------------------------------------------------
// Snapshot / restore original gains
// ---------------------------------------------------------------------------

static void snapshotGains(int axis)
{
    autotuneAxisState_t *as = &autotuneState.axisState[axis];
    as->originalP = readGainFloat(axis, 0);
    as->originalI = readGainFloat(axis, 1);
    as->originalD = readGainFloat(axis, 2);
    as->originalF = readGainFloat(axis, 3);
    as->bestP = as->originalP;
    as->bestI = as->originalI;
    as->bestD = as->originalD;
    as->bestF = as->originalF;
}

static void restoreOriginalGains(int axis)
{
    autotuneAxisState_t *as = &autotuneState.axisState[axis];
    writeGainU8(axis, 0, as->originalP, PTERM_SCALE);
    writeGainU8(axis, 1, as->originalI, ITERM_SCALE);
    writeGainU8(axis, 2, as->originalD, DTERM_SCALE);
}

// ---------------------------------------------------------------------------
// Oscillation detection
// ---------------------------------------------------------------------------

// Update the slew-rate metric from the current PID output.
// Returns true if oscillation is detected.
static bool updateOscillation(autotuneAxisState_t *as, float pidOutput, float dt)
{
    if (dt <= 0.0f) {
        return false;
    }

    // First derivative of PID output (slew rate)
    const float slewRate = fabsf(pidOutput - as->pidOutputPrev) / dt;
    as->pidOutputPrev = pidOutput;

    // Low-pass filter the slew rate
    as->oscillationMetric += SLEW_RATE_FILTER_ALPHA * (slewRate - as->oscillationMetric);

    // Check if the filtered metric has risen above the threshold
    if (as->oscillationMetric > as->oscillationThreshold) {
        as->oscillationDetected = true;
        as->oscillationTimeMs = currentTimeMs();
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Gain ramping
// ---------------------------------------------------------------------------

// Exponentially ramp a gain: gain *= rampRate ^ dt.
static float rampGain(float currentGain, float rampRate, float dt)
{
    return currentGain * powf(rampRate, dt);
}

// ---------------------------------------------------------------------------
// Apply tuned gains to the live profile
// ---------------------------------------------------------------------------

static void applyBestGains(int axis)
{
    autotuneAxisState_t *as = &autotuneState.axisState[axis];

    writeGainU8(axis, 0, as->bestP, PTERM_SCALE);
    writeGainU8(axis, 1, as->bestI, ITERM_SCALE);
    writeGainU8(axis, 2, as->bestD, DTERM_SCALE);
}

// ---------------------------------------------------------------------------
// Phase handlers
// ---------------------------------------------------------------------------

static void phaseInit(void)
{
    autotuneState.currentAxis = AUTOTUNE_AXIS_ROLL;

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        snapshotGains(axis);

        autotuneAxisState_t *as = &autotuneState.axisState[axis];
        as->tuneComplete = false;
        as->oscillationDetected = false;
        as->pidOutputPrev = 0.0f;
        as->oscillationMetric = 0.0f;
        as->oscillationMetricPrev = 0.0f;
        as->rampStartTimeMs = 0;
        as->oscillationTimeMs = 0;
        as->oscillationThreshold = autotuneState.oscillationSlewThreshold;
    }

    // Start with D-gain tuning on the first axis
    autotuneState.phase = AUTOTUNE_PHASE_TUNE_D;
    autotuneState.phaseStartTimeMs = currentTimeMs();

    // Prepare D-gain ramp for the current axis
    autotuneAxisState_t *as = &autotuneState.axisState[autotuneState.currentAxis];
    as->currentTestGain = as->originalD;
    as->lastTestGain = as->originalD;
    as->rampStartTimeMs = currentTimeMs();
    as->oscillationDetected = false;
    as->oscillationMetric = 0.0f;

    beeper(BEEPER_READY_BEEP);
}

static void phaseTuneD(int axis, float pidOutput, float dt)
{
    autotuneAxisState_t *as = &autotuneState.axisState[axis];
    const uint32_t now = currentTimeMs();

    if ((int)axis != (int)autotuneState.currentAxis) {
        return;  // Only process the axis we are currently tuning
    }

    // Wait for settle time
    if ((now - as->rampStartTimeMs) < autotuneState.settleTimeMs) {
        as->pidOutputPrev = pidOutput;
        return;
    }

    // Check timeout
    if ((now - autotuneState.phaseStartTimeMs) > autotuneState.tuneTimeoutMs) {
        // Timeout — keep best D found so far (which is the original if no oscillation was found)
        as->bestD = as->currentTestGain * autotuneState.gainMargin;
        if (as->bestD < as->originalD * 0.5f) {
            as->bestD = as->originalD;  // Don't reduce below 50% of original
        }
        goto nextPhase;
    }

    // Check for maximum gain multiplier
    if (as->currentTestGain > as->originalD * autotuneState.maxGainMultiplier) {
        // Hit the ceiling — use current gain with margin
        as->bestD = as->currentTestGain * autotuneState.gainMargin;
        goto nextPhase;
    }

    // Ramp the gain
    as->currentTestGain = rampGain(as->currentTestGain, autotuneState.gainRampRate, dt);

    // Apply the ramped D gain in real-time
    writeGainU8(axis, 2, as->currentTestGain, DTERM_SCALE);

    // Check for oscillation
    if (updateOscillation(as, pidOutput, dt)) {
        // Oscillation detected — apply margin and move on
        as->bestD = as->lastTestGain * autotuneState.gainMargin;
        if (as->bestD < as->originalD * 0.5f) {
            as->bestD = as->originalD;
        }
        // Restore a safe D value immediately
        writeGainU8(axis, 2, as->bestD, DTERM_SCALE);
        goto nextPhase;
    }

    as->lastTestGain = as->currentTestGain;
    return;

nextPhase:
    // Apply the best D found
    writeGainU8(axis, 2, as->bestD, DTERM_SCALE);

    // Move to P tuning for this axis
    autotuneState.phase = AUTOTUNE_PHASE_TUNE_P;
    autotuneState.phaseStartTimeMs = now;
    as->currentTestGain = as->originalP;
    as->lastTestGain = as->originalP;
    as->rampStartTimeMs = now;
    as->oscillationDetected = false;
    as->oscillationMetric = 0.0f;
    as->pidOutputPrev = pidOutput;
}

static void phaseTuneP(int axis, float pidOutput, float dt)
{
    autotuneAxisState_t *as = &autotuneState.axisState[axis];
    const uint32_t now = currentTimeMs();

    if ((int)axis != (int)autotuneState.currentAxis) {
        return;
    }

    // Wait for settle time
    if ((now - as->rampStartTimeMs) < autotuneState.settleTimeMs) {
        as->pidOutputPrev = pidOutput;
        return;
    }

    // Check timeout
    if ((now - autotuneState.phaseStartTimeMs) > autotuneState.tuneTimeoutMs) {
        as->bestP = as->currentTestGain * autotuneState.gainMargin;
        if (as->bestP < as->originalP * 0.5f) {
            as->bestP = as->originalP;
        }
        goto nextAxis;
    }

    // Check ceiling
    if (as->currentTestGain > as->originalP * autotuneState.maxGainMultiplier) {
        as->bestP = as->currentTestGain * autotuneState.gainMargin;
        goto nextAxis;
    }

    // Ramp P
    as->currentTestGain = rampGain(as->currentTestGain, autotuneState.gainRampRate, dt);

    // Apply the ramped P gain in real-time
    writeGainU8(axis, 0, as->currentTestGain, PTERM_SCALE);

    // Check oscillation
    if (updateOscillation(as, pidOutput, dt)) {
        as->bestP = as->lastTestGain * autotuneState.gainMargin;
        if (as->bestP < as->originalP * 0.5f) {
            as->bestP = as->originalP;
        }
        writeGainU8(axis, 0, as->bestP, PTERM_SCALE);
        goto nextAxis;
    }

    as->lastTestGain = as->currentTestGain;
    return;

nextAxis:
    // Derive I from P
    as->bestI = as->bestP * autotuneState.piRatio;

    // D-P coupling: if best D < original D, reduce P proportionally
    if (as->bestD < as->originalD * 0.95f && as->originalD > 0.0f) {
        float dRatio = as->bestD / as->originalD;
        as->bestP *= dRatio;
        as->bestI *= dRatio;
    }

    // Apply best gains for this axis
    applyBestGains(axis);
    as->tuneComplete = true;

    // Advance to next axis
    int nextAxisIdx = (int)autotuneState.currentAxis + 1;

    // Skip yaw if not configured
    if (nextAxisIdx == AUTOTUNE_AXIS_YAW && !autotuneState.tuneYaw) {
        nextAxisIdx++;
    }

    if (nextAxisIdx >= AUTOTUNE_AXIS_COUNT) {
        // All axes done — move to save phase
        autotuneState.phase = AUTOTUNE_PHASE_SAVE;
        autotuneState.phaseStartTimeMs = currentTimeMs();
    } else {
        // Start D tuning on next axis
        autotuneState.currentAxis = (autotuneAxis_e)nextAxisIdx;
        autotuneState.phase = AUTOTUNE_PHASE_TUNE_D;
        autotuneState.phaseStartTimeMs = currentTimeMs();

        autotuneAxisState_t *nextAs = &autotuneState.axisState[nextAxisIdx];
        nextAs->currentTestGain = nextAs->originalD;
        nextAs->lastTestGain = nextAs->originalD;
        nextAs->rampStartTimeMs = currentTimeMs();
        nextAs->oscillationDetected = false;
        nextAs->oscillationMetric = 0.0f;
        nextAs->pidOutputPrev = 0.0f;
    }
}

static void phaseSave(void)
{
    // Save to EEPROM
    if (!autotuneState.savedToEeprom) {
        writeEEPROM();
        autotuneState.savedToEeprom = true;
    }

    autotuneState.phase = AUTOTUNE_PHASE_COMPLETE;
    beeper(BEEPER_READY_BEEP);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void autotuneInit(void)
{
    memset(&autotuneState, 0, sizeof(autotuneState));
    autotuneState.phase = AUTOTUNE_PHASE_IDLE;
}

void autotuneStartTuning(void)
{
    if (autotuneState.phase != AUTOTUNE_PHASE_IDLE &&
        autotuneState.phase != AUTOTUNE_PHASE_COMPLETE &&
        autotuneState.phase != AUTOTUNE_PHASE_FAILED) {
        return;  // Already running
    }

    // Load configuration from the current PID profile
    const pidProfile_t *pp = currentPidProfile;
    autotuneState.gainRampRate = pp->autotune_gain_ramp_rate / 100.0f;      // stored as 107 → 1.07
    autotuneState.gainMargin = pp->autotune_gain_margin / 100.0f;           // stored as 60 → 0.60
    autotuneState.oscillationSlewThreshold = (float)pp->autotune_osc_threshold;
    autotuneState.piRatio = pp->autotune_pi_ratio / 100.0f;                 // stored as 80 → 0.80
    autotuneState.maxGainMultiplier = pp->autotune_max_gain_multiplier / 10.0f; // stored as 40 → 4.0
    autotuneState.settleTimeMs = pp->autotune_settle_time_ms;
    autotuneState.tuneTimeoutMs = pp->autotune_timeout_ms;
    autotuneState.tuneYaw = pp->autotune_tune_yaw;
    autotuneState.savedToEeprom = false;

    // Sanity clamp
    if (autotuneState.gainRampRate < 1.01f) autotuneState.gainRampRate = 1.01f;
    if (autotuneState.gainRampRate > 1.50f) autotuneState.gainRampRate = 1.50f;
    if (autotuneState.gainMargin < 0.30f) autotuneState.gainMargin = 0.30f;
    if (autotuneState.gainMargin > 0.90f) autotuneState.gainMargin = 0.90f;

    autotuneState.phase = AUTOTUNE_PHASE_INIT;
    phaseInit();
}

void autotuneStopTuning(void)
{
    if (autotuneState.phase == AUTOTUNE_PHASE_IDLE) {
        return;
    }

    if (autotuneState.phase == AUTOTUNE_PHASE_COMPLETE) {
        // Gains were already saved — nothing to revert
        autotuneState.phase = AUTOTUNE_PHASE_IDLE;
        return;
    }

    // Revert gains to originals if we were interrupted
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        restoreOriginalGains(axis);
    }

    autotuneState.phase = AUTOTUNE_PHASE_IDLE;
    beeper(BEEPER_RX_SET);
}

void autotuneUpdateAxis(int axis, float pidOutput, timeUs_t currentTimeUs)
{
    (void)currentTimeUs;

    if (autotuneState.phase == AUTOTUNE_PHASE_IDLE ||
        autotuneState.phase == AUTOTUNE_PHASE_COMPLETE ||
        autotuneState.phase == AUTOTUNE_PHASE_FAILED) {
        return;
    }

    // Only armed tuning
    if (!ARMING_FLAG(ARMED)) {
        return;
    }

    // Require angle mode for safety
    if (!FLIGHT_MODE(ANGLE_MODE)) {
        return;
    }

    const float dt = pidGetDT();

    switch (autotuneState.phase) {
    case AUTOTUNE_PHASE_TUNE_D:
        phaseTuneD(axis, pidOutput, dt);
        break;

    case AUTOTUNE_PHASE_TUNE_P:
        phaseTuneP(axis, pidOutput, dt);
        break;

    case AUTOTUNE_PHASE_SAVE:
        if (axis == FD_ROLL) {  // Only do this once per PID loop
            phaseSave();
        }
        break;

    default:
        break;
    }

    // Debug output
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_AUTOTUNE, 0, lrintf(autotuneState.phase));
        DEBUG_SET(DEBUG_AUTOTUNE, 1, lrintf(autotuneState.currentAxis));
        DEBUG_SET(DEBUG_AUTOTUNE, 2, lrintf(autotuneState.axisState[autotuneState.currentAxis].oscillationMetric * 10.0f));
        DEBUG_SET(DEBUG_AUTOTUNE, 3, lrintf(autotuneState.axisState[autotuneState.currentAxis].currentTestGain * 10000.0f));
    }
}

autotunePhase_e autotuneGetPhase(void)
{
    return autotuneState.phase;
}

bool autotuneIsActive(void)
{
    return autotuneState.phase != AUTOTUNE_PHASE_IDLE;
}

bool autotuneIsComplete(void)
{
    return autotuneState.phase == AUTOTUNE_PHASE_COMPLETE;
}

autotuneAxis_e autotuneGetCurrentAxis(void)
{
    return autotuneState.currentAxis;
}

uint8_t autotuneGetProgress(void)
{
    // Rough progress: each axis has D + P phase.
    // Total phases = (tuneYaw ? 3 : 2) axes * 2 phases = 4 or 6
    const int totalAxes = autotuneState.tuneYaw ? 3 : 2;
    const int axesDone = (int)autotuneState.currentAxis;
    int phasesPerAxis = 2;  // D + P

    int completedPhases = axesDone * phasesPerAxis;
    if (autotuneState.phase == AUTOTUNE_PHASE_TUNE_P) {
        completedPhases += 1;  // D done for current axis
    } else if (autotuneState.phase >= AUTOTUNE_PHASE_SAVE) {
        completedPhases = totalAxes * phasesPerAxis;
    }

    int totalPhases = totalAxes * phasesPerAxis;
    if (totalPhases == 0) return 0;

    return (uint8_t)constrain(completedPhases * 100 / totalPhases, 0, 100);
}

const autotuneAxisState_t *autotuneGetAxisState(int axis)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return NULL;
    }
    return &autotuneState.axisState[axis];
}

#endif // USE_AUTOTUNE
