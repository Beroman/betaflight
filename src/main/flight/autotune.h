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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/time.h"

// ---- Autotune state machine states ----
typedef enum {
    AUTOTUNE_PHASE_IDLE = 0,          // Not running
    AUTOTUNE_PHASE_INIT,              // Initialising — snapshot original gains
    AUTOTUNE_PHASE_TUNE_D,            // Ramping D gain until oscillation
    AUTOTUNE_PHASE_TUNE_P,            // Ramping P gain until oscillation
    AUTOTUNE_PHASE_SAVE,              // Applying final gains and saving
    AUTOTUNE_PHASE_COMPLETE,          // Tuning complete, waiting for mode switch off
    AUTOTUNE_PHASE_FAILED,            // Tuning failed or aborted
} autotunePhase_e;

// Which axes have been tuned
typedef enum {
    AUTOTUNE_AXIS_ROLL = 0,
    AUTOTUNE_AXIS_PITCH,
    AUTOTUNE_AXIS_YAW,
    AUTOTUNE_AXIS_COUNT
} autotuneAxis_e;

// Per-axis autotune runtime state
typedef struct autotuneAxisState_s {
    float originalP;
    float originalI;
    float originalD;
    float originalF;

    float currentTestGain;          // Current gain value under test (float scale)
    float lastTestGain;             // Previous iteration test gain

    float bestP;
    float bestI;
    float bestD;
    float bestF;

    float oscillationMetric;        // Filtered PID output derivative (slew rate)
    float oscillationMetricPrev;
    float oscillationThreshold;     // Adaptive threshold for oscillation detection

    float pidOutputPrev;            // Previous PID output for slew rate calculation

    bool tuneComplete;
    bool oscillationDetected;
    uint32_t rampStartTimeMs;       // When the current ramp started
    uint32_t oscillationTimeMs;     // When oscillation was last detected
} autotuneAxisState_t;

// Main autotune state
typedef struct autotuneState_s {
    autotunePhase_e phase;
    autotuneAxis_e currentAxis;     // Which axis is currently being tuned

    autotuneAxisState_t axisState[XYZ_AXIS_COUNT];

    uint32_t phaseStartTimeMs;
    uint32_t lastUpdateTimeMs;

    bool wasArmed;
    bool savedToEeprom;

    // Configuration (copied from pidProfile at init)
    float gainRampRate;             // Multiplier per second (e.g., 1.05 = 5% per second)
    float gainMargin;               // Safety margin (e.g., 0.6 = keep 60% of oscillation gain)
    float oscillationSlewThreshold; // PID output derivative threshold for oscillation detection (deg/s/s)
    float piRatio;                  // I = P * piRatio
    float maxGainMultiplier;        // Maximum allowed gain multiplier above original
    uint16_t settleTimeMs;          // Time to wait after setting gain before measuring
    uint16_t tuneTimeoutMs;         // Maximum time per axis per phase
    bool tuneYaw;                   // Whether to tune yaw axis
} autotuneState_t;

// ---- Public API ----

// Called once from pidInit to initialise autotune structures
void autotuneInit(void);

// Called every PID loop iteration (from pidController)
// Returns adjustment to PID error if autotune is injecting anything (currently 0)
void autotuneUpdateAxis(int axis, float pidOutput, timeUs_t currentTimeUs);

// Called from processRxModes() in core.c when AUTOTUNE mode is activated/deactivated
void autotuneStartTuning(void);
void autotuneStopTuning(void);

// Query state for OSD / status
autotunePhase_e autotuneGetPhase(void);
bool autotuneIsActive(void);
bool autotuneIsComplete(void);
uint8_t autotuneGetProgress(void);  // 0-100 percent
autotuneAxis_e autotuneGetCurrentAxis(void);

// Access tuned gains for debug/display
const autotuneAxisState_t *autotuneGetAxisState(int axis);
