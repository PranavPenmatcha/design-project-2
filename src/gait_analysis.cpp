#include "gait_analysis.h"
#include "config.h"
#include "Arduino.h"
#include <math.h>

float g_stepIntervalMeanMs  = 0.0f;
float g_stepIntervalCvPct   = 0.0f;
bool  g_stopGate            = true;
bool  g_locomotionConfirmed = false;

// Ring buffer of recent step timestamps (ms) — used to compute interval variability.
static constexpr uint8_t  STEP_WIN = 8;
static uint32_t           stepTimes[STEP_WIN] = {0};
static uint8_t            stepHead    = 0;
static uint8_t            stepFilled  = 0;
static uint16_t           lastSteps   = 0;

void onStepEvent(uint32_t nowMs) {
  stepTimes[stepHead] = nowMs;
  stepHead = (stepHead + 1) % STEP_WIN;
  if (stepFilled < STEP_WIN) stepFilled++;
}

static void computeIntervalStats() {
  if (stepFilled < 3) {
    g_stepIntervalMeanMs = 0.0f;
    g_stepIntervalCvPct  = 0.0f;
    return;
  }
  // Walk the ring oldest→newest, accumulate intervals.
  float sum = 0.0f;
  uint8_t  count = 0;
  uint32_t prev  = 0;
  for (uint8_t i = 0; i < stepFilled; i++) {
    const uint8_t idx = (stepHead + STEP_WIN - stepFilled + i) % STEP_WIN;
    const uint32_t t = stepTimes[idx];
    if (i > 0) { sum += static_cast<float>(t - prev); count++; }
    prev = t;
  }
  if (count == 0) { g_stepIntervalMeanMs = 0.0f; g_stepIntervalCvPct = 0.0f; return; }
  const float mean = sum / static_cast<float>(count);

  float sqSum = 0.0f;
  prev = 0;
  uint8_t c2 = 0;
  for (uint8_t i = 0; i < stepFilled; i++) {
    const uint8_t idx = (stepHead + STEP_WIN - stepFilled + i) % STEP_WIN;
    const uint32_t t = stepTimes[idx];
    if (i > 0) {
      const float d = static_cast<float>(t - prev) - mean;
      sqSum += d * d;
      c2++;
    }
    prev = t;
  }
  const float var = (c2 > 0) ? sqSum / static_cast<float>(c2) : 0.0f;
  const float sd  = sqrtf(var);

  g_stepIntervalMeanMs = mean;
  g_stepIntervalCvPct  = (mean > 1.0f) ? (sd / mean) * 100.0f : 0.0f;
}

void updateGaitFeatures(uint16_t steps, float cadence, float hzMag, float hzZ) {
  // Detect new steps since last call → log timestamp(s).
  if (steps > lastSteps) {
    onStepEvent(static_cast<uint32_t>(millis()));
  }
  lastSteps = steps;

  computeIntervalStats();

  // Locomotion gate:
  //   - cadence above walking threshold
  //   - dominant accel peak in stride band (0.8–3.5 Hz)
  //   - step intervals consistent (CV < 35 %)
  //   - at least 3 steps in window
  const bool strideFreq = (hzMag >= WALKING_HZ_MIN && hzMag <= WALKING_HZ_MAX) ||
                          (hzZ   >= WALKING_HZ_MIN && hzZ   <= WALKING_HZ_MAX);
  const bool intervalsOk = (stepFilled >= 3) && (g_stepIntervalCvPct < 35.0f);
  const bool cadenceOk   = (cadence >= WALKING_CADENCE_MIN);

  g_locomotionConfirmed = cadenceOk && strideFreq && intervalsOk;
  g_stopGate            = !g_locomotionConfirmed;
}

const char* stopGateName() {
  return g_stopGate ? "STOPPED" : "WALKING";
}
