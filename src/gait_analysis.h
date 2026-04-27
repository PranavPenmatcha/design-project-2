#pragma once
#include <stdint.h>

// Step-interval and gait-consistency features.
// "Stop gate" = boolean: is the user in a non-locomotion state?
// True locomotion requires consistent step timing AND step-shaped accel waveform.

extern float g_stepIntervalMeanMs;     // mean ms between recent steps
extern float g_stepIntervalCvPct;      // coefficient of variation (%) — gait regularity
extern bool  g_stopGate;               // true = NOT walking (shaking, still, etc.)
extern bool  g_locomotionConfirmed;    // true = consistent stride pattern detected

void onStepEvent(uint32_t nowMs);                 // call when a step is detected
void updateGaitFeatures(uint16_t steps, float cadence, float hzMag, float hzZ);
const char* stopGateName();                        // "WALKING" / "STOPPED"
