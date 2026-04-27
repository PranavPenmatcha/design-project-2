#pragma once
#include <stdint.h>

// Stage A: high-level activity context. This is a coarse "what is the body doing"
// label, set BEFORE any pathology classification.
enum ActivityContext {
  ACT_STILL = 0,         // motionless / seated
  ACT_STANDING_MICRO,    // standing with small motion (postural sway, light tremor)
  ACT_WALKING,           // confirmed locomotion
  ACT_IRREGULAR,         // movement but not locomotion (turning, shaking, FOG candidate)
};

extern ActivityContext g_activity;

void updateActivity(float cadence, float bandEnergyMag,
                    bool stopGate, bool locomotionConfirmed);
const char* activityName(ActivityContext a);
