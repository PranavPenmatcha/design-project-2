#include "activity_context.h"
#include "config.h"

ActivityContext g_activity = ACT_STILL;

void updateActivity(float cadence, float bandEnergyMag,
                    bool stopGate, bool locomotionConfirmed) {
  if (locomotionConfirmed) {
    g_activity = ACT_WALKING;
    return;
  }

  // Not walking. Distinguish still vs micro-motion vs irregular movement.
  if (bandEnergyMag < BAND_ENERGY_FLOOR) {
    g_activity = ACT_STILL;
    return;
  }
  if (bandEnergyMag < TREMOR_ENERGY_MIN) {
    g_activity = ACT_STANDING_MICRO;
    return;
  }
  // Energy is high but no consistent stride → irregular movement (shaking, turning, FOG).
  g_activity = ACT_IRREGULAR;
  (void)cadence; (void)stopGate;
}

const char* activityName(ActivityContext a) {
  switch (a) {
    case ACT_STILL:          return "Still";
    case ACT_STANDING_MICRO: return "StandMicro";
    case ACT_WALKING:        return "Walking";
    case ACT_IRREGULAR:      return "Irregular";
    default:                 return "Still";
  }
}
