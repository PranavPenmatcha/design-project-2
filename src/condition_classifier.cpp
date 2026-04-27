#include "condition_classifier.h"
#include "activity_context.h"
#include "tremor_classifier.h"
#include "gait_analysis.h"
#include "fft_tremor.h"
#include "step_cadence.h"
#include "config.h"
#include "Arduino.h"

ConditionLabel g_condition = COND_NORMAL;

// Track recent walking → if it stops abruptly with high tremor energy in place,
// that is a FOG candidate.
static uint32_t lastWalkingMs = 0;

void runConditionPipeline() {
  const uint32_t nowMs = millis();
  if (g_activity == ACT_WALKING) lastWalkingMs = nowMs;

  // Stage B branches on activity.
  switch (g_activity) {
    case ACT_STILL:
    case ACT_STANDING_MICRO: {
      classifyTremorKind(peakFrequencyHzMag,       peakFrequencyHzZ,
                         bandEnergyMag,            bandEnergyZ,
                         peakFrequencyHzTremorMag, peakFrequencyHzTremorZ,
                         bandEnergyTremorMag,      bandEnergyTremorZ);
      switch (g_tremorKind) {
        case TR_REGULAR:      g_condition = COND_TREMOR_REST;   return;
        case TR_DYSKINESIA:   g_condition = COND_DYSKINESIA;    return;
        case TR_BRADYKINESIA: g_condition = COND_BRADYKINESIA;  return;
        default:              g_condition = COND_NORMAL;        return;
      }
    }

    case ACT_WALKING: {
      // Walking. Check for bradykinesia (slow but regular) or dyskinesia overlay.
      classifyTremorKind(peakFrequencyHzMag,       peakFrequencyHzZ,
                         bandEnergyMag,            bandEnergyZ,
                         peakFrequencyHzTremorMag, peakFrequencyHzTremorZ,
                         bandEnergyTremorMag,      bandEnergyTremorZ);
      if (g_tremorKind == TR_DYSKINESIA) { g_condition = COND_DYSKINESIA; return; }
      if (cadenceSpm < 60.0f && cadenceSpm >= WALKING_CADENCE_MIN) {
        g_condition = COND_BRADYKINESIA; return;
      }
      g_condition = COND_NORMAL_WALK;
      return;
    }

    case ACT_IRREGULAR: {
      // Movement without locomotion. If recent walking → FOG candidate.
      const bool recentlyWalking = (nowMs - lastWalkingMs) < 5000;
      classifyTremorKind(peakFrequencyHzMag,       peakFrequencyHzZ,
                         bandEnergyMag,            bandEnergyZ,
                         peakFrequencyHzTremorMag, peakFrequencyHzTremorZ,
                         bandEnergyTremorMag,      bandEnergyTremorZ);
      if (recentlyWalking && bandEnergyTremorMag >= TREMOR_ENERGY_MIN) {
        g_condition = COND_FOG_CANDIDATE; return;
      }
      if (g_tremorKind == TR_DYSKINESIA) { g_condition = COND_DYSKINESIA; return; }
      if (g_tremorKind == TR_REGULAR)    { g_condition = COND_TREMOR_REST; return; }
      g_condition = COND_NORMAL;
      return;
    }
  }
}

const char* conditionName(ConditionLabel c) {
  switch (c) {
    case COND_NORMAL:         return "Normal";
    case COND_TREMOR_REST:    return "RestTremor";
    case COND_DYSKINESIA:     return "Dyskinesia";
    case COND_BRADYKINESIA:   return "Bradykinesia";
    case COND_FOG_CANDIDATE:  return "FOG?";
    case COND_NORMAL_WALK:    return "Walk-OK";
    default:                  return "Normal";
  }
}
