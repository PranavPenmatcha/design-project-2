#include "algorithms.h"
#include "config.h"
#include "tremor_classifier.h"

TremorType  g_tremorType  = TREMOR_NONE;
MotionState g_motionState = MOTION_STOPPED;

MotionState classifyMotion(float cadence, float hzMag, float hzZ) {
  if (cadence < WALKING_CADENCE_MIN) return MOTION_STOPPED;
  const bool strideFreq = (hzMag >= WALKING_HZ_MIN && hzMag <= WALKING_HZ_MAX) ||
                          (hzZ   >= WALKING_HZ_MIN && hzZ   <= WALKING_HZ_MAX);
  return strideFreq ? MOTION_WALKING : MOTION_STOPPED;
}

void syncTremorTypeFromKind(void) {
  switch (g_tremorKind) {
    case TR_NONE:         g_tremorType = TREMOR_NONE; break;
    case TR_REGULAR:      g_tremorType = TREMOR_REGULAR; break;
    case TR_DYSKINESIA:   g_tremorType = TREMOR_DYSKINESIA; break;
    case TR_BRADYKINESIA: g_tremorType = TREMOR_BRADYKINESIA; break;
    default:              g_tremorType = TREMOR_NONE; break;
  }
}

const char* tremorName(TremorType t) {
  switch (t) {
    case TREMOR_NONE:         return "None        ";
    case TREMOR_REGULAR:      return "Tremor      ";
    case TREMOR_DYSKINESIA:   return "Dyskinesia  ";
    case TREMOR_BRADYKINESIA: return "Bradykinesia";
    default:                  return "None        ";
  }
}

void updateClassifiers(float cadence, float hzMag, float hzZ) {
  g_motionState = classifyMotion(cadence, hzMag, hzZ);
}
