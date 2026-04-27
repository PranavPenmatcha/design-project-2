#include "algorithms.h"
#include "config.h"

TremorType  g_tremorType  = TREMOR_NONE;
MotionState g_motionState = MOTION_STOPPED;

MotionState classifyMotion(float cadence, float hzMag, float hzZ) {
  if (cadence < WALKING_CADENCE_MIN) return MOTION_STOPPED;
  const bool strideFreq = (hzMag >= WALKING_HZ_MIN && hzMag <= WALKING_HZ_MAX) ||
                          (hzZ   >= WALKING_HZ_MIN && hzZ   <= WALKING_HZ_MAX);
  return strideFreq ? MOTION_WALKING : MOTION_STOPPED;
}

TremorType classifyTremor(float hzMag, float hzZ, float energyMag, float energyZ) {
  const float energy = (energyMag + energyZ) * 0.5f;

  if (energy >= DYSKINESIA_ENERGY_MIN) {
    const float hz = (hzMag * energyMag + hzZ * energyZ) /
                     (energyMag + energyZ + 1e-30f);
    if (hz >= 4.0f && hz <= 12.0f) return TREMOR_DYSKINESIA;
  }

  if (energy < TREMOR_ENERGY_MIN) {
    if (energy >= BRADYKINESIA_ENERGY_MIN && energy < BRADYKINESIA_ENERGY_MAX) {
      const float hz = (hzMag * energyMag + hzZ * energyZ) /
                       (energyMag + energyZ + 1e-30f);
      if (hz > 0.0f && hz < 2.5f) return TREMOR_BRADYKINESIA;
    }
    return TREMOR_NONE;
  }

  const float hz = (hzMag * energyMag + hzZ * energyZ) /
                   (energyMag + energyZ + 1e-30f);
  if (hz >= 3.0f && hz <= 6.5f) return TREMOR_REGULAR;
  if (hz >  0.0f && hz <  2.5f) return TREMOR_BRADYKINESIA;
  return TREMOR_NONE;
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

void updateClassifiers(float cadence, float hzMag, float hzZ,
                       float hzTremorMag, float hzTremorZ,
                       float energyTremorMag, float energyTremorZ) {
  g_motionState = classifyMotion(cadence, hzMag, hzZ);
  g_tremorType  = classifyTremor(hzTremorMag, hzTremorZ, energyTremorMag, energyTremorZ);
}
