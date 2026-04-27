#pragma once
#include <stdint.h>

enum TremorType {
  TREMOR_NONE = 0,
  TREMOR_REGULAR,
  TREMOR_DYSKINESIA,
  TREMOR_BRADYKINESIA,
};

enum MotionState {
  MOTION_STOPPED = 0,
  MOTION_WALKING,
};

extern TremorType  g_tremorType;
extern MotionState g_motionState;

MotionState classifyMotion(float cadence, float hzMag, float hzZ);
TremorType  classifyTremor(float hzMag, float hzZ, float energyMag, float energyZ);
const char* tremorName(TremorType t);
void        updateClassifiers(float cadence, float hzMag, float hzZ,
                              float hzTremorMag, float hzTremorZ,
                              float energyTremorMag, float energyTremorZ);
