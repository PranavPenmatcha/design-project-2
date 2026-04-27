#pragma once
#include <stdint.h>
#include "LSM6DS3.h"

extern volatile bool g_stepFlag;
extern volatile bool g_sampleFlag;
extern LSM6DS3       xIMU;
extern bool          imuReady;
extern float         cadenceSpm;

bool     initIMU();
void     setupSampleTimer();
uint16_t pollSteps();
void     readAccel(float& ax, float& ay, float& az);
void     updateCadence(uint16_t steps);
