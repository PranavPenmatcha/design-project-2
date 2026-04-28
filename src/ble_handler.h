#pragma once
#include <stdint.h>

void setupBle();
void pushBleData(uint16_t steps, float cadence, float hzMag, float hzZ, float hzMotion, float engBand, uint8_t motionState);
void logSerialData(uint16_t steps, float cadence, float hzMag, float hzZ, float engBand);
