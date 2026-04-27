#pragma once
#include <stdint.h>
#include "U8x8lib.h"

extern U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8;

void initDisplay();
void displayMessage(const char* msg);
// steps, cadence (spm), slow-motion FFT peak Hz, wide FFT peak Hz, tremor-band peak Hz
void drawDisplay(uint16_t steps, float cadence, float motionHz, float fftHz, float tremorHz);
