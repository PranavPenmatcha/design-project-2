#include "display.h"
#include <stdio.h>
#include <math.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE, SCL, SDA);

void initDisplay() {
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
}

void displayMessage(const char* msg) {
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print(msg);
}

void drawDisplay(uint16_t steps, float cadence, float motionHz, float fftHz, float tremorHz) {
  char line[17];  // 16 chars + null (128-px / 8-px font → 16 cols, 8 rows)

  // Line 0: step count
  snprintf(line, sizeof(line), "Steps:  %5u", static_cast<unsigned>(steps));
  u8x8.setCursor(0, 0); u8x8.print(line);

  // Line 1: cadence in spm
  snprintf(line, sizeof(line), "Cd:   %4d spm", static_cast<int>(cadence + 0.5f));
  u8x8.setCursor(0, 1); u8x8.print(line);

  // Line 2: periodic hand / slow motion from IMU |a| buffer + FFT (see MOTION_BAND_* in config)
  snprintf(line, sizeof(line), "Freq:   %5.2f", motionHz);
  u8x8.setCursor(0, 2); u8x8.print(line);

  // Line 3: IMU step frequency (cadence / 60 → Hz, derived from hardware pedometer)
  snprintf(line, sizeof(line), "StepHz: %5.2f", cadence / 60.0f);
  u8x8.setCursor(0, 3); u8x8.print(line);

  // Line 4: FFT wide-band peak (0.5–12 Hz, accel magnitude)
  snprintf(line, sizeof(line), "FFTHz:  %5.2f", fftHz);
  u8x8.setCursor(0, 4); u8x8.print(line);

  // Line 5: tremor-band peak (3–12 Hz scan)
  snprintf(line, sizeof(line), "TrHz:   %5.2f", tremorHz);
  u8x8.setCursor(0, 5); u8x8.print(line);
}
