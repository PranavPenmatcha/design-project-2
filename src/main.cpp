#include "Arduino.h"
#include "Wire.h"

#include "step_cadence.h"
#include "fft_tremor.h"
#include "algorithms.h"
#include "display.h"
#include "ble_handler.h"
#include "gait_analysis.h"
#include "tremor_classifier.h"
#include "activity_context.h"
#include "condition_classifier.h"

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initDisplay();
  displayMessage("Starting...");

  precomputeHannWindow();
  initIMU();

  setupBle();
  setupSampleTimer();

  displayMessage(imuReady ? "Ready" : "IMU FAIL");
}

void loop() {
  static uint32_t nextBleMs     = 0;
  static uint32_t nextDisplayMs = 0;

  if (!g_sampleFlag) return;
  noInterrupts(); g_sampleFlag = false; interrupts();

  const uint32_t nowMs = millis();
  if (!imuReady) return;

  float ax, ay, az;
  readAccel(ax, ay, az);
  pushFifoSample(ax, ay, az);

  const uint16_t steps = pollSteps();

  if (nowMs >= nextDisplayMs) {
    nextDisplayMs = nowMs + 120;
    drawDisplay(steps, cadenceSpm, peakFrequencyHzMotion, peakFrequencyHzMag,
                peakFrequencyHzTremorMag);
  }

  if (nowMs >= nextBleMs) {
    nextBleMs = nowMs + 1000;
    updateCadence(steps);
    updateClassifiers(cadenceSpm, peakFrequencyHzMag, peakFrequencyHzZ);
    updateGaitFeatures(steps, cadenceSpm, peakFrequencyHzMag, peakFrequencyHzZ);
    updateActivity(cadenceSpm, bandEnergyMag, g_stopGate, g_locomotionConfirmed);
    runConditionPipeline();
    syncTremorTypeFromKind();
    logSerialData(steps, cadenceSpm, peakFrequencyHzMag, peakFrequencyHzZ, bandEnergyMag);
    Serial.print("Tremor: "); Serial.print(tremorKindName(g_tremorKind));
    Serial.print(" | StopGate: "); Serial.print(g_stopGate ? "YES" : "NO");
    Serial.print(" | Act: "); Serial.print(activityName(g_activity));
    Serial.print(" | Cond: "); Serial.print(conditionName(g_condition));
    Serial.print(" | WideHz: "); Serial.print(g_widePeakHz, 2);
    Serial.print(" TrHz: "); Serial.print(g_tremorPeakHz, 2);
    Serial.print(" Df: "); Serial.print(g_peakSeparationHz, 2);
    Serial.print(" Ratio: "); Serial.print(g_tremorRatio, 2);
    Serial.print(" StepCV%: "); Serial.println(g_stepIntervalCvPct, 1);
    pushBleData(steps, cadenceSpm, peakFrequencyHzMag, peakFrequencyHzZ, peakFrequencyHzMotion, bandEnergyMag,
                static_cast<uint8_t>(g_motionState));
  }
}
