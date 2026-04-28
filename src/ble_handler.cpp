#include "ble_handler.h"
#include "Arduino.h"
#include <bluefruit.h>

static BLEDis  bledis;
static BLEDfu  bledfu;
static BLEUart bleuart;

static BLEService        walkSvc      ("57a70000-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrSteps     ("57a70001-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrCadence   ("57a70002-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrState     ("57a70003-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrPeakHzMag ("57a70004-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrPeakHzZ   ("57a70007-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrBandEnergy  ("57a70006-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrPeakHzMotion("57a70008-9350-11ed-a1eb-0242ac120002");

static void setupWalkCharacteristics() {
  walkSvc.begin();

  chrSteps.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrSteps.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrSteps.setFixedLen(4);
  chrSteps.begin();
  chrSteps.write32(0);

  chrCadence.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrCadence.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrCadence.setFixedLen(4);
  chrCadence.begin();
  chrCadence.write32(0);

  chrState.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrState.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrState.setFixedLen(1);
  chrState.begin();
  chrState.write8(0);

  chrPeakHzMag.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrPeakHzMag.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrPeakHzMag.setFixedLen(4);
  chrPeakHzMag.begin();
  chrPeakHzMag.writeFloat(0.0f);

  chrPeakHzZ.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrPeakHzZ.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrPeakHzZ.setFixedLen(4);
  chrPeakHzZ.begin();
  chrPeakHzZ.writeFloat(0.0f);

  chrBandEnergy.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrBandEnergy.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrBandEnergy.setFixedLen(4);
  chrBandEnergy.begin();
  chrBandEnergy.writeFloat(0.0f);

  chrPeakHzMotion.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrPeakHzMotion.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrPeakHzMotion.setFixedLen(4);
  chrPeakHzMotion.begin();
  chrPeakHzMotion.writeFloat(0.0f);
}

static void connectCallback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char name[32] = {0};
  connection->getPeerName(name, sizeof(name));
  Serial.print("BLE connected: ");
  Serial.println(name);
}

static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle; (void)reason;
  Serial.println("BLE disconnected");
}

static void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(walkSvc);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setupBle() {
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XIAO WalkSense");
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  bledis.setManufacturer("Seeed");
  bledis.setModel("XIAO nRF52840 Sense");
  bledis.begin();
  bledfu.begin();

  setupWalkCharacteristics();
  bleuart.begin();
  startAdv();
  Serial.println("BLE: XIAO WalkSense");
}

void logSerialData(uint16_t steps, float cadence, float hzMag, float hzZ, float engBand) {
  Serial.print("St:"); Serial.print(steps);
  Serial.print(" Cd:"); Serial.print(static_cast<int>(cadence + 0.5f));
  Serial.print("spm HzMag:"); Serial.print(hzMag, 2);
  Serial.print(" HzZ:"); Serial.print(hzZ, 2);
  Serial.print(" Eb:"); Serial.println(engBand, 5);
}

// CSV via Nordic UART: millis,steps,cadenceSpm,hzMag,hzZ,bandEnergy
void pushBleData(uint16_t steps, float cadence, float hzMag, float hzZ, float hzMotion, float engBand, uint8_t motionState) {
  if (!Bluefruit.connected()) return;

  chrSteps.notify32(static_cast<uint32_t>(steps));
  chrCadence.notify32(static_cast<uint32_t>(cadence + 0.5f));
  chrState.notify8(motionState);
  chrPeakHzMag.notify(&hzMag, sizeof(hzMag));
  chrPeakHzZ.notify(&hzZ, sizeof(hzZ));
  chrPeakHzMotion.notify(&hzMotion, sizeof(hzMotion));
  chrBandEnergy.notify(&engBand, sizeof(engBand));

  if (bleuart.notifyEnabled()) {
    char line[96];
    const int n = snprintf(line, sizeof(line),
      "%lu,%u,%d,%.2f,%.2f,%.5f\r\n",
      static_cast<unsigned long>(millis()),
      static_cast<unsigned>(steps),
      static_cast<int>(cadence + 0.5f),
      hzMag, hzZ, engBand);
    if (n > 0 && n < static_cast<int>(sizeof(line)))
      bleuart.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(n));
  }
}
