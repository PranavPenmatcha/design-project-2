#include "Arduino.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include "U8x8lib.h"
#include <math.h>
#include <bluefruit.h>

// ── BLE objects ───────────────────────────────────────────────────────────────
static BLEDis  bledis;
static BLEDfu  bledfu;
static BLEUart bleuart;

static BLEService        walkSvc       ("57a70000-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrSteps      ("57a70001-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrCadence    ("57a70002-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic chrPeakHzMag  ("57a70004-9350-11ed-a1eb-0242ac120002");  // accel magnitude FFT
static BLECharacteristic chrPeakHzZ    ("57a70007-9350-11ed-a1eb-0242ac120002");  // Z-axis FFT
static BLECharacteristic chrBandEnergy ("57a70006-9350-11ed-a1eb-0242ac120002");

// ── INT1 / step-detector interrupt ───────────────────────────────────────────
#if defined(PIN_LSM6DS3TR_C_INT1)
  static constexpr int INT1_PIN = PIN_LSM6DS3TR_C_INT1;
#elif defined(PIN_LSM6DS3_INT1)
  static constexpr int INT1_PIN = PIN_LSM6DS3_INT1;
#else
  static constexpr int INT1_PIN = 3;
#endif

// ── Sampling rate (must match LSM6DS3 ODR in configPedometer) ────────────────
constexpr float SAMPLE_RATE_HZ = 52.0f;

volatile bool g_stepFlag   = false;
volatile bool g_sampleFlag = false;  // set by hardware timer ISR at SAMPLE_RATE_HZ

static void stepISR() { g_stepFlag = true; }

// ── Hardware timer for precise IMU sampling ───────────────────────────────────
// TIMER4 (32-bit, unused by SoftDevice/BSP).
// Prescaler 4 → 16 MHz / 2^4 = 1 MHz tick.  CC[0] = 1e6 / Fs ticks.
// The ISR only sets a flag; all I2C work stays in the main loop.
static void setupSampleTimer() {
  NRF_TIMER4->TASKS_STOP  = 1;
  NRF_TIMER4->MODE        = TIMER_MODE_MODE_Timer;
  NRF_TIMER4->BITMODE     = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER4->PRESCALER   = 4;
  NRF_TIMER4->CC[0]       = 1000000UL / static_cast<uint32_t>(SAMPLE_RATE_HZ);
  NRF_TIMER4->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
  NRF_TIMER4->INTENSET    = TIMER_INTENSET_COMPARE0_Msk;
  NVIC_SetPriority(TIMER4_IRQn, 3);
  NVIC_EnableIRQ(TIMER4_IRQn);
  NRF_TIMER4->TASKS_START = 1;
}

extern "C" void TIMER4_IRQHandler() {
  if (NRF_TIMER4->EVENTS_COMPARE[0]) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    g_sampleFlag = true;
  }
}

// ── Peripherals ───────────────────────────────────────────────────────────────
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE, SCL, SDA);
LSM6DS3 xIMU(I2C_MODE, 0x6A);
bool imuReady = false;

// ── FFT ───────────────────────────────────────────────────────────────────────
// N=128 @ 52 Hz → 2.46 s window, 0.406 Hz/bin, updated every 0.5 s (stride=26).
// Reported peak scans 0.5–12 Hz so hand motion up to 12 Hz is reported correctly;
// a separate 3–12 Hz scan feeds the tremor classifier.
constexpr uint16_t FFT_SIZE          = 128;
constexpr uint16_t FFT_STRIDE        = 26;      // slide window every 0.5 s @ 52 Hz
constexpr float    PEAK_BAND_LO_HZ   = 0.5f;
constexpr float    PEAK_BAND_HI_HZ   = 12.0f;
constexpr float    TREMOR_BAND_LO_HZ = 3.0f;
constexpr float    TREMOR_BAND_HI_HZ = 12.0f;
constexpr float    BAND_ENERGY_FLOOR = 0.00133f;

// Sliding FIFO: circular buffer, FFT runs every FFT_STRIDE new samples once full.
static float    fifoBufMag[FFT_SIZE];
static float    fifoBufZ[FFT_SIZE];
static uint16_t fifoWrite       = 0;
static uint16_t samplesSinceFft = 0;
static bool     fifoFilled      = false;

float peakFrequencyHzMag       = 0.0f;
float bandEnergyMag            = 0.0f;
float peakFrequencyHzTremorMag = 0.0f;
float bandEnergyTremorMag      = 0.0f;

float peakFrequencyHzZ       = 0.0f;
float bandEnergyZ            = 0.0f;
float peakFrequencyHzTremorZ = 0.0f;
float bandEnergyTremorZ      = 0.0f;

// Shared FFT scratch arrays and Hann window
float fftReal[FFT_SIZE];
float fftImag[FFT_SIZE];
float hannCoeff[FFT_SIZE];
float hannPowerGain = 1.0f;  // sum(w[n]^2)/N — corrects Parseval's energy for window loss

// ── Cadence ───────────────────────────────────────────────────────────────────
static uint16_t stepHistory[10] = {0};
static uint8_t  stepHistHead    = 0;
float           cadenceSpm      = 0.0f;

// ── Tremor / motion classification ────────────────────────────────────────────
enum TremorType {
  TREMOR_NONE = 0,
  TREMOR_REGULAR,      // 3–6.5 Hz rhythmic — classic PD resting tremor
  TREMOR_DYSKINESIA,   // 4–12 Hz, high energy — involuntary drug-induced jerks
  TREMOR_BRADYKINESIA, // <2.5 Hz, low-moderate energy — slow/reduced movement
};

enum MotionState {
  MOTION_STOPPED = 0,
  MOTION_WALKING,
};

// Band energy thresholds (scaled ×8/3 from original to match window-corrected Parseval energy).
constexpr float TREMOR_ENERGY_MIN      = 0.01333f;
constexpr float DYSKINESIA_ENERGY_MIN  = 0.0667f;
constexpr float BRADYKINESIA_ENERGY_MAX = 0.04f;
constexpr float BRADYKINESIA_ENERGY_MIN = 0.00213f;

// Walking: cadence gate (spm) AND the IMU peak must be in stride-frequency range.
// Shaking/tremor produces peaks >3.5 Hz; walking strides peak at 1–3 Hz.
constexpr float WALKING_CADENCE_MIN   = 40.0f;
constexpr float WALKING_HZ_MAX        = 3.5f;
constexpr float WALKING_HZ_MIN        = 0.8f;

TremorType  g_tremorType  = TREMOR_NONE;
MotionState g_motionState = MOTION_STOPPED;

// ─────────────────────────────────────────────────────────────────────────────
//  IMU helpers
// ─────────────────────────────────────────────────────────────────────────────

static uint16_t readStepCount() {
  uint8_t lo = 0, hi = 0;
  if (xIMU.readRegister(&lo, LSM6DS3_ACC_GYRO_STEP_COUNTER_L) != IMU_SUCCESS) return 0;
  if (xIMU.readRegister(&hi, LSM6DS3_ACC_GYRO_STEP_COUNTER_H) != IMU_SUCCESS) return 0;
  return static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

static int configPedometer() {
  uint8_t err = 0;
  uint8_t v   = 0;

  err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL,
                             static_cast<uint8_t>(LSM6DS3_ACC_GYRO_ODR_XL_52Hz)
                             | static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_XL_2g));

  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_CTRL3_C) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE);
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, v);
  } else { err++; }

  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_CTRL10_C) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
    v |= 0x10u;
    v |= 0x20u;
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, v);
  } else { err++; }

  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_TAP_CFG1) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED);
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED);
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, v);
  } else { err++; }

  err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL,
                             static_cast<uint8_t>(LSM6DS3_ACC_GYRO_INT1_PEDO_ENABLED));

  xIMU.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80u);
  delayMicroseconds(20);
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN, 0x06u);
  delayMicroseconds(20);
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00u);
  delayMicroseconds(20);

  return err;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Signal processing
// ─────────────────────────────────────────────────────────────────────────────

static void precomputeHannWindow() {
  float powerSum = 0.0f;
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    hannCoeff[i] = 0.5f * (1.0f - cosf((2.0f * PI * i) / (FFT_SIZE - 1)));
    powerSum += hannCoeff[i] * hannCoeff[i];
  }
  hannPowerGain = powerSum / static_cast<float>(FFT_SIZE);  // ≈ 3/8 for Hann
}

static float parabolicInterp(float p_prev, float p_peak, float p_next) {
  const float denom = p_prev - 2.0f * p_peak + p_next;
  if (fabsf(denom) < 1e-30f) return 0.0f;
  float d = 0.5f * (p_prev - p_next) / denom;
  if (d < -0.5f) d = -0.5f;
  if (d >  0.5f) d =  0.5f;
  return d;
}

static void fftInPlace(float* real, float* imag, uint16_t n) {
  uint16_t j = 0;
  for (uint16_t i = 1; i < n; i++) {
    uint16_t bit = n >> 1;
    while (j & bit) { j ^= bit; bit >>= 1; }
    j ^= bit;
    if (i < j) {
      float tr = real[i]; real[i] = real[j]; real[j] = tr;
      float ti = imag[i]; imag[i] = imag[j]; imag[j] = ti;
    }
  }
  for (uint16_t len = 2; len <= n; len <<= 1) {
    const float angle = -2.0f * PI / static_cast<float>(len);
    const float wlC = cosf(angle);
    const float wlS = sinf(angle);
    for (uint16_t i = 0; i < n; i += len) {
      float wC = 1.0f, wS = 0.0f;
      for (uint16_t k = 0; k < len / 2; k++) {
        const uint16_t u = i + k, v = i + k + len / 2;
        const float vr = real[v] * wC - imag[v] * wS;
        const float vi = real[v] * wS + imag[v] * wC;
        real[v] = real[u] - vr;  imag[v] = imag[u] - vi;
        real[u] += vr;           imag[u] += vi;
        const float nC = wC * wlC - wS * wlS;
        wS = wC * wlS + wS * wlC;
        wC = nC;
      }
    }
  }
}

// Copy FIFO into fftReal/fftImag, remove DC, apply Hann window, run FFT.
// `start` is the index of the oldest sample in the circular buffer so the
// window is applied in chronological order.
static void prepFftFromBuf(const float* buf, uint16_t start) {
  float mean = 0.0f;
  for (uint16_t i = 0; i < FFT_SIZE; i++) mean += buf[i];
  mean /= FFT_SIZE;
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    const uint16_t idx = (start + i) & (FFT_SIZE - 1);
    fftReal[i] = (buf[idx] - mean) * hannCoeff[i];
    fftImag[i] = 0.0f;
  }
  fftInPlace(fftReal, fftImag, FFT_SIZE);
}

// Step 2: scan a frequency band in the already-transformed fftReal/fftImag.
// Extracts the peak frequency (parabolic interpolation) and one-sided band energy.
// Boundary bins are excluded so the peak can't anchor to the band edge.
static void scanFftBand(float loHz, float hiHz, float& outHz, float& outEnergy) {
  const uint16_t kLo = static_cast<uint16_t>(
      loHz * static_cast<float>(FFT_SIZE) / SAMPLE_RATE_HZ + 0.5f);
  const uint16_t kHi = static_cast<uint16_t>(
      hiHz * static_cast<float>(FFT_SIZE) / SAMPLE_RATE_HZ);
  const uint16_t b0 = (kLo < 1u) ? 1u : kLo;
  const uint16_t b1 = (kHi >= FFT_SIZE / 2) ? static_cast<uint16_t>(FFT_SIZE / 2 - 1) : kHi;

  float    bandPwr = 0.0f, maxPwr = 0.0f;
  uint16_t maxK    = b0;
  for (uint16_t k = b0; k <= b1; k++) {
    const float pwr = fftReal[k]*fftReal[k] + fftImag[k]*fftImag[k];
    bandPwr += pwr;
    if (pwr > maxPwr) { maxPwr = pwr; maxK = k; }
  }

  // Parseval's theorem: E = 2/N * sum|X[k]|^2, corrected for Hann window power loss
  outEnergy = 2.0f * bandPwr / (static_cast<float>(FFT_SIZE) * hannPowerGain);

  float delta = 0.0f;
  if (maxK > b0 && maxK < b1) {
    const float pp = fftReal[maxK-1]*fftReal[maxK-1] + fftImag[maxK-1]*fftImag[maxK-1];
    const float pn = fftReal[maxK+1]*fftReal[maxK+1] + fftImag[maxK+1]*fftImag[maxK+1];
    delta = parabolicInterp(pp, maxPwr, pn);
  }
  outHz = (static_cast<float>(maxK) + delta) * SAMPLE_RATE_HZ / static_cast<float>(FFT_SIZE);

  if (outEnergy < BAND_ENERGY_FLOOR) outHz = 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Motion and tremor classification
// ─────────────────────────────────────────────────────────────────────────────

// WALKING requires cadence above threshold AND the dominant IMU peak in the
// stride-frequency band (0.8–3.5 Hz).  Shaking and tremor produce peaks above
// 3.5 Hz, so they no longer trigger a false WALKING state.
static MotionState classifyMotion(uint16_t steps, float cadence,
                                  uint16_t prevSteps,
                                  float hzMag, float hzZ) {
  (void)steps; (void)prevSteps;
  if (cadence < WALKING_CADENCE_MIN) return MOTION_STOPPED;
  // At least one pipeline must show a stride-range peak.
  const bool strideFreq = (hzMag >= WALKING_HZ_MIN && hzMag <= WALKING_HZ_MAX) ||
                          (hzZ   >= WALKING_HZ_MIN && hzZ   <= WALKING_HZ_MAX);
  return strideFreq ? MOTION_WALKING : MOTION_STOPPED;
}

// Classifies tremor/movement disorder from peak frequency and band energy.
//
// Dyskinesia   : involuntary, high-amplitude, 4–12 Hz jerky movements
//                (drug-induced, high energy, broad-spectrum)
// Regular tremor: rhythmic 3–6.5 Hz oscillation (classic PD resting tremor)
// Bradykinesia  : low-amplitude, slow (<2.5 Hz) movements; energy well below
//                 normal walking but above noise floor
//
// Energy-weighted average of both FFT pipelines for robustness.
static TremorType classifyTremor(float hzMag, float hzZ,
                                 float energyMag, float energyZ) {
  const float energy = (energyMag + energyZ) * 0.5f;

  // Dyskinesia first — high energy gate overrides frequency check.
  if (energy >= DYSKINESIA_ENERGY_MIN) {
    const float hz = (hzMag * energyMag + hzZ * energyZ) /
                     (energyMag + energyZ + 1e-30f);
    if (hz >= 4.0f && hz <= 12.0f) return TREMOR_DYSKINESIA;
  }

  if (energy < TREMOR_ENERGY_MIN) {
    // Below tremor floor — check bradykinesia band (very low energy, slow signal).
    if (energy >= BRADYKINESIA_ENERGY_MIN && energy < BRADYKINESIA_ENERGY_MAX) {
      const float hz = (hzMag * energyMag + hzZ * energyZ) /
                       (energyMag + energyZ + 1e-30f);
      if (hz > 0.0f && hz < 2.5f) return TREMOR_BRADYKINESIA;
    }
    return TREMOR_NONE;
  }

  // Moderate energy — classify by frequency.
  const float hz = (hzMag * energyMag + hzZ * energyZ) /
                   (energyMag + energyZ + 1e-30f);

  if (hz >= 3.0f && hz <= 6.5f) return TREMOR_REGULAR;
  if (hz >  0.0f && hz <  2.5f) return TREMOR_BRADYKINESIA;
  return TREMOR_NONE;
}

static const char* tremorName(TremorType t) {
  switch (t) {
    case TREMOR_NONE:         return "None        ";
    case TREMOR_REGULAR:      return "Tremor      ";
    case TREMOR_DYSKINESIA:   return "Dyskinesia  ";
    case TREMOR_BRADYKINESIA: return "Bradykinesia";
    default:                  return "None        ";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Display
// ─────────────────────────────────────────────────────────────────────────────

// hzTremor / engTremor are from the 3–12 Hz band scan used by classifyTremor.
static void drawDisplay(uint16_t steps, float cadence,
                        float hzTremor, float engTremor,
                        MotionState motion, TremorType tremor) {
  char line[17]; // 16 chars + null (u8x8 128-px / 8-px font → 16 cols, 8 rows)

  // Line 0: step count
  snprintf(line, sizeof(line), "Steps:%5u", static_cast<unsigned>(steps));
  u8x8.setCursor(0, 0); u8x8.print(line);

  // Line 1: cadence
  snprintf(line, sizeof(line), "Cd:%4dspm", static_cast<int>(cadence + 0.5f));
  u8x8.setCursor(0, 1); u8x8.print(line);

  // Line 2: motion state
  u8x8.setCursor(0, 2);
  u8x8.print(motion == MOTION_WALKING ? "State:WALKING   " : "State:STOPPED   ");

  // Line 3: tremor-band peak frequency (3–12 Hz scan)
  snprintf(line, sizeof(line), "TrHz:%5.2f", hzTremor);
  u8x8.setCursor(0, 3); u8x8.print(line);

  // Line 4: tremor-band energy
  snprintf(line, sizeof(line), "TrEb:%.5f", engTremor);
  u8x8.setCursor(0, 4); u8x8.print(line);

  // Line 5: tremor classification label
  u8x8.setCursor(0, 5);
  u8x8.print("Tr:");
  u8x8.setCursor(3, 5);
  u8x8.print(tremorName(tremor));
}

// ─────────────────────────────────────────────────────────────────────────────
//  BLE
// ─────────────────────────────────────────────────────────────────────────────

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

static void setupBle() {
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

// CSV via Nordic UART: millis,steps,cadenceSpm,hzMag,hzZ,bandEnergy
static void pushBleData(uint16_t steps, float cadence,
                        float hzMag, float hzZ, float engBand) {
  if (!Bluefruit.connected()) return;

  chrSteps.notify32(static_cast<uint32_t>(steps));
  chrCadence.notify32(static_cast<uint32_t>(cadence + 0.5f));
  chrPeakHzMag.notify(&hzMag, sizeof(hzMag));
  chrPeakHzZ.notify(&hzZ, sizeof(hzZ));
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

// ─────────────────────────────────────────────────────────────────────────────
//  setup / loop
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Wire.begin();

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print("Starting...");

  precomputeHannWindow();

  if (xIMU.begin() != 0) {
    Serial.println("IMU init failed.");
    u8x8.clear(); u8x8.setCursor(0, 0); u8x8.print("IMU FAIL");
    imuReady = false;
  } else {
    Serial.println("IMU OK.");
    delay(10);
    const int pedoErr = configPedometer();
    if (pedoErr != 0) {
      Serial.print("Pedometer config errors: ");
      Serial.println(pedoErr);
    } else {
      Serial.println("Pedometer ON @ 52 Hz.");
    }
    imuReady = true;
    pinMode(INT1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT1_PIN), stepISR, RISING);
  }

  setupBle();
  setupSampleTimer();

  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print(imuReady ? "Ready" : "IMU FAIL");
}

void loop() {
  static uint32_t nextBleMs     = 0;
  static uint32_t nextDisplayMs = 0;

  // Block until the hardware timer fires at exactly SAMPLE_RATE_HZ.
  if (!g_sampleFlag) return;
  noInterrupts(); g_sampleFlag = false; interrupts();

  const uint32_t nowMs = millis();

  if (!imuReady) return;

  // ── Read accelerometer ────────────────────────────────────────────────────
  const float ax = xIMU.readFloatAccelX();
  const float ay = xIMU.readFloatAccelY();
  const float az = xIMU.readFloatAccelZ();
  const float mag = sqrtf(ax*ax + ay*ay + az*az);

  // Sliding FIFO: circular write, FFT every FFT_STRIDE samples (0.5 s) once full.
  // Oldest sample is always at `fifoWrite` (the slot that will be overwritten next).
  fifoBufMag[fifoWrite] = mag;
  fifoBufZ[fifoWrite]   = az;
  fifoWrite = (fifoWrite + 1) & (FFT_SIZE - 1);
  if (fifoWrite == 0) fifoFilled = true;
  if (samplesSinceFft < 0xFFFF) samplesSinceFft++;

  if (fifoFilled && samplesSinceFft >= FFT_STRIDE) {
    samplesSinceFft = 0;

    prepFftFromBuf(fifoBufMag, fifoWrite);
    scanFftBand(PEAK_BAND_LO_HZ,   PEAK_BAND_HI_HZ,   peakFrequencyHzMag,       bandEnergyMag);
    scanFftBand(TREMOR_BAND_LO_HZ, TREMOR_BAND_HI_HZ, peakFrequencyHzTremorMag, bandEnergyTremorMag);

    prepFftFromBuf(fifoBufZ, fifoWrite);
    scanFftBand(PEAK_BAND_LO_HZ,   PEAK_BAND_HI_HZ,   peakFrequencyHzZ,       bandEnergyZ);
    scanFftBand(TREMOR_BAND_LO_HZ, TREMOR_BAND_HI_HZ, peakFrequencyHzTremorZ, bandEnergyTremorZ);
  }

  // ── Step ISR latch ────────────────────────────────────────────────────────
  if (g_stepFlag) {
    noInterrupts();
    g_stepFlag = false;
    interrupts();
  }
  const uint16_t steps = readStepCount();

  // ── Display (~8 Hz) ───────────────────────────────────────────────────────
  if (nowMs >= nextDisplayMs) {
    nextDisplayMs = nowMs + 120;
    drawDisplay(steps, cadenceSpm, peakFrequencyHzTremorMag, bandEnergyTremorMag,
                g_motionState, g_tremorType);
  }

  // ── BLE + cadence (1 Hz) ─────────────────────────────────────────────────
  if (nowMs >= nextBleMs) {
    nextBleMs = nowMs + 1000;

    const uint16_t prevSteps = stepHistory[stepHistHead]; // oldest slot before overwrite
    stepHistory[stepHistHead] = steps;
    stepHistHead = (stepHistHead + 1) % 10;
    const uint16_t oldSteps = stepHistory[stepHistHead];
    cadenceSpm = (steps >= oldSteps) ? (steps - oldSteps) * 6.0f : 0.0f;

    g_motionState = classifyMotion(steps, cadenceSpm, prevSteps,
                                   peakFrequencyHzMag, peakFrequencyHzZ);
    g_tremorType  = classifyTremor(peakFrequencyHzTremorMag, peakFrequencyHzTremorZ,
                                   bandEnergyTremorMag, bandEnergyTremorZ);

    Serial.print("St:"); Serial.print(steps);
    Serial.print(" Cd:"); Serial.print(static_cast<int>(cadenceSpm + 0.5f));
    Serial.print("spm HzMag:"); Serial.print(peakFrequencyHzMag, 2);
    Serial.print(" HzZ:"); Serial.print(peakFrequencyHzZ, 2);
    Serial.print(" Eb:"); Serial.println(bandEnergyMag, 5);

    pushBleData(steps, cadenceSpm, peakFrequencyHzMag, peakFrequencyHzZ, bandEnergyMag);
  }
}
