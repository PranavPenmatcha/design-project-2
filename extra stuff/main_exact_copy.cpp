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

static BLEService         walkSvc      ("57a70000-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic  chrSteps     ("57a70001-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic  chrIdleMs    ("57a70002-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic  chrState     ("57a70003-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic  chrPeakHz    ("57a70004-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic  chrBandEnergy("57a70006-9350-11ed-a1eb-0242ac120002");
static BLECharacteristic  chrMotionScore("57a70009-9350-11ed-a1eb-0242ac120002");

// ── INT1 / step-detector interrupt ───────────────────────────────────────────
// The built-in LSM6DS3(TR-C) on the XIAO nRF52840 Sense routes INT1 to D3 /
// P0.03.  The board variant may define PIN_LSM6DS3TR_C_INT1 or similar; fall
// back to pin 3 when it does not.
#if defined(PIN_LSM6DS3TR_C_INT1)
  static constexpr int INT1_PIN = PIN_LSM6DS3TR_C_INT1;
#elif defined(PIN_LSM6DS3_INT1)
  static constexpr int INT1_PIN = PIN_LSM6DS3_INT1;
#else
  static constexpr int INT1_PIN = 3;  // D3 / P0.03 on XIAO nRF52840 Sense
#endif

// Volatile flag set in ISR, cleared in main loop.
volatile bool g_stepFlag = false;
static void stepISR() { g_stepFlag = true; }

// ── Peripherals ───────────────────────────────────────────────────────────────
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
LSM6DS3 xIMU(I2C_MODE, 0x6A);
bool imuReady = false;

// ── Sampling ──────────────────────────────────────────────────────────────────
// 52 Hz: ST pedometer supports 26/52/104/208 Hz. 52 Hz gives finer cadence
// resolution than 26 Hz and still runs the embedded pedometer algorithm.
constexpr float    SAMPLE_RATE_HZ   = 52.0f;
constexpr uint32_t SAMPLE_PERIOD_US =
    static_cast<uint32_t>(1000000.0f / SAMPLE_RATE_HZ + 0.5f);  // 19 231 µs

// ── FFT ───────────────────────────────────────────────────────────────────────
// N=128 @ 52 Hz → 2.46 s window, 0.406 Hz/bin.
// Sliding-window: circular FIFO filled at 52 Hz; FFT re-runs every FFT_STRIDE
//   new samples (75% overlap).  Parabolic interpolation refines to ~±0.02 Hz.
//   FFT_STRIDE = 26  →  update every 0.5 s.
// Walking cadence band 0.5–3.5 Hz  →  bins 1–9 at 52 Hz / N=128.
constexpr uint16_t FFT_SIZE        = 128;
constexpr uint16_t FFT_STRIDE      = 26;   // re-run FFT every 26 new samples (0.5 s)
constexpr float    WALK_BAND_LO_HZ = 0.5f;
constexpr float    WALK_BAND_HI_HZ = 3.5f;

// ── Tremor detection (gyro FFT pipeline) ──────────────────────────────────────
// Dedicated circular buffer of gyro-magnitude samples fed at 52 Hz.  A
// separate FFT (N=128, same as walking FFT, reuses fftReal/fftImag) runs
// every TREMOR_FFT_STRIDE new samples (= every 0.5 s, 75 % overlap).
// Peak frequency in the 4–6 Hz band is extracted; a 1-s rolling RMS of gyro
// magnitude gates amplitude.  Three consecutive candidate windows → isTremor.
//
// Bin map @ 52 Hz, N=128:  4 Hz → bin 10,  6 Hz → bin 15  (0.406 Hz/bin)
//
// Tune TREMOR_RMS_MIN / TREMOR_RMS_MAX to the device's noise floor and the
// upper bound of intentional wrist movement (deg/s).
constexpr float    TREMOR_LO_HZ        = 3.5f;   // widen: covers 3.5–7 Hz (Parkinsonian + essential)
constexpr float    TREMOR_HI_HZ        = 7.0f;
constexpr float    TREMOR_RMS_MIN      = 0.5f;   // deg/s — lower floor to catch subtle tremor
constexpr float    TREMOR_RMS_MAX      = 25.0f;  // deg/s — below gross movement
constexpr float    TREMOR_STRONG_THRESH = 5.0f;  // deg/s — Weak vs Strong boundary
constexpr float    TREMOR_PEAK_FRAC    = 0.30f;  // peak bin ≥ 30 % of band power (was 40 %, too strict)
constexpr uint16_t TREMOR_FFT_STRIDE   = 26;     // update every 0.5 s
constexpr uint16_t TREMOR_RMS_WIN      = 52;     // 1-s rolling RMS window
constexpr uint8_t  TREMOR_CONFIRM_WIN  = 3;      // 3 × 0.5 s = 1.5 s to confirm
constexpr uint8_t  TREMOR_CLEAR_WIN    = 5;      // 5 × 0.5 s = 2.5 s to clear

// Circular FIFO: sampleHead is the next write slot; sampleFill tracks how
// many slots are valid (0..FFT_SIZE).  Once full, the oldest sample is at
// sampleBuf[sampleHead] and wraps forward.
static float    sampleBuf[FFT_SIZE];
static uint16_t sampleHead  = 0;
static uint16_t sampleFill  = 0;
static uint16_t strideCount = 0;  // samples pushed since last FFT

float    fftReal[FFT_SIZE];
float    fftImag[FFT_SIZE];
float    hannCoeff[FFT_SIZE];  // precomputed Hann window coefficients

float    peakFrequencyHz = 0.0f;
float    signalEnergy    = 0.0f;  // total spectral power (g²)
float    bandEnergy      = 0.0f;  // walking-band power   (g²)
uint16_t peakBin         = 0;

// ── Gyro tremor pipeline ──────────────────────────────────────────────────────
static float    gyroSampleBuf[FFT_SIZE];  // circular FIFO of gyro-magnitude samples
static uint16_t gyroSampleHead  = 0;
static uint16_t gyroSampleFill  = 0;
static uint16_t gyroStrideCount = 0;

static float    gyroRmsBuf[TREMOR_RMS_WIN];  // circular buffer for rolling RMS
static uint16_t gyroRmsHead  = 0;
static uint16_t gyroRmsCount = 0;
static float    gyroRmsSumSq = 0.0f;

float    tremorPeakHz     = 0.0f;  // dominant Hz in 4–6 Hz band (from gyro FFT)
float    tremorRmsG       = 0.0f;  // 1-s rolling RMS of gyro magnitude (deg/s)
bool     isTremor         = false;
static uint8_t tremorConfirmCount = 0;
static uint8_t tremorClearCount   = 0;

// ── Cadence ───────────────────────────────────────────────────────────────────
// Rolling 10-second window: store step count sampled every 1 s, compute
// cadence = (steps_now − steps_10s_ago) × 6  →  steps per minute.
static uint16_t stepHistory[10] = {0};
static uint8_t  stepHistHead    = 0;
float           cadenceSpm      = 0.0f;

// ── Sliding-window motion detector ───────────────────────────────────────────
// Each accelerometer sample casts a binary vote: 1 if |mag−1g| ≥ threshold.
// motionScore = fraction of votes in the last MOTION_WIN_SIZE samples that are 1.
// Hysteresis: enter WALKING at ≥25 %, stay until score drops to ≤8 %.
constexpr uint16_t MOTION_WIN_SIZE  = 52;      // 1 s at 52 Hz
constexpr float    MOTION_THRESH_G  = 0.060f;  // per-sample vote threshold (g)
constexpr float    WALK_SCORE_ENTER = 0.25f;   // score to enter  WALKING (25 %)
constexpr float    WALK_SCORE_EXIT  = 0.08f;   // score to exit → STOPPED  (8 %)

static uint8_t  motionWin[MOTION_WIN_SIZE];
static uint16_t motionWinHead  = 0;
static uint16_t motionWinCount = 0;
static uint16_t motionWinSum   = 0;
float           motionScore    = 0.0f;

// ── Motion state ──────────────────────────────────────────────────────────────
enum class MotionState : uint8_t { STOPPED = 0, WALKING = 1 };
MotionState motionState = MotionState::STOPPED;

// WALKING requires motionScore AND recent step-detector events.
// If no step ISR fires for STEP_TIMEOUT_MS, the state drops to STOPPED
// regardless of motionScore — prevents arm swings / table knocks from
// registering as walking.
constexpr uint32_t STEP_TIMEOUT_MS = 3000;  // 3 s without a step ISR — covers ~20 spm slow walk
static uint32_t    lastStepMs      = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  IMU helpers
// ─────────────────────────────────────────────────────────────────────────────

// Read STEP_COUNTER_L/H with BDU enabled, so both bytes belong to the same
// hardware snapshot (no torn read across a counter increment).
static uint16_t readStepCount() {
  uint8_t lo = 0, hi = 0;
  if (xIMU.readRegister(&lo, LSM6DS3_ACC_GYRO_STEP_COUNTER_L) != IMU_SUCCESS) return 0;
  if (xIMU.readRegister(&hi, LSM6DS3_ACC_GYRO_STEP_COUNTER_H) != IMU_SUCCESS) return 0;
  return static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

// Configure the LSM6DS3 for walking detection.
//
// Register map highlights:
//   CTRL1_XL  — ODR=52 Hz, FS=±2g (ST pedometer requirement)
//   CTRL3_C   — BDU=1: coherent 16-bit register reads (no torn STEP_COUNTER)
//   CTRL10_C  — FUNC_EN=1 (bit 2): enable embedded-function engine
//   TAP_CFG1  — PEDO_EN=1 (bit 6) + TIMER_EN=1 (bit 7): start pedometer + timer
//   INT1_CTRL — bit 7 (INT1_STEP_DETECTOR / INT1_PEDO_ENABLED = 0x80): route
//               step-detect event to INT1 pin.
//
// Note: the previous code wrote 0x10 to INT1_CTRL which enables FIFO-overrun
// (not step-detect).  That bit is corrected here to 0x80.
static int configPedometer() {
  uint8_t err = 0;
  uint8_t v   = 0;

  // CTRL1_XL: ODR=52 Hz, FS=±2g
  err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL,
                             static_cast<uint8_t>(LSM6DS3_ACC_GYRO_ODR_XL_52Hz)
                             | static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_XL_2g));

  // CTRL3_C: set BDU; preserve other bits
  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_CTRL3_C) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE);
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, v);
  } else { err++; }

  // CTRL10_C: FUNC_EN (bit 2) + PEDO_EN (bit 4) + TIMER_EN (bit 5).
  // In the original LSM6DS3 register map, PEDO_EN and TIMER_EN live in CTRL10_C
  // at bits 4 and 5.  In the TR-C variant they moved to TAP_CFG1, but setting
  // these bits in CTRL10_C is harmless on TR-C and required on original.
  // Read-modify-write to avoid touching FUNC_CFG_EN (bit 7).
  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_CTRL10_C) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);  // 0x04
    v |= 0x10u;   // PEDO_EN  — original LSM6DS3 location (bit 4)
    v |= 0x20u;   // TIMER_EN — original LSM6DS3 location (bit 5)
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, v);
  } else { err++; }

  // TAP_CFG1: PEDO_EN + TIMER_EN (required by ST embedded algorithm)
  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_TAP_CFG1) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED);
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED);
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, v);
  } else { err++; }

  // INT1_CTRL: route STEP_DETECT to INT1 (active-HIGH push-pull).
  // INT1_PEDO_ENABLED = 0x80 = bit 7 of INT1_CTRL.
  // (Previous code wrote 0x10 = FIFO_OVR — wrong bit, steps never fired.)
  err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL,
                             static_cast<uint8_t>(LSM6DS3_ACC_GYRO_INT1_PEDO_ENABLED));

  // CONFIG_PEDO_THS_MIN — accessed via embedded function register bank.
  // Default threshold is 0x10 (16), which is tuned for a device strapped to
  // the body.  Lower to 0x06 (6) so hand-held or wrist-worn motion registers.
  //   FUNC_CFG_ACCESS (0x01) bit 7 = 1  →  embedded bank selected
  //   Register 0x0F in that bank = CONFIG_PEDO_THS_MIN
  //   [7:5] PEDO_FS = 000 (±2g, matching CTRL1_XL); [4:0] = threshold
  //   FUNC_CFG_ACCESS bit 7 = 0  →  return to normal register bank
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80u);   // open embedded bank
  delayMicroseconds(20);
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN, 0x06u); // threshold=6
  delayMicroseconds(20);
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00u);   // close embedded bank
  delayMicroseconds(20);

  return err;
}

// Poll STATUS_REG for new accelerometer data (XLDA bit).
// Prevents reading stale register values between ODR ticks.
static bool isAccelReady() {
  uint8_t status = 0;
  if (xIMU.readRegister(&status, LSM6DS3_ACC_GYRO_STATUS_REG) != IMU_SUCCESS) return true;
  return (status & static_cast<uint8_t>(LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL)) != 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Signal processing
// ─────────────────────────────────────────────────────────────────────────────

static void precomputeHannWindow() {
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    hannCoeff[i] = 0.5f * (1.0f - cosf((2.0f * PI * i) / (FFT_SIZE - 1)));
  }
}

// Push one magnitude sample into the circular FIFO and advance stride counter.
static void pushSample(float v) {
  sampleBuf[sampleHead] = v;
  sampleHead = (sampleHead + 1) % FFT_SIZE;
  if (sampleFill < FFT_SIZE) sampleFill++;
  strideCount++;
}

// Parabolic interpolation: given power at bins (k-1, k, k+1), return the
// fractional bin offset that best locates the true spectral peak.
// Clamped to [-0.5, 0.5] to prevent extrapolation beyond adjacent bins.
static float parabolicInterp(float p_prev, float p_peak, float p_next) {
  const float denom = p_prev - 2.0f * p_peak + p_next;
  if (fabsf(denom) < 1e-30f) return 0.0f;
  float d = 0.5f * (p_prev - p_next) / denom;
  if (d < -0.5f) d = -0.5f;
  if (d >  0.5f) d =  0.5f;
  return d;
}

// Update the sliding window with a new magnitude sample and derive motionState.
// O(1): uses a running integer sum in a circular buffer — no per-window scan.
static void updateMotionWindow(float mag) {
  const uint8_t vote = (fabsf(mag - 1.0f) >= MOTION_THRESH_G) ? 1u : 0u;
  if (motionWinCount == MOTION_WIN_SIZE) {
    motionWinSum -= motionWin[motionWinHead];
  } else {
    motionWinCount++;
  }
  motionWin[motionWinHead] = vote;
  motionWinHead = (motionWinHead + 1) % MOTION_WIN_SIZE;
  motionWinSum += vote;
  motionScore = static_cast<float>(motionWinSum) / static_cast<float>(motionWinCount);
  // State machine updated in loop() where step timing is available.
}

// Cooley-Tukey radix-2 DIT FFT, in-place.
static void fftInPlace(float* real, float* imag, uint16_t n) {
  // Bit-reversal permutation
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
  // Butterfly stages — trig per stage (not per butterfly) via iterative rotation
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

// Sliding-window FFT: called every FFT_STRIDE new samples once the FIFO is full.
// Copies the circular FIFO (oldest-first) into fftReal/fftImag, removes DC,
// applies Hann window, runs FFT, then extracts peakFrequencyHz (with parabolic
// sub-bin interpolation) and bandEnergy.
//
// Frequency resolution:  Fs/N = 52/128 ≈ 0.406 Hz/bin  (+parabolic → ~0.02 Hz)
// Walking band bins:      kLo=1 (0.41 Hz) … kHi=9 (3.66 Hz)  @ 52 Hz, N=128
// Update rate:            every FFT_STRIDE=26 samples → every 0.5 s
static void computeFftEnergy() {
  // Copy circular FIFO into FFT arrays, oldest sample first.
  // When sampleFill == FFT_SIZE the oldest entry sits at sampleHead.
  const uint16_t oldest = (sampleFill == FFT_SIZE) ? sampleHead : 0u;
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    fftReal[i] = sampleBuf[(oldest + i) % FFT_SIZE];
    fftImag[i] = 0.0f;
  }

  // Remove DC (gravity ~1 g) and apply Hann window in one pass.
  float mean = 0.0f;
  for (uint16_t i = 0; i < FFT_SIZE; i++) mean += fftReal[i];
  mean /= FFT_SIZE;
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    fftReal[i] = (fftReal[i] - mean) * hannCoeff[i];
  }

  fftInPlace(fftReal, fftImag, FFT_SIZE);

  // Total spectral energy (Parseval, normalised by N).
  float totalPwr = 0.0f;
  for (uint16_t k = 0; k < FFT_SIZE; k++) {
    totalPwr += fftReal[k] * fftReal[k] + fftImag[k] * fftImag[k];
  }
  signalEnergy = totalPwr / FFT_SIZE;

  // Walking-band bin limits: 0.5–3.5 Hz
  //   kLo = round(0.5 * 128 / 52) = 1   (0.41 Hz)
  //   kHi = round(3.5 * 128 / 52) = 9   (3.66 Hz)
  const uint16_t kLo = static_cast<uint16_t>(
      WALK_BAND_LO_HZ * static_cast<float>(FFT_SIZE) / SAMPLE_RATE_HZ + 0.5f);
  const uint16_t kHi = static_cast<uint16_t>(
      WALK_BAND_HI_HZ * static_cast<float>(FFT_SIZE) / SAMPLE_RATE_HZ + 0.5f);
  const uint16_t b0 = (kLo < 1u) ? 1u : kLo;
  const uint16_t b1 = (kHi >= FFT_SIZE / 2) ? static_cast<uint16_t>(FFT_SIZE / 2 - 1) : kHi;

  float    bandPwr = 0.0f;
  float    maxPwr  = 0.0f;
  uint16_t maxK    = b0;

  for (uint16_t k = b0; k <= b1; k++) {
    const float pwr = fftReal[k] * fftReal[k] + fftImag[k] * fftImag[k];
    bandPwr += pwr;
    if (pwr > maxPwr) { maxPwr = pwr; maxK = k; }
  }

  bandEnergy = bandPwr / static_cast<float>(FFT_SIZE);
  peakBin    = maxK;

  // Parabolic sub-bin interpolation: refines peak from ±0.2 Hz to ~±0.02 Hz.
  // Only valid when the peak bin has neighbours within the search band.
  float delta = 0.0f;
  if (maxK > b0 && maxK < b1) {
    const float pp = fftReal[maxK-1]*fftReal[maxK-1] + fftImag[maxK-1]*fftImag[maxK-1];
    const float pn = fftReal[maxK+1]*fftReal[maxK+1] + fftImag[maxK+1]*fftImag[maxK+1];
    delta = parabolicInterp(pp, maxPwr, pn);
  }
  // Bin → Hz: f = (k + fractional_offset) * Fs / N  (no 2π — that would give rad/s)
  peakFrequencyHz = (static_cast<float>(maxK) + delta)
                    * SAMPLE_RATE_HZ / static_cast<float>(FFT_SIZE);

}

// ─────────────────────────────────────────────────────────────────────────────
//  Gyro tremor pipeline
// ─────────────────────────────────────────────────────────────────────────────

// Push one gyro-magnitude sample into the dedicated circular FIFO.
static void pushGyroSample(float v) {
  gyroSampleBuf[gyroSampleHead] = v;
  gyroSampleHead = (gyroSampleHead + 1) % FFT_SIZE;
  if (gyroSampleFill < FFT_SIZE) gyroSampleFill++;
  gyroStrideCount++;
}

// O(1) rolling RMS of gyro magnitude over TREMOR_RMS_WIN samples (1 s).
static void updateGyroRms(float v) {
  if (gyroRmsCount == TREMOR_RMS_WIN) {
    gyroRmsSumSq -= gyroRmsBuf[gyroRmsHead] * gyroRmsBuf[gyroRmsHead];
  } else {
    gyroRmsCount++;
  }
  gyroRmsBuf[gyroRmsHead] = v;
  gyroRmsSumSq += v * v;
  gyroRmsHead = (gyroRmsHead + 1) % TREMOR_RMS_WIN;
  tremorRmsG = (gyroRmsCount > 0) ? sqrtf(gyroRmsSumSq / gyroRmsCount) : 0.0f;
}

// Gyro-based FFT tremor detector.  Reuses fftReal/fftImag (safe: called
// sequentially after computeFftEnergy which has already consumed its results).
//
// Algorithm:
//   1. Copy oldest-first gyro FIFO into FFT arrays.
//   2. Remove DC, apply precomputed Hann window.
//   3. Run FFT, search 4–6 Hz band for peak bin.
//   4. Candidate = RMS in range AND peak is narrow (≥40 % of band power)
//                  AND motionState == STOPPED (resting-tremor gate).
//   5. Hysteresis: 3 consecutive candidates → isTremor; 5 clear → not tremor.
static void computeTremorFft() {
  // Step 1: copy FIFO oldest-first
  const uint16_t oldest = (gyroSampleFill == FFT_SIZE) ? gyroSampleHead : 0u;
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    fftReal[i] = gyroSampleBuf[(oldest + i) % FFT_SIZE];
    fftImag[i] = 0.0f;
  }

  // Step 2: remove DC, apply Hann window
  float mean = 0.0f;
  for (uint16_t i = 0; i < FFT_SIZE; i++) mean += fftReal[i];
  mean /= FFT_SIZE;
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    fftReal[i] = (fftReal[i] - mean) * hannCoeff[i];
  }

  // Step 3: FFT, then search 4–6 Hz band
  fftInPlace(fftReal, fftImag, FFT_SIZE);

  const uint16_t kLo = static_cast<uint16_t>(
      TREMOR_LO_HZ * static_cast<float>(FFT_SIZE) / SAMPLE_RATE_HZ + 0.5f);
  const uint16_t kHi = static_cast<uint16_t>(
      TREMOR_HI_HZ * static_cast<float>(FFT_SIZE) / SAMPLE_RATE_HZ + 0.5f);
  const uint16_t b0 = (kLo < 1u) ? 1u : kLo;
  const uint16_t b1 = (kHi >= FFT_SIZE / 2)
                      ? static_cast<uint16_t>(FFT_SIZE / 2 - 1) : kHi;

  float    bandPwr = 0.0f, maxPwr = 0.0f;
  uint16_t maxK    = b0;
  for (uint16_t k = b0; k <= b1; k++) {
    const float pwr = fftReal[k]*fftReal[k] + fftImag[k]*fftImag[k];
    bandPwr += pwr;
    if (pwr > maxPwr) { maxPwr = pwr; maxK = k; }
  }

  // Peak frequency in Hz (direct bin→Hz, no 2π factor)
  tremorPeakHz = static_cast<float>(maxK) * SAMPLE_RATE_HZ
                 / static_cast<float>(FFT_SIZE);

  // Step 4: candidate conditions
  //   - RMS in valid tremor amplitude range
  //   - Peak bin holds ≥ 40 % of band power (narrowband = oscillatory)
  //   - Person is STOPPED (resting tremor; suppressed during walking)
  const float peakFrac  = (bandPwr > 1e-9f) ? (maxPwr / bandPwr) : 0.0f;
  const bool candidate  = (tremorRmsG  >= TREMOR_RMS_MIN)
                       && (tremorRmsG  <  TREMOR_RMS_MAX)
                       && (peakFrac    >= TREMOR_PEAK_FRAC)
                       && (motionState == MotionState::STOPPED);

  // Step 5: hysteresis counters
  if (candidate) {
    tremorClearCount = 0;
    if (tremorConfirmCount < TREMOR_CONFIRM_WIN) tremorConfirmCount++;
  } else {
    tremorConfirmCount = 0;
    if (tremorClearCount < TREMOR_CLEAR_WIN) tremorClearCount++;
  }
  if (tremorConfirmCount >= TREMOR_CONFIRM_WIN) isTremor = true;
  if (tremorClearCount   >= TREMOR_CLEAR_WIN)   isTremor = false;
}


// ─────────────────────────────────────────────────────────────────────────────
//  Display & serial
// ─────────────────────────────────────────────────────────────────────────────

static const char* motionStateToText(MotionState state) {
  return (state == MotionState::WALKING) ? "WALKING" : "STOPPED";
}

static void drawDisplay(uint16_t steps, float cadence, float frequencyHz,
                        float bandEng, bool tremor, float tremorEng,
                        float tremorHz, MotionState state) {
  char line[20];

  u8x8.clearLine(0); u8x8.clearLine(1); u8x8.clearLine(2);
  u8x8.clearLine(3); u8x8.clearLine(4); u8x8.clearLine(5);
  u8x8.clearLine(6); u8x8.clearLine(7);

  if (!imuReady) {
    u8x8.setCursor(0, 0);
    u8x8.print("IMU: FAIL");
    return;
  }

  // Line 0: step count
  snprintf(line, sizeof(line), "Steps:%5u", static_cast<unsigned>(steps));
  u8x8.setCursor(0, 0);
  u8x8.print(line);

  // Line 1: cadence (steps per minute, 10-s rolling window)
  snprintf(line, sizeof(line), "Cadnc:%4dspm", static_cast<int>(cadence + 0.5f));
  u8x8.setCursor(0, 1);
  u8x8.print(line);

  // Line 2: dominant walking-band frequency
  snprintf(line, sizeof(line), "Hz:%6.2f", frequencyHz);
  u8x8.setCursor(0, 2);
  u8x8.print(line);

  // Line 3: walking-band energy (g²)
  snprintf(line, sizeof(line), "Eband:%.4f", bandEng);
  u8x8.setCursor(0, 3);
  u8x8.print(line);

  // Line 4: tremor detection + strength
  if (tremor) {
    const char* strength = (tremorEng < TREMOR_STRONG_THRESH) ? "Weak" : "Str ";
    snprintf(line, sizeof(line), "Tremor:YES %s", strength);
  } else {
    snprintf(line, sizeof(line), "Tremor: NO");
  }
  u8x8.setCursor(0, 4);
  u8x8.print(line);

  // Line 5: dominant frequency inside the tremor band (3–6 Hz)
  snprintf(line, sizeof(line), "THz:%5.2f", tremorHz);
  u8x8.setCursor(0, 5);
  u8x8.print(line);

  // Line 6: motion state
  snprintf(line, sizeof(line), "%-16s", motionStateToText(state));
  u8x8.setCursor(0, 6);
  u8x8.print(line);

  // Line 7: BLE connection status
  u8x8.setCursor(0, 7);
  u8x8.print(Bluefruit.connected() ? "BLE: OK " : "BLE: --- ");
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

  chrIdleMs.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrIdleMs.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrIdleMs.setFixedLen(4);
  chrIdleMs.begin();
  chrIdleMs.write32(0);

  chrState.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrState.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrState.setFixedLen(1);
  chrState.begin();
  chrState.write8(0);

  chrPeakHz.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrPeakHz.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrPeakHz.setFixedLen(4);
  chrPeakHz.begin();
  chrPeakHz.writeFloat(0.0f);

  chrBandEnergy.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrBandEnergy.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrBandEnergy.setFixedLen(4);
  chrBandEnergy.begin();
  chrBandEnergy.writeFloat(0.0f);

  chrMotionScore.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  chrMotionScore.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrMotionScore.setFixedLen(4);
  chrMotionScore.begin();
  chrMotionScore.writeFloat(0.0f);
}

static void connectCallback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char name[32] = {0};
  connection->getPeerName(name, sizeof(name));
  Serial.print("BLE central: ");
  Serial.println(name);
}

static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
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
  Serial.println("BLE: XIAO WalkSense (custom svc + Nordic UART)");
}

// Sends all walking metrics as BLE notifications + a Nordic UART CSV line.
// CSV format: millis,steps,cadenceSpm,peakHz,bandEnergy,motionScore,state(0/1),tremor(0/1),tremorHz
static void pushBleData(uint16_t steps, float cadence, MotionState state,
                        float peakHz, float engBand, float score,
                        bool tremor, float tremorHz) {
  if (!Bluefruit.connected()) return;

  chrSteps.notify32(static_cast<uint32_t>(steps));
  chrIdleMs.notify32(static_cast<uint32_t>(cadence + 0.5f));  // repurposed: cadence spm
  chrState.notify8(static_cast<uint8_t>(state));
  chrPeakHz.notify(&peakHz, sizeof(peakHz));
  chrBandEnergy.notify(&engBand, sizeof(engBand));
  chrMotionScore.notify(&score, sizeof(score));

  if (bleuart.notifyEnabled()) {
    char line[128];
    const int n = snprintf(line, sizeof(line),
      "%lu,%u,%d,%.2f,%.5f,%.3f,%u,%u,%.2f\r\n",
      static_cast<unsigned long>(millis()),
      static_cast<unsigned>(steps),
      static_cast<int>(cadence + 0.5f),
      peakHz, engBand, score,
      static_cast<uint8_t>(state),
      tremor ? 1u : 0u,
      tremorHz);
    if (n > 0 && n < static_cast<int>(sizeof(line)))
      bleuart.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(n));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup / loop
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

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
    imuReady = false;
  } else {
    Serial.println("IMU OK.");
    delay(10);  // let ODR stabilise before reconfiguring
    const int pedoErr = configPedometer();
    if (pedoErr != 0) {
      Serial.print("Pedometer config errors: ");
      Serial.println(pedoErr);
    } else {
      Serial.println("Pedometer ON @ 52 Hz, INT1=STEP_DET.");
    }
    imuReady = true;

    // Attach hardware interrupt: RISING edge on INT1 → LSM6DS3 STEP_DET event.
    // The ISR only sets a flag — no Wire/I2C calls inside the ISR.
    pinMode(INT1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT1_PIN), stepISR, RISING);
    Serial.print("INT1 attached on pin ");
    Serial.println(INT1_PIN);
  }

  setupBle();

  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print(imuReady ? "Ready" : "IMU FAIL");
}

void loop() {
  // Drift-free scheduling: increment by a fixed period each time rather than
  // rescheduling from "now", so the average sample rate stays at 52 Hz.
  static uint32_t nextSampleUs = 0;
  static uint32_t nextDisplayMs = 0;
  static uint32_t nextBleMs = 0;

  const uint32_t nowUs = micros();

  // Guard against long absence (e.g., Serial.print stalls): if we are more
  // than one full period behind, resync to avoid burst catch-up.
  if ((int32_t)(nowUs - nextSampleUs) > (int32_t)(SAMPLE_PERIOD_US * 4)) {
    nextSampleUs = nowUs;
  }
  if ((int32_t)(nowUs - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_PERIOD_US;

  const uint32_t nowMs = millis();

  if (!imuReady) {
    Serial.println("IMU not ready.");
    return;
  }

  // ── Read accelerometer ────────────────────────────────────────────────────
  // Samples are evenly spaced by the 52 Hz scheduler above — read straight
  // through exactly like the basic IMU example (no STATUS_REG polling needed).
  const float ax = xIMU.readFloatAccelX();
  const float ay = xIMU.readFloatAccelY();
  const float az = xIMU.readFloatAccelZ();
  const float mag = sqrtf(ax * ax + ay * ay + az * az);

  // ── Read gyroscope (for tremor pipeline) ─────────────────────────────────
  const float gx = xIMU.readFloatGyroX();
  const float gy = xIMU.readFloatGyroY();
  const float gz = xIMU.readFloatGyroZ();
  const float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);

  // ── Walking FFT (accelerometer magnitude) ────────────────────────────────
  // Push into sliding FIFO; run FFT every FFT_STRIDE=26 new samples (0.5 s)
  // once the buffer is fully populated (first FFT after 128 samples = 2.46 s).
  pushSample(mag);
  if (sampleFill >= FFT_SIZE && strideCount >= FFT_STRIDE) {
    strideCount = 0;
    computeFftEnergy();
  }
  updateMotionWindow(mag);

  // ── Tremor FFT (gyro magnitude) ───────────────────────────────────────────
  updateGyroRms(gyroMag);
  pushGyroSample(gyroMag);
  if (gyroSampleFill >= FFT_SIZE && gyroStrideCount >= TREMOR_FFT_STRIDE) {
    gyroStrideCount = 0;
    computeTremorFft();
  }

  // ── Step count + ISR flag ─────────────────────────────────────────────────
  // Atomically latch and clear the ISR flag.
  bool isrStep = false;
  if (g_stepFlag) {
    noInterrupts();
    g_stepFlag = false;
    interrupts();
    isrStep = true;
    lastStepMs = nowMs;  // record when the pedometer last fired
  }
  const uint16_t steps = readStepCount();

  // ── Walking state machine ─────────────────────────────────────────────────
  // WALKING only when the pedometer is actively firing (recentStep) AND the
  // accelerometer motion score is high enough.  This prevents arm swings,
  // tremors, or table knocks from triggering the walking state.
  // !recentStep is an immediate override — no hysteresis when the pedometer
  // has gone silent.  Only within the hysteresis band (EXIT < score < ENTER)
  // with an active step stream does the state stay unchanged.
  const bool recentStep = (nowMs - lastStepMs) < STEP_TIMEOUT_MS;
  if (isrStep && motionScore >= WALK_SCORE_EXIT) {
    // Fresh hardware step with any meaningful motion → enter WALKING immediately.
    // Using WALK_SCORE_EXIT (not ENTER) here avoids missing the first few steps
    // of a walk before the score window fills up.
    motionState = MotionState::WALKING;
  } else if (!recentStep && motionScore <= WALK_SCORE_EXIT) {
    // Step stream silent for >3 s AND accel motion has died → STOPPED.
    // Requiring BOTH prevents a single missed ISR from incorrectly dropping state.
    motionState = MotionState::STOPPED;
  } else if (recentStep) {
    // Step still within timeout window: use score hysteresis.
    if      (motionScore >= WALK_SCORE_ENTER) motionState = MotionState::WALKING;
    else if (motionScore <= WALK_SCORE_EXIT)  motionState = MotionState::STOPPED;
    // else: hysteresis band — keep current state unchanged
  }
  // !recentStep but score is in hysteresis band → keep current state (don't snap)

  // ── Serial output ─────────────────────────────────────────────────────────
  Serial.print("St:");    Serial.print(steps);
  Serial.print(" Cd:");   Serial.print(static_cast<int>(cadenceSpm + 0.5f));
  Serial.print("spm Hz:"); Serial.print(peakFrequencyHz, 2);
  Serial.print(" Eb:");   Serial.print(bandEnergy, 5);
  Serial.print(" Tr:");   Serial.print(isTremor ? "YES" : "NO");
  Serial.print(" THz:");  Serial.print(tremorPeakHz, 2);
  Serial.print(" TRMS:"); Serial.print(tremorRmsG, 2);
  Serial.print(" mg:");   Serial.print(mag, 3);
  Serial.print(" ");      Serial.println(motionStateToText(motionState));

  // ── Display (throttled to ~8 Hz to keep I2C bus clear for IMU reads) ──────
  if (nowMs >= nextDisplayMs) {
    nextDisplayMs = nowMs + 120;
    drawDisplay(steps, cadenceSpm, peakFrequencyHz, bandEnergy,
                isTremor, tremorRmsG, tremorPeakHz, motionState);
  }

  // ── BLE notification (throttled to 1 Hz) ─────────────────────────────────
  if (nowMs >= nextBleMs) {
    nextBleMs = nowMs + 1000;
    // Cadence: slide the 10-s step history window forward
    stepHistory[stepHistHead] = steps;
    stepHistHead = (stepHistHead + 1) % 10;
    const uint16_t oldSteps = stepHistory[stepHistHead]; // oldest slot = 10 s ago
    cadenceSpm = (steps >= oldSteps) ? (steps - oldSteps) * 6.0f : 0.0f;

    pushBleData(steps, cadenceSpm, motionState, peakFrequencyHz, bandEnergy,
                motionScore, isTremor, tremorPeakHz);
  }

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
