#include "fft_tremor.h"
#include "Arduino.h"
#include <math.h>

// ── Sliding FIFO ──────────────────────────────────────────────────────────────
static float    fifoBufMag[FFT_SIZE];
static float    fifoBufZ[FFT_SIZE];
static uint16_t fifoWrite       = 0;
static uint16_t samplesSinceFft = 0;
static bool     fifoFilled      = false;

// ── Exported globals ──────────────────────────────────────────────────────────
float peakFrequencyHzMag       = 0.0f;
float bandEnergyMag            = 0.0f;
float peakFrequencyHzTremorMag = 0.0f;
float bandEnergyTremorMag      = 0.0f;

float peakFrequencyHzZ       = 0.0f;
float bandEnergyZ            = 0.0f;
float peakFrequencyHzTremorZ = 0.0f;
float bandEnergyTremorZ      = 0.0f;

float peakFrequencyHzMotion  = 0.0f;
float bandEnergyMotion       = 0.0f;

// ── FFT scratch buffers and Hann window ───────────────────────────────────────
static float fftReal[FFT_SIZE];
static float fftImag[FFT_SIZE];
static float hannCoeff[FFT_SIZE];
static float hannPowerGain = 1.0f;  // sum(w[n]^2)/N — corrects Parseval energy for window loss

void precomputeHannWindow() {
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

// Copies the circular FIFO into fftReal/fftImag in chronological order,
// removes DC, applies Hann window, then runs the FFT in-place.
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

// Scans [loHz, hiHz] in the already-transformed fftReal/fftImag.
// Extracts peak frequency (parabolic interpolation) and one-sided band energy.
static void scanFftBand(float loHz, float hiHz, float& outHz, float& outEnergy,
                        float energyFloor) {
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

  if (outEnergy < energyFloor) outHz = 0.0f;
}

void pushFifoSample(float ax, float ay, float az) {
  const float mag = sqrtf(ax*ax + ay*ay + az*az);

  fifoBufMag[fifoWrite] = mag;
  fifoBufZ[fifoWrite]   = az;
  fifoWrite = (fifoWrite + 1) & (FFT_SIZE - 1);
  if (fifoWrite == 0) fifoFilled = true;
  if (samplesSinceFft < 0xFFFF) samplesSinceFft++;

  if (fifoFilled && samplesSinceFft >= FFT_STRIDE) {
    samplesSinceFft = 0;

    prepFftFromBuf(fifoBufMag, fifoWrite);
    scanFftBand(PEAK_BAND_LO_HZ,   PEAK_BAND_HI_HZ,   peakFrequencyHzMag,       bandEnergyMag,
                BAND_ENERGY_FLOOR);
    scanFftBand(TREMOR_BAND_LO_HZ, TREMOR_BAND_HI_HZ, peakFrequencyHzTremorMag, bandEnergyTremorMag,
                BAND_ENERGY_FLOOR);
    scanFftBand(MOTION_BAND_LO_HZ, MOTION_BAND_HI_HZ, peakFrequencyHzMotion,    bandEnergyMotion,
                MOTION_ENERGY_FLOOR);

    prepFftFromBuf(fifoBufZ, fifoWrite);
    scanFftBand(PEAK_BAND_LO_HZ,   PEAK_BAND_HI_HZ,   peakFrequencyHzZ,       bandEnergyZ,
                BAND_ENERGY_FLOOR);
    scanFftBand(TREMOR_BAND_LO_HZ, TREMOR_BAND_HI_HZ, peakFrequencyHzTremorZ, bandEnergyTremorZ,
                BAND_ENERGY_FLOOR);
  }
}
