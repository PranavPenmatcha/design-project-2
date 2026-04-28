#include "tremor_classifier.h"
#include "config.h"
#include <math.h>

TremorKind g_tremorKind       = TR_NONE;
float      g_spectralWidthHz  = 0.0f;
float      g_tremorPeakHz     = 0.0f;
float      g_tremorEnergy     = 0.0f;
float      g_tremorRatio      = 0.0f;
float      g_widePeakHz       = 0.0f;
float      g_peakSeparationHz = 0.0f;

static float energyWeighted(float aMag, float aZ, float wMag, float wZ) {
  return (aMag * wMag + aZ * wZ) / (wMag + wZ + 1e-30f);
}

/** True when tremor-band peak sits in the physiological tremor band (clinical bands overlap here). */
static bool tremorOscillationBand(float tremorHz) {
  return tremorHz >= TREMOR_BAND_LO_HZ && tremorHz <= TREMOR_BAND_HI_HZ;
}

/**
 * Spectral consistency: dominant motion frequency (wide FFT) and tremor-band peak frequency must
 * tell the same story. If the wide peak sits in gait/sway (below ~3 Hz), harmonic leakage can
 * still show a peak in the tremor scan — require closer peak agreement and a higher energy ratio.
 */
static bool spectralAgreementForTremor(float wideHz, float tremorHz, float ratio) {
  const float sep = fabsf(wideHz - tremorHz);
  g_peakSeparationHz = sep;

  if (wideHz >= TREMOR_FREQ_DOM_MIN_HZ)
    return true;

  const bool peaksAligned = sep <= TREMOR_WIDE_TREMOR_MATCH_HZ;
  const bool strictRatio    = ratio >= TREMOR_RATIO_STRICT_LOW_WIDE;
  return peaksAligned && strictRatio && tremorOscillationBand(tremorHz);
}

void classifyTremorKind(float widePeakMag, float widePeakZ,
                        float wideEnergyMag, float wideEnergyZ,
                        float tremorPeakMag, float tremorPeakZ,
                        float tremorEnergyMag, float tremorEnergyZ) {
  const float tremorEnergy = (tremorEnergyMag + tremorEnergyZ) * 0.5f;
  const float wideEnergy   = (wideEnergyMag   + wideEnergyZ)   * 0.5f;

  const float tremorHz = energyWeighted(tremorPeakMag, tremorPeakZ,
                                        tremorEnergyMag, tremorEnergyZ);
  const float wideHz   = energyWeighted(widePeakMag, widePeakZ,
                                        wideEnergyMag, wideEnergyZ);
  const float width    = fabsf(tremorPeakMag - tremorPeakZ);

  const float ratio = (wideEnergy > 1e-9f) ? (tremorEnergy / wideEnergy) : 0.0f;

  g_tremorEnergy    = tremorEnergy;
  g_tremorPeakHz    = tremorHz;
  g_widePeakHz      = wideHz;
  g_spectralWidthHz = width;
  g_tremorRatio     = ratio;
  g_peakSeparationHz = fabsf(wideHz - tremorHz);

  // ── Bradykinesia: dominant motion is slow and low-energy (wide spectrum only) ──
  if (wideEnergy >= BRADYKINESIA_ENERGY_MIN && wideEnergy < BRADYKINESIA_ENERGY_MAX &&
      wideHz > 0.0f && wideHz < 2.5f) {
    g_tremorKind = TR_BRADYKINESIA;
    return;
  }

  // ── Tremor presence: energy + both frequency axes ─────────────────────────────
  // 1) Tremor band must carry enough of total motion energy.
  // 2) tremorHz must fall in the scanned tremor band (meaningful oscillation there).
  // 3) wideHz vs tremorHz must agree per spectralAgreementForTremor().
  const bool energyOk = ratio >= TREMOR_RATIO_MIN;
  const bool hasTremorOsc =
      energyOk &&
      tremorOscillationBand(tremorHz) &&
      spectralAgreementForTremor(wideHz, tremorHz, ratio);

  if (!hasTremorOsc) {
    g_tremorKind = TR_NONE;
    return;
  }

  const float peakSep = g_peakSeparationHz;

  // ── Subtype: use tremor frequency + peak separation + axiswidth + energy ────
  // Dyskinesia: higher-frequency / chaotic content — broader axis spread, higher energy,
  // or tremor peak clearly in the dyskinesia range with incoherent wide peak.
  const bool dyskHighEnergy =
      tremorEnergy >= DYSKINESIA_ENERGY_MIN &&
      tremorHz >= 4.0f && tremorHz <= TREMOR_BAND_HI_HZ;
  const bool dyskWideSpectrum =
      tremorEnergy >= TREMOR_ENERGY_MIN &&
      tremorHz >= 4.0f && tremorHz <= TREMOR_BAND_HI_HZ &&
      width > 1.5f;
  const bool dyskIncoherentPeaks =
      tremorEnergy >= TREMOR_ENERGY_MIN &&
      tremorHz >= 5.0f &&
      peakSep > TREMOR_COHERENCE_MAX_HZ;

  if (dyskHighEnergy || dyskWideSpectrum || dyskIncoherentPeaks) {
    g_tremorKind = TR_DYSKINESIA;
    return;
  }

  // Parkinsonian resting tremor: rhythmic ~3–7 Hz, narrow tremor-axis peaks, coherent spectrum.
  if (tremorEnergy >= TREMOR_ENERGY_MIN &&
      tremorHz >= 3.0f && tremorHz <= 7.0f &&
      width <= 1.5f &&
      peakSep <= TREMOR_COHERENCE_MAX_HZ) {
    g_tremorKind = TR_REGULAR;
    return;
  }

  g_tremorKind = TR_NONE;
}

const char* tremorKindName(TremorKind k) {
  switch (k) {
    case TR_NONE:         return "None";
    case TR_REGULAR:      return "Tremor";
    case TR_DYSKINESIA:   return "Dyskinesia";
    case TR_BRADYKINESIA: return "Bradykinesia";
    default:              return "None";
  }
}
