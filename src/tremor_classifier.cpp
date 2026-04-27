#include "tremor_classifier.h"
#include "config.h"
#include <math.h>

TremorKind g_tremorKind       = TR_NONE;
float      g_spectralWidthHz  = 0.0f;
float      g_tremorPeakHz     = 0.0f;
float      g_tremorEnergy     = 0.0f;
float      g_tremorRatio      = 0.0f;
float      g_widePeakHz       = 0.0f;

// Tremor band must dominate total motion energy by this fraction before we
// call anything a tremor. Below this, the tremor-band "peak" is just whatever
// noise happens to be loudest in 3–12 Hz — not real oscillation.
static constexpr float TREMOR_RATIO_MIN = 0.45f;

// And the full-band peak must itself fall in tremor frequencies. If wide peak
// is in walking band (0.8–3.5 Hz) the dominant motion is gait/sway, not tremor.
static constexpr float TREMOR_WIDE_PEAK_MIN = 3.0f;

static float energyWeighted(float aMag, float aZ, float wMag, float wZ) {
  return (aMag * wMag + aZ * wZ) / (wMag + wZ + 1e-30f);
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

  // Tremor-band energy as a fraction of total motion energy.
  const float ratio = (wideEnergy > 1e-9f) ? (tremorEnergy / wideEnergy) : 0.0f;

  g_tremorEnergy    = tremorEnergy;
  g_tremorPeakHz    = tremorHz;
  g_widePeakHz      = wideHz;
  g_spectralWidthHz = width;
  g_tremorRatio     = ratio;

  // ── Bradykinesia: very low-energy slow movement ─────────────────────────────
  // Use wide-band: dominant motion must be slow (<2.5 Hz) and quiet.
  if (wideEnergy >= BRADYKINESIA_ENERGY_MIN && wideEnergy < BRADYKINESIA_ENERGY_MAX &&
      wideHz > 0.0f && wideHz < 2.5f) {
    g_tremorKind = TR_BRADYKINESIA;
    return;
  }

  // ── Tremor-band must dominate total energy AND wide peak must agree ─────────
  // If the wide-band peak is in walking range (0.8–3.5 Hz), the tremor-band
  // peak is just harmonic noise — not a real tremor.
  const bool tremorDominates = (ratio >= TREMOR_RATIO_MIN) &&
                               (wideHz >= TREMOR_WIDE_PEAK_MIN);

  if (!tremorDominates) {
    g_tremorKind = TR_NONE;
    return;
  }

  // Now the dominant motion really is in 3–12 Hz. Decide subtype.

  // Dyskinesia: high absolute energy OR wide spectrum at moderate-high freq.
  if (tremorEnergy >= DYSKINESIA_ENERGY_MIN && tremorHz >= 4.0f && tremorHz <= 12.0f) {
    g_tremorKind = TR_DYSKINESIA;
    return;
  }
  if (tremorEnergy >= TREMOR_ENERGY_MIN && width > 1.5f &&
      tremorHz >= 4.0f && tremorHz <= 12.0f) {
    g_tremorKind = TR_DYSKINESIA;
    return;
  }

  // Regular Parkinsonian rest tremor: narrow peak, 3–7 Hz, energy above floor.
  if (tremorEnergy >= TREMOR_ENERGY_MIN &&
      tremorHz >= 3.0f && tremorHz <= 7.0f && width <= 1.5f) {
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
