#pragma once
#include <stdint.h>

// Output of the tremor sub-classifier (run only when stop gate = STOPPED).
enum TremorKind {
  TR_NONE = 0,
  TR_REGULAR,      // narrow peak ~3–7 Hz — Parkinsonian resting tremor
  TR_DYSKINESIA,   // wide / chaotic 4–12 Hz spectrum
  TR_BRADYKINESIA, // very low energy, slow signal (<2.5 Hz)
};

extern TremorKind g_tremorKind;
extern float      g_spectralWidthHz;   // |hzMag - hzZ|; wide = chaotic / dyskinesia
extern float      g_tremorPeakHz;      // energy-weighted tremor-band peak
extern float      g_tremorEnergy;      // (energyMag + energyZ) / 2 in tremor band
extern float      g_tremorRatio;       // tremor_energy / total_energy — MUST dominate
extern float      g_widePeakHz;        // energy-weighted full-band (0.5–12 Hz) peak

// Combined classifier. Uses BOTH the full-band and tremor-band features:
//   - widePeak / wideEnergy : where the dominant motion actually lives
//   - tremorPeak / tremorEnergy : strength of high-freq sub-band
// A tremor is only declared when the tremor band dominates total energy AND
// the wide-band peak agrees that the dominant motion is in tremor frequencies.
void classifyTremorKind(float widePeakMag, float widePeakZ,
                        float wideEnergyMag, float wideEnergyZ,
                        float tremorPeakMag, float tremorPeakZ,
                        float tremorEnergyMag, float tremorEnergyZ);
const char* tremorKindName(TremorKind k);
