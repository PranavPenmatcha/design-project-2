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
extern float      g_peakSeparationHz;  // |wide peak Hz − tremor peak Hz|

// Uses full-band dominant frequency + tremor-band peak frequency together:
//   - widePeak / wideEnergy (0.5–12 Hz): dominant motion frequency
//   - tremorPeak / tremorEnergy (3–12 Hz): oscillation peak in tremor band
// Presence requires energy ratio + tremorHz in band + spectral agreement between the two peaks.
// Subtype uses tremorHz, axis peak spread, peak separation |wide−tremor|, and energy tiers.
void classifyTremorKind(float widePeakMag, float widePeakZ,
                        float wideEnergyMag, float wideEnergyZ,
                        float tremorPeakMag, float tremorPeakZ,
                        float tremorEnergyMag, float tremorEnergyZ);
const char* tremorKindName(TremorKind k);
