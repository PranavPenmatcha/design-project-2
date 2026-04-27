#pragma once
#include <stdint.h>
#include "config.h"

// Peak frequency and band energy for the full scan range (0.5–12 Hz).
extern float peakFrequencyHzMag;
extern float bandEnergyMag;
extern float peakFrequencyHzZ;
extern float bandEnergyZ;

// Peak frequency and band energy from the tremor sub-band scan (3–12 Hz).
extern float peakFrequencyHzTremorMag;
extern float bandEnergyTremorMag;
extern float peakFrequencyHzTremorZ;
extern float bandEnergyTremorZ;

// Low-band (slow periodic) motion on |a|; 0 Hz when energy below MOTION_ENERGY_FLOOR.
extern float peakFrequencyHzMotion;
extern float bandEnergyMotion;

void precomputeHannWindow();
void pushFifoSample(float ax, float ay, float az);
