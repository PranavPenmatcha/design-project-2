#pragma once
#include <stdint.h>

constexpr float    SAMPLE_RATE_HZ          = 52.0f;

constexpr uint16_t FFT_SIZE                = 128;
constexpr uint16_t FFT_STRIDE              = 26;
constexpr float    PEAK_BAND_LO_HZ         = 0.5f;
constexpr float    PEAK_BAND_HI_HZ         = 12.0f;
constexpr float    TREMOR_BAND_LO_HZ       = 3.0f;
constexpr float    TREMOR_BAND_HI_HZ       = 12.0f;
constexpr float    BAND_ENERGY_FLOOR       = 0.00133f;

// Rhythmic hand / slow motion: dominant FFT peak in this band; zeroed when below floor.
constexpr float    MOTION_BAND_LO_HZ       = 0.45f;
constexpr float    MOTION_BAND_HI_HZ       = 4.5f;
constexpr float    MOTION_ENERGY_FLOOR     = 0.0045f;

constexpr float    TREMOR_ENERGY_MIN       = 0.01333f;
constexpr float    DYSKINESIA_ENERGY_MIN   = 0.0667f;
constexpr float    BRADYKINESIA_ENERGY_MAX = 0.04f;
constexpr float    BRADYKINESIA_ENERGY_MIN = 0.00213f;

constexpr float    WALKING_CADENCE_MIN     = 40.0f;
constexpr float    WALKING_HZ_MAX          = 3.5f;
constexpr float    WALKING_HZ_MIN          = 0.8f;
