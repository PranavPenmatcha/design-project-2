#pragma once
#include <stdint.h>

// Stage B: condition label. Combines activity context + tremor kind + gait stats
// into a single user-visible classification.
enum ConditionLabel {
  COND_NORMAL = 0,        // healthy / nothing detected
  COND_TREMOR_REST,       // resting tremor (Parkinsonian) while still
  COND_DYSKINESIA,        // chaotic involuntary movement
  COND_BRADYKINESIA,      // slow / reduced movement (walking or still)
  COND_FOG_CANDIDATE,     // freezing-of-gait candidate: trembling-in-place after walking
  COND_NORMAL_WALK,       // walking, no pathology overlay
};

extern ConditionLabel g_condition;

void runConditionPipeline();
const char* conditionName(ConditionLabel c);
