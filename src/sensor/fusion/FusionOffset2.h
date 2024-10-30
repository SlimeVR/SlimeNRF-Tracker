/**
 * @file FusionOffset.h
 * @author Seb Madgwick
 * @brief Gyroscope offset correction algorithm for run-time calibration of the
 * gyroscope offset.
 */

#ifndef FUSION_OFFSET2_H
#define FUSION_OFFSET2_H

//------------------------------------------------------------------------------
// Includes

#include "../../../Fusion/Fusion/FusionOffset.h"

//------------------------------------------------------------------------------
// Function declarations

void FusionOffsetInitialise2(FusionOffset *const offset, const unsigned int sampleRate);

FusionVector FusionOffsetUpdate2(FusionOffset *const offset, FusionVector gyroscope);

#endif

//------------------------------------------------------------------------------
// End of file
