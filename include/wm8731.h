#pragma once

#include "codec.h"

/* I2C configuration */
#define CODEC_I2C_ADDR 0x1A /* CSB=0, should be 0x1B if CSB=1 */

extern _Atomic unsigned samplecounter;
extern _Atomic CodecIntSample peakIn;
extern _Atomic CodecIntSample peakOut;

void codecInit(void);

/**
 * Get a pointer to where in the whole audio input buffer the ADC
 * DMA job is currently writing.
 */
void codecPeek(const int16_t** buffer, unsigned* buffersamples, unsigned* writepos);
