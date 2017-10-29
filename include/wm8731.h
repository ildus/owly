#pragma once

#include "codec.h"

/* I2C configuration */
#define CODEC_I2C_ADDR 0b0011010 /* CSB=0, should be 0011011 if CSB=1 */

extern _Atomic unsigned samplecounter;
extern _Atomic CodecIntSample peakIn;
extern _Atomic CodecIntSample peakOut;

void codecInit(void);

/**
 * Get a pointer to where in the whole audio input buffer the ADC
 * DMA job is currently writing.
 */
void codecPeek(const int16_t** buffer, unsigned* buffersamples, unsigned* writepos);
