/*
 * calculateVectors.h
 *
 * Author: Michael Reinhart Sandstedt
 *
 * First release: September 14th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */
#ifndef SPINTRONICS_STRUCTS_H
#include "spintronicsStructs.h"
#endif

#ifndef CALCULATE_VECTORS_H
#define CALCULATE_VECTORS_H

extern inline void spawnVectorCalcThread(uint16_t delayCycles, uint8_t sensorIndex, uint64_t *cosAccumulator, uint64_t *sinAccumulator, bool bridgeADCClip, bool coilADCClip, bool bridgeDigitalClip, bool f1PlusF2OutOfRange, float implementedBridgeGain);
extern void calculateFinalVectors(void);

extern uint8_t sensorAddressCapture;
extern accumulator_t cosAccumulatorCapture;
extern accumulator_t sinAccumulatorCapture;
extern bool bridgeADCClipCapture;
extern bool coilADCClipCapture;
extern bool bridgeDigitalClipCapture;
extern bool f1PlusF2OutOfRangeCapture;
extern float inverseBridgeAnalogGainCapture;
extern bool waitForVectorCalculation;

#endif
