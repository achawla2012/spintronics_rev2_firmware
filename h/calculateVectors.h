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

extern void spawnVectorCalcThread(uint16_t delayCycles, uint8_t sensorIndex, uint64_t *cosAccumulator, uint64_t *sinAccumulator, bool bridgeADCClip, bool coilADCClip, bool bridgeDigitalClip, bool f1PlusF2OutOfRange);
extern void calculateFinalVectors(void);
