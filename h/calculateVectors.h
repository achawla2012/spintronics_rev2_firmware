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

extern inline void spawnVectorCalcThread(uint16_t delayCycles, uint8_t sensorIndex, uint64_t *cosAccumulator, uint64_t *sinAccumulator, bool bridgeADCClip, bool coilADCClip, bool bridgeDigitalClip, bool f1PlusF2OutOfRange, float implementedBridgeGain);
extern void calculateFinalVectors(void);

extern uint8_t sensorAddressCapture;
extern int64_t cosAccumulatorCapture[5];
extern int64_t sinAccumulatorCapture[5];
extern bool bridgeADCClipCapture;
extern bool coilADCClipCapture;
extern bool bridgeDigitalClipCapture;
extern bool f1PlusF2OutOfRangeCapture;
extern float implementedBridgeGainCapture;
