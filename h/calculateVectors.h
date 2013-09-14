extern void spawnVectorCalcThread(uint16_t delayCycles, uint8_t sensorIndex, uint64_t *cosAccumulator, uint64_t *sinAccumulator, bool bridgeADCClip, bool coilADCClip, bool bridgeDigitalClip);
extern void calculateFinalVectors(void);
