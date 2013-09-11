/*
 * uartDrv.h
 *
 * designed and written
 * by Michael Reinhart Sandstedt
 *
 * First release: May 7th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

//function prototypes
extern void uartInit (void);
extern void transmitResults(uint8_t sensor, double *phaseAngle, float *amplitude, bool bridgeADCClipFlag, bool coilADCClipFlag, bool bridgeDigitalClipFlag);
extern void transmitError(uint8_t errorCode);

//global variables
extern bool GUIRequestingRun;
extern bool resetStateMachine;
extern uint32_t measurementTime;//units are samples
extern _Q15 f1;//units are half-cycles per sample-period
extern _Q15 f2;//units are half-cycles per sample-period
extern _Q15 fdiff;//units are half-cycles per sample-period
extern _Q15 fsum;//units are half-cycles per sample-period
extern _Q15 a1;
extern _Q15 a2;
extern uint8_t sensorAddressTable[256];
extern uint8_t numberOfSensors;

/****************************************************************
 * The gain factor can be 0 to 8;
 * Gain = 2^bridgeADCGainFactor;
 *
 * All internal processing is 16 bit.  Thus, the lowest 8 bits of
 * incoming 24bit ADC samples are truncated. This parameter left shifts
 * the 24bit samples from the Wheatstone bridge ADC prior to truncation
 *
 * Note that the communication protocol specifies Gain explicitly
 * (e.g. 1, 2, 4, 8, 16).  Also, the protocol does not allow for
 * values above 16.  BUT, digital gain stage can actually deal with
 * all possible values: 1, 2, 4, 8, 16, 32, 64, 128, and 256
 * To extend this functionality, add cases to this switch statement:
 * switch(GUISpeciedBridgeGainFactor) in uartDrv.c/processStartCommand()
 * and to this switch statement:
 * switch(bridgeADCGainFactor) in uartDrv.c/transmitResults
****************************************************************/
extern uint8_t bridgeADCGainFactor;
