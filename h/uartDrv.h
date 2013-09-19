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

#define MAX_MUX_ADDRESS_TABLE_SIZE 32

//function prototypes
extern void uartInit (void);
extern void transmitResults(uint8_t sensor, float *phaseAngle, float *amplitude, bool bridgeADCClipFlag, bool coilADCClipFlag, bool bridgeDigitalClipFlag);
extern void transmitError(uint8_t errorCode);
extern void processStartCommand(void);

//global variables
extern uint8_t global_state;
extern uint32_t measurementTime;//units are samples
extern _Q15 f1;//units are half-cycles per sample-period
extern _Q15 f2;//units are half-cycles per sample-period
extern _Q15 fdiff;//units are half-cycles per sample-period
extern _Q15 fsum;//units are half-cycles per sample-period
extern _Q15 a1;
extern _Q15 a2;
extern uint8_t sensorAddressTable[MAX_MUX_ADDRESS_TABLE_SIZE];
extern uint8_t numberOfSensors;
extern bool f1PlusF2OutOfRange;
extern _Q15 bridge_balance_amplitude;
extern _Q15 bridge_balance_frequency;
extern uint8_t latest_command;


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
extern uint8_t u24_code;
extern float implementedBridgeGain;
