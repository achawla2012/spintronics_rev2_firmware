/* 
 * File:   commsDefines.h
 * Author: Michael R Sandstedt
 *
 * Created on September 15, 2013, 8:16 PM
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

//message type received from GUI
#define StartCommand 0x00
#define StopCommand 0x01
#define MuxAddressing 0x04

//confirmation message type to GUI
#define confirm_StartCommand 0x80
#define confirm_StopCommand 0x81
#define confirm_MuxAddressing 0x84

//error codes
#define Bad_Packet_XOR 0x01
#define A1_OUT_OF_RANGE 0x02
#define F1_OUT_OF_RANGE 0x03
#define A2_OUT_OF_RANGE 0x04
#define F2_OUT_OF_RANGE 0x05
#define F1_PLUS_F2_OUT_OF_RANGE 0x06
#define T_OUT_OF_RANGE 0x07
#define INVALID_DIGITAL_GAIN_VALUE 0x08
#define BRIDGE_ADC_CLIP 0x09
#define COIL_ADC_CLIP 0x0A
#define BRIDGE_DIGITAL_CLIP 0x0B
#define BRIDGE_BALANCE_FAILURE 0x0C
#define A1_CHANGED_DURING_BRIDGE_BALANCE 0x0D
#define F1_CHANGED_DURING_BRIDGE_BALANCE 0x0E
#define ATTEMPT_MEASURE_WITHOUT_BALANCED_BRIDGE 0x0F
