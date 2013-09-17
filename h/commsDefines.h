/* 
 * File:   commsDefines.h
 * Author: Michael R Sandstedt
 *
 * Created on September 15, 2013, 8:16 PM
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#define START_MESSAGE               0xFE

//message type received from GUI
#define START_COMMAND               0x00
#define STOP_COMMAND                0x01
#define CONFIG_MUX_ADDRESSING       0x04
#define BALANCE_WHEASTONE_BRIDGE    0x05

//message type sent to GUI
#define CONFIRM_START_COMMAND       0x80
#define CONFIRM_STOP_COMMAND        0x81
#define REPORT_VALUES               0x82
#define REPORT_ERROR                0x83
#define CONFIRM_MUX_ADDRESSING      0x84
#define CONFIRM_BALANCE_BRIDGE      0x85

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
#define BRIDGE_BALANCE_VOLTAGE_OUT_OF_RANGE 0x0C
#define BRIDGE_BALANCE_FREQUENCY_OUT_OF_RANGE 0x0D
#define BRIDGE_BALANCE_FAILURE 0x0E
#define RECEIVED_PACKET_WITH_INCORRECT_SIZE 0x0F
#define UNRECOGNIZED_COMMAND_RECEIVED 0x10
#define ANALOG_GAIN_OUT_OF_RANGE 0x11
