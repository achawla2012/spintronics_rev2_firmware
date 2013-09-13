/*
 * spintronics.h
 *
 * written
 * by Michael Reinhart Sandstedt
 *
 * First release: May 7th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

//#define MEASURE_F2_AT_BRIDGE
//#define SIMULATION_MODE //un-comment this for a simulation specific build
//#define CODEC_USES_I2S //un-comment this if tx/rx samples from the CODEC are in I2S format; otherwise samples are left-justified
#define TRIG_USES_LUT//un-comment this to enable the LUT-based sin / cos functions; otherwise, the C30 _Q15cosPI(), _Q15sinPI() functions are used
//#define REPORT_RECTANGULAR_VECTORS_FOR_BRIDGE//un-comment this to switch reporting from amplitude / phase to inPhase volts and quadrature volts

//uncomment PROBE and one of the probe sites to output that signal to the COIL DAC
//#define PROBE
//#define PROBE_BRIDGE_ADC
//#define PROBE_COIL_ADC
//#define PROBE_BRIDGE_X_COS_F1_T
//#define PROBE_BRIDGE_X_COS_F2_T
//#define PROBE_BRIDGE_X_COS_FSUM_T
//#define PROBE_BRIDGE_X_COS_FDIFF_T
//#define PROBE_COIL_X_COS_F2_T
//#define PROBE_BRIDGE_X_SIN_F1_T
//#define PROBE_BRIDGE_X_SIN_F2_T
//#define PROBE_BRIDGE_X_SIN_FSUM_T
//#define PROBE_BRIDGE_X_SIN_FDIFF_T
//#define PROBE_COIL_X_SIN_F2_T
//#define PROBE_BRIDGE_X_COS_F1_T_PLUS_PHI
//#define PROBE_BRIDGE_X_COS_F2_T_PLUS_PHI
//#define PROBE_BRIDGE_X_COS_FSUM_T_PLUS_PHI
//#define PROBE_BRIDGE_X_COS_FDIFF_T_PLUS_PHI
//#define PROBE_COIL_X_COS_F2_T_PLUS_PHI

//un-comment one of these only; this should be the same as the sample rate determined by the CODEC's xtal
//#define FORTY_EIGHT_KHZ
#define THIRTY_EIGHT_POINT_FOUR_KHZ
//#define THIRTY_TWO_KHZ
//#define TWENTY_FOUR_KHZ

#ifdef FORTY_EIGHT_KHZ
#define SAMPLE_RATE 48000 //units are samples per second
#define HALF_SAMPLE_RATE 24000 //units are samples per second
#define TWICE_SAMPLE_PERIOD 41.6666666e-6 //units are seconds
#endif

#ifdef THIRTY_EIGHT_POINT_FOUR_KHZ
#define SAMPLE_RATE 38400 //units are samples per second
#define HALF_SAMPLE_RATE 19200 //units are samples per second
#define TWICE_SAMPLE_PERIOD 52.0833333e-6 //units are seconds
#endif

#ifdef THIRTY_TWO_KHZ
#define SAMPLE_RATE 32000 //units are samples per second
#define HALF_SAMPLE_RATE 16000 //units are samples per second
#define TWICE_SAMPLE_PERIOD 62.5e-6 //units are seconds
#endif

#ifdef TWENTY_FOUR_KHZ
#define SAMPLE_RATE 24000 //units are samples per second
#define HALF_SAMPLE_RATE 12000 //units are samples per second
#define TWICE_SAMPLE_PERIOD 83.3333333e-6 //units are seconds
#endif

#define MAX_OUTPUT_FREQUENCY 0.9 * HALF_SAMPLE_RATE //units are samples per second
#define PROCESSOR_CYCLES_PER_SECOND 70000000
#define PROCESSOR_CYCLES_PER_MS 70000
#define PROCESSOR_CYCLES_PER_US 70

#ifdef SIMULATION_MODE
#define MEASUREMENT_SETUP_TIME 10
#define GAIN_CHECK_SETUP_TIME 10
#define BRIDGE_CHECK_SETUP_TIME 10
#define GAIN_CHECK_MEASURE_CYCLES 1
#define BRIDGE_CHECK_MEASURE_TIME_MULTIPLIER 1
#else
//units are samples//the amount of time to let the transients settle after switching sensors
#define MEASUREMENT_SETUP_TIME 1000
#define GAIN_CHECK_SETUP_TIME 50
#define BRIDGE_CHECK_SETUP_TIME 100
#define GAIN_CHECK_MEASURE_CYCLES 2
#define BRIDGE_CHECK_MEASURE_TIME_MULTIPLIER 3
#endif

//coil ADC//uncomment ONE of these, depending on the current sense resistor value used in hardware
//#define ONE_HUNDRED_MILLIOHM_CURRENT_SENSE_RESITOR
#define TEN_MILLIOHM_CURRENT_SENSE_RESISTOR
#ifdef ONE_HUNDRED_MILLIOHM_CURRENT_SENSE_RESITOR
#define A_COIL_CORRESPONDING_TO_ADC_FULLSCALE 1.33079341//units are amperes//the coil current that corresponds to a full-scale input//As of 5/6/2013: The CS4272ADC accepts 1.4Vpeak on each diffential pin, giving 2.8Vpeak total; the coil ADC buffer has a gain of 10k/499 + 1 = 21.04; 2.8/21.04 = 13.3079341; the current sense resistor is 0.1ohm; thus the full-scale current is 1.33079341 amps
#endif
#ifdef TEN_MILLIOHM_CURRENT_SENSE_RESISTOR
#define A_COIL_CORRESPONDING_TO_ADC_FULLSCALE 13.3079341//units are amperes//the coil current that corresponds to a full-scale input//As of 5/6/2013: The CS4272ADC accepts 1.4Vpeak on each diffential pin, giving 2.8Vpeak total; the coil ADC buffer has a gain of 10k/499 + 1 = 21.04; 2.8/21.04 = 13.3079341; the current sense resistor is 0.01ohm; thus the full-scale current is 13.3079341 amps
#endif

//bridge ADC
#define ADCDAC_GROUP_DELAY 26//units are samples//Note that pointer and pointerPlusOne in generateAndProcessSamples.c/ShiftRegister are uint8_t, so max ADCDAC_GROUP_DELAY is 254
//ADC Wheastone Bridge Voltages: units are volts//As of 5/6/2013: The maximum accepted full-scale voltage is ~2Volts; The CS4272ADC accepts 1.4Vpeak on each diffential pin, giving 2.8Vpeak total; the bridge ADC buffer has a gain of 309/221 = 1.39819004; 2.8V/1.39819004 = 2.00258900; 2.00258900/256 = 7.82261327e-3
#define FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS 1.66452288
#define FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_2 832.26144e-3
#define FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_4 416.13072-3
#define FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_8 208.06536e-3
#define FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_16 104.03268e-3
#define BRIDGE_ADC_F1_PHASE_OFFSET 0.0;
#define BRIDGE_ADC_F2_PHASE_OFFSET 0.0;
#define BRIDGE_ADC_FDIFF_PHASE_OFFSET 0.0;
#define BRIDGE_ADC_FSUM_PHASE_OFFSET 0.0;

//DAC
#define FULLSCALE_BRIDGE_DAC_VOLTAGE 2.20901803//units are volts//the Wheatstone bridge voltage that corresponds to a full-scale output//as of 5/6/2013: the DAC outputs 1.25Vpeak on two differential pins giving 2.5peak total; the DAC buffer gain is 6.04k/4.99k; 2.5 * 6.04 / 4.99 = 3.02605210
#define FULLSCALE_COIL_DAC_VOLTAGE 2.20901803//untis are volts//the output voltage for the coil channel that corresponds to a full-scale output//as of 5/6/2013: the DAC outputs 1.25Vpeak on two differential pins giving 2.5peak total; the DAC buffer gain is 6.04k/4.99k; 2.5 * 6.04 / 4.99 = 3.02605210

#define MAX_MEASUREMENT_SAMPLES 0x7FFF0000//the counters that keep track of this are uint32_t; must be strictly less than 0x8000000 to avoid ambiguity when casting to int32_t
#define MIN_MEASUREMENT_SAMPLES 1000//TODO: what's our minimum measurement period before UART transmission overhead causes the state machine to fail? appropriate value TBD

//states for bridgeBalanceFSM() must have bit 7 set!
#define BALANCE_BRIDGE_FSM_MASK                 0x40
#define IDLE                                    0x40
#define START_BRIDGE_BALANCE_FSM                0x41
#define START_GAIN_CAL                          0x42
#define SET_R_AMP_TO_LO_MID                     0x43
#define R_AMP_LO_MID_SIGNAL_SETTLING            0x44
#define R_AMP_LO_MID_CLIP_TEST                  0x45
#define SET_R_AMP_TO_HI_MID                     0x46
#define R_AMP_HI_MID_SIGNAL_SETTLING            0x47
#define R_AMP_HI_MID_CLIP_TEST                  0x48
#define START_BRIDGE_CAL                        0x49
#define SET_R_BRIDGE_TO_LO_MID                  0x4A
#define R_BRIDGE_LO_MID_SIGNAL_SETTLING         0x4B
#define R_BRIDGE_LO_MID_AMP_MEASURE             0x4C
#define R_BRIDGE_LO_MID_AMP_CALC                0x4D
#define SET_R_BRIDGE_TO_HI_MID                  0x4E
#define R_BRIDGE_HI_MID_SIGNAL_SETTLING         0x4F
#define R_BRIDGE_HI_MID_AMP_MEASURE             0x50
#define R_BRIDGE_HI_MID_AMP_CALC                0x51

//states for measurementFSM() must have bit 8 set!
#define MEASUREMENT_FSM_MASK                    0x80
#define START_MEASUREMENT_FSM                   0x81
#define START_SIGNAL_GEN                        0x82
#define	MEASURE                                 0x83
#define	CALCULATE_VECTORS                       0x84

//signal generator state encoding
#define RESET_SIGNAL_GEN 0
#define RUN_SIGNAL_GEN 1

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

#include <math.h>
#include <libq.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


int32_t asm16X16Mult(int16_t, int16_t);

extern inline void START_ATOMIC(void);
extern inline void END_ATOMIC(void);
extern inline void NOP(void);