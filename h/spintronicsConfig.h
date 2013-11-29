/*
 * spintronicsConfig.h (formerly spintronics.h)
 *
 * written
 * by Michael Reinhart Sandstedt
 *
 * First release: May 7th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#ifndef SPINTRONICS_CONFIG_H
#define	SPINTRONICS_CONFIG_H

//#define MEASURE_F2_AT_BRIDGE
//#define MEASURE_F2_AT_COIL
//#define SIMULATION_MODE //un-comment this for a simulation specific build
//#define CODEC_USES_I2S //un-comment this if tx/rx samples from the CODEC are in I2S format; otherwise samples are left-justified
#define TRIG_USES_LUT//un-comment this to enable the LUT-based sin / cos functions; otherwise, the C30 _Q15cosPI(), _Q15sinPI() functions are used

#define NO_GUI
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

//bridge ADC
#define ADCDAC_GROUP_DELAY 26//units are samples//Note that pointer and pointerPlusOne in generateAndProcessSamples.c/ShiftRegister are uint8_t, so max ADCDAC_GROUP_DELAY is 254

#define MAX_MEASUREMENT_SAMPLES 0x7FFF0000//the counters that keep track of this are uint32_t; must be strictly less than 0x8000000 to avoid ambiguity when casting to int32_t
#define MIN_MEASUREMENT_SAMPLES 3840//TODO: what's our minimum measurement period before UART transmission overhead causes the state machine to fail? appropriate value TBD

#define DELAY_TO_VECTOR_CALC_THREAD 150//processor cycles

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

#endif