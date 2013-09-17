/* 
 * File:   generateConstants.h
 * Author: Michael R Sandstedt
 *
 * Created on September 15, 2013, 9:01 PM
 *
 * Use this file to calibrate the device to output calibrated values in volts
 * and amperes.  These constants must be calibrated based upon the actual
 * behavior of the on-board analog circuitry
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

//DAC
#define FULLSCALE_BRIDGE_DAC_VOLTAGE 2.20901803//units are volts//the Wheatstone bridge voltage that corresponds to a full-scale output//as of 5/6/2013: the DAC outputs 1.25Vpeak on two differential pins giving 2.5peak total; the DAC buffer gain is 6.04k/4.99k; 2.5 * 6.04 / 4.99 = 3.02605210
#define FULLSCALE_COIL_DAC_VOLTAGE 2.20901803//untis are volts//the output voltage for the coil channel that corresponds to a full-scale output//as of 5/6/2013: the DAC outputs 1.25Vpeak on two differential pins giving 2.5peak total; the DAC buffer gain is 6.04k/4.99k; 2.5 * 6.04 / 4.99 = 3.02605210

extern void generateConstants(void);

extern float half_sample_rate;
extern float twice_sample_period;
extern float max_output_hz;
extern _Q15 default_balance_frequency; //units are Q15 half-cycles per sample-period
extern _Q15 default_balance_amplitude; //units are Q15 fraction of full-scale
extern float coil_ADC_scale_factor; //units are amperes
extern float bridge_ADC_scale_factor;//units are volts
extern float bridge_ADC_scale_factor_by_2;//units are volts
extern float bridge_ADC_scale_factor_by_4;//units are volts
extern float bridge_ADC_scale_factor_by_8;//units are volts
extern float bridge_ADC_scale_factor_by_16;//units are volts

