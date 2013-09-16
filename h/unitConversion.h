/* 
 * File:   unitConversion.h
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

//coil ADC//uncomment ONE of these, depending on the current sense resistor value used in hardware
//#define ONE_HUNDRED_MILLIOHM_CURRENT_SENSE_RESITOR
#define TEN_MILLIOHM_CURRENT_SENSE_RESISTOR
#ifdef ONE_HUNDRED_MILLIOHM_CURRENT_SENSE_RESITOR
#define COIL_ADC_SCALE_FACTOR 1.23939795e-9//amperage corresponding to ADC full-scale divided by 1.07374182e9  1.33079341//Where does 1.07374182e9 come from?  I forget, but this works
#endif
#ifdef TEN_MILLIOHM_CURRENT_SENSE_RESISTOR
#define COIL_ADC_SCALE_FACTOR 12.3939795e-9//units are amperes//the coil current that corresponds to a full-scale input//As of 5/6/2013: The CS4272ADC accepts 1.4Vpeak on each diffential pin, giving 2.8Vpeak total; the coil ADC buffer has a gain of 10k/499 + 1 = 21.04; 2.8/21.04 = 13.3079341; the current sense resistor is 0.01ohm; thus the full-scale current is 13.3079341 amps
#endif

//BRIDGE_ADC_SCALE_FACTOR is voltage across the bridge corresponding to ADC full-scale divided by 1.07374182e9;  Where does 1.07374182e9 come from?  I forget, but this works
//TODO: this assumes 40uV across bridge is full-scale and the wheatstone bridge buffer has fixed gain; neither are true!
#define BRIDGE_ADC_SCALE_FACTOR 37.2529031e-15
#define BRIDGE_ADC_SCALE_FACTOR_BY_2 18.6264516e-15
#define BRIDGE_ADC_SCALE_FACTOR_BY_4 9.31322578e-15
#define BRIDGE_ADC_SCALE_FACTOR_BY_8 4.65661289e-15
#define BRIDGE_ADC_SCALE_FACTOR_BY_16 2.32830645e-15

//DAC
#define FULLSCALE_BRIDGE_DAC_VOLTAGE 2.20901803//units are volts//the Wheatstone bridge voltage that corresponds to a full-scale output//as of 5/6/2013: the DAC outputs 1.25Vpeak on two differential pins giving 2.5peak total; the DAC buffer gain is 6.04k/4.99k; 2.5 * 6.04 / 4.99 = 3.02605210
#define FULLSCALE_COIL_DAC_VOLTAGE 2.20901803//untis are volts//the output voltage for the coil channel that corresponds to a full-scale output//as of 5/6/2013: the DAC outputs 1.25Vpeak on two differential pins giving 2.5peak total; the DAC buffer gain is 6.04k/4.99k; 2.5 * 6.04 / 4.99 = 3.02605210
