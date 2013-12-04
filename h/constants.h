/* 
 * File:   constants.h
 * Author: Michael R Sandstedt
 *
 * Created on September 16, 2013, 9:34 PM
 */

//these values are accurate
#define PROCESSOR_CYCLES_PER_MS 70000
#define CURRENT_SENSE_RESISTOR_OHMS 10e-3
#define SCALE_DIVIDER 1.07374182e9 //Where does 1.07374182e9 come from?  I forget, but this works
#define ADC_FULLSCALE_VOLTS 2.8

//these values depend upon the sample rate
#define SAMPLE_RATE 32000 //units are samples per second
#define HALF_SAMPLE_RATE 16000.0 //SAMPLE_RATE * 0.5
#define TWICE_SAMPLE_PERIOD 62.5e-6 //1.0 / HALF_SAMPLE_RATE
#define MAX_OUTPUT_HZ 14400.0 //0.9 * HALF_SAMPLE_RATE
#define DEFAULT_BALANCE_FREQUENCY 2048//1000Hz, _Q15ftoi(1000.0 * twice_sample_period);

//TODO: these values need calibration
#define FULLSCALE_BRIDGE_DAC_VOLTAGE 2.20901803//units are volts//the Wheatstone bridge voltage that corresponds to a full-scale output//as of 5/6/2013: the DAC outputs 1.25Vpeak on two differential pins giving 2.5peak total; the DAC buffer gain is 6.04k/4.99k; 2.5 * 6.04 / 4.99 = 3.02605210
#define FULLSCALE_COIL_DAC_VOLTAGE 2.20901803//untis are volts//the output voltage for the coil channel that corresponds to a full-scale output//as of 5/6/2013: the DAC outputs 1.25Vpeak on two differential pins giving 2.5peak total; the DAC buffer gain is 6.04k/4.99k; 2.5 * 6.04 / 4.99 = 3.02605210
#define U2_BUF_GAIN 1.39819005
#define U2_INVERSE_GAIN .715210354
#define COIL_ADC_BUFFER_GAIN 20.179

//TODO: these values depend upon the above calibrated values
#define DEFAULT_BALANCE_AMPLITUDE 14834 //1Volt, _Q15ftoi(1.0 / FULLSCALE_BRIDGE_DAC_VOLTAGE);
#define COIL_ADC_SCALE_FACTOR 12.9228565e-9 //ADC_FULLSCALE_VOLTS / CURRENT_SENSE_RESISTOR_OHMS / COIL_ADC_BUFFER_GAIN / SCALE_DIVIDER
#define BRIDGE_ADC_SCALE_FACTOR 2.60770322e-9 //ADC_FULLSCALE_VOLTS / SCALE_DIVIDER
#define BRIDGE_ADC_SCALE_FACTOR_BY_2 1.30385161e-9 //BRIDGE_ADC_SCALE_FACTOR / 2.0
#define BRIDGE_ADC_SCALE_FACTOR_BY_4 651.925805e-12 //BRIDGE_ADC_SCALE_FACTOR / 4.0
#define BRIDGE_ADC_SCALE_FACTOR_BY_8 325.962902e-12 //BRIDGE_ADC_SCALE_FACTOR / 8.0
#define BRIDGE_ADC_SCALE_FACTOR_BY_16 162.981451e-12 //BRIDGE_ADC_SCALE_FACTOR / 16.0
#define BRIDGE_ADC_BUFFER_MIN_GAIN 9.38784748 //getBridgeBufGainFromU24Code(0)
#define BRIDGE_ADC_BUFFER_MAX_GAIN 157.022822 //getBridgeBufGainFromU24Code(0xFF)
#define INVERSE_BRIDGE_ANALOG_MIN_GAIN 106.520691e-3
#define INVERSE_BRIDGE_ANALOG_MAX_GAIN 6.36850101e-3
#define INVERSE_U2_BUF_GAIN 715.210354e-3 //1.0 / U2_BUF_GAIN;

