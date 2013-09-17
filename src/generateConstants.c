#include "spintronicsIncludes.h"
#include "spintronicsConfig.h"
#include "generateConstants.h"

#define CURRENT_SENSE_RESISTOR_OHMS 10e-3
#define ADC_FULLSCALE_VOLTS 2.8
#define COIL_ADC_BUFFER_GAIN 20.179

#define U2_BUF_GAIN 1.39819005

#define SCALE_DIVIDER 1.07374182e9 //Where does 1.07374182e9 come from?  I forget, but this works

float half_sample_rate;
float twice_sample_period;
float max_output_hz;
_Q15 default_balance_frequency; //units are Q15 half-cycles per sample-period
_Q15 default_balance_amplitude; //units are Q15 fraction of full-scale
float coil_ADC_scale_factor; //units are amperes
float bridge_ADC_scale_factor;//units are volts
float bridge_ADC_scale_factor_by_2;//units are volts
float bridge_ADC_scale_factor_by_4;//units are volts
float bridge_ADC_scale_factor_by_8;//units are volts
float bridge_ADC_scale_factor_by_16;//units are volts

void generateConstants(void)
{
    float coil_ADC_fullscale_amperes;
    float bridge_ADC_fullscale_volts;

    half_sample_rate = SAMPLE_RATE * 0.5;
    twice_sample_period = 1.0 / half_sample_rate;

    max_output_hz = 0.9 * half_sample_rate;
    
    default_balance_frequency = _Q15ftoi(DEFAULT_BALANCE_HZ * twice_sample_period);
    default_balance_amplitude = _Q15ftoi(DEFAULT_BALANCE_VOLTS / FULLSCALE_BRIDGE_DAC_VOLTAGE);

    coil_ADC_fullscale_amperes = ADC_FULLSCALE_VOLTS / CURRENT_SENSE_RESISTOR_OHMS / COIL_ADC_BUFFER_GAIN;
    coil_ADC_scale_factor = coil_ADC_fullscale_amperes / SCALE_DIVIDER;

    bridge_ADC_fullscale_volts = ADC_FULLSCALE_VOLTS / U2_BUF_GAIN; //full-scale voltage AFTER U25; must calculate the gain of U25 based upon digi-pot U24 in order to return results that are scaled properly
    bridge_ADC_scale_factor = bridge_ADC_fullscale_volts / SCALE_DIVIDER;
    bridge_ADC_scale_factor_by_2 = bridge_ADC_scale_factor / 2.0;
    bridge_ADC_scale_factor_by_4 = bridge_ADC_scale_factor / 4.0;
    bridge_ADC_scale_factor_by_8 = bridge_ADC_scale_factor / 8.0;
    bridge_ADC_scale_factor_by_16 = bridge_ADC_scale_factor / 16.0;

}
