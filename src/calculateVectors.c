/*
 * calculateVectors.c
 *
 * Author: Michael Reinhart Sandstedt
 *
 * First release: September 14th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "spintronicsIncludes.h"
#include "spintronicsConfig.h"
#include "constants.h"
#include "uartDrv.h"
#include "spintronicsStructs.h"
#include "calculateVectors.h"

#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

//global variables
uint8_t sensorAddressCapture;
accumulator_t cosAccumulatorCapture;
accumulator_t sinAccumulatorCapture;
bool bridgeADCClipCapture;
bool coilADCClipCapture;
bool bridgeDigitalClipCapture;
bool f1PlusF2OutOfRangeCapture;
float inverseBridgeAnalogGainCapture;

void calculateFinalVectors(void)
{
    uint8_t i;
    double_array_t cosAccumulatorFloat;
    double_array_t sinAccumulatorFloat;
    double_array_t phaseAngle;
    float_array_t amplitude;
    float_array_t phaseAngle32;

    uint8_t localSensorAddress;
    bool localBridgeADCClip;
    bool localCoilADCClip;
    bool localBridgeDigitalClip;
    bool localF1PlusF2OutOfRange;

    //copy in global variables
    START_ATOMIC();//begin critical section; must be atomic!
    localSensorAddress = sensorAddressCapture;
    cosAccumulatorFloat.bridge_f1 = (double)(cosAccumulatorCapture.bridge_f1);
    sinAccumulatorFloat.bridge_f1 = (double)(sinAccumulatorCapture.bridge_f1);
#ifdef MEASURE_F2_AT_BRIDGE
    cosAccumulatorFloat.bridge_f2 = (double)(cosAccumulatorCapture.bridge_f2);
    sinAccumulatorFloat.bridge_f2 = (double)(sinAccumulatorCapture.bridge_f2);
#endif
    cosAccumulatorFloat.bridge_fdiff = (double)(cosAccumulatorCapture.bridge_fdiff);
    sinAccumulatorFloat.bridge_fdiff = (double)(sinAccumulatorCapture.bridge_fdiff);
    cosAccumulatorFloat.bridge_fsum = (double)(cosAccumulatorCapture.bridge_fsum);
    sinAccumulatorFloat.bridge_fsum = (double)(sinAccumulatorCapture.bridge_fsum);
#ifdef MEASURE_F2_AT_COIL
    cosAccumulatorFloat.coil_f2 = (double)(cosAccumulatorCapture.coil_f2);
    sinAccumulatorFloat.coil_f2 = (double)(sinAccumulatorCapture.coil_f2);
#endif
    localBridgeADCClip = bridgeADCClipCapture;
    localCoilADCClip = coilADCClipCapture;
    localBridgeDigitalClip = bridgeDigitalClipCapture;
    localF1PlusF2OutOfRange = f1PlusF2OutOfRangeCapture;
    END_ATOMIC();//end critical section

    phaseAngle.bridge_f1 = atan2(sinAccumulatorFloat.bridge_f1, cosAccumulatorFloat.bridge_f1);
    if (fabs(cosAccumulatorFloat.bridge_f1) > fabs(sinAccumulatorFloat.bridge_f1)) {
        amplitude.bridge_f1 = cosAccumulatorFloat.bridge_f1 / (cos(phaseAngle.bridge_f1) * measurementTime);
    } else {
        amplitude.bridge_f1 = sinAccumulatorFloat.bridge_f1 / (sin(phaseAngle.bridge_f1) * measurementTime);
    }
    if(amplitude.bridge_f1 < 0) {//make sure amplitude is positive
        amplitude.bridge_f1 = fabsf(amplitude.bridge_f1);
        if (phaseAngle.bridge_f1 < 0) {
            phaseAngle.bridge_f1 = phaseAngle.bridge_f1 + M_PI_2;
        } else {
            phaseAngle.bridge_f1 = phaseAngle.bridge_f1 - M_PI_2;
        }
    }

#ifdef MEASURE_F2_AT_BRIDGE
    phaseAngle.bridge_f2 = atan2(sinAccumulatorFloat.bridge_f2, cosAccumulatorFloat.bridge_f2);
    if (fabs(cosAccumulatorFloat.bridge_f2) > fabs(sinAccumulatorFloat.bridge_f2)) {
        amplitude.bridge_f2 = cosAccumulatorFloat.bridge_f2 / (cos(phaseAngle.bridge_f2) * measurementTime);
    } else {
        amplitude.bridge_f2 = sinAccumulatorFloat.bridge_f2 / (sin(phaseAngle.bridge_f2) * measurementTime);
    }
    if(amplitude.bridge_f2 < 0) {
        amplitude.bridge_f2 = fabsf(amplitude.bridge_f2);
        if (phaseAngle.bridge_f2 < 0) {
            phaseAngle.bridge_f2 = phaseAngle.bridge_f2 + M_PI_2;
        } else {
            phaseAngle.bridge_f2 = phaseAngle.bridge_f2 - M_PI_2;
        }
    }
#endif

    phaseAngle.bridge_fdiff = atan2(sinAccumulatorFloat.bridge_fdiff, cosAccumulatorFloat.bridge_fdiff);
    if (fabs(cosAccumulatorFloat.bridge_fdiff) > fabs(sinAccumulatorFloat.bridge_fdiff)) {
        amplitude.bridge_fdiff = cosAccumulatorFloat.bridge_fdiff / (cos(phaseAngle.bridge_fdiff) * measurementTime);
    } else {
        amplitude.bridge_fdiff = sinAccumulatorFloat.bridge_fdiff / (sin(phaseAngle.bridge_fdiff) * measurementTime);
    }
    if(amplitude.bridge_fdiff < 0) {//make sure amplitude is positive
        amplitude.bridge_fdiff = fabsf(amplitude.bridge_fdiff);
        if (phaseAngle.bridge_fdiff < 0) {
            phaseAngle.bridge_fdiff = phaseAngle.bridge_fdiff + M_PI_2;
        } else {
            phaseAngle.bridge_fdiff = phaseAngle.bridge_fdiff - M_PI_2;
        }
    }

    phaseAngle.bridge_fsum = atan2(sinAccumulatorFloat.bridge_fsum, cosAccumulatorFloat.bridge_fsum);
    if (fabs(cosAccumulatorFloat.bridge_fsum) > fabs(sinAccumulatorFloat.bridge_fsum)) {
        amplitude.bridge_fsum = cosAccumulatorFloat.bridge_fsum / (cos(phaseAngle.bridge_fsum) * measurementTime);
    } else {
        amplitude.bridge_fsum = sinAccumulatorFloat.bridge_fsum / (sin(phaseAngle.bridge_fsum) * measurementTime);
    }
    if(amplitude.bridge_fsum < 0) {//make sure amplitude is positive
        amplitude.bridge_fsum = fabsf(amplitude.bridge_fsum);
        if (phaseAngle.bridge_fsum < 0) {
            phaseAngle.bridge_fsum = phaseAngle.bridge_fsum + M_PI_2;
        } else {
            phaseAngle.bridge_fsum = phaseAngle.bridge_fsum - M_PI_2;
        }
    }
#ifdef MEASURE_F2_AT_COIL
    phaseAngle.coil_f2 = atan2(sinAccumulatorFloat.coil_f2, cosAccumulatorFloat.coil_f2);
    if (fabs(cosAccumulatorFloat.coil_f2) > fabs(sinAccumulatorFloat.coil_f2)) {
        amplitude.coil_f2 = cosAccumulatorFloat.coil_f2 / (cos(phaseAngle.coil_f2) * measurementTime);
    } else {
        amplitude.coil_f2 = sinAccumulatorFloat.coil_f2 / (sin(phaseAngle.coil_f2) * measurementTime);
    }
    if(amplitude.coil_f2 < 0) {//make sure amplitude is positive
        amplitude.coil_f2 = fabsf(amplitude.coil_f2);
        if (phaseAngle.coil_f2 < 0) {
            phaseAngle.coil_f2 = phaseAngle.coil_f2 + M_PI_2;
        } else {
            phaseAngle.coil_f2 = phaseAngle.coil_f2 - M_PI_2;
        }
    }
#endif

    switch(bridgeADCGainFactor)//if digital gain was added, need to scale the reported voltage appropriately
    {
        case 0:

            amplitude.bridge_f1 = amplitude.bridge_f1 * BRIDGE_ADC_SCALE_FACTOR * inverseBridgeAnalogGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude.bridge_f2 = amplitude.bridge_f2 * BRIDGE_ADC_SCALE_FACTOR * inverseBridgeAnalogGainCapture;
            #endif

            amplitude.bridge_fdiff = amplitude.bridge_fdiff * BRIDGE_ADC_SCALE_FACTOR * inverseBridgeAnalogGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude.bridge_fsum = 0;
            }
            else
            {
                amplitude.bridge_fsum = amplitude.bridge_fsum * BRIDGE_ADC_SCALE_FACTOR * inverseBridgeAnalogGainCapture;
            }
            break;


        case 1:

            amplitude.bridge_f1 = amplitude.bridge_f1 * BRIDGE_ADC_SCALE_FACTOR_BY_2 * inverseBridgeAnalogGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude.bridge_f2 = amplitude.bridge_f2 * BRIDGE_ADC_SCALE_FACTOR_BY_2 * inverseBridgeAnalogGainCapture;
            #endif

            amplitude.bridge_fdiff = amplitude.bridge_fdiff * BRIDGE_ADC_SCALE_FACTOR_BY_2 * inverseBridgeAnalogGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude.bridge_fsum = 0;
            }
            else
            {
                amplitude.bridge_fsum = amplitude.bridge_fsum * BRIDGE_ADC_SCALE_FACTOR_BY_2 * inverseBridgeAnalogGainCapture;
            }
            break;


        case 2:

            amplitude.bridge_f1 = amplitude.bridge_f1 * BRIDGE_ADC_SCALE_FACTOR_BY_4 * inverseBridgeAnalogGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude.bridge_f2 = amplitude.bridge_f2 * BRIDGE_ADC_SCALE_FACTOR_BY_4 * inverseBridgeAnalogGainCapture;
            #endif

            amplitude.bridge_fdiff = amplitude.bridge_fdiff * BRIDGE_ADC_SCALE_FACTOR_BY_4 * inverseBridgeAnalogGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude.bridge_fsum = 0;
            }
            else
            {
                amplitude.bridge_fsum = amplitude.bridge_fsum * BRIDGE_ADC_SCALE_FACTOR_BY_4 * inverseBridgeAnalogGainCapture;
            }
            break;


        case 3:

            amplitude.bridge_f1 = amplitude.bridge_f1 * BRIDGE_ADC_SCALE_FACTOR_BY_8 * inverseBridgeAnalogGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude.bridge_f2 = amplitude.bridge_f2 * BRIDGE_ADC_SCALE_FACTOR_BY_8 * inverseBridgeAnalogGainCapture;
            #endif

            amplitude.bridge_fdiff = amplitude.bridge_fdiff * BRIDGE_ADC_SCALE_FACTOR_BY_8 * inverseBridgeAnalogGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude.bridge_fsum = 0;
            }
            else
            {
                amplitude.bridge_fsum = amplitude.bridge_fsum * BRIDGE_ADC_SCALE_FACTOR_BY_8 * inverseBridgeAnalogGainCapture;
            }
            break;


        case 4:

            amplitude.bridge_f1 = amplitude.bridge_f1 * BRIDGE_ADC_SCALE_FACTOR_BY_16 * inverseBridgeAnalogGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude.bridge_f2 = amplitude.bridge_f2 * BRIDGE_ADC_SCALE_FACTOR_BY_16 * inverseBridgeAnalogGainCapture;
            #endif

            amplitude.bridge_fdiff = amplitude.bridge_fdiff * BRIDGE_ADC_SCALE_FACTOR_BY_16 * inverseBridgeAnalogGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude.bridge_fsum = 0;
            }
            else
            {
                amplitude.bridge_fsum = amplitude.bridge_fsum * BRIDGE_ADC_SCALE_FACTOR_BY_16 * inverseBridgeAnalogGainCapture;
            }
            break;


        default:

            amplitude.bridge_f1 = amplitude.bridge_f1 * BRIDGE_ADC_SCALE_FACTOR * inverseBridgeAnalogGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude.bridge_f2 = amplitude.bridge_f2 * BRIDGE_ADC_SCALE_FACTOR * inverseBridgeAnalogGainCapture;
            #endif

            amplitude.bridge_fdiff = amplitude.bridge_fdiff * BRIDGE_ADC_SCALE_FACTOR * inverseBridgeAnalogGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude.bridge_fsum = 0;
            }
            else
            {
                amplitude.bridge_fsum = amplitude.bridge_fsum * BRIDGE_ADC_SCALE_FACTOR * inverseBridgeAnalogGainCapture;
            }
            break;

    }

#ifdef MESSURE_F2_AT_COIL
    amplitude.coil_f2 = amplitude.coil_f2 * COIL_ADC_SCALE_FACTOR;
#endif

    phaseAngle32.bridge_f1 = phaseAngle.bridge_f1;
#ifdef MEASURE_F2_AT_BRIDGE
    phaseAngle32.bridge_f2 = phaseAngle.bridge_f2;
#endif
    phaseAngle32.bridge_fdiff = phaseAngle.bridge_fdiff;
    phaseAngle32.bridge_fsum = phaseAngle.bridge_fsum;
#ifdef MEASURE_F2_AT_COIL
    phaseAngle32.coil_f2 = phaseAngle.coil_f2;
#endif

    transmitResults(localSensorAddress, &phaseAngle32, &amplitude,
                    localBridgeADCClip, localCoilADCClip,
                    localBridgeDigitalClip);

}
