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

#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

//global variables
uint8_t sensorAddressCapture;
int64_t cosAccumulatorCapture[5];
int64_t sinAccumulatorCapture[5];
bool bridgeADCClipCapture;
bool coilADCClipCapture;
bool bridgeDigitalClipCapture;
bool f1PlusF2OutOfRangeCapture;
float implementedBridgeGainCapture;

void calculateFinalVectors(void)
{
    uint8_t i;
    double cosAccumulatorFloat[5];
    double sinAccumulatorFloat[5];
    double phaseAngle[5];
    float amplitude[5];
    float phaseAngle32[5];

    uint8_t localSensorAddress;
    bool localBridgeADCClip;
    bool localCoilADCClip;
    bool localBridgeDigitalClip;
    bool localF1PlusF2OutOfRange;

    //copy in global variables
    START_ATOMIC();//begin critical section; must be atomic!
    localSensorAddress = sensorAddressCapture;
    for (i = 0; i < 5; ++i) {
        #ifndef MEASURE_F2_AT_BRIDGE
        //do not calculate this vector
        if (1 == i) {
            continue;
        }
        #endif
        cosAccumulatorFloat[i] = (double)cosAccumulatorCapture[i];
        sinAccumulatorFloat[i] = (double)sinAccumulatorCapture[i];
    }
    localBridgeADCClip = bridgeADCClipCapture;
    localCoilADCClip = coilADCClipCapture;
    localBridgeDigitalClip = bridgeDigitalClipCapture;
    localF1PlusF2OutOfRange = f1PlusF2OutOfRangeCapture;
    END_ATOMIC();//end critical section

    for (i = 0; i < 5; ++i) {

        #ifndef MEASURE_F2_AT_BRIDGE
        //do not calculate this vector
        if (1 == i) {
            phaseAngle[1] = 0;
            amplitude[1] = 0;
            continue;
        }
        #endif

        phaseAngle[i] = atan2(sinAccumulatorFloat[i], cosAccumulatorFloat[i]);
        if (fabs(cosAccumulatorFloat[i]) > fabs(sinAccumulatorFloat[i])) {
            amplitude[i] = cosAccumulatorFloat[i] / (cos(phaseAngle[i]) * measurementTime);
        } else {
            amplitude[i] = sinAccumulatorFloat[i] / (sin(phaseAngle[i]) * measurementTime);
        }
        if(amplitude[i] < 0) {//make sure amplitude is positive
            amplitude[i] = fabsf(amplitude[i]);
            if (phaseAngle[i] < 0) {
                phaseAngle[i] = phaseAngle[i] + M_PI_2;
            } else {
                phaseAngle[i] = phaseAngle[i] - M_PI_2; }
        }
    }

    switch(bridgeADCGainFactor)//if digital gain was added, need to scale the reported voltage appropriately
    {
        case 0:

            amplitude[0] = amplitude[0] * BRIDGE_ADC_SCALE_FACTOR * implementedBridgeGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * BRIDGE_ADC_SCALE_FACTOR * implementedBridgeGainCapture;
            #endif

            amplitude[2] = amplitude[2] * BRIDGE_ADC_SCALE_FACTOR * implementedBridgeGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * BRIDGE_ADC_SCALE_FACTOR * implementedBridgeGainCapture;
            }
            break;


        case 1:

            amplitude[0] = amplitude[0] * BRIDGE_ADC_SCALE_FACTOR_BY_2 * implementedBridgeGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * BRIDGE_ADC_SCALE_FACTOR_BY_2 * implementedBridgeGainCapture;
            #endif

            amplitude[2] = amplitude[2] * BRIDGE_ADC_SCALE_FACTOR_BY_2 * implementedBridgeGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * BRIDGE_ADC_SCALE_FACTOR_BY_2 * implementedBridgeGainCapture;
            }
            break;


        case 2:

            amplitude[0] = amplitude[0] * BRIDGE_ADC_SCALE_FACTOR_BY_4 * implementedBridgeGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * BRIDGE_ADC_SCALE_FACTOR_BY_4 * implementedBridgeGainCapture;
            #endif

            amplitude[2] = amplitude[2] * BRIDGE_ADC_SCALE_FACTOR_BY_4 * implementedBridgeGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * BRIDGE_ADC_SCALE_FACTOR_BY_4 * implementedBridgeGainCapture;
            }
            break;


        case 3:

            amplitude[0] = amplitude[0] * BRIDGE_ADC_SCALE_FACTOR_BY_8 * implementedBridgeGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * BRIDGE_ADC_SCALE_FACTOR_BY_8 * implementedBridgeGainCapture;
            #endif

            amplitude[2] = amplitude[2] * BRIDGE_ADC_SCALE_FACTOR_BY_8 * implementedBridgeGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * BRIDGE_ADC_SCALE_FACTOR_BY_8 * implementedBridgeGainCapture;
            }
            break;


        case 4:

            amplitude[0] = amplitude[0] * BRIDGE_ADC_SCALE_FACTOR_BY_16 * implementedBridgeGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * BRIDGE_ADC_SCALE_FACTOR_BY_16 * implementedBridgeGainCapture;
            #endif

            amplitude[2] = amplitude[2] * BRIDGE_ADC_SCALE_FACTOR_BY_16 * implementedBridgeGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * BRIDGE_ADC_SCALE_FACTOR_BY_16 * implementedBridgeGainCapture;
            }
            break;


        default:

            amplitude[0] = amplitude[0] * BRIDGE_ADC_SCALE_FACTOR * implementedBridgeGainCapture;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * BRIDGE_ADC_SCALE_FACTOR * implementedBridgeGainCapture;
            #endif

            amplitude[2] = amplitude[2] * BRIDGE_ADC_SCALE_FACTOR * implementedBridgeGainCapture;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * BRIDGE_ADC_SCALE_FACTOR * implementedBridgeGainCapture;
            }
            break;

    }

    amplitude[4] = amplitude[4] * COIL_ADC_SCALE_FACTOR;

    for (i = 0; i < 5; ++ i) {
        phaseAngle32[i] = phaseAngle[i];
    }

    transmitResults(localSensorAddress, phaseAngle32, amplitude, localBridgeADCClip, localCoilADCClip, localBridgeDigitalClip);

}
