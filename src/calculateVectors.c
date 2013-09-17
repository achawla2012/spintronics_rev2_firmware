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
#include "uartDrv.h"
#include "generateConstants.h"

#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

//file-scope global variables
static uint8_t sensorAddressCapture;
static int64_t cosAccumulatorCapture[5];
static int64_t sinAccumulatorCapture[5];
static bool bridgeADCClipCapture;
static bool coilADCClipCapture;
static bool bridgeDigitalClipCapture;
static bool f1PlusF2OutOfRangeCapture;

void spawnVectorCalcThread(uint16_t delayCycles, uint8_t sensorIndex, uint64_t *cosAccumulator, uint64_t *sinAccumulator, bool bridgeADCClip, bool coilADCClip, bool bridgeDigitalClip, bool f1PlusF2OutOfRange)
{
    uint8_t i;

    START_ATOMIC();//begin critical section; must be atomic!
    //capture static variables from measurementFSM so it can continue
    sensorAddressCapture = sensorAddressTable[sensorIndex];
    for (i = 0; i < 5; ++i) {
        cosAccumulatorCapture[i] = cosAccumulator[i];
        sinAccumulatorCapture[i] = sinAccumulator[i];
    }
    bridgeADCClipCapture = bridgeADCClip;
    coilADCClipCapture = coilADCClip;
    bridgeDigitalClipCapture = bridgeDigitalClip;
    f1PlusF2OutOfRangeCapture = f1PlusF2OutOfRange;
    END_ATOMIC();//end critical section

    //start the timer
    PR1 = delayCycles;
    T1CONbits.TON = 1;

}

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

            amplitude[0] = amplitude[0] * bridge_ADC_scale_factor;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * bridge_ADC_scale_factor;
            #endif

            amplitude[2] = amplitude[2] * bridge_ADC_scale_factor;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * bridge_ADC_scale_factor;
            }
            break;


        case 1:

            amplitude[0] = amplitude[0] * bridge_ADC_scale_factor_by_2;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * bridge_ADC_scale_factor_by_2;
            #endif

            amplitude[2] = amplitude[2] * bridge_ADC_scale_factor_by_2;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * bridge_ADC_scale_factor_by_2;
            }
            break;


        case 2:

            amplitude[0] = amplitude[0] * bridge_ADC_scale_factor_by_4;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * bridge_ADC_scale_factor_by_4;
            #endif

            amplitude[2] = amplitude[2] * bridge_ADC_scale_factor_by_4;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * bridge_ADC_scale_factor_by_4;
            }
            break;


        case 3:

            amplitude[0] = amplitude[0] * bridge_ADC_scale_factor_by_8;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * bridge_ADC_scale_factor_by_8;
            #endif

            amplitude[2] = amplitude[2] * bridge_ADC_scale_factor_by_8;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * bridge_ADC_scale_factor_by_8;
            }
            break;


        case 4:

            amplitude[0] = amplitude[0] * bridge_ADC_scale_factor_by_16;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * bridge_ADC_scale_factor_by_16;
            #endif

            amplitude[2] = amplitude[2] * bridge_ADC_scale_factor_by_16;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * bridge_ADC_scale_factor_by_16;
            }
            break;


        default:

            amplitude[0] = amplitude[0] * bridge_ADC_scale_factor;

            #ifdef MEASURE_F2_AT_BRIDGE
            amplitude[1] = amplitude[1] * bridge_ADC_scale_factor;
            #endif

            amplitude[2] = amplitude[2] * bridge_ADC_scale_factor;
            if (localF1PlusF2OutOfRange)
            {
                amplitude[3] = 0;
            }
            else
            {
                amplitude[3] = amplitude[3] * bridge_ADC_scale_factor;
            }
            break;

    }

    amplitude[4] = amplitude[4] * coil_ADC_scale_factor;

    for (i = 0; i < 5; ++ i) {
        phaseAngle32[i] = phaseAngle[i];
    }

    transmitResults(localSensorAddress, phaseAngle32, amplitude, localBridgeADCClip, localCoilADCClip, localBridgeDigitalClip);

}
