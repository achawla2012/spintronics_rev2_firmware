/*
 * generateAndProcessSamples.c
 * 
 * designed and written
 * by Michael Reinhart Sandstedt
 *
 * First release: May 7th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

//TODO: add FIR HPF

#include "p33exxxx.h"
#include "spintronicsIncludes.h"
#include "spintronicsConfig.h"
#include "fsmStates.h"
#include "uartDrv.h"
#include "muxControl.h"
#include "balanceBridge.h"
#include "calculateVectors.h"
#include "commsDefines.h"
#include "utility.h"
#include "spintronicsStructs.h"
#include "FIR.h"

_Q15 readBridgeSampleAndApplyGain(bool* bridgeDigitalClip);
static void signalGenerator(unsigned char runOrReset, angle_array_t *freqT, __eds__ _Q15 *cosOmega1T, __eds__ _Q15 *cosOmega2T, _Q15 local_a2, _Q15 local_f2);
static void measure(_Q15 bridgeSample,
#ifdef MEASURE_F2_AT_COIL
                    _Q15 coilSample,
#endif
                    angle_array_t *freqT, _Q15 cosOmega1TTimeAligned,
#if defined(MEASURE_F2_AT_BRIGE) || defined(MEASURE_F2_AT_COIL)
                    _Q15 cosOmega2TTimeAligned,
#endif
                    accumulator_t *cosAccumulator,
                    accumulator_t *sinAccumulator);

#ifdef TRIG_USES_LUT
extern _Q15 _Q15cosPILUT(_Q15 phiOverPI);
extern _Q15 _Q15sinPILUT(_Q15 phiOverPI);
#endif

void measurementFSM(void)
{
    volatile _Q15 bridgeSample;
    volatile _Q15 coilSample;

    bool balanceBridgeFirst = false;

    static uint8_t sensorAddress;
    static uint32_t timer;
    static uint8_t sensorIndex;
    _Q15 cosOmega1T;//fractions of full-scale
    _Q15 cosOmega2T;//fractions of full-scale
    static angle_array_t freqT;//array contents: 2*f1*t, 2*f2*t, 2*fdiff*t, 2*fsum*t
    static accumulator_t cosAccumulator;
    static accumulator_t sinAccumulator;
    static bool bridgeADCClip;
    static bool coilADCClip;
    static bool bridgeDigitalClip;
    static _Q15 current_a2 = 0;

    //local copies of globals
    static uint8_t local_state = IDLE;
    static _Q15 local_f2;//units are Q15 half-cycles per sample-period
    static _Q15 local_a2;
    static bool local_f1PlusF2OutOfRange;
    static float local_inverseBridgeAnalogGain;

    //START_ATOMIC() called from calling ISR

    local_state = global_state;

    /*
     * clear the global state register so we can tell
     * when another request comes in via UART
     */
    global_state = 0;
    END_ATOMIC();//end critical section

#ifdef SIMULATION_MODE
    uint16_t RXBUF2 = rand();//only to give random stimulus during simulation
#endif


    if (RXBUF0 == 0x7FFF || RXBUF0 == 0x8000)
    {
        bridgeADCClip = true;
    }
    if (RXBUF2 == 0x7FFF || RXBUF2 == 0x8000)
    {
        coilADCClip = true;
    }
    coilSample = RXBUF2;

    switch (local_state)
    {

        case RAMP_DOWN_COIL_QUIT:

            if (current_a2 != 0) {
                --current_a2;
            }
            signalGenerator(RESET_BRIDGE_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
            if (0 == current_a2) {
                local_state = IDLE;
            }
            break;


        case RAMP_DOWN_COIL_RESTART:

            if (current_a2 != 0) {
                --current_a2;
            }
            signalGenerator(RESET_BRIDGE_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
            if (0 == current_a2) {
                local_state = START_MEASUREMENT_FSM;
            }
            break;


        case RAMP_DOWN_COIL_BALANCE_BRIDGE:

            if (current_a2 != 0) {
                --current_a2;
            }
            signalGenerator(RESET_BRIDGE_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
            if (0 == current_a2) {
                local_state = START_BRIDGE_BALANCE_FSM;
            }
            break;


        case START_MEASUREMENT_FSM:

            START_ATOMIC();//begin critical section; must be atomic!
            if (!sensorRBridgeTableValid) {
                balanceBridgeFirst = true;
            }

            /*
             * capture f2/a2 so that coil signal can ramp down when a new START command
             * interrupts the current measurment
             */
            local_f2 = f2;
            local_a2 = a2;

            /*
             * f1PlusF2OutOfRange TBD at time START command is issued;
             * capture now to prevent the value being overwritten by receipt of
             * a new START command during the last cycle with the previous
             * START command's values
             */
            local_f1PlusF2OutOfRange = f1PlusF2OutOfRange;

            /*
             * implementedBridgeGain TBD at time START command is issued;
             * capture now to prevent the value being overwritten by receipt of
             * a new START command during the last cycle with the previous
             * START command's values
             */
            local_inverseBridgeAnalogGain = inverseBridgeAnalogGain;

            END_ATOMIC();//end critical section

            if (balanceBridgeFirst) {

                local_state = START_BRIDGE_BALANCE_FSM | START_MEASUREMENT_AFTER_BALANCE_MASK;

            } else {
                sensorIndex = 0;
                signalGenerator(RESET_SIGNAL_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
                local_state = RAMP_UP_COIL;
                current_a2 = 0;
                setRAmp(u24_code);
            }
            break;


        case RAMP_UP_COIL:

            signalGenerator(RESET_BRIDGE_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
            ++current_a2;
            if (current_a2 == local_a2) {
                local_state = START_NEW_MEASUREMENT_CYCLE;
            }
            break;


        case START_NEW_MEASUREMENT_CYCLE:
        {
            uint16_t r_bridge;

            timer = 0;
            signalGenerator(RESET_BRIDGE_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
            cosAccumulator.bridge_f1 = 0;
#ifdef MEASURE_F2_AT_BRIDGE
            cosAccumulator.bridge_f2 = 0;
#endif
            cosAccumulator.bridge_fdiff = 0;
            cosAccumulator.bridge_fsum = 0;
#ifdef MEASURE_F2_AT_COIL
            cosAccumulator.coil_f2 = 0;
#endif
            sinAccumulator.bridge_f1 = 0;
#ifdef MEASURE_F2_AT_BRIDGE
            sinAccumulator.bridge_f2 = 0;
#endif
            sinAccumulator.bridge_fdiff = 0;
            sinAccumulator.bridge_fsum = 0;
#ifdef MEASURE_F2_AT_COIL
            cosAccumulator.coil_f2 = 0;
#endif
            bridgeADCClip = false;
            coilADCClip = false;
            bridgeDigitalClip = false;

            START_ATOMIC();
            sensorAddress = sensorAddressTable[sensorIndex];
            r_bridge = sensorRBridgeTable[sensorIndex];
            END_ATOMIC();
            
            configSensor(sensorAddress);
            setRBridge(r_bridge);

            local_state = WAIT_FOR_COIL_0RAD;
            break;
        }


        case WAIT_FOR_COIL_0RAD:

            signalGenerator(RESET_BRIDGE_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
            if (0 == freqT.two_f2_t && !waitForVectorCalculation) {
                local_state = START_SIGNAL_GEN;
            }
            break;


        case START_SIGNAL_GEN:

            /* need to start before MEASURE state because of FIR group delay */
            bridgeSample = readBridgeSampleAndApplyGain(&bridgeDigitalClip);
            signalGenerator(RUN_SIGNAL_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
            ++timer;
            if (timer == MEASUREMENT_SETUP_TIME)
            {
                timer = 0;
                local_state = MEASURE;
            }
            break;            


        case MEASURE:

            bridgeSample = readBridgeSampleAndApplyGain(&bridgeDigitalClip);
            measure(bridgeSample,
#ifdef MEASURE_F2_AT_COIL
                    coilSample,
#endif
                    &freqT, cosOmega1T,
#if defined(MEASURE_F2_AT_BRIGE) || defined(MEASURE_F2_AT_COIL)
                    cosOmega2T,
#endif
                    &cosAccumulator, &sinAccumulator);
            signalGenerator(RUN_SIGNAL_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);
            ++timer;
            if (timer == measurementTime)
            {
                timer = 0;
                local_state = CALCULATE_VECTORS;
            }
            break;


        case CALCULATE_VECTORS:

            signalGenerator(RESET_BRIDGE_GEN, &freqT, &cosOmega1T, &cosOmega2T, current_a2, local_f2);

            //capture necessary variables for processing the vector calc thread
            sensorAddressCapture = sensorAddress;
            cosAccumulatorCapture.bridge_f1 = cosAccumulator.bridge_f1;
            sinAccumulatorCapture.bridge_f1 = sinAccumulator.bridge_f1;
            #ifdef MEASURE_F2_AT_BRIDGE
            cosAccumulatorCapture.bridge_f2 = cosAccumulator.bridge_f2;
            sinAccumulatorCapture.bridge_f2 = sinAccumulator.bridge_f2;
            #endif
            cosAccumulatorCapture.bridge_fdiff = cosAccumulator.bridge_fdiff;
            sinAccumulatorCapture.bridge_fdiff = sinAccumulator.bridge_fdiff;
            cosAccumulatorCapture.bridge_fsum = cosAccumulator.bridge_fsum;
            sinAccumulatorCapture.bridge_fsum = sinAccumulator.bridge_fsum;
            #ifdef MEASURE_F2_AT_COIL
            cosAccumulatorCapture.coil_f2 = cosAccumulator.coil_f2;
            sinAccumulatorCapture.coil_f2 = sinAccumulator.coil_f2;
            #endif
            bridgeADCClipCapture = bridgeADCClip;
            coilADCClipCapture = coilADCClip;
            bridgeDigitalClipCapture = bridgeDigitalClip;
            f1PlusF2OutOfRangeCapture = local_f1PlusF2OutOfRange;
            inverseBridgeAnalogGainCapture = local_inverseBridgeAnalogGain;

            //start the timer to spawn the vector calc thread
            waitForVectorCalculation = true;
            PR1 = DELAY_TO_VECTOR_CALC_THREAD;
            T1CONbits.TON = 1;

#ifdef SIMULATION_MODE
            calculateFinalVectors();
#endif

            ++sensorIndex;
            if (sensorIndex >= numberOfSensors) {

                /*
                 * must test >= in case numberOfSensors was updated since
                 * the last iteration of this FSM
                 */
                sensorIndex = 0;
            }
            local_state = START_NEW_MEASUREMENT_CYCLE;
            break;


        default:
            local_state = IDLE;
            break;
    }

    START_ATOMIC();//begin critical section; must be atomic!
    if (!global_state) {
        /*
         * global_state register is clear; no pending requests have arrived over
         * UART so it's safe to write in our new state
         */
        global_state = local_state;

    }
    END_ATOMIC();//end critical section

}

_Q15 readBridgeSampleAndApplyGain(bool* bridgeDigitalClip)
{

#ifdef SIMULATION_MODE
    uint16_t RXBUF0 = rand();//only to give random stimulus during simulation
    uint16_t RXBUF1 = rand();//only to give random stimulus during simulation
#endif
        
    if (bridgeADCGainFactor == 0)
    {
        return FIR(RXBUF0);
    }
    else
    {
        int16_t clipTest;
        int16_t maskForTruncationBitsPlusOne = 0x8000;//make it signed so that the shift uses sign extension
        maskForTruncationBitsPlusOne = maskForTruncationBitsPlusOne >> bridgeADCGainFactor;
        clipTest = RXBUF0 & maskForTruncationBitsPlusOne;
        if (clipTest != 0x0000 && clipTest != maskForTruncationBitsPlusOne)
        {
            *bridgeDigitalClip = true;
            if ((int16_t)RXBUF0 < 0)
            {
                return FIR(0x8000);//interpret the clipped sample as the most negative value
            }
            else
            {
                return FIR(0x7FFF);//interpret the clipped sample as the most positive value
            }
        }
        else
        {
            uint32_t tempSample;
            __eds__ uint16_t *tempSampleLowWord = (__eds__ uint16_t *)&tempSample;
            __eds__ uint16_t *tempSampleHighWord = tempSampleLowWord + 1;
            *tempSampleLowWord = RXBUF1;
            *tempSampleHighWord = RXBUF0;
            *tempSampleLowWord = *tempSampleLowWord >> (16 - bridgeADCGainFactor);
            *tempSampleHighWord = *tempSampleHighWord << bridgeADCGainFactor;
            return FIR(*tempSampleLowWord + *tempSampleHighWord);
        }
    }
}

static inline void
signalGenerator(uint8_t runOrReset, angle_array_t *freqT,
                __eds__ _Q15 *cosOmega1T, __eds__ _Q15 *cosOmega2T,
                _Q15 current_a2, _Q15 local_f2)
{
    if (runOrReset == RUN_SIGNAL_GEN) {

#ifdef TRIG_USES_LUT
        *cosOmega1T = _Q15cosPILUT(freqT->two_f1_t);//generating cos(omega1 * t)
        *cosOmega2T = _Q15cosPILUT(freqT->two_f2_t);//generating cos(omega2 * t)
#else
        *cosOmega1T = _Q15cosPI(freqT->two_f1_t);//generating cos(omega1 * t)
        *cosOmega2T = _Q15cosPI(freqT->two_f2_t);//generating cos(omega2 * t)
#endif

        uint32_t tempSample;
        if (a1 == 0x7FFF) {
            TXBUF0 = *cosOmega1T;
            TXBUF1 = 0x0000;
        }
        else {
            tempSample = asm16X16Mult(*cosOmega1T, a1);
            TXBUF0 = *((__eds__ uint16_t *)&tempSample + 1);
            TXBUF1 = *((__eds__ uint16_t *)&tempSample);
        }

        #ifndef PROBE
        if (current_a2 == 0x7FFF) {
            TXBUF2 = *cosOmega2T;
            TXBUF3 = 0x0000;
        }
        else {
            tempSample = asm16X16Mult(*cosOmega2T, current_a2);
            TXBUF2 = *((__eds__ uint16_t *)&tempSample + 1);
            TXBUF3 = *((__eds__ uint16_t *)&tempSample);
        }

        #elif defined(PROBE_BRIDGE_ADC)
        TXBUF2 = RXBUF0;
        TXBUF3 = RXBUF1;
        #elif defined(PROBE_COIL_ADC)
        TXBUF2 = RXBUF2;
        TXBUF3 = RXBUF3;
        #endif

        freqT->two_f1_t += f1;
        freqT->two_f2_t += local_f2;
        freqT->two_fdiff_t += fdiff;
        freqT->two_fsum_t += fsum;

    } else if (runOrReset == RESET_BRIDGE_GEN) {

        #ifdef TRIG_USES_LUT
        *cosOmega2T = _Q15cosPILUT(freqT->two_f2_t);//generating cos(omega2 * t)
        #else
        *cosOmega2T = _Q15cosPI(freqT->two_f2_t);//generating cos(omega2 * t)
        #endif

        uint32_t tempSample;

        #ifndef PROBE
        if (current_a2 == 0x7FFF) {
            TXBUF2 = *cosOmega2T;
            TXBUF3 = 0x0000;
        } else {
            tempSample = asm16X16Mult(*cosOmega2T, current_a2);
            TXBUF2 = *((__eds__ uint16_t *)&tempSample + 1);
            TXBUF3 = *((__eds__ uint16_t *)&tempSample);
        }

        #elif defined(PROBE_BRIDGE_ADC)
        TXBUF2 = RXBUF0;
        TXBUF3 = RXBUF1;
        #elif defined(PROBE_COIL_ADC)
        TXBUF2 = RXBUF2;
        TXBUF3 = RXBUF3;
        #endif

        freqT->two_f2_t += local_f2;

        *cosOmega1T = 0;
        TXBUF0 = 0x0000;
        TXBUF1 = 0x0000;

        freqT->two_f1_t = 0x0000;
        freqT->two_fdiff_t = 0x0000;
        freqT->two_fsum_t = 0x0000;

    } else {

        *cosOmega1T = 0;
        *cosOmega2T = 0;
        TXBUF0 = 0x0000;
        TXBUF1 = 0x0000;
        TXBUF2 = 0x0000;
        TXBUF3 = 0x0000;
        
        freqT->two_f1_t = 0x0000;
        freqT->two_f2_t = 0x0000;
        freqT->two_fdiff_t = 0x0000;
        freqT->two_fsum_t = 0x0000;
    }
}

void measure(_Q15 bridgeSample,
#ifdef MEASURE_F2_AT_COIL
             _Q15 coilSample,
#endif
              angle_array_t *freqT, _Q15 cosOmega1T,
#if defined(MEASURE_F2_AT_BRIGE) || defined(MEASURE_F2_AT_COIL)
              _Q15 cosOmega2T,
#endif
              accumulator_t *cosAccumulator,
              accumulator_t *sinAccumulator)
{
    int32_t bridgeByCosF1T, bridgeByCosFDiffT, bridgeByCosFSumT, bridgeBySinF1T, bridgeBySinFDiffT, bridgeBySinFSumT;
#ifdef MEASURE_F2_AT_BRIDGE
    int32_t bridgeByCosF2T, bridgeBySinF2T;
#endif
#ifdef MEASURE_F2_AT_COIL
     int32_t coilBySinF2T, coilByCosF2T;
#endif
#ifdef TRIG_USES_LUT
    _Q15 cosFDiffT = _Q15cosPILUT(freqT->two_fdiff_t);
    _Q15 cosFSumT = _Q15cosPILUT(freqT->two_fsum_t);
    _Q15 sinF1T = _Q15sinPILUT(freqT->two_f1_t);
#if defined(MEASURE_F2_AT_BRIGE) || defined(MEASURE_F2_AT_COIL)
    _Q15 sinF2T = _Q15sinPILUT(freqT->two_f2_t);
#endif
    _Q15 sinFDiffT = _Q15sinPILUT(freqT->two_fdiff_t);
    _Q15 sinFSumT = _Q15sinPILUT(freqT->two_fsum_t);
#else
    _Q15 cosFDiffT = _Q15cosPI(freqT->two_fdiff_t);
    _Q15 cosFSumT = _Q15cosPI(freqT->two_fsum_t);
    _Q15 sinF1T = _Q15sinPI(freqT->two_f1_t);
#if defined(MEASURE_F2_AT_BRIGE) || defined(MEASURE_F2_AT_COIL)
    _Q15 sinF2T = _Q15sinPI(freqT->two_f2_t);
#endif
    _Q15 sinFDiffT = _Q15sinPI(freqT->two_fdiff_t);
    _Q15 sinFSumT = _Q15sinPI(freqT->two_fsum_t);
#endif
    bridgeByCosF1T = asm16X16Mult(bridgeSample, cosOmega1T);
    cosAccumulator->bridge_f1 += bridgeByCosF1T;
#ifdef MEASURE_F2_AT_BRIDGE
    bridgeByCosF2T = asm16X16Mult(bridgeSample, cosOmega2T);
    cosAccumulator->bridge_f2 += bridgeByCosF2T;
#endif
    bridgeByCosFDiffT = asm16X16Mult(bridgeSample, cosFDiffT);
    cosAccumulator->bridge_fdiff += bridgeByCosFDiffT;
    bridgeByCosFSumT =  asm16X16Mult(bridgeSample, cosFSumT);
    cosAccumulator->bridge_fsum += bridgeByCosFSumT;
#ifdef MEASURE_F2_AT_COIL
    coilByCosF2T = asm16X16Mult(coilSample, cosOmega2T);
    cosAccumulator->coil_f2 += coilByCosF2T;
#endif
    bridgeBySinF1T = asm16X16Mult(bridgeSample, sinF1T);
    sinAccumulator->bridge_f1 += bridgeBySinF1T;
#ifdef MEASURE_F2_AT_BRIDGE
    bridgeBySinF2T = asm16X16Mult(bridgeSample, sinF2T);
    sinAccumulator->bridge_f2 += bridgeBySinF2T;
#endif
    bridgeBySinFDiffT = asm16X16Mult(bridgeSample, sinFDiffT);
    sinAccumulator->bridge_fdiff += bridgeBySinFDiffT;
    bridgeBySinFSumT = asm16X16Mult(bridgeSample, sinFSumT);
    sinAccumulator->bridge_fsum += bridgeBySinFSumT;
#ifdef MEASURE_F2_AT_COIL
    coilBySinF2T = asm16X16Mult(coilSample, sinF2T);
    sinAccumulator->coil_f2 += coilBySinF2T;
#endif

#if defined(PROBE) && defined(PROBE_BRIDGE_X_COS_F1_T)
    TXBUF2 = *((uint16_t *)&bridgeByCosF1T + 1);
    TXBUF3 = *((uint16_t *)&bridgeByCosF1T);
#elif defined(PROBE) && defined(PROBE_BRIDGE_X_COS_F2_T) && defined(MEASURE_F2_AT_BRIDGE)
    TXBUF2 = 0x00;//*((uint16_t *)&bridgeByCosF2T + 1);
    TXBUF3 = 0x00;//*((uint16_t *)&bridgeByCosF2T);
#elif defined(PROBE) && defined(PROBE_BRIDGE_X_COS_FDIFF_T)
    TXBUF2 = *((uint16_t *)&bridgeByCosFDiffT + 1);
    TXBUF3 = *((uint16_t *)&bridgeByCosFDiffT);
#elif defined(PROBE) && defined(PROBE_BRIDGE_X_COS_FSUM_T)
    TXBUF2 = *((uint16_t *)&bridgeByCosFSumT + 1);
    TXBUF3 = *((uint16_t *)&bridgeByCosFSumT);
#elif defined(PROBE) && defined(PROBE_COIL_X_COS_F2_T)
    TXBUF2 = *((uint16_t *)&coilByCosF2T + 1);
    TXBUF3 = *((uint16_t *)&coilByCosF2T);
#elif defined(PROBE) && defined(PROBE_BRIDGE_X_SIN_F1_T)
    TXBUF2 = *((uint16_t *)&bridgeBySinF1T + 1);
    TXBUF3 = *((uint16_t *)&bridgeBySinF1T);
#elif defined(PROBE) && defined(PROBE_BRIDGE_X_SIN_F2_T) && defined(MEASURE_F2_AT_BRIDGE)
    TXBUF2 = *((uint16_t *)&bridgeBySinF2T + 1);
    TXBUF3 = *((uint16_t *)&bridgeBySinF2T);
#elif defined(PROBE) && defined(PROBE_BRIDGE_X_SIN_FDIFF_T)
    TXBUF2 = *((uint16_t *)&bridgeBySinFDiffT + 1);
    TXBUF3 = *((uint16_t *)&bridgeBySinFDiffT);
#elif defined(PROBE) && defined(PROBE_BRIDGE_X_SIN_FSUM_T)
    TXBUF2 = *((uint16_t *)&bridgeBySinFSumT + 1);
    TXBUF3 = *((uint16_t *)&bridgeBySinFSumT);
#elif defined(PROBE) && defined(PROBE_COIL_X_SIN_F2_T)
    TXBUF2 = *((uint16_t *)&coilBySinF2T + 1);
    TXBUF3 = *((uint16_t *)&coilBySinF2T);
#endif
}
