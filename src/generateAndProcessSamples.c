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

#include "p33exxxx.h"
#include "spintronics.h"
#include "uartDrv.h"
#include "muxControl.h"
#include "balanceBridge.h"

#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

_Q15 readBridgeSampleAndApplyGain(bool* bridgeDigitalClipFlag);
static void signalGenerator(unsigned char runOrReset, _Q15 *freqT, __eds__ _Q15 *cosOmega1T, __eds__ _Q15 *cosOmega2T);
static void shiftRegister( _Q15 cosOmega1T, _Q15 cosOmega2T, _Q15 *cosOmega1TTimeAligned, _Q15 *cosOmega2TTimeAligned);
static void measure(_Q15 bridgeSample, _Q15 coilSample, _Q15 *freqT, _Q15 cosOmega1TTimeAligned, _Q15 cosOmega2TTimeAligned, int64_t *cosAccumulator, int64_t *sinAccumulator);

#ifdef TRIG_USES_LUT
extern _Q15 _Q15cosPILUT(_Q15 phiOverPI);
extern _Q15 _Q15sinPILUT(_Q15 phiOverPI);
#endif

void measurementFSM(void)
{
    static uint8_t sensorAddress;
    volatile _Q15 bridgeSample;
    volatile _Q15 coilSample;
    static uint8_t local_state;
    static uint32_t timer;
    static uint8_t sensorIndex;
    _Q15 cosOmega1T;//fractions of full-scale //aligned to compensate for ADCDAC_GROUP_DELAY
    _Q15 cosOmega2T;//fractions of full-scale //aligned to compensate for ADCDAC_GROUP_DELAY
    static _Q15 cosOmega1TTimeAligned;//fractions of full-scale
    static _Q15 cosOmega2TTimeAligned;//fractions of full-scale
    static _Q15 freqT[6];//array contents: 2*f1*t, 2*f2*t, 2*f1*(t + ADCDAC_GROUP_DELAY), 2*f2*(t + ADCDAC_GROUP_DELAY), 2*fdiff*(t + ADCDAC_GROUP_DELAY), 2*fsum*(t + ADCDAC_GROUP_DELAY)
    static int64_t cosAccumulator[5];
    static int64_t sinAccumulator[5];
    static bool bridgeADCClipFlag;
    static bool coilADCClipFlag;
    static bool bridgeDigitalClipFlag;

    local_state = global_state;
    END_ATOMIC();

#ifdef SIMULATION_MODE
    uint16_t RXBUF2 = rand();//only to give random stimulus during simulation
#endif


    if (RXBUF0 == 0x7FFF || RXBUF0 == 0x8000)
    {
        bridgeADCClipFlag = true;
    }
    if (RXBUF2 == 0x7FFF || RXBUF2 == 0x8000)
    {
        coilADCClipFlag = true;
    }
    bridgeSample = readBridgeSampleAndApplyGain(&bridgeDigitalClipFlag);
    coilSample = RXBUF2;

    switch (local_state)
    {

        case START_MEASUREMENT_FSM:

            sensorIndex = 0;
            signalGenerator(RESET_SIGNAL_GEN, freqT, &cosOmega1T, &cosOmega2T);
            local_state = START_NEW_MEASUREMENT_CYCLE;
            break;

        case START_NEW_MEASUREMENT_CYCLE:
        {
            uint16_t r_bridge;

            timer = 0;
            signalGenerator(RESET_SIGNAL_GEN, freqT, &cosOmega1T, &cosOmega2T);
            shiftRegister(cosOmega1T, cosOmega2T, &cosOmega1TTimeAligned, &cosOmega2TTimeAligned);
            cosAccumulator[0] = 0; cosAccumulator[1] = 0; cosAccumulator[2] = 0; cosAccumulator[3] = 0; cosAccumulator[4] = 0;
            sinAccumulator[0] = 0; sinAccumulator[1] = 0; sinAccumulator[2] = 0; sinAccumulator[3] = 0; sinAccumulator[4] = 0;
            bridgeADCClipFlag = false;
            coilADCClipFlag = false;
            bridgeDigitalClipFlag = false;

            START_ATOMIC();
            sensorAddress = sensorAddressTable[sensorIndex];
            r_bridge = sensorRBridgeTable[sensorIndex];
            END_ATOMIC();
            
            configSensor(sensorAddress);
            setRBridge(r_bridge);
            
        }
        case START_SIGNAL_GEN:

            signalGenerator(RUN_SIGNAL_GEN, freqT, &cosOmega1T, &cosOmega2T);
            shiftRegister(cosOmega1T, cosOmega2T, &cosOmega1TTimeAligned, &cosOmega2TTimeAligned);
            ++timer;
            if (timer == MEASUREMENT_SETUP_TIME)
            {
                timer = 0;
                local_state = MEASURE;
            }
            break;            

        case MEASURE:
            measure(bridgeSample, coilSample, freqT, cosOmega1TTimeAligned, cosOmega2TTimeAligned, cosAccumulator, sinAccumulator);
            signalGenerator(RUN_SIGNAL_GEN, freqT, &cosOmega1T, &cosOmega2T);
            shiftRegister(cosOmega1T, cosOmega2T, &cosOmega1TTimeAligned, &cosOmega2TTimeAligned);
            ++timer;
            if (timer == measurementTime)
            {
                timer = 0;
                local_state = CALCULATE_VECTORS;
            }
            break;

        case CALCULATE_VECTORS:
        {
            uint8_t i;
            double cosAccumulatorFloat[5];
            double sinAccumulatorFloat[5];
            double phaseAngle[5];
            float amplitude[5];

            IEC3bits.DCIIE = 0;//disable the interrupt while this calculation is taking place because the floating point calculations take much longer than one sample period

            cosAccumulatorFloat[0] = (double)cosAccumulator[0];
            sinAccumulatorFloat[0] = (double)sinAccumulator[0];
            phaseAngle[0] = atan2(sinAccumulatorFloat[0], cosAccumulatorFloat[0]);
            if (fabs(cosAccumulatorFloat[0]) > fabs(sinAccumulatorFloat[0]))
            {
                amplitude[0] = cosAccumulatorFloat[0] / (1.07374182e9 * cosf(phaseAngle[0]) * measurementTime);
            }
            else
            {
                amplitude[0] = sinAccumulatorFloat[0] / (1.07374182e9 * sinf(phaseAngle[0]) * measurementTime);
            }
            if(amplitude[0] < 0)//make sure amplitude is positive
            {
                amplitude[0] = fabsf(amplitude[0]);
                if (phaseAngle[0] < 0)
                {
                    phaseAngle[0] = phaseAngle[0] + M_PI_2;
                }
                else
                {
                    phaseAngle[0] = phaseAngle[0] - M_PI_2;
                }
            }
#ifdef MEASURE_F2_AT_BRIDGE
            cosAccumulatorFloat[1] = (double)cosAccumulator[1];
            sinAccumulatorFloat[1] = (double)sinAccumulator[1];
            phaseAngle[1] = atan2(sinAccumulatorFloat[1], cosAccumulatorFloat[1]);
            if (fabs(cosAccumulatorFloat[1]) > fabs(sinAccumulatorFloat[1]))
            {
                amplitude[1] = cosAccumulatorFloat[1] / (1.07374182e9 * cos(phaseAngle[1]) * measurementTime);
            }
            else
            {
                amplitude[1] = sinAccumulatorFloat[1] / (1.07374182e9 * sin(phaseAngle[1]) * measurementTime);
            }
            if(amplitude[1] < 0)//make sure amplitude is positive
            {
                amplitude[1] = fabsf(amplitude[0]);
                if (phaseAngle[1] < 0)
                {
                    phaseAngle[1] = phaseAngle[1] + M_PI_2;
                }
                else
                {
                    phaseAngle[1] = phaseAngle[1] - M_PI_2;
                }
            }
#else
            phaseAngle[1] = 0;
            amplitude[1] = 0;
#endif
            for (i = 2; i < 5; ++i)
            {
                cosAccumulatorFloat[i] = (double)cosAccumulator[i];
                sinAccumulatorFloat[i] = (double)sinAccumulator[i];
                phaseAngle[i] = atan2(sinAccumulatorFloat[i], cosAccumulatorFloat[i]);
                if (fabs(cosAccumulatorFloat[i]) > fabs(sinAccumulatorFloat[i]))
                {
                    amplitude[i] = cosAccumulatorFloat[i] / (1.07374182e9 * cos(phaseAngle[i]) * measurementTime);
                }
                else
                {
                    amplitude[i] = sinAccumulatorFloat[i] / (1.07374182e9 * sin(phaseAngle[i]) * measurementTime);
                }
                if(amplitude[i] < 0)//make sure amplitude is positive
                {
                    amplitude[i] = fabsf(amplitude[i]);
                    if (phaseAngle[i] < 0)
                    {
                        phaseAngle[i] = phaseAngle[i] + M_PI_2;
                    }
                    else
                    {
                        phaseAngle[i] = phaseAngle[i] - M_PI_2;
                    }
                }
            }
            //transmit results
            transmitResults(sensorAddress, phaseAngle, amplitude, bridgeADCClipFlag, coilADCClipFlag, bridgeDigitalClipFlag);

            ++sensorIndex;
            if (sensorIndex == numberOfSensors)
            {
                sensorIndex = 0;
            }
            local_state = START_NEW_MEASUREMENT_CYCLE;
            IEC3bits.DCIIE = 1;//re-enable the DCI interrupt
            break;
        }
        default:
            local_state = IDLE;
            break;
    }

    START_ATOMIC();
    if (   global_state != IDLE
        && global_state != START_BRIDGE_BALANCE_FSM
        && global_state != START_MEASUREMENT_FSM) {

        /*
         * if we hit here, we're assured that no messages from UART should
         * override our next state
         */
        global_state = local_state;

    }
    END_ATOMIC();

}

_Q15 readBridgeSampleAndApplyGain(bool* bridgeDigitalClipFlag)
{

#ifdef SIMULATION_MODE
    uint16_t RXBUF0 = rand();//only to give random stimulus during simulation
    uint16_t RXBUF1 = rand();//only to give random stimulus during simulation
#endif
        
    if (bridgeADCGainFactor == 0)
    {
        return RXBUF0;
    }
    else
    {
        int16_t clipTest;
        int16_t maskForTruncationBitsPlusOne = 0x8000;//make it signed so that the shift uses sign extension
        maskForTruncationBitsPlusOne = maskForTruncationBitsPlusOne >> bridgeADCGainFactor;
        clipTest = RXBUF0 & maskForTruncationBitsPlusOne;
        if (clipTest != 0x0000 && clipTest != maskForTruncationBitsPlusOne)
        {
            *bridgeDigitalClipFlag = true;
            if ((int16_t)RXBUF0 < 0)
            {
                return 0x8000;//interpret the clipped sample as the most negative value
            }
            else
            {
                return 0x7FFF;//interpret the clipped sample as the most positive value
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
            return *tempSampleLowWord + *tempSampleHighWord;
        }
    }
}

void shiftRegister(_Q15 cosOmega1T, _Q15 cosOmega2T, _Q15 *cosOmega1TTimeAligned, _Q15 *cosOmega2TTimeAligned)
{
    static _Q15 delayArrayA[ADCDAC_GROUP_DELAY];
    static _Q15 delayArrayB[ADCDAC_GROUP_DELAY];

    static int16_t pointer = 0;
    int16_t pointerPlusOne = pointer + 1;
    if(pointerPlusOne == ADCDAC_GROUP_DELAY)
    {
        pointerPlusOne = 0;
    }
    delayArrayA[pointer] = cosOmega1T;
    delayArrayB[pointer] = cosOmega2T;
    *cosOmega1TTimeAligned = delayArrayA[pointerPlusOne];
    *cosOmega2TTimeAligned = delayArrayB[pointerPlusOne];
    ++pointer;
        if(pointer == ADCDAC_GROUP_DELAY)
    {
        pointer = 0;
    }
}

void signalGenerator(unsigned char runOrReset, _Q15 *freqT, __eds__ _Q15 *cosOmega1T, __eds__ _Q15 *cosOmega2T)
{
    if (runOrReset == RUN_SIGNAL_GEN)
    {
#ifdef TRIG_USES_LUT
        *cosOmega1T = _Q15cosPILUT(freqT[0]);//generating cos(omega1 * t)
        *cosOmega2T = _Q15cosPILUT(freqT[1]);//generating cos(omega2 * t)
#else
        *cosOmega1T = _Q15cosPI(freqT[0]);//generating cos(omega1 * t)
        *cosOmega2T = _Q15cosPI(freqT[1]);//generating cos(omega2 * t)
#endif

        uint32_t tempSample;
        if (a1 == 0x7FFF)
        {
            TXBUF0 = *cosOmega1T;
            TXBUF1 = 0x0000;
        }
        else
        {
            tempSample = asm16X16Mult(*cosOmega1T, a1);
            TXBUF0 = *((__eds__ uint16_t *)&tempSample + 1);
            TXBUF1 = *((__eds__ uint16_t *)&tempSample);
        }
#ifndef PROBE
        if (a2 == 0x7FFF)
        {
            TXBUF2 = *cosOmega2T;
            TXBUF3 = 0x0000;
        }
        else
        {
            tempSample = asm16X16Mult(*cosOmega2T, a2);
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
        freqT[0] += f1;
        freqT[1] += f2;
        freqT[2] += f1;
        freqT[3] += f2;
        freqT[4] += fdiff;
        freqT[5] += fsum;
    }
    
    else
    {
        *cosOmega1T = 0;
        *cosOmega2T = 0;
        TXBUF0 = 0x0000;
        TXBUF1 = 0x0000;
        TXBUF2 = 0x0000;
        TXBUF3 = 0x0000;
        
        freqT[0] = (f1 * ADCDAC_GROUP_DELAY);//advance to compensate for ADC/DAC FIR group delay
        freqT[1] = (f2 * ADCDAC_GROUP_DELAY);//advance to compensate for ADC/DAC FIR group delay
        freqT[2] = 0x0000;
        freqT[3] = 0x0000;
        freqT[4] = 0x0000;
        freqT[5] = 0x0000;
    }
}

void measure(_Q15 bridgeSample, _Q15 coilSample, _Q15 *freqT, _Q15 cosOmega1TTimeAligned, _Q15 cosOmega2TTimeAligned, int64_t *cosAccumulator, int64_t *sinAccumulator)
{
    int32_t bridgeByCosF1T, bridgeByCosFDiffT, bridgeByCosFSumT, coilByCosF2T, bridgeBySinF1T, bridgeBySinFDiffT, bridgeBySinFSumT, coilBySinF2T;
#ifdef MEASURE_F2_AT_BRIDGE
    int32_t bridgeByCosF2T, bridgeBySinF2T;
#endif
#ifdef TRIG_USES_LUT
    _Q15 cosFDiffT = _Q15cosPILUT(freqT[4]);
    _Q15 cosFSumT = _Q15cosPILUT(freqT[5]);
    _Q15 sinF1T = _Q15sinPILUT(freqT[2]);
    _Q15 sinF2T = _Q15sinPILUT(freqT[3]);
    _Q15 sinFDiffT = _Q15sinPILUT(freqT[4]);
    _Q15 sinFSumT = _Q15sinPILUT(freqT[5]);
#else
    _Q15 cosFDiffT = _Q15cosPI(freqT[4]);
    _Q15 cosFSumT = _Q15cosPI(freqT[5]);
    _Q15 sinF1T = _Q15sinPI(freqT[2]);
    _Q15 sinF2T = _Q15sinPI(freqT[3]);
    _Q15 sinFDiffT = _Q15sinPI(freqT[4]);
    _Q15 sinFSumT = _Q15sinPI(freqT[5]);
#endif
    bridgeByCosF1T = asm16X16Mult(bridgeSample, cosOmega1TTimeAligned);
    cosAccumulator[0] += bridgeByCosF1T;
#ifdef MEASURE_F2_AT_BRIDGE
    bridgeByCosF2T = asm16X16Mult(bridgeSample, cosOmega2TTimeAligned);
    cosAccumulator[1] += bridgeByCosF2T;
#endif
    bridgeByCosFDiffT = asm16X16Mult(bridgeSample, cosFDiffT);
    cosAccumulator[2] += bridgeByCosFDiffT;
    bridgeByCosFSumT =  asm16X16Mult(bridgeSample, cosFSumT);
    cosAccumulator[3] += bridgeByCosFSumT;
    coilByCosF2T = asm16X16Mult(coilSample, cosOmega2TTimeAligned);
    cosAccumulator[4] += coilByCosF2T;

    bridgeBySinF1T = asm16X16Mult(bridgeSample, sinF1T);
    sinAccumulator[0] += bridgeBySinF1T;
#ifdef MEASURE_F2_AT_BRIDGE
    bridgeBySinF2T = asm16X16Mult(bridgeSample, sinF2T);
    sinAccumulator[1] += bridgeBySinF2T;
#endif
    bridgeBySinFDiffT = asm16X16Mult(bridgeSample, sinFDiffT);
    sinAccumulator[2] += bridgeBySinFDiffT;
    bridgeBySinFSumT = asm16X16Mult(bridgeSample, sinFSumT);
    sinAccumulator[3] += bridgeBySinFSumT;
    coilBySinF2T = asm16X16Mult(coilSample, sinF2T);
    sinAccumulator[4] += coilBySinF2T;

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
