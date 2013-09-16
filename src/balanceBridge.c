/*
 * balanceBridge.c
 *
 * designed and written
 * by Michael Reinhart Sandstedt
 *
 * First release: September 12th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "spintronicsIncludes.h"
#include "spintronicsConfig.h"
#include "muxControl.h"
#include "digiPotDrv.h"
#include "uartDrv.h"
#include "fsmStates.h"
#include "commsDefines.h"
#include "utility.h"

//how close do we tune the digiPots?
#define R_AMP_MARGIN 2//out of 255
#define R_BRIDGE_MARGIN 5//out of 1535

void balanceBridgeGenerator(uint8_t runOrReset, _Q15 amplitude, _Q15 frequency, __eds__ _Q15 *cosOmegaT, __eds__ _Q15 *sinOmegaT);
void balanceBridgeMeasure(_Q15 bridgeSample, _Q15 cosOmegaT, _Q15 sinOmegaT, int64_t *cosAccumulator, int64_t *sinAccumulator);

#ifdef TRIG_USES_LUT
extern _Q15 _Q15cosPILUT(_Q15 phiOverPI);
extern _Q15 _Q15sinPILUT(_Q15 phiOverPI);
#endif

uint16_t sensorRBridgeTable[256];
bool sensorRBridgeTableValid = false;

void balanceBridgeFSM(void)
{
    static uint8_t local_state;
    static _Q15 local_a1 = 0;
    static _Q15 local_f1 = 0;
    static _Q15 gain_check_measure_time;
    static _Q15 bridge_check_measure_time;
    static uint32_t timer;
    static uint8_t sensorIndex;
    _Q15 cosOmegaT;//fractions of full-scale
    _Q15 sinOmegaT;//fractions of full-scale
    static int64_t cosAccumulator;
    static int64_t sinAccumulator;

    static uint8_t r_amp;
    static uint8_t r_amp_min;
    static uint8_t r_amp_lo_mid;
    static uint8_t r_amp_hi_mid;
    static uint8_t r_amp_max;
    static bool r_amp_lo_mid_clip;
    static bool r_amp_hi_mid_clip;

    static uint16_t r_bridge_min;
    static uint16_t r_bridge_lo_mid;
    static uint16_t r_bridge_hi_mid;
    static uint16_t r_bridge_max;
    static float r_bridge_lo_mid_amplitude;
    static float r_bridge_hi_mid_amplitude;

    bool advance_state = false;
    bool a1_match = true;
    bool f1_match = true;

    //START_ATOMIC() called from calling ISR
    local_state = global_state;
    /*
     * clear the global state register so we can tell
     * when another request comes in via UART
     */
    global_state = 0;
    END_ATOMIC();//end critical section

    switch (local_state)
    {
        case IDLE:
            sensorIndex = 0;
            balanceBridgeGenerator(RESET_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);//use the same frequency and amplitude to balance the bridge as will be used during measurement
            break;


        case START_BRIDGE_BALANCE_FSM:
        {

            uint32_t gain_check_measure_cycles = GAIN_CHECK_MEASURE_CYCLES;
            uint32_t gain_check_measure_time_32;
            uint32_t bridge_check_measure_time_32;

            sensorIndex = 0;

            START_ATOMIC;//begin critical section; must be atomic!
            local_a1 = a1;
            local_f1 = f1;
            END_ATOMIC();//end critical section

            IEC3bits.DCIIE = 0;//disable the DCI interrupt in case the divide take longer than one sample period

            gain_check_measure_time_32 = gain_check_measure_cycles * 65536 / (uint16_t)local_f1;
            if (gain_check_measure_time_32 & 0xFFFF0000 != 0) {
                //gain_check_measure_time_32 will overflow the int16_t
                gain_check_measure_time = local_f1;
            } else {
                gain_check_measure_time = gain_check_measure_time_32;
            }

            bridge_check_measure_time_32 = gain_check_measure_time_32 * BRIDGE_CHECK_MEASURE_TIME_MULTIPLIER;
            if (bridge_check_measure_time_32 & 0xFFFF0000 != 0) {
                //bridge_check_measure_time_32 will overflow the int16_t
                bridge_check_measure_time = local_f1;
            } else {
                bridge_check_measure_time = bridge_check_measure_time_32;
            }

            IEC3bits.DCIIE = 1;//re-enable DCI interrupt

            balanceBridgeGenerator(RESET_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);//use the same frequency and amplitude to balance the bridge as will be used during measurement
            local_state = START_GAIN_CAL;
            break;
        }

        
        case START_GAIN_CAL:
        {
            uint8_t sensorAddress;

            timer = 0;

            START_ATOMIC;//begin critical section; must be atomic!
            if (local_a1 != a1) {
                a1_match = false;
            }
            if (local_f1 != f1) {
                f1_match = false;
            }
            sensorAddress = sensorAddressTable[sensorIndex];
            END_ATOMIC();//end critical section

            if (!a1_match) {
                transmitError(A1_CHANGED_DURING_BRIDGE_BALANCE);
                local_state = IDLE;
            } else if (!f1_match) {
                transmitError(F1_CHANGED_DURING_BRIDGE_BALANCE);
                local_state = IDLE;
            } else {

                configSensor(sensorAddress);

                //set r_bridge to initial value
                setRBridge(R_BRIDGE_MID);
            
                balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

                r_amp_min = R_AMP_MIN;
                r_amp_max = R_AMP_MAX;

                local_state = SET_R_AMP_TO_LO_MID;
            }
            break;
        }

        case SET_R_AMP_TO_LO_MID:

            balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            r_amp_lo_mid_clip = false;

            //set setRAmp to r_amp_lo_mid
            r_amp_lo_mid = (r_amp_max - r_amp_min) / 4 + r_amp_min;
            setRAmp(r_amp_lo_mid);

            local_state = R_AMP_LO_MID_SIGNAL_SETTLING;
            break;


        case R_AMP_LO_MID_SIGNAL_SETTLING:

            balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            //wait for any transients to settle
            ++timer;
            if (GAIN_CHECK_SETUP_TIME == timer) {
                timer = 0;
                local_state = R_AMP_LO_MID_CLIP_TEST;
            }
            break;


        case R_AMP_LO_MID_CLIP_TEST:

            //see if the signal is clipping with this gain
            if (RXBUF0 == 0x7FFF || RXBUF0 == 0x8000) {
                r_amp_lo_mid_clip = true;
                advance_state = true;
            }

            balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            ++timer;
            if (advance_state || gain_check_measure_time == timer) {
                timer = 0;
                local_state = SET_R_AMP_TO_HI_MID;
            }
            break;

        case SET_R_AMP_TO_HI_MID:

            balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);
            r_amp_hi_mid_clip = false;

            //set setRAmp to r_amp_hi_mid
            r_amp_hi_mid = (uint16_t)(r_amp_max - r_amp_min) * 3 / 4 + r_amp_min;
            setRAmp(r_amp_hi_mid);

            local_state = R_AMP_HI_MID_SIGNAL_SETTLING;
            break;

        case R_AMP_HI_MID_SIGNAL_SETTLING:

            balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            //wait for any transients to settle
            ++timer;
            if (GAIN_CHECK_SETUP_TIME == timer) {
                timer = 0;
                local_state = R_AMP_HI_MID_CLIP_TEST;
            }
            break;

        case R_AMP_HI_MID_CLIP_TEST:

            //see if the signal is clipping with this gain
            if (RXBUF0 == 0x7FFF || RXBUF0 == 0x8000) {
                r_amp_hi_mid_clip = true;
                advance_state = true;
            }

            balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            ++timer;
            if (advance_state || gain_check_measure_time == timer) {
                timer = 0;

                if (!r_amp_lo_mid_clip && !r_amp_hi_mid_clip) {
                    r_amp_min = r_amp_lo_mid;
                } else if (r_amp_lo_mid_clip) {
                    r_amp_max = r_amp_lo_mid;
                } else {
                    r_amp_min = r_amp_lo_mid;
                    r_amp_max = r_amp_hi_mid;
                }

                if (   (r_amp_min > r_amp_max)
                    || (r_amp_max - r_amp_min < R_AMP_MARGIN)) {

                    if (r_amp_min > R_AMP_MARGIN) {
                        r_amp = r_amp_min - R_AMP_MARGIN;
                        setRAmp(r_amp);
                    } else {
                        r_amp = 0;
                        setRAmp(r_amp);
                    }

                    local_state = START_BRIDGE_CAL;

                } else {

                    local_state = SET_R_AMP_TO_LO_MID;

                }
            }
            break;


        case START_BRIDGE_CAL:

            balanceBridgeGenerator(RESET_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            r_bridge_min = R_BRIDGE_MIN;
            r_bridge_max = R_BRIDGE_MAX;

            local_state = SET_R_BRIDGE_TO_LO_MID;

            break;


        case SET_R_BRIDGE_TO_LO_MID:

            balanceBridgeGenerator(RESET_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            cosAccumulator = 0;
            sinAccumulator = 0;
            
            //set set RBridge to r_bridge_lo_mid
            r_bridge_lo_mid = (r_bridge_max - r_bridge_min) / 4 + r_bridge_min;
            setRBridge(r_bridge_lo_mid);

            local_state = R_BRIDGE_LO_MID_SIGNAL_SETTLING;
            break;


        case R_BRIDGE_LO_MID_SIGNAL_SETTLING:

            balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            //wait for any transients to settle
            ++timer;
            if (BRIDGE_CHECK_SETUP_TIME == timer) {
                timer = 0;
                local_state = R_BRIDGE_LO_MID_AMP_MEASURE;
            }
            break;

        case R_BRIDGE_LO_MID_AMP_MEASURE:
        {
            volatile _Q15 bridgeSample;

            bridgeSample = RXBUF0;

            //see if the signal is clipping with this gain
            if (bridgeSample == 0x7FFF || bridgeSample == 0x8000) {

                //we clipped! reduce gain and abort this measurement
                timer = 0;
                if (0 == r_amp) {
                    //bridge amp is at minimum gain; calibration has failed
                    local_state = IDLE;
                    transmitError(BRIDGE_BALANCE_FAILURE);
                } else if (r_amp > R_AMP_MARGIN) {
                    r_amp -= R_AMP_MARGIN;
                    setRAmp(r_amp);
                    local_state = START_BRIDGE_CAL;
                } else {
                    r_amp = 0;
                    setRAmp(r_amp);
                    local_state = START_BRIDGE_CAL;
                }

            } else {

                balanceBridgeMeasure(bridgeSample, cosOmegaT, sinOmegaT, &cosAccumulator, &sinAccumulator);
                ++timer;
                if (bridge_check_measure_time == timer) {
                    timer = 0;
                    local_state = R_BRIDGE_LO_MID_AMP_CALC;
                } else {
                    balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);
                }

            }
            break;
        }


        case R_BRIDGE_LO_MID_AMP_CALC:
        {
            double cosAccumulatorFloat;
            double sinAccumulatorFloat;
            double phaseAngle;

            IEC3bits.DCIIE = 0;//disable the interrupt while this calculation is taking place because the floating point calculations take much longer than one sample period

            cosAccumulatorFloat = (double)cosAccumulator;
            sinAccumulatorFloat = (double)sinAccumulator;

            phaseAngle = atan2(sinAccumulatorFloat, cosAccumulatorFloat);

            if (fabs(cosAccumulatorFloat) > fabs(sinAccumulatorFloat))
            {
                //NOTE: units are arbitrary!
                r_bridge_lo_mid_amplitude = cosAccumulatorFloat / cosf(phaseAngle);
            }
            else
            {
                //NOTE: units are arbitrary!
                r_bridge_lo_mid_amplitude = sinAccumulatorFloat / sinf(phaseAngle);
            }

            if(r_bridge_lo_mid_amplitude < 0)//make sure amplitude is positive
            {
                r_bridge_lo_mid_amplitude = fabsf(r_bridge_lo_mid_amplitude);
            }

            local_state = SET_R_BRIDGE_TO_HI_MID;
            IEC3bits.DCIIE = 1;//re-enable the DCI interrupt
            break;
        }


        case SET_R_BRIDGE_TO_HI_MID:

            balanceBridgeGenerator(RESET_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            cosAccumulator = 0;
            sinAccumulator = 0;

            //set set RBridge to r_bridge_hi_mid
            r_bridge_hi_mid = (r_bridge_max - r_bridge_min) * 3 / 4 + r_bridge_min;
            setRBridge(r_bridge_lo_mid);

            local_state = R_BRIDGE_HI_MID_SIGNAL_SETTLING;
            break;


        case R_BRIDGE_HI_MID_SIGNAL_SETTLING:

            balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);

            //wait for any transients to settle
            ++timer;
            if (BRIDGE_CHECK_SETUP_TIME == timer) {
                timer = 0;
                local_state = R_BRIDGE_HI_MID_AMP_MEASURE;
            }
            break;

        case R_BRIDGE_HI_MID_AMP_MEASURE:
        {
            volatile _Q15 bridgeSample;

            bridgeSample = RXBUF0;

            //see if the signal is clipping with this gain
            if (bridgeSample == 0x7FFF || bridgeSample == 0x8000) {

                //we clipped! reduce gain and abort this measurement
                timer = 0;
                if (0 == r_amp) {
                    //bridge amp is at minimum gain; calibration has failed
                    local_state = IDLE;
                    transmitError(BRIDGE_BALANCE_FAILURE);
                } else if (r_amp > R_AMP_MARGIN) {
                    r_amp -= R_AMP_MARGIN;
                    setRAmp(r_amp);
                    local_state = START_BRIDGE_CAL;
                } else {
                    r_amp = 0;
                    setRAmp(r_amp);
                    local_state = START_BRIDGE_CAL;
                }

            } else {
                
                balanceBridgeMeasure(bridgeSample, cosOmegaT, sinOmegaT, &cosAccumulator, &sinAccumulator);
                ++timer;
                if (bridge_check_measure_time == timer) {
                    timer = 0;
                    local_state = R_BRIDGE_HI_MID_AMP_CALC;
                } else {
                    balanceBridgeGenerator(RUN_SIGNAL_GEN, local_a1, local_f1, &cosOmegaT, &sinOmegaT);
                }

            }
            break;
        }


        case R_BRIDGE_HI_MID_AMP_CALC:
        {
            double cosAccumulatorFloat;
            double sinAccumulatorFloat;
            double phaseAngle;
            uint16_t cal_window;

            IEC3bits.DCIIE = 0;//disable the interrupt while this calculation is taking place because the floating point calculations take much longer than one sample period

            cosAccumulatorFloat = (double)cosAccumulator;
            sinAccumulatorFloat = (double)sinAccumulator;

            phaseAngle = atan2(sinAccumulatorFloat, cosAccumulatorFloat);

            if (fabs(cosAccumulatorFloat) > fabs(sinAccumulatorFloat))
            {
                //NOTE: units are arbitrary!
                r_bridge_hi_mid_amplitude = cosAccumulatorFloat / cosf(phaseAngle);
            }
            else
            {
                //NOTE: units are arbitrary!
                r_bridge_hi_mid_amplitude = sinAccumulatorFloat / sinf(phaseAngle);
            }

            if(r_bridge_hi_mid_amplitude < 0)//make sure amplitude is positive
            {
                r_bridge_hi_mid_amplitude = fabsf(r_bridge_lo_mid_amplitude);
            }

            if (r_bridge_lo_mid_amplitude > r_bridge_hi_mid_amplitude) {
                r_bridge_min = r_bridge_lo_mid_amplitude;
            } else {
                r_bridge_max = r_bridge_hi_mid_amplitude;
            }
            
            if (r_bridge_min < r_bridge_max) {
                cal_window = r_bridge_max - r_bridge_min;
            } else {
                cal_window = r_bridge_min - r_bridge_max;
            }
            
            if (cal_window <= R_BRIDGE_MARGIN) {

                sensorRBridgeTable[sensorIndex] = r_bridge_min;
                ++sensorIndex;
                START_ATOMIC();
                if (sensorIndex >= numberOfSensors) {

                    /*
                     * must test >= in case numberOfSensors was updated since
                     * the last iteration of this FSM
                     */
                    sensorRBridgeTableValid = true;
                    local_state = START_MEASUREMENT_FSM;
                } else {
                    local_state = START_GAIN_CAL;
                }

            } else {

                local_state = SET_R_BRIDGE_TO_LO_MID;

            }

            IEC3bits.DCIIE = 1;//re-enable the DCI interrupt
            break;
        }

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

void balanceBridgeGenerator(uint8_t runOrReset, _Q15 amplitude, _Q15 frequency, __eds__ _Q15 *cosOmegaT, __eds__ _Q15 *sinOmegaT)
{
    static _Q15 freqT;
    if (runOrReset == RESET_SIGNAL_GEN)
    {
        TXBUF0 = 0x0000;
        TXBUF1 = 0x0000;
        TXBUF2 = 0x0000;
        TXBUF3 = 0x0000;

        freqT = 0;
    }
    else
    {
#ifdef TRIG_USES_LUT
        *cosOmegaT = _Q15cosPILUT(freqT);//generating cos(omega * t)
        *sinOmegaT = _Q15sinPILUT(freqT);//generating sin(omega * t)
#else
        *cosOmegaT = _Q15cosPI(freqT);//generating cos(omega * t)
        *sinOmegaT = _Q15sinPI(freqT);//generating sin(omega * t)
#endif

        uint32_t tempSample;
        if (amplitude == 0x7FFF)
        {
            TXBUF0 = *cosOmegaT;
            TXBUF1 = 0x0000;
        }
        else
        {
            tempSample = asm16X16Mult(*cosOmegaT, a1);
            TXBUF0 = *((__eds__ uint16_t *)&tempSample + 1);
            TXBUF1 = *((__eds__ uint16_t *)&tempSample);
        }
#ifndef PROBE
        TXBUF2 = 0x0000;
        TXBUF3 = 0x0000;
#elif defined(PROBE_BRIDGE_ADC)
        TXBUF2 = RXBUF0;
        TXBUF3 = RXBUF1;
#elif defined(PROBE_COIL_ADC)
        TXBUF2 = RXBUF2;
        TXBUF3 = RXBUF3;
#endif
        freqT += frequency;
    }
}
void balanceBridgeMeasure(_Q15 bridgeSample, _Q15 cosOmegaT, _Q15 sinOmegaT, int64_t *cosAccumulator, int64_t *sinAccumulator)
{
    int32_t bridgeByCosFT, bridgeBySinFT;

    bridgeByCosFT = asm16X16Mult(bridgeSample, cosOmegaT);
    *cosAccumulator += bridgeByCosFT;

    bridgeBySinFT = asm16X16Mult(bridgeSample, sinOmegaT);
    *sinAccumulator += bridgeBySinFT;
}