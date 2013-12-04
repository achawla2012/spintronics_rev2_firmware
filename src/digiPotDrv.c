/*
 * digiPotDrv.c
 *
 * Author: Michael Reinhart Sandstedt
 *
 * First release: September 11th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "spintronicsIncludes.h"
#include "spiTx.h"
#include "constants.h"

float getRBridgeOhms(uint16_t val);
float getRAmpOhms(uint8_t val);
float getU25GainFromU24Ohms(float rg_ohms);
float getU25GainFromU24Code(uint8_t u24_code);
float getU25InverseGainFromU24Ohms(float rg_ohms);
float getU25InverseGainFromU24Code(uint8_t u24_code);
float getU24OhmsFromU25Gain(float u25_gain);
uint8_t getU24CodeFromU25Gain(float u25_gain);

/*
 * void setRBridge(uint16_t val)
 *
 * sets the resisttance of RBridge by controlling U20 and U23 in order to
 * balance the Wheatstone bridge. Units of val are arbitrary
 * 
 * See getRBridgeOhms() for a translation algorithm to physical units
 *
 * uint16_t val: value to set RBridge to, in the range of 0 to 1535
 */

void setRBridge(uint16_t val)
{
    uint8_t u20_val;
    uint8_t u23_val;

    if (val > 1535)
    {
        //maximum value is 1535
        val = 1535;
    }

    /*
     * Bits 8-10 are for U20; grab thes.
     *
     * U20 is an AD5160BRJ25-RL7 and requires an 8-bit code, but we are only not
     * going to use all of the lower bits.  See getRBridgeOhms() for more
     * explanation.  Multiplying bits 8-10 by 51 gives the desired range 0-255
     */
    u20_val = (val >> 8) * 51;
    spiTx(U20, u20_val);

    /*
     * Bits 0-7 are for U23; grab these.
     *
     * U23 is an AD8400ARZI and requires a 10-bit code, where the lower 8 bits
     * correspond to the resistance value and the upper two bits are an address,
     * which is always 0b00 in ths case.
     */
    u23_val = val & 0x00FF;
    spiTx(U23, u23_val);
}


/*
 * void setRAmp(uint8_t val)
 *
 * Set the input resistance for the Wheatstone bridge buffer amplifier.
 * This controls the gain for that amplifier.  Units are arbitrary;
 *
 * See getRAmpOhms() for a translation algorithm to physical units
 *
 * uint8_t val: value to set RBridge to
 */

void setRAmp(uint8_t val)
{
    spiTx(U24, val);
}


/*
 * float getRBridgeOhms(uint16_t val)
 *
 * convert values used internally by the firmware into a resistance of the
 * variable leg of the Wheatstone bridge in units of ohms.
 *
 * U20 (AD5160BRJ25-RL7) and U23 (AD8400ARZI) are connected in series
 * to create this resistance on the pcb
 *
 * uint16_t val: arbitrary units used by the firmware
 *
 * return: Wheatstone bridge resistance in ohms corresonding to val
 */

float getRBridgeOhms(uint16_t val)
{
    uint8_t u20_val;
    uint8_t u23_val;

    if (val > 1535)
    {
        //maximum value is 1535
        val = 1535;
    }

    u20_val = (val >> 8) * 51;
    u23_val = val & 0x00FF;

    return 110.0 + (  (256 - u20_val) * 5000.0
                    + (256 - u23_val) * 1000.0 ) / 256.0;
}


/*
 * float getRAmpOhms(uint8_t val)
 *
 * convert values used internally by the firmware into the feedback resistance
 * for the Wheatstone bridge buffer amplifier in units of ohms.
 *
 * uint8_t val: arbitrary units used by the firmware
 *
 * return: Wheatstone bridge buffer amp r_feedback in ohms
 */

inline float getRAmpOhms(uint8_t val)
{
    /*
     * val corresponds to values sent to U24 (AD8400ARZI).  See the Analog
     * Devices datasheet for more info.
     */
    
    return 50.0 + (256 - val) * 1000.0 / 256.0;
}

inline float getU25GainFromU24Ohms(float rg_ohms)
{
    return 1 + 6000.0 / rg_ohms;
}

inline float getU25GainFromU24Code(uint8_t u24_code)
{
    float rg_ohms;
    rg_ohms = getRAmpOhms(u24_code);
    return getU25GainFromU24Ohms(rg_ohms);
}

inline float getBridgeBufGainFromU24Code(uint8_t u24_code)
{
    return getU25GainFromU24Code(u24_code) * U2_BUF_GAIN;
}

inline float getU25InverseGainFromU24Ohms(float rg_ohms)
{
    return rg_ohms / (6000.0 + rg_ohms);
}

inline float getU25InverseGainFromU24Code(uint8_t u24_code)
{
    float rg_ohms;
    rg_ohms = getRAmpOhms(u24_code);
    return getU25InverseGainFromU24Ohms(rg_ohms);
}

inline float getBridgeInverseGainFromU24Code(uint8_t u24_code)
{
    return getU25InverseGainFromU24Code(u24_code) * U2_INVERSE_GAIN;
}

inline float getU24OhmsFromU25Gain(float u25_gain)
{
    return 6000.0 / (u25_gain - 1);
}

inline uint8_t getU24CodeFromU25Gain(float u25_gain)
{
    uint8_t u24_code;
    float u24_float_code;
    float u24_ohms;

    u24_ohms = getU24OhmsFromU25Gain(u25_gain);

    u24_float_code = 256 - 256 * (u24_ohms - 50.0) * 1e-3;

    if (u24_float_code < 0.0) {
        u24_code = 0;
    } else if (u24_float_code > 255.0) {
        u24_code = 0xFF;
    } else {
        u24_code = u24_float_code;
    }

    return u24_code;

}

inline uint8_t getU24CodeFromBrdigeBufGain(float bridge_gain) {

    return getU24CodeFromU25Gain(bridge_gain * U2_INVERSE_GAIN);

}
