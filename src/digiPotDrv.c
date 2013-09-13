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
#include "spintronics.h"
#include "spiTx.h"

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
    uint8_t u23_val[2];

    if (val > 1535)
    {
        //maximum value is 1535
        val = 1535;
    }

    /*
     * Using WA tap for U20 / U23; min_code gives RMax, max_code gives Rmin
     */
    val = 1535 - val;

    /*
     * Bits 8-10 are for U20; grab thes.
     *
     * U20 is an AD5160BRJ25-RL7 and requires an 8-bit code, but we are only not
     * going to use all of the lower bits.  See getRBridgeOhms() for more
     * explanation.  Multiplying bits 8-10 by 51 gives the desired range 0-255
     */
    u20_val = (val >> 8) * 51;
    spiTx(U20, 1, &u20_val);

    /*
     * Bits 0-7 are for U23; grab these.
     *
     * U23 is an AD8400ARZI and requires a 10-bit code, where the lower 8 bits
     * correspond to the resistance value and the upper two bits are an address,
     * which is always 0b00 in ths case.
     *
     * SPI transmits MSB first, so the address is copied to index 0 and the
     * value is copied to index 1.
     */
    u23_val[0] = 0;
    u23_val[1] = val & 0x00FF;
    spiTx(U23, 2, u23_val);
}


/*
 * void setRAmp(uint8_t val)
 *
 * Set the feedback resistance for the Wheatstone bridge buffer amplifier.
 * This controls the gain for that amplifier.  Units are arbitrary;
 *
 * See getRAmpOhms() for a translation algorithm to physical units
 *
 * This is a feedback resistor, so Rmax (val=255) gives most amplifcation;
 * Rmin (val=0) gives least amplification; amplification is approximately
 * proportional to val.  I.e., increase val to increase amplification.
 *
 * uint8_t val: value to set RBridge to
 */

void setRAmp(uint8_t val)
{
    uint8_t u24_val[2];

    /*
     * Using WA tap for U24; min_code gives RMax, max_code gives Rmin
     */
    val = 255 - val;

    /*
     * U24 is an AD8400ARZI and requires a 10-bit code, where the lower 8 bits
     * correspond to the resistance value and the upper two bits are an address,
     * which is always 0b00 in ths case.
     *
     * SPI transmits MSB first, so the address is copied to index 0 and the
     * value is copied to index 1.
     */
    u24_val[0] = 0;
    u24_val[1] = val;

    spiTx(U24, 2, u24_val);
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
    float ohms;

    if (val > 1535)
    {
        //maximum value is 1535
        val = 1535;
    }

    /*
     * Using WA tap for U20 / U23; min_code gives RMax, max_code gives Rmin
     */
    val = 1535 - val;

    u20_val = (val >> 8) * 51;
    u23_val = val & 0x00FF;

    ohms = 110.0 + (  (256 - u20_val) * 5000.0
                    + (256 - u23_val) * 1000.0 ) / 256.0;

    return ohms;
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

float getRAmpOhms(uint8_t val)
{
    /*
     * Using WA tap for U24; min_code gives RMax, max_code gives Rmin
     */
    val = 255 - val;

    /*
     * val corresponds to values sent to U24 (AD8400ARZI).  See the Analog
     * Devices datasheet for more info.
     */
    
    float ohms;
    
    ohms = 50.0 + (256 - val) * 1000.0 / 256.0;

    return ohms;
}