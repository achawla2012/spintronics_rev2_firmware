/*
 * i2sDrv.c
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
#include "spintronicsIncludes.h"
#include "fsmStates.h"
#include "generateAndProcessSamples.h"
#include "balanceBridge.h"
#include "uartDrv.h"
#include "utility.h"

/*
i2sInit(): Initialise DCI for I2S
*/

void i2sInit (void)
{

    //make RE5, RE6, RE7 digital pins
    ANSELEbits.ANSE5 = 0;
    ANSELEbits.ANSE6 = 0;
    ANSELEbits.ANSE7 = 0;

    //make RG6 a digital pin
    ANSELGbits.ANSG6 = 0;

    RPINR25bits.COFSR = 118;// route PIN4/RP118/RG6 to COFS
    RPINR24bits.CSCKR = 87; // route PIN3/RP87/RE7 to CSCK
    RPINR24bits.CSDIR = 86; // route PIN2/RP86/RE6 to CSDI
    RPOR6bits.RP85R = 0b001011;// route CSDO to PIN1/RP85/RE5

// DCI Control Register DCICON1 Initialization
    DCICON1bits.DCISIDL = 0;// module operates in CPU Idle Mode
    DCICON1bits.DLOOP = 0;  // Digital Loopback disabled
    DCICON1bits.CSCKD = 1;  // CSCK is an input
    DCICON1bits.CSCKE = 1;  // data sampled on sck rising edge
    DCICON1bits.COFSD = 1;  // COFS is an input
    DCICON1bits.UNFM = 0;   // xmit 0 on underflow
    DCICON1bits.CSDOM = 0;  // CSDO drives 0 during disabled time slots
#if defined(CODEC_USES_I2S)
    DCICON1bits.DJST = 0;   // xmit/rx begins on serial clock after fsync
#else
    DCICON1bits.DJST = 1;   // xmit/rx begins same serial clock as fsync
#endif
    DCICON1bits.COFSM = 1;  // i2s mode

// DCI Control Register DCICON2 Initialization
    DCICON2bits.BLEN = 3;   // 4 words = 64 bits buffered between ineterupts
    DCICON2bits.COFSG = 1;  // data frame has 4 words = 64 bits
    DCICON2bits.WS = 15;    // data word size is 16 bits

// DCI Control Register DCICON3 Initialization
    DCICON3 = 0;            //serial clock provided externally

// Transmit Slot Control Register Initialization
    TSCONbits.TSE0 = 1;     // Transmit on Time Slot 0
    TSCONbits.TSE1 = 1;	    // Transmit on Time Slot 1

// Receiver Slot Control Register Initialization
    RSCONbits.RSE0 = 1;	    // Receive on Time Slot 0
    RSCONbits.RSE1 = 1;	    // Receive on Time Slot 1

	
// Initialize the TX buffers
    TXBUF0 = 0x0000;
    TXBUF1 = 0x0000;
    TXBUF2 = 0x0000;
    TXBUF3 = 0x0000;

// Enable DCI module
    IPC15bits.DCIIP = 6;    // Set the interrupt priority
    IFS3bits.DCIIF = 0;     // Clear the interrupt flag
    IEC3bits.DCIIE = 1;     // Enable the interrupt
    DCICON1bits.DCIEN = 1;  //DCI Module Enabled
}

void __attribute__((__interrupt__, no_auto_psv)) _DCIInterrupt(void)
{
    uint8_t state;
    
    IFS3bits.DCIIF = 0;

    START_ATOMIC();//begin critical section; must be atomic!
    state = global_state;

    //must call END_ATOMIC() right away in these functions!
    if (state & BALANCE_BRIDGE_FSM_MASK) {
        balanceBridgeFSM();
    } else {
        measurementFSM();
    }
    RETFIE();
}
