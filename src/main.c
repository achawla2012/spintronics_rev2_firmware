/*
 * main.c
 *
 * designed and written
 * by the Spintronics Senior Design Embedded Team:
 *     Jonathan Pechuman
 *     Hajime Makino
 *     Samiha Sultana
 *     Michael Sandstedt
 *
 * First release: May 7th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "spintronicsIncludes.h"
#include "spintronicsConfig.h"
#include "timerDrv.h"
#include "spiTx.h"
#include "CS4272.h"
#include "muxControl.h"
#include "uartDrv.h"

#ifdef SIMULATION_MODE
#include "balanceBridge.h"
#include "fsmStates.h"
#endif

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC & IESO_OFF);                 // OSC2 Pin Function: OSC2 is Clock Output                                                        // Primary Oscillator Mode: Disabled
// Enable Clock Switching and Configure POSC in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FWDT(FWDTEN_OFF);                              // Watchdog Timer Enabled/disabled by user software
						// (LPRC can be disabled by clearing SWDTEN bit in RCON register
_FPOR(FPWRT_PWR1 & ALTI2C1_ON );  		// Turn off the power-up timers.
//_FGS(GCP_OFF);                                // Disable Code Protection

int main(void)
{
    // Configure Oscillator to operate the device at 140Mhz
    CLKDIVbits.PLLPRE = 0;
    CLKDIVbits.PLLPOST = 0;
    PLLFBDbits.PLLDIV = 110;        //PLL 5MHz to 140 MHz
	
    // Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);
	
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC!= 0b011);
    // Wait for PLL to lock
    while (OSCCONbits.LOCK!= 1);
	
    // Disable Watch Dog Timer
    RCONbits.SWDTEN=0;

    //initialize components; order is important; DO NOT CHANGE
    timerInit();//initialize the timer
    spiInit();// init SPI for controlling digi pots
    cs4272Init();// establish communication with the CS4272
    muxInit();// setup pins to communicate with the multiplexer
    uart_Init();// Init UART for GUI communication
    
#ifdef SIMULATION_MODE
    processStartCommand();
    global_state = RAMP_DOWN_COIL_RESTART;
    sensorRBridgeTableValid = true;
#endif
    while(1)
    {
#ifdef SIMULATION_MODE
        measurementFSM();
        //balanceBridgeFSM();
#endif
    }
}
