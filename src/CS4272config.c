/*
 * CS4272config.c
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
#include "spintronicsConfig.h"
#include "timerDrv.h"

#define CS4272_RST_BAR PORTEbits.RE0

void cs4272Init(void)
{
    // Setup RE1 for connection to RSTbar on CS4272
    ANSELEbits.ANSE0 = 0; //set RE0 to be a digital pin
    TRISEbits.TRISE0 = 0; // set RE0 to be an output

#ifndef SIMULATION_MODE
    CS4272_RST_BAR = 0;
    busy_wait_ms(100); // wait 100ms
    CS4272_RST_BAR = 1;
    busy_wait_ms(1); // wait 2ms
#endif
    i2sInit();//initialize the DCI module for communication with CS4272
}