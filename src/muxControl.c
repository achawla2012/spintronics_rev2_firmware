/*
 * muxControl.c
 *
 * designed and written
 * by Michael Reinhart Sandstedt
 * and Todd Klein
 *
 * First release: May 7th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "spintronics.h"

void muxInit (void)
{
    /*
     * mux connections:
     * A0  <- RD9
     * A1  <- RD10
     * A2  <- RD11
     * A3  <- RD0
     * EN0 <- RD1
     * EN1 <- RD2
     * 
     * None are analog-capable, so no need to set ANSEL
     */

    //set relevant PORTD pins as outputs
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD9 = 0;
    TRISDbits.TRISD10 = 0;
    TRISDbits.TRISD11 = 0;
    
    //make the relevant pins open drain / 5V tolerant
    ODCDbits.ODCD0 = 1;
    ODCDbits.ODCD1 = 1;
    ODCDbits.ODCD2 = 1;
    ODCDbits.ODCD9 = 1;
    ODCDbits.ODCD10 = 1;
    ODCDbits.ODCD11 = 1;

    //clear the pins
    PORTDbits.RD0 = 0;
    PORTDbits.RD1 = 0;
    PORTDbits.RD2 = 0;
    PORTDbits.RD9 = 0;
    PORTDbits.RD10 = 0;
    PORTDbits.RD11 = 0;
}

void configSensor(uint8_t sensor)
{
    if(sensor & 0x01) {
        PORTDbits.RD9 = 1;
    } else {
        PORTDbits.RD9 = 0;
    }

    if(sensor & 0x02) {
        PORTDbits.RD10 = 1;
    } else {
        PORTDbits.RD10 = 0;
    }

    if(sensor & 0x04) {
        PORTDbits.RD11 = 1;
    } else {
        PORTDbits.RD11 = 0;
    }

    if(sensor & 0x08) {
        PORTDbits.RD0 = 1;
    } else {
        PORTDbits.RD0 = 0;
    }

    if(sensor & 0x10) {
        PORTDbits.RD1 = 1;
    } else {
        PORTDbits.RD1 = 0;
    }

    if(sensor & 0x20) {
        PORTDbits.RD2 = 1;
    } else {
        PORTDbits.RD2 = 0;
    }

    //top two bits of sensor address are ignored (not connected in hardware)

}

