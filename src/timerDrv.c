/*
 * timerDrv.c
 *
 * Author: Michael Reinhart Sandstedt
 *
 * First release: September 14th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "timer.h"
#include "spintronics.h"
#include "calculateVectors.h"

void timerInit(void)
{
    T1CONbits.TON = 0;// stop Timer1
    T1CONbits.TSIDL = 1; // Discontinue timer operation when device enters Idle mode
    T1CONbits.TGATE = 0; // Gated time accumulation disabled
    T1CONbits.TCKPS = 0b00; // 1:1 prescale value
    T1CONbits.TCS = 0; // Internal clock as source
    TMR1 = 0x00; // clear Timer1
    PR1 = 0xFFFF; // load an arbitrary value for comparison (not 0)
    IPC0bits.T1IP = 0x03; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;
    T1CONbits.TON = 0;
    TMR1 = 0x00;
    calculateFinalVectors();
}

