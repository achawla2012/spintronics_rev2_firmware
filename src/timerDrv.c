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
#include "spintronicsIncludes.h"
#include "timer.h"
#include "calculateVectors.h"

#define PROCESSOR_CYCLES_PER_MS 70000

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

    T3CONbits.TON = 0; // Stop any 16-bit Timer3 operation
    T2CONbits.TON = 0; // Stop any 16/32-bit Timer3 operation
    T2CONbits.T32 = 1; // Enable 32-bit Timer mode
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR3 = 0x00; // Clear 32-bit Timer (msw)
    TMR2 = 0x00; // Clear 32-bit Timer (lsw)
    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 1; // Disable Timer3 interrupt

}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;
    T1CONbits.TON = 0;
    TMR1 = 0x00;
    calculateFinalVectors();
}

void busy_wait_ms(uint16_t ms)
{
    uint32_t countTarget = PROCESSOR_CYCLES_PER_MS * ms - 87;//subtract cycles to compensate for multiplication and funciton call overhead
    uint32_t count;

    if (countTarget > 0)
    {
        TMR3 = 0x00; // Clear 32-bit Timer (msw)
        TMR2 = 0x00; // Clear 32-bit Timer (lsw)

        PR2 = countTarget & 0xFFFF;
        PR3 = *((__eds__ uint16_t *)&countTarget + 1);
        T2CONbits.TON = 1;

        do {
            count = TMR2;
            *((__eds__ uint16_t *)&count + 1) = TMR3HLD;
        } while (count < countTarget);

        T2CONbits.TON = 0;
    }
}

