/*
 * File:   asmFIR.c
 * Author: Michael Sandstedt
 *
 * Created on December 2, 2013, 7:02 PM
 */

#include "asmFIR.h"
#include "spintronicsIncludes.h"

static const int16_t coef_tbl[FILTER_TAPS] __attribute__((space(ymemory), eds, aligned)) = {0x7FFF, 0x0000};
static int16_t delayLine[FILTER_TAPS] __attribute__((space(xmemory), eds, aligned));

inline int16_t FIR(int16_t input)
{
    static int16_t *dly_ptr = delayLine;
    volatile register int16_t *reg_dly_ptr asm("w9");
    volatile register int16_t *reg_coef_ptr;
    volatile register uint16_t en_modulo = 0x8009;
    volatile register int16_t result;

    reg_dly_ptr = dly_ptr;
    reg_coef_ptr = coef_tbl;




    volatile __asm__("MOV %2, MODCON\n"
                     "MOV %3, [%4]\n"
                     "CLR A, [%4]-=2, w4, [%5]+=2, w5\n"
                     "DO #15, ACCUMULATE\n"
                     "MAC w4*w5, A, [%4]-=2, w4, [%5]+=2, w5\n"
                     "ACCUMULATE: MAC w4*w5, A, [%4]-=2, w4, [%5]+=2, w5\n"
                     "MAC w4*w5, A, [%4]-=2, w4, [%5]+=2, w5\n"
                     "MAC w4*w5, A, [%4], w4, [%5], w5\n"
                     "MAC w4*w5, A\n"
                     "CLR MODCON\n"
                     "MOV ACCAH, %0"
                     : "=r"(result), "+x"(reg_dly_ptr)
                     : "r"(en_modulo), "r"(input), "+x"(reg_dly_ptr), "y"(reg_coef_ptr)
                     : "w4", "w5", "ACCAL", "ACCAH", "ACCAU");

    dly_ptr = reg_dly_ptr;
    return result;
}

void firInit(void)
{
    XMODSRT = (uint16_t)delayLine;
    XMODEND = (uint16_t)(delayLine + FILTER_TAPS - 1);
}