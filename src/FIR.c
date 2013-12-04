/*
 * File:   asmFIR.c
 * Author: Michael Sandstedt
 *
 * Created on December 2, 2013, 7:02 PM
 */

#include "spintronicsIncludes.h"

#define EVEN_ORDER_FILTER
#define FILTER_TAPS 150

static const int16_t coef_tbl[FILTER_TAPS] __attribute__((space(ymemory), eds, aligned)) = {0x7FFF, 0x0000};
static int16_t delayLine[FILTER_TAPS] __attribute__((space(xmemory), eds, aligned));

inline int16_t FIR(register int16_t input)
{
    static int16_t *dly_ptr = delayLine;
    volatile register int16_t *reg_dly_ptr asm("w9");
    volatile register int16_t *reg_coef_ptr asm("w11");
    register uint16_t en_modulo = 0x8009;
    volatile register int16_t result;

    reg_dly_ptr = dly_ptr;
    reg_coef_ptr = (int16_t *)coef_tbl;

#ifdef EVEN_ORDER_FILTER
    __asm__ volatile ("MOV %4, MODCON\n" // enable modulo addressing for w9
                      "MOV %2, [%1]\n" // copy the new sample into the delay line
                      "CLR A, [%1]-=2, w4, [%3]+=2, w5\n" // clr A, prefetch w4, w5, postdec w9, postinc w11
                      "DO #73, ACCUMULATE\n" // ulit must be (FILTER_TAPS - 2) / 2 - 1
                      "MAC w4*w5, A, [%1]-=2, w4, [%3]+=2, w5\n" // MAC, prefetch w4/w5, postdec w9, postinc w11
                      "ACCUMULATE: MAC w4*w5, A, [%1]-=2, w4, [%3]+=2, w5\n" // MAC, prefetch w4/w5, postdec w9, postinc w11
                      "MAC w4*w5, A, [%1], w4, [%3], w5\n" // MAC, prefetch w4/w5
                      "MAC w4*w5, A\n" // MAC
                      "CLR MODCON\n" // disable modulo addressing
                      "MOV ACCAH, %0" // return result
                      : "=r"(result), "+x"(reg_dly_ptr)
                      : "r"(input), "y"(reg_coef_ptr), "r"(en_modulo)
                      : "w4", "w5");
#else
    __asm__ volatile ("MOV %4, MODCON\n" // enable modulo addressing for w9
                      "MOV %2, [%1]\n" // copy the new sample into the delay line
                      "CLR A, [%1]-=2, w4, [%3]+=2, w5\n" // clr A, prefetch w4, w5, postdec w9, postinc w11
                      "DO #28, ACCUMULATE\n" // ulit must be (FILTER_TAPS - 3) / 2 - 1
                      "MAC w4*w5, A, [%1]-=2, w4, [%3]+=2, w5\n" // MAC, prefetch w4/w5, postdec w9, postinc w11
                      "ACCUMULATE: MAC w4*w5, A, [%1]-=2, w4, [%3]+=2, w5\n" // MAC, prefetch w4/w5, postdec w9, postinc w11
                      "MAC w4*w5, A, [%1]-=2, w4, [%3]+=2, w5\n" // MAC, prefetch w4/w5, postdec w9, postinc w11
                      "MAC w4*w5, A, [%1], w4, [%3], w5\n" // MAC, prefetch w4/w5
                      "MAC w4*w5, A\n" // MAC
                      "CLR MODCON\n" // disable modulo addressing
                      "MOV ACCAH, %0" // return result
                      : "=r"(result), "+x"(reg_dly_ptr)
                      : "r"(input), "y"(reg_coef_ptr), "r"(en_modulo)
                      : "w4", "w5");
#endif
    dly_ptr = (int16_t *)reg_dly_ptr;
    return result;
}

void firInit(void)
{
    XMODSRT = (uint16_t)delayLine;
    XMODEND = (uint16_t)(delayLine + FILTER_TAPS - 1);
}