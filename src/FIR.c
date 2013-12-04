/*
 * File:   asmFIR.c
 * Author: Michael Sandstedt
 *
 * Created on December 2, 2013, 7:02 PM
 */

#include "spintronicsIncludes.h"

#define EVEN_ORDER_FILTER
#define FILTER_TAPS 160

static const int16_t coef_tbl[FILTER_TAPS] __attribute__((space(ymemory), eds, aligned)) = {1, 1, 1, 1, 0, -1, -5, -9, -14, -21, -28, -36, -44, -51, -57, -62, -65, -64, -60, -53, -42, -28, -11, 8, 28, 48, 67, 83, 96, 104, 106, 104, 96, 84, 68, 51, 34, 20, 10, 6, 10, 24, 47, 79, 118, 163, 209, 254, 292, 320, 332, 325, 295, 240, 160, 55, -72, -217, -375, -538, -697, -844, -969, -1065, -1123, -1137, -1102, -1017, -883, -702, -481, -228, 47, 332, 614, 881, 1120, 1318, 1468, 1560, 1592, 1560, 1468, 1318, 1120, 881, 614, 332, 47, -228, -481, -702, -883, -1017, -1102, -1137, -1123, -1065, -969, -844, -697, -538, -375, -217, -72, 55, 160, 240, 295, 325, 332, 320, 292, 254, 209, 163, 118, 79, 47, 24, 10, 6, 10, 20, 34, 51, 68, 84, 96, 104, 106, 104, 96, 83, 67, 48, 28, 8, -11, -28, -42, -53, -60, -64, -65, -62, -57, -51, -44, -36, -28, -21, -14, -9, -5, -1, 0, 1, 1, 1};
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
                      "DO #78, ACCUMULATE\n" // ulit must be (FILTER_TAPS - 2) / 2 - 1
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