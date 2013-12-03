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
    return asmFIR(input, &dly_ptr, coef_tbl);
}

void firInit(void)
{
    XMODSRT = (uint16_t)delayLine;
    XMODEND = (uint16_t)(delayLine + FILTER_TAPS - 1);
}