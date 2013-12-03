/*
 * File:   asmFIR.c
 * Author: Michael Sandstedt
 *
 * Created on December 2, 2013, 7:02 PM
 */

#include "asmFIR.h"
#include "spintronicsIncludes.h"


//static int16_t coef_tbl[FILTER_TAPS];

inline int16_t FIR(int16_t input)
{
    static int16_t delayLine[FILTER_TAPS];
    static int16_t coef_tbl[FILTER_TAPS];
    static bool first = true;
    static int16_t *dly_ptr = delayLine;
    int16_t *coef_ptr = coef_tbl;

    if (first) {

        XMODSRT = (uint16_t)delayLine;
        XMODEND = (uint16_t)(delayLine + FILTER_TAPS - 1);
        first = false;
    }

    return asmFIR(input, &dly_ptr, &coef_ptr);

}