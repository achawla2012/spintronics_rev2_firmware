/*
 * File:   asmFIR.h
 * Author: Michael Sandstedt
 *
 * Created on December 2, 2013, 7:02 PM
 */

#include "spintronicsIncludes.h"

#ifndef ASMFIR_H
#define	ASMFIR_H

#define FILTER_TAPS 6
#define FILTER_TAPS_LESS_3 #3



/*
 * int16_t: the incoming sample
 * int16_t **: ptr to ptr to current location in the delay line
 * int16_t **: ptr to ptr to the first element int he FIR coefficient array
 */
int16_t asmFIR(int16_t, int16_t **, int16_t **);

#endif	/* ASMFIR_H */
