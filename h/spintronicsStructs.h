/* 
 * File:   spintronics_structs.h
 * Author: Michael R Sandstedt
 *
 * Created on November 28, 2013, 11:29 AM
 */

#ifndef SPINTRONICS_CONFIG_H
#include "spintronicsConfig.h"
#endif

#ifndef SPINTRONICS_STRUCTS_H
#define	SPINTRONICS_STRUCTS_H

typedef struct accumulator_s {
    int64_t bridge_f1;
#ifdef MEASURE_F2_AT_BRIDGE
    int64_t bridge_f2;
#endif
    int64_t bridge_fdiff;
    int64_t bridge_fsum;
#ifdef MEASURE_F2_AT_COIL
    int64_t coil_f2;
#endif
} accumulator_t;

typedef struct float_array_s {
    float bridge_f1;
#ifdef MEASURE_F2_AT_BRIDGE
    float bridge_f2;
#endif
    float bridge_fdiff;
    float bridge_fsum;
#ifdef MEASURE_F2_AT_COIL
    float coil_f2;
#endif
} float_array_t;

typedef struct double_array_s {
    double bridge_f1;
#ifdef MEASURE_F2_AT_BRIDGE
    double bridge_f2;
#endif
    double bridge_fdiff;
    double bridge_fsum;
#ifdef MEASURE_F2_AT_COIL
    double coil_f2;
#endif
} double_array_t;

#endif	/* SPINTRONICS_STRUCTS_H */

