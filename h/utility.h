/* 
 * File:   utility.h
 * Author: Michael R Sandstedt
 *
 * Created on September 15, 2013, 8:21 PM
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

int32_t asm16X16Mult(int16_t, int16_t);
extern inline void START_ATOMIC(void);
extern inline void END_ATOMIC(void);
extern inline void NOP(void);