/*
 * asmFIR.s
 *
 * designed and written
 * by Michael Sandstedt
 *
 * First release: Dec 2nd, 2013
 *
 */

#include "p33exxxx.h"
#include "asmFIR.h"

.global _asmFIR
_asmFIR:

MOV #0x8009, w3     ; enable modulo addressing for w9
MOV w3, MODCON
MOV [w1], w9        ; point w9 to the proper index in the delay line
MOV w0, [w9]        ; copy the new sample into the delay line
MOV w2, w11         ; point w11 to the first coefficient in the table
CLR A, [w9]-=2, w4, [w11]+=2, w5 ; clr A, prefetch w4, w5, postdec w9, postinc w11
DO #15, ACCUMULATE ; const must be (FILTER_TAPS - 3) / 2 - 1
MAC w4*w5, A, [w9]-=2, w4, [w11]+=2, w5 ; MAC, prefetch w4/w5, postdec w9, postinc w11
ACCUMULATE: MAC w4*w5, A, [w9]-=2, w4, [w11]+=2, w5 ; MAC, prefetch w4/w5, postdec w9, postinc w11
MAC w4*w5, A, [w9]-=2, w4, [w11]+=2, w5 ; MAC, prefetch w4/w5, postdec w9, postinc w11
MAC w4*w5, A, [w9], w4, [w11], w5   ; MAC, prefetch w4/w5
MAC w4*w5, A        ; MAC
MOV w9, [w1]        ; store delay line ptr for next call
CLR MODCON     ; disable modulo addressing
MOV ACCAH, w0       ; return accumulated result
return
.end
