/*
 * delay.c
 *
 * designed and written
 * by Michael Reinhart Sandstedt
 *
 * First release: September 10th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "spintronics.h"
#include <TIME.H>

void busy_wait_ms(uint16_t ms)
{
    clock_t start, stop, elapsed;
    int32_t cyclesToSleep = PROCESSOR_CYCLES_PER_MS * ms - 87;//subtract cycles to compensate for multiplication and funciton call overhead

    if (cyclesToSleep > 0)
    {
        start = clock();
        do
        {
            stop = clock();
            if (start < stop)
            {
                elapsed = stop - start;
            }
            else
            {
                elapsed = (clock_t)((uint32_t)stop - (uint32_t)start);
            }
        } while (elapsed < cyclesToSleep);
    }
}
