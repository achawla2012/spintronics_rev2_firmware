#include "p33exxxx.h"

inline void START_ATOMIC(void)
{
    //call this twice in case an interrupt occurs during the first call
    __asm__ volatile ("DISI #0x3FFF");
    __asm__ volatile ("DISI #0x3FFF");
}

inline void END_ATOMIC(void)
{
    DISICNT = 0;
}

inline void NOP(void)
{
    __asm__ volatile ("nop");
}
