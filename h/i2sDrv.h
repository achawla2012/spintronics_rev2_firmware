/*
 * i2sDrv.h
 *
 * designed and written
 * by Michael Reinhart Sandstedt
 *
 * First release: May 7th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

// External Functions
extern void i2sInit(void);
extern void __attribute__((__interrupt__)) _DCIInterrupt(void);
