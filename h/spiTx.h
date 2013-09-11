/*
 * spiTx.h
 *
 * designed and written
 * by Michael Reinhart Sandstedt
 *
 * First release: September 10th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

extern void spiInit(void);
extern void spiTx(uint8_t addr, uint8_t numBytes, uint8_t *pl);