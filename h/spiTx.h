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

//addresses for U20, U23 and U24 to be used for spiTx();
#define U20 0x01
#define U23 0x02
#define U24 0x03

extern void spiInit(void);
extern void spiTx(uint8_t addr, uint8_t numBytes, __eds__ uint8_t *pl);