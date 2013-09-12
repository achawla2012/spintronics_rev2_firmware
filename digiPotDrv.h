/*
 * digiPotDrv.h
 *
 * Author: Michael Reinhart Sandstedt
 *
 * First release: September 11th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

extern void setRBridge(uint16_t val);
extern void setRAmp(uint8_t val);
extern float getRBridgeOhms(uint16_t val);
extern float getRAmpeOhms(uint8_t val);
