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

//see getRBridgeOhms(uint16_t val) for corresponding values in ohms
#define R_BRIDGE_MAX 1535//actually corresponds to Rmin
#define R_BRIDGE_MIN 0//actually corresponds to Rmax
#define R_BRIDGE_MID 768

//see getRAmpOhms(uint8_t val) for corresponding values in ohms
#define R_AMP_MAX 255//actually corresponds to Rmin
#define R_AMP_MIN 0//actually corresponds to Rmax
#define R_AMP_MID 128

extern void setRBridge(uint16_t val);
extern void setRAmp(uint8_t val);
extern float getRBridgeOhms(uint16_t val);
extern float getRAmpeOhms(uint8_t val);
