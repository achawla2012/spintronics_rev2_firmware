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

#ifndef DIGI_POT_DRV_H
#define DIGI_POT_DRV_H

//see getRBridgeOhms(uint16_t val) for corresponding values in ohms
#define R_BRIDGE_MAX 1535
#define R_BRIDGE_MIN 0
#define R_BRIDGE_MID 768

//see getRAmpOhms(uint8_t val) for corresponding values in ohms
#define R_AMP_MAX 255
#define R_AMP_MIN 0
#define R_AMP_MID 128

extern void setRBridge(uint16_t val);
extern void setRAmp(uint8_t val);
extern inline float getBridgeBufGainFromU24Code(uint8_t u24_code);
extern inline float getBridgeInverseGainFromU24Code(uint8_t u24_code);
extern inline uint8_t getU24CodeFromBrdigeBufGain(float bridge_gain);

#endif