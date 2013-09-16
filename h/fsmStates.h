/* 
 * File:   fsmStates.h
 * Author: Michael R Sandstedt
 *
 * Created on September 15, 2013, 8:11 PM
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

//states for bridgeBalanceFSM() must have bit 7 set!
#define BALANCE_BRIDGE_FSM_MASK                 0x40
#define IDLE                                    0x40
#define START_BRIDGE_BALANCE_FSM                0x41
#define START_GAIN_CAL                          0x42
#define SET_R_AMP_TO_LO_MID                     0x43
#define R_AMP_LO_MID_SIGNAL_SETTLING            0x44
#define R_AMP_LO_MID_CLIP_TEST                  0x45
#define SET_R_AMP_TO_HI_MID                     0x46
#define R_AMP_HI_MID_SIGNAL_SETTLING            0x47
#define R_AMP_HI_MID_CLIP_TEST                  0x48
#define START_BRIDGE_CAL                        0x49
#define SET_R_BRIDGE_TO_LO_MID                  0x4A
#define R_BRIDGE_LO_MID_SIGNAL_SETTLING         0x4B
#define R_BRIDGE_LO_MID_AMP_MEASURE             0x4C
#define R_BRIDGE_LO_MID_AMP_CALC                0x4D
#define SET_R_BRIDGE_TO_HI_MID                  0x4E
#define R_BRIDGE_HI_MID_SIGNAL_SETTLING         0x4F
#define R_BRIDGE_HI_MID_AMP_MEASURE             0x50
#define R_BRIDGE_HI_MID_AMP_CALC                0x51

//states for measurementFSM() must have bit 8 set!
#define MEASUREMENT_FSM_MASK                    0x80
#define START_MEASUREMENT_FSM                   0x81
#define RAMP_UP_COIL                            0x82
#define START_NEW_MEASUREMENT_CYCLE             0x83
#define WAIT_FOR_COIL_0RAD                      0x84
#define START_SIGNAL_GEN                        0x85
#define	MEASURE                                 0x86
#define	CALCULATE_VECTORS                       0x87
#define RAMP_DOWN_COIL_QUIT                     0x88
#define RAMP_DOWN_COIL_RESTART                  0x89

//signal generator state encoding
#define RESET_SIGNAL_GEN 0
#define RESET_BRIDGE_GEN 1
#define RUN_SIGNAL_GEN 2
