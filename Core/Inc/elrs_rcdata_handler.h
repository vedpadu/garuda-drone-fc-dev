/*
 * elrsToRcData.h
 * Util files for converting ExpressLRS packets to readable RC data
 * Mostly pulled from Betaflight
 *
 *  Created on: Jul 23, 2024
 *      Author: vedpadu
 */

#ifndef INC_ELRS_RCDATA_HANDLER_H_
#define INC_ELRS_RCDATA_HANDLER_H_

#include "stdint.h"
#include "math.h"
#include "expresslrs.h"

#define ROLL_STICK_IND 0
#define PITCH_STICK_IND 1
#define THROTTLE_STICK_IND 2
#define YAW_STICK_IND 3
#define SWITCH_A_IND 4
#define SWITCH_B_IND 5
#define SWITCH_C_IND 6
#define SWITCH_D_IND 7

void unpackChannelDataHybridWide(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr);
uint16_t convertSwitchNb(const uint16_t val, const uint16_t max);
uint16_t convertSwitch1b(const uint16_t val);
void unpackAnalogChannelData(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr);
void hybridWideNonceToSwitchIndex(const uint8_t nonce);


#endif /* INC_ELRS_RCDATA_HANDLER_H_ */
