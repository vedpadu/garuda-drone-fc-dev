/*
 * elrsToRcData.h
 * Util files for converting ExpressLRS packets to readable RC data
 * Mostly pulled from Betaflight
 *
 *  Created on: Jul 23, 2024
 *      Author: vedpadu
 */

#ifndef INC_ELRSTORCDATA_H_
#define INC_ELRSTORCDATA_H_

#include "stdint.h"
#include "math.h"
#include "expresslrs.h"

void unpackChannelDataHybridWide(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr);
uint16_t convertSwitchNb(const uint16_t val, const uint16_t max);
uint16_t convertSwitch1b(const uint16_t val);
void unpackAnalogChannelData(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr);
void hybridWideNonceToSwitchIndex(const uint8_t nonce);


#endif /* INC_ELRSTORCDATA_H_ */
