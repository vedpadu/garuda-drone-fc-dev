/*
 * elrsToRcData.c
 * Converts Express LRS packets to readable data
 *
 *  Created on: Jul 23, 2024
 *      Author: vedpadu
 */
#include "elrsToRcData.h"

uint8_t wideSwitchIndex = 0; // necessary?

/**
 * HybridWide switches decoding of over the air data
 *
 * Hybrid switches uses 10 bits for each analog channel,
 * 1 bits for the low latency switch[0]
 * 6 or 7 bits for the round-robin switch
 * 1 bit for the TelemetryStatus, which may be in every packet or just idx 7
 * depending on TelemetryRatio
 *
 * Output: crsf.PackedRCdataOut, crsf.LinkStatistics.uplink_TX_Power
 * Returns: TelemetryStatus bit
 */
void unpackChannelDataHybridWide(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr)
{
    unpackAnalogChannelData(rcData, otaPktPtr);
    const uint8_t switchByte = otaPktPtr->rc.switches;

    if (wideSwitchIndex >= 7) {
        //txPower = switchByte & 0x3F;
    } else {
        uint8_t bins;
        uint16_t switchValue;
		bins = 127;
		switchValue = switchByte & 0x7F; // 7-bit


        rcData[5 + wideSwitchIndex] = convertSwitchNb(switchValue, bins);
    }
}

void unpackAnalogChannelData(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr)
{
    const uint8_t numOfChannels = 4;
    const uint8_t srcBits = 10;
    const uint16_t inputChannelMask = (1 << srcBits) - 1;

    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    uint8_t readByteIndex = 0;
    for (uint8_t n = 0; n < numOfChannels; n++) {
        while (bitsMerged < srcBits) {
            uint8_t readByte = otaPktPtr->rc.ch[readByteIndex++];
            readValue |= ((uint32_t) readByte) << bitsMerged;
            bitsMerged += 8;
        }
        rcData[n] = 988 + (readValue & inputChannelMask);
        readValue >>= srcBits;
        bitsMerged -= srcBits;
    }

    // The low latency switch
    rcData[4] = convertSwitch1b(otaPktPtr->rc.ch4);
}

uint16_t convertSwitch1b(const uint16_t val)
{
    return val ? 2000 : 1000;
}

uint16_t convertSwitchNb(const uint16_t val, const uint16_t max)
{
    return (val > max) ? 1500 : val * 1000 / max + 1000;
}

void hybridWideNonceToSwitchIndex(uint8_t nonce)
{
    // Returns the sequence (0 to 7, then 0 to 7 rotated left by 1):
    // 0, 1, 2, 3, 4, 5, 6, 7,
    // 1, 2, 3, 4, 5, 6, 7, 0
    // Because telemetry can occur on every 2, 4, 8, 16, 32, 64, 128th packet
    // this makes sure each of the 8 values is sent at least once every 16 packets
    // regardless of the TLM ratio
    // Index 7 also can never fall on a telemetry slot
    wideSwitchIndex = ((nonce & 0x07) + ((nonce >> 3) & 0x01)) % 8;
}
