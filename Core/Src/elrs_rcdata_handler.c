/*
 * elrsToRcData.c
 * Converts Express LRS packets to readable data
 *
 *  Created on: Jul 23, 2024
 *      Author: vedpadu
 */
#include <elrs_rcdata_handler.h>

uint8_t wide_switch_index = 0;

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
    const uint8_t switch_byte = otaPktPtr->rc.switches;

    if (wide_switch_index < 7){
        uint8_t bins;
        uint16_t switch_value;
		bins = 127;
		switch_value = switch_byte & 0x7F; // 7-bit


        rcData[SWITCH_B_IND + wide_switch_index] = convertSwitchNb(switch_value, bins);
    }
}

void unpackAnalogChannelData(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr)
{
    const uint8_t num_of_channels = 4;
    const uint8_t src_bits = 10;
    const uint16_t input_channel_mask = (1 << src_bits) - 1;

    uint8_t bits_merged = 0;
    uint32_t read_value = 0;
    uint8_t read_byte_index = 0;
    for (uint8_t n = 0; n < num_of_channels; n++) {
        while (bits_merged < src_bits) {
            uint8_t read_byte = otaPktPtr->rc.ch[read_byte_index++];
            read_value |= ((uint32_t) read_byte) << bits_merged;
            bits_merged += 8;
        }
        rcData[n] = 988 + (read_value & input_channel_mask);
        read_value >>= src_bits;
        bits_merged -= src_bits;
    }

    // The low latency switch -> arming switch
    rcData[SWITCH_A_IND] = convertSwitch1b(otaPktPtr->rc.ch4);
}

// on off switch (switch C and switch D)
uint16_t convertSwitch1b(const uint16_t val)
{
    return val ? 2000 : 1000;
}

// 3 state switch (switch A and switch B)
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
    wide_switch_index = ((nonce & 0x07) + ((nonce >> 3) & 0x01)) % 8;
}
