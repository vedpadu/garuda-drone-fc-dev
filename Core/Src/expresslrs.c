/*
 * expresslrs.c
 * expresslrs frequency hopping and packet decoding
 *
 *  Created on: Jul 18, 2024
 *      Author: vedpadu
 */

#include "elrs_rcdata_handler.h"
#include "expresslrs.h"

elrsReceiver_t receiver;
elrsPhase_t phase_locker;
uint8_t binding_UID[6] = {0,1,2,3,4,5}; // seed for binding
uint8_t UID[6];

uint8_t rc_payload[8] = {0};

uint8_t fhss_sequence[ELRS_NR_SEQUENCE_ENTRIES] = {0};
uint8_t fhss_index = 0;
static uint16_t seq_count = 0;
static uint8_t sync_channel = 0;
static uint32_t freq_spread = 0;
uint32_t seed = 0;

// FHSS Configs for the SX1280
// start and stop frequencies are pulled from betaflight
uint32_t freq_start = (uint32_t)((2400400000)/ SX1280_PLL_STEP);
uint32_t freq_stop = (uint32_t)((2479400000)/ SX1280_PLL_STEP);
uint8_t freq_count = 80;

// ripped off betaflight -> settings for each frequency the radio can transmit at.
	elrsModSettings_t air_rate_config[4] = {
	{0, 500, LORA_BW_800, LORA_SF_5, LORA_CR_LI_4_6, 2000, 4, 12},
	{1, 250, LORA_BW_800, LORA_SF_6, LORA_CR_LI_4_8, 4000, 4, 14},
	{2, 150, LORA_BW_800, LORA_SF_7, LORA_CR_LI_4_8, 6666, 4, 12},
	{3, 50, LORA_BW_800, LORA_SF_8, LORA_CR_LI_4_8, 20000, 2, 12} // bind mode rate index
};

void initExpressLRS(){
	HAL_TIM_Base_Start_IT(htim_elrs);

	initFlashMemoryConfig(2);
	uint8_t init_configs[8] = {0x00};
	readCurrentConfig(init_configs);
	// reading config
	UID[0] = init_configs[0];
	UID[1] = init_configs[1];
	UID[2] = init_configs[2];
	UID[3] = init_configs[3];
	UID[4] = init_configs[4];
	UID[5] = init_configs[5];
	receiver.rate_index = init_configs[6];
	receiver.switch_mode = init_configs[7];

	// no UID in flash memory
	if(UID[0] == 0 && UID[1] == 0 && UID[2] == 0 && UID[3] == 0 && UID[4] == 0 && UID[5] == 0){
		setBindingMode();
	}else{
		refreshExpressLRS(receiver.rate_index);
	}
}

// deals with frequency hopping and LORA params
void refreshExpressLRS(uint8_t newIndex){
	fhssGenSequence(UID);
	receiver.nonce_RX = 0;
	receiver.connected = ELRS_DISCONNECTED;
	receiver.nonce_disconnected = 0;

	uint32_t time_micros = micros();
	phase_locker.last_clock_time_micros = time_micros;
	phase_locker.last_packet_time_micros = time_micros; // reset so we don't have crazy long differences
	phase_locker.raw_phase_diff = 0;

	receiver.current_freq = fhssGetInitialFreq();
	sx1280_init(); // necessary in refresh?

	changeRateIndex(newIndex, receiver.current_freq, UID[5]);
}

// run on sequence a little before when we think a packet should arrive,
// so that we can change the frequency to ensure that we get all packets
void clockPhaseUpdate(uint32_t timeMicros){
	if(!isDisconnected()){
		phase_locker.raw_phase_diff = getDeltaTime(phase_locker.last_clock_time_micros, phase_locker.last_packet_time_micros);
		if(phase_locker.raw_phase_diff > DC_TIMEOUT_MICROS){
			displayInt("DCED", 1);
			disconnect(timeMicros);
			return;
		}

		int32_t default_interval = (int32_t)air_rate_config[receiver.rate_index].interval;
		int32_t offset;
		phase_locker.raw_phase_diff = phase_locker.raw_phase_diff % default_interval;
		int32_t phase_signed = (int32_t)phase_locker.raw_phase_diff;

		offset = default_interval/2 - phase_signed; // need to be half out of phase so that the interrupts do not coliide!!
		offset = intClamp(offset, default_interval/MAX_SHIFT_SCALE, -default_interval/MAX_SHIFT_SCALE); // don't phase too much.

		if(receiver.connected == ELRS_TENTATIVE && absInt(offset) < 50){
			receiver.connected = ELRS_CONNECTED;
			displayInt("CONNECTED!", 1);
		}
		if(offset > LOCKED_ON_THRESHOLD || offset < -LOCKED_ON_THRESHOLD){
			__HAL_TIM_SET_AUTORELOAD(htim_elrs, default_interval + (offset * OFFSET_SHIFT_KP) - 1);
		}else{
			// don't phase if already locked on
			__HAL_TIM_SET_AUTORELOAD(htim_elrs, default_interval - 1);
		}
		phase_locker.offset = offset;
	}else{
		receiver.nonce_disconnected++;
		phase_locker.last_clock_time_micros = timeMicros;
		phase_locker.last_packet_time_micros = timeMicros;
		if(receiver.nonce_disconnected > (air_rate_config[receiver.rate_index].rate * 11)/10){
			receiver.rate_index = (receiver.rate_index + 1) % 4;
			refreshExpressLRS(receiver.rate_index);
		}
	}
}

// run when a packet is actually received
void processRFPacket(uint8_t* packet, uint32_t timeMicros){
	elrsOtaPacket_t * const ota_pkt_ptr = (elrsOtaPacket_t * const) packet;
	switch(ota_pkt_ptr->type) {
	    case ELRS_SYNC_PACKET:
	    	processSyncPacket(ota_pkt_ptr, timeMicros);
		    break;
	    case ELRS_RC_DATA_PACKET:
	    	if(receiver.switch_mode == 0){
	    		hybridWideNonceToSwitchIndex(receiver.nonce_RX);
	    	}
	    	memcpy((uint8_t *) rc_payload, (uint8_t *) packet, 8);
		    break;
	    case ELRS_MSP_DATA_PACKET:
	    	if (receiver.in_binding_mode && ota_pkt_ptr->msp_ul.package_index == 1 && ota_pkt_ptr->msp_ul.payload[0] == ELRS_MSP_BIND) {
			    uint8_t* packy = (uint8_t*)&ota_pkt_ptr->msp_ul.payload[1];
			    processBindPacket(packy);
			    char* data4 = "BIND PACK\n";
			    CDC_Transmit_FS((uint8_t *)data4, strlen(data4));
		    }
		    break;
	    default:
	    	break;
	}
}

void processBindPacket(uint8_t* packet){
	UID[2] = packet[0];
	UID[3] = packet[1];
	UID[4] = packet[2];
	UID[5] = packet[3];

	receiver.in_binding_mode = 0;
	receiver.rate_index = 0; // default value, will search for connection after
	writeCurrentConfigsToFlash();
	refreshExpressLRS(receiver.rate_index);
}

uint8_t processSyncPacket(elrsOtaPacket_t * const otaPktPtr, uint32_t timeMicros){
	// Verify the first two of three bytes of the binding ID, which should always match
	if (otaPktPtr->sync.UID3 != UID[3] || otaPktPtr->sync.UID4 != UID[4]) {
		return 0;
	}
	uint8_t config_write_needed = 0;

	receiver.next_rate_index = airRateIndexToIndex24(otaPktPtr->sync.rate_index, receiver.rate_index);
	if(receiver.next_rate_index != receiver.rate_index){
		config_write_needed = 1;
		receiver.rate_index = receiver.next_rate_index;
	}

	// need to update switch mode
	uint8_t switch_enc_mode = otaPktPtr->sync.switch_enc_mode;

	if(switch_enc_mode != receiver.switch_mode){
		receiver.switch_mode = switch_enc_mode;
		config_write_needed = 1;
	}

	if(config_write_needed){
		writeCurrentConfigsToFlash();
	}

	if (receiver.nonce_RX != otaPktPtr->sync.nonce || fhss_index != otaPktPtr->sync.fhss_index || receiver.connected == ELRS_DISCONNECTED) {
		fhss_index = (otaPktPtr->sync.fhss_index) % seq_count;
		receiver.nonce_RX = otaPktPtr->sync.nonce;

		if(receiver.connected == ELRS_DISCONNECTED){
			tentativeConnection(timeMicros);
		}

	}
	return 1;
}

void tentativeConnection(uint32_t timeMicros){
	receiver.connected = ELRS_TENTATIVE;

	phase_locker.last_clock_time_micros = timeMicros;
	phase_locker.last_packet_time_micros = timeMicros;
	phase_locker.raw_phase_diff = 0;
}

void disconnect(uint32_t timeMicros){
	receiver.connected = ELRS_DISCONNECTED;
	phase_locker.last_clock_time_micros = timeMicros;
	phase_locker.last_packet_time_micros = timeMicros; // reset so we don't have crazy long differences
	receiver.nonce_RX = 0;
	receiver.nonce_disconnected = 0;
	setPrescaleForRateIndex(receiver.rate_index);
	phase_locker.raw_phase_diff = 0;

	receiver.current_freq = fhssGetInitialFreq();
	sx1280_write_RF_frequency(receiver.current_freq);
}

void writeCurrentConfigsToFlash(){
	uint8_t buf[8] = {UID[0], UID[1], UID[2], UID[3], UID[4], UID[5], receiver.rate_index, receiver.switch_mode};
	writeNewConfig(buf);
}

void changeRateIndex(uint8_t newIndex, uint32_t freq, uint8_t uid5){
	receiver.rate_index = newIndex;
	elrsModSettings_t newSettings = air_rate_config[receiver.rate_index];
	receiver.mod_params = newSettings;
	setPrescaleForRateIndex(newIndex);
	sx1280_set_RF_rate(newSettings.sf, newSettings.bw, newSettings.cr, newSettings.preamble_len, freq, uid5 & 0x01);
}

void expressLrsSetRcDataFromPayload(uint16_t *rcData)
{
	volatile elrsOtaPacket_t * const ota_pkt_ptr = (elrsOtaPacket_t * const) rc_payload;
	unpackChannelDataHybridWide(rcData, ota_pkt_ptr);
}

void setBindingMode(){
	if(!receiver.in_binding_mode){
		receiver.in_binding_mode = 1;
		memcpy(UID, binding_UID, 6);

		refreshExpressLRS(BIND_RATE_INDEX);
	}
}

void exitBindMode(){
	receiver.in_binding_mode = 0;
	receiver.rate_index = 1;
	refreshExpressLRS(receiver.rate_index);
}

uint8_t isDisconnected(){
	if(receiver.connected == ELRS_DISCONNECTED){
		return 1;
	}
	return 0;
}

uint8_t isBindingMode(){
	return receiver.in_binding_mode;
}

void setPrescaleForRateIndex(uint8_t index){
	__HAL_TIM_SET_AUTORELOAD(htim_elrs, air_rate_config[index].interval - 1);
}

void setLastClockTime(uint32_t timeMicros){
	phase_locker.last_clock_time_micros = timeMicros;
}

void setLastPacketTime(uint32_t timeMicros){
	phase_locker.last_packet_time_micros = timeMicros;
}

/**
 *
 * Taken and very slightly modified from Betaflight
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pseudorandom

Approach:
  Fill the sequence array with the sync channel every FHSS_FREQ_CNT
  Iterate through the array, and for each block, swap each entry in it with
  another random entry, excluding the sync channel.

  The UID of the receiver is used as the seed

  The domain of the frequency is ignored as compared to Betaflight as the SX1280's settings do not vary
  based on the domain.

*/
void fhssGenSequence(const uint8_t inputUID[])
{
    seed = (((long)inputUID[2] << 24) + ((long)inputUID[3] << 16) + ((long)inputUID[4] << 8) + inputUID[5]) ^ ELRS_OTA_VERSION_ID;
    seq_count = (256 / maxf(freq_count, 1)) * freq_count;
    sync_channel = (freq_count / 2) + 1;
    freq_spread = (freq_stop - freq_start) * ELRS_FREQ_SPREAD_SCALE / maxf((freq_count - 1), 1);

    // initialize the sequence array
    for (uint16_t i = 0; i < seq_count; i++) {
        if (i % freq_count == 0) {
            fhss_sequence[i] = sync_channel;
        } else if (i % freq_count == sync_channel) {
            fhss_sequence[i] = 0;
        } else {
            fhss_sequence[i] = i % freq_count;
        }
    }

    for (uint16_t i = 0; i < seq_count; i++) {
        // if it's not the sync channel
        if (i % freq_count != 0) {
            uint8_t offset = (i / freq_count) * freq_count; // offset to start of current block
            uint8_t rand = rngN(freq_count - 1) + 1; // random number between 1 and numFreqs

            // switch this entry and another random entry in the same block
            uint8_t temp = fhss_sequence[i];
            fhss_sequence[i] = fhss_sequence[offset + rand];
            fhss_sequence[offset + rand] = temp;
        }
    }
}

uint8_t doFhssIrq(){
	receiver.nonce_RX += 1;
	uint8_t mod_result_FHSS = (receiver.nonce_RX) % receiver.mod_params.fhss_hop_interval;

	if((receiver.in_binding_mode == 0) && mod_result_FHSS == 0 && !isDisconnected()){
		receiver.current_freq = fhssGetNextFreq();
		sx1280_write_RF_frequency(receiver.current_freq);
		return 1;
	}
	return 0;
}

uint32_t fhssGetInitialFreq()
{
    return freq_start + (sync_channel * freq_spread / ELRS_FREQ_SPREAD_SCALE);
}

uint32_t fhssGetNextFreq()
{
    fhss_index = (fhss_index + 1) % seq_count;
    uint32_t freq = freq_start + (freq_spread * fhss_sequence[fhss_index] / ELRS_FREQ_SPREAD_SCALE);
    return freq;
}


uint8_t rngN(const uint8_t max)
{
    const uint32_t m = 2147483648;
    const uint32_t a = 214013;
    const uint32_t c = 2531011;
    seed = (a * seed + c) % m;
    return (seed >> 16) % max;
}

uint8_t airRateIndexToIndex24(uint8_t airRate, uint8_t currentIndex)
{
    switch (airRate) {
    case 0:
        return currentIndex;
    case 1:
        return currentIndex;
    case 2:
        return currentIndex;
    case 3:
        return currentIndex;
    case 4:
        return 0;
    case 5:
        return currentIndex;
    case 6:
        return 1;
    case 7:
        return 2;
    case 8:
        return currentIndex;
    case 9:
        return 3;
    default:
        return currentIndex;
    }
}
