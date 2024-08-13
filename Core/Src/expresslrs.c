/*
 * expresslrs.c
 *
 *  Created on: Jul 18, 2024
 *      Author: vedpa
 */
#include <elrs_rcdata_handler.h>
#include "expresslrs.h"
#include <string.h>
#include "stdlib.h"
#include "usbd_cdc_if.h"
#include "math_util.h"
#include "com_debugging.h"
// TODO: change order to be better

elrsReceiver_t receiver;
elrsPhase_t phaseLocker;
uint8_t BindingUID[6] = {0,1,2,3,4,5}; // seed for binding
uint8_t UID[6];

uint8_t rcPayload[8] = {0};
int bindRateIndex = 3;

uint8_t fhssSequence[ELRS_NR_SEQUENCE_ENTRIES] = {0};
uint8_t fhssIndex = 0;
static uint16_t seqCount = 0;
static uint8_t syncChannel = 0;
static uint32_t freqSpread = 0;
uint32_t seed = 0;
uint32_t lastSync = 0;
uint32_t lastPacketMicros = 0;
uint32_t hopCountDisconnected = 0;

// FHSS Configs for the SX1280
// start and stop frequencies are pulled from betaflight
uint32_t freqStart =  (uint32_t)((2400400000)/ SX1280_PLL_STEP);
uint32_t freqStop = (uint32_t)((2479400000)/ SX1280_PLL_STEP);
uint8_t freqCount = 80;

// ripped off betaflight -> settings for each frequency the radio can transmit at.
elrsModSettings_t airRateConfig[4] = {
	{0, 500, LORA_BW_800, LORA_SF_5, LORA_CR_LI_4_6, 2000, 4, 12},
	{1, 250, LORA_BW_800, LORA_SF_6, LORA_CR_LI_4_8, 4000, 4, 14},
	{2, 150, LORA_BW_800, LORA_SF_7, LORA_CR_LI_4_8, 6666, 4, 12},
	{3, 50, LORA_BW_800, LORA_SF_8, LORA_CR_LI_4_8, 20000, 2, 12} // bind mode rate index
};

void setBindingMode(){
	if(!receiver.inBindingMode){
		receiver.inBindingMode = 1;
		memcpy(UID, BindingUID, 6);

		refreshExpressLRS(bindRateIndex);
	}
}

void setPrescaleForRateIndex(uint8_t index){
	__HAL_TIM_SET_AUTORELOAD(elrs_tim, airRateConfig[index].interval - 1);
}

void setLastClockTime(uint32_t timeMicros){
	phaseLocker.lastClockTimeMicros = timeMicros;
	phaseLocker.lastUpdateClock = 1;
}

void setLastPacketTime(uint32_t timeMicros){
	phaseLocker.lastPacketTimeMicros = timeMicros;
	phaseLocker.lastUpdateClock = 0;
}

void clockPhaseUpdate(uint32_t timeMicros){
	if(!isDisconnected()){
		phaseLocker.rawPhaseDiff = getDeltaTime(phaseLocker.lastClockTimeMicros, phaseLocker.lastPacketTimeMicros);
		if(phaseLocker.rawPhaseDiff > 1000000){
			displayInt("DCED", 1);
			disconnect(timeMicros);
			return;
		}

		int32_t defaultInt = (int32_t)airRateConfig[receiver.rateIndex].interval;
		int32_t offset;
		phaseLocker.rawPhaseDiff = phaseLocker.rawPhaseDiff % defaultInt;
		int32_t phaseSigned = (int32_t)phaseLocker.rawPhaseDiff;

		offset = defaultInt/2 - phaseSigned; // need to be half out of phase so that the interrupts do not coliide!!
		offset = constrain(offset, -defaultInt/4, defaultInt/4); // don't phase too much.

		if(receiver.connected == ELRS_TENTATIVE && absInt(offset) < 50){
			receiver.connected = ELRS_CONNECTED;
			displayInt("CONNECTED!", 1);
		}
		if(offset > 10 || offset < -10){
			__HAL_TIM_SET_AUTORELOAD(elrs_tim, defaultInt + (offset/3) - 1);
		}else{
			// dont phase if already locked on
			__HAL_TIM_SET_AUTORELOAD(elrs_tim, defaultInt - 1);
		}
		phaseLocker.offset = offset;
	}else{
		receiver.nonceDisconnected++;
		phaseLocker.lastClockTimeMicros = timeMicros;
		phaseLocker.lastPacketTimeMicros = timeMicros;
		if(receiver.nonceDisconnected > (airRateConfig[receiver.rateIndex].rate * 11)/10){
			receiver.rateIndex = (receiver.rateIndex + 1) % 4;
			refreshExpressLRS(receiver.rateIndex);
		}
		// do receiver refreshing
	}
}

uint8_t isDisconnected(){
	if(receiver.connected == ELRS_DISCONNECTED){
		return 1;
	}
	return 0;
}

// deals with frequency hopping and LORA params
void refreshExpressLRS(uint8_t newIndex){
	fhssGenSequence(UID);
	receiver.nonceRX = 0;
	receiver.rssi = 0;
	receiver.rssiFiltered = 0;
	receiver.snr = 0;
	receiver.connected = ELRS_DISCONNECTED;
	receiver.nonceDisconnected = 0;

	uint32_t timeMicros = micros();
	phaseLocker.lastClockTimeMicros = timeMicros;
	phaseLocker.lastPacketTimeMicros = timeMicros; // reset so we don't have crazy long differences
	phaseLocker.lastUpdateClock = 0;
	phaseLocker.rawPhaseDiff = 0;

	receiver.currentFreq = fhssGetInitialFreq();
	sx1280_init(); // necessary in refresh?

	changeRateIndex(newIndex, receiver.currentFreq, UID[5]);
}

void processRFPacket(uint8_t* packet, uint32_t timeMicros){
	elrsOtaPacket_t * const otaPktPtr = (elrsOtaPacket_t * const) packet;

	lastPacketMicros = timeMicros;
	switch(otaPktPtr->type) {
	    case ELRS_SYNC_PACKET:
	    	uint8_t sync_success = processSyncPacket(otaPktPtr, timeMicros);
	    	if(sync_success){
	    		lastSync = timeMicros;
	    	}
		    break;
	    case ELRS_RC_DATA_PACKET:
	    	if(receiver.switchMode == 0){
	    		hybridWideNonceToSwitchIndex(receiver.nonceRX);
	    	}
	    	memcpy((uint8_t *) rcPayload, (uint8_t *) packet, 8);
		    break;
	    case ELRS_MSP_DATA_PACKET:
	    	//displayInts4("nonceRx", receiver.nonceRX, "fhssInd", fhssIndex, "time", timeMicros, "phase", phaseLocker.rawPhaseDiff);
		    if (receiver.inBindingMode && otaPktPtr->msp_ul.packageIndex == 1 && otaPktPtr->msp_ul.payload[0] == ELRS_MSP_BIND) {
			    uint8_t* packy = (uint8_t*)&otaPktPtr->msp_ul.payload[1];
			    processBindPacket(packy);
			    char* data4 = "BIND PACK\n";
			    CDC_Transmit_FS((uint8_t *)data4, strlen(data4));
		    }
		    break;
	    default:
	    	break;
	}

}

void tentativeConnection(uint32_t timeMicros){
	receiver.connected = ELRS_TENTATIVE;

	phaseLocker.lastClockTimeMicros = timeMicros;
	phaseLocker.lastPacketTimeMicros = timeMicros;
	phaseLocker.rawPhaseDiff = 0;
}

void disconnect(uint32_t timeMicros){
	receiver.connected = ELRS_DISCONNECTED;
	phaseLocker.lastClockTimeMicros = timeMicros;
	phaseLocker.lastPacketTimeMicros = timeMicros; // reset so we don't have crazy long differences
	receiver.nonceRX = 0;
	receiver.nonceDisconnected = 0;
	setPrescaleForRateIndex(receiver.rateIndex);
	phaseLocker.lastUpdateClock = 0;
	phaseLocker.rawPhaseDiff = 0;

	receiver.currentFreq = fhssGetInitialFreq();
	sx1280_write_RF_frequency(receiver.currentFreq);
}

void processBindPacket(uint8_t* packet){
	UID[2] = packet[0];
	UID[3] = packet[1];
	UID[4] = packet[2];
	UID[5] = packet[3];

	receiver.inBindingMode = 0;
	receiver.rateIndex = 0; // TEMP!!! make connection searching alg
	writeCurrentConfigsToFlash();
	refreshExpressLRS(receiver.rateIndex);
}

void exitBindMode(){
	receiver.inBindingMode = 0;
	receiver.rateIndex = 1;
	refreshExpressLRS(receiver.rateIndex);
}

uint8_t isBindingMode(){
	return receiver.inBindingMode;
}

uint8_t processSyncPacket(elrsOtaPacket_t * const otaPktPtr, uint32_t timeMicros){
	// Verify the first two of three bytes of the binding ID, which should always match
	if (otaPktPtr->sync.UID3 != UID[3] || otaPktPtr->sync.UID4 != UID[4]) {
		return 0;
	}
	uint8_t config_write_needed = 0;

	// need to change in loop
	receiver.nextRateIndex = airRateIndexToIndex24(otaPktPtr->sync.rateIndex, receiver.rateIndex);
	if(receiver.nextRateIndex != receiver.rateIndex){
		config_write_needed = 1;
		receiver.rateIndex = receiver.nextRateIndex;
	}

	// need to update switch mode
	uint8_t switchEncMode = otaPktPtr->sync.switchEncMode;

	if(switchEncMode != receiver.switchMode){
		receiver.switchMode = switchEncMode;
		config_write_needed = 1;
	}

	if(config_write_needed){
		writeCurrentConfigsToFlash();
	}

	if (receiver.nonceRX != otaPktPtr->sync.nonce || fhssIndex != otaPktPtr->sync.fhssIndex || receiver.connected == ELRS_DISCONNECTED) {
		fhssIndex = (otaPktPtr->sync.fhssIndex) % seqCount;
		receiver.nonceRX = otaPktPtr->sync.nonce;

		if(receiver.connected == ELRS_DISCONNECTED){
			tentativeConnection(timeMicros);
		}

	}
	return 1;
}

void writeCurrentConfigsToFlash(){
	uint8_t buf[8] = {UID[0], UID[1], UID[2], UID[3], UID[4], UID[5], receiver.rateIndex, receiver.switchMode};
	writeNewConfig(buf);
}

void initExpressLRS(){
	HAL_TIM_Base_Start_IT(htim_elrs);

	initFlashMemoryConfig(2);
	uint8_t initConfigs[8] = {0x00};
	readCurrentConfig(initConfigs);
	UID[0] = initConfigs[0];
	UID[1] = initConfigs[1];
	UID[2] = initConfigs[2];
	UID[3] = initConfigs[3];
	UID[4] = initConfigs[4];
	UID[5] = initConfigs[5];
	receiver.rateIndex = initConfigs[6]; // TEMP
	receiver.switchMode = initConfigs[7];

	if(UID[0] == 0 && UID[1] == 0 && UID[2] == 0 && UID[3] == 0 && UID[4] == 0 && UID[5] == 0){
		setBindingMode();
	}else{
		refreshExpressLRS(receiver.rateIndex);
	}
}

void changeRateIndex(uint8_t newIndex, uint32_t freq, uint8_t uid5){
	receiver.rateIndex = newIndex;
	elrsModSettings_t newSettings = airRateConfig[receiver.rateIndex];
	receiver.modParams = newSettings;
	setPrescaleForRateIndex(newIndex);
	sx1280_set_RF_rate(newSettings.sf, newSettings.bw, newSettings.cr, newSettings.preambleLen, freq, uid5 & 0x01);
}



void expressLrsSetRcDataFromPayload(uint16_t *rcData)
{
	volatile elrsOtaPacket_t * const otaPktPtr = (elrsOtaPacket_t * const) rcPayload;
	unpackChannelDataHybridWide(rcData, otaPktPtr);
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
    seqCount = (256 / maxf(freqCount, 1)) * freqCount;
    syncChannel = (freqCount / 2) + 1;
    freqSpread = (freqStop - freqStart) * ELRS_FREQ_SPREAD_SCALE / maxf((freqCount - 1), 1);

    // initialize the sequence array
    for (uint16_t i = 0; i < seqCount; i++) {
        if (i % freqCount == 0) {
            fhssSequence[i] = syncChannel;
        } else if (i % freqCount == syncChannel) {
            fhssSequence[i] = 0;
        } else {
            fhssSequence[i] = i % freqCount;
        }
    }

    for (uint16_t i = 0; i < seqCount; i++) {
        // if it's not the sync channel
        if (i % freqCount != 0) {
            uint8_t offset = (i / freqCount) * freqCount; // offset to start of current block
            uint8_t rand = rngN(freqCount - 1) + 1; // random number between 1 and numFreqs

            // switch this entry and another random entry in the same block
            uint8_t temp = fhssSequence[i];
            fhssSequence[i] = fhssSequence[offset + rand];
            fhssSequence[offset + rand] = temp;
        }
    }
}

uint8_t doFhssIrq(){
	receiver.nonceRX += 1;
	uint8_t modResultFHSS = (receiver.nonceRX) % receiver.modParams.fhssHopInterval;

	if((receiver.inBindingMode == 0) && modResultFHSS == 0 && !isDisconnected()){
		receiver.currentFreq = fhssGetNextFreq();
		sx1280_write_RF_frequency(receiver.currentFreq);
		return 1;
	}
	return 0;
}

uint32_t fhssGetInitialFreq()
{
    return freqStart + (syncChannel * freqSpread / ELRS_FREQ_SPREAD_SCALE);
}

uint32_t fhssGetNextFreq()
{
    fhssIndex = (fhssIndex + 1) % seqCount;
    uint32_t freq = freqStart + (freqSpread * fhssSequence[fhssIndex] / ELRS_FREQ_SPREAD_SCALE);
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
