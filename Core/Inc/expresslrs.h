/*
 * expresslrs.h
 *
 *  Created on: Jul 18, 2024
 *      Author: vedpa
 */

#ifndef INC_EXPRESSLRS_H_
#define INC_EXPRESSLRS_H_

#include <flash_memory_handler.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "sx1280.h"
#include "tim.h"
#include "com_debugging.h"

#define ELRS_OTA_VERSION_ID 3

#define ELRS_MSP_BYTES_PER_CALL 5
#define ELRS_TELEMETRY_SHIFT 2
#define ELRS_TELEMETRY_BYTES_PER_CALL 5

#define ELRS_NR_SEQUENCE_ENTRIES 256
#define ELRS_FREQ_SPREAD_SCALE 256

#define ELRS_MSP_BIND 0x09


extern uint8_t isBinding;

#define elrs_tim (&htim9)

#define maxf(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })


typedef enum {
    ELRS_RC_DATA_PACKET = 0x00,
    ELRS_MSP_DATA_PACKET = 0x01,
    ELRS_SYNC_PACKET = 0x02,
    ELRS_TLM_PACKET = 0x03,
} elrsPacketType_e;

typedef enum {
    ELRS_CONNECTED,
    ELRS_TENTATIVE,
    ELRS_DISCONNECTED
} connectionState_e;

typedef struct elrsOtaPacket_s {
    // The packet type must always be the low two bits of the first byte of the
    // packet to match the same placement in OTA_Packet8_s
    uint8_t type : 2,
            crcHigh : 6;
    union {
        /** PACKET_TYPE_RCDATA **/
        struct {
            uint8_t ch[5];
            uint8_t switches : 7,
                    ch4 : 1;
        } rc;
        /** PACKET_TYPE_MSP **/
        struct {
            uint8_t packageIndex;
            uint8_t payload[ELRS_MSP_BYTES_PER_CALL];
        } msp_ul;
        /** PACKET_TYPE_SYNC **/
        struct {
            uint8_t fhssIndex;
            uint8_t nonce;
            uint8_t switchEncMode : 1,
                    newTlmRatio : 3,
                    rateIndex : 4;
            uint8_t UID3;
            uint8_t UID4;
            uint8_t UID5;
        } sync;
        /** PACKET_TYPE_TLM **/
        struct {
            uint8_t type : ELRS_TELEMETRY_SHIFT,
                    packageIndex : (8 - ELRS_TELEMETRY_SHIFT);
            union {
                struct {
                    uint8_t uplink_RSSI_1 : 7,
                            antenna : 1;
                    uint8_t uplink_RSSI_2 : 7,
                            modelMatch : 1;
                    uint8_t lq : 7,
                            mspConfirm : 1;
                    int8_t SNR;
                    uint8_t free;
                } ul_link_stats;
                uint8_t payload[ELRS_TELEMETRY_BYTES_PER_CALL];
            };
        } tlm_dl;
    };
    uint8_t crcLow;
} __attribute__ ((__packed__)) elrsOtaPacket_t;

// different settings for rate indices of expressLRS
typedef struct elrsModSettings_s {
    uint8_t index;
    uint32_t rate;            // Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t bw;
    uint8_t sf;
    uint8_t cr;
    uint32_t interval;                  // interval in us seconds that corresponds to that frequency
    uint8_t fhssHopInterval;            // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t preambleLen;
} elrsModSettings_t;

static inline int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

typedef struct clockScaleSettings_s {
	uint8_t index;
	uint16_t prescale;
	uint32_t period;
} clockScaleSettings_t;

typedef struct elrsReceiver_s {
	uint32_t currentFreq;
	uint8_t rateIndex;
	uint8_t nextRateIndex;
	connectionState_e connected;

	uint32_t nonceRX; // frequency index that we think we are on, without sync packet
	uint32_t nonceDisconnected;

	elrsModSettings_t modParams;

	int8_t rssi;
	int8_t snr;
	int8_t rssiFiltered;

	uint8_t inBindingMode;
	uint8_t switchMode;
} elrsReceiver_t;

typedef struct elrsPhase_s {
	uint32_t lastPacketTimeMicros;
	uint32_t lastClockTimeMicros;
	uint32_t rawPhaseDiff; // lpf??? also deltaPhaseDiff?
	uint8_t lastUpdateClock;
	int32_t offset;
}elrsPhase_t;

uint8_t rngN(const uint8_t max);
uint32_t fhssGetInitialFreq();
void fhssGenSequence(const uint8_t inputUID[]);
void initExpressLRS();
void refreshExpressLRS(uint8_t newIndex);
void setBindingMode();
void processRFPacket(uint8_t* packet, uint32_t timeMicros);
void writeCurrentConfigsToFlash();
void changeRateIndex(uint8_t newIndex, uint32_t freq, uint8_t uid5);
void setPrescaleForRateIndex(uint8_t index);
uint8_t airRateIndexToIndex24(uint8_t airRate, uint8_t currentIndex);
uint8_t doFhssIrq();
uint32_t fhssGetNextFreq();

void clockPhaseUpdate(uint32_t timeMicros);
void setLastClockTime(uint32_t timeMicros);
void setLastPacketTime(uint32_t timeMicros);
//void handleConnectionState(uint32_t timeMicros);

void tentativeConnection(uint32_t timeMicros);
void disconnect(uint32_t timeMicros);

uint8_t processSyncPacket(elrsOtaPacket_t * const otaPktPtr, uint32_t timeMicros);
void processBindPacket(uint8_t* packet);
void expressLrsSetRcDataFromPayload(uint16_t *rcData);
void exitBindMode();

uint8_t isDisconnected();
uint8_t isBindingMode();


#endif /* INC_EXPRESSLRS_H_ */
