//
// Created by robert on 3/31/23.
//

#include <memory.h>
#include <math.h>
#include <stdbool.h>
#include "../clk/comp.h"
#include "../clk/mono.h"
#include "../clk/tai.h"
#include "../clk/util.h"
#include "../format.h"
#include "../led.h"
#include "../net.h"
#include "../net/eth.h"
#include "../net/ip.h"
#include "../net/udp.h"
#include "../net/util.h"
#include "../run.h"
#include "common.h"
#include "pll.h"
#include "ptp.h"

#define PTP_MAX_SRCS (8)
#define PTP_MIN_ACCURACY (250e-9f)

typedef struct PtpSource {
    uint64_t mac;
    uint64_t rxLocal;
    uint64_t rxRemote;
    uint64_t txLocal;
    uint64_t txRemote;
    uint32_t delay;
    uint16_t seqDelay;
    uint16_t seqSync;
    int8_t syncRate;
} PtpSource;


uint8_t ptpClockId[8];

static volatile uint64_t lastUpdate;
static volatile uint32_t refId;
static volatile float rootDelay;
static volatile float rootDispersion;

static PtpSource sources[PTP_MAX_SRCS];

// source internal state functions
static void sourceDelay(PtpSource *src);
static void sourceDelayTx(void *ref, uint8_t *frame, int flen);
static void sourceRequestDelay(PtpSource *src);
static void sourceRx(PtpSource *src, uint8_t *frame, int flen);
static void sourceSync(PtpSource *src, PTP2_TIMESTAMP *ts);

static void runDelay(void *ref);
static void runSelect(void *ref);

// chronyc request handler
static void chronycRequest(uint8_t *frame, int flen);

void PTP_init() {
    PLL_init();

    // set clock ID to MAC address
    getMAC(ptpClockId + 2);
    // update source delay at 2 Hz
    runSleep(1u << (32 - 1), runDelay, NULL);
    // update source selection at 16 Hz
    runSleep(1u << (32 - 4), runSelect, NULL);
}

/**
 * Handles raw ethernet PTP frames
 * @param frame
 * @param flen
 */
void PTP_process(uint8_t *frame, int flen) {
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_PTP *headerPTP = (HEADER_PTP *) (headerEth + 1);

    // ignore anything we sent ourselves
    if(isMyMAC(headerEth->macSrc) == 0)
        return;
    // ignore unsupported versions
    if(headerPTP->versionPTP != PTP2_VERSION) return;
    // flip mac address for reply
    copyMAC(headerEth->macDst, headerEth->macSrc);

    // indicate time-server activity
    LED_act0();

    // convert MAC to integer for fast comparison
    uint64_t mac = 0;
    copyMAC(&mac, headerEth->macSrc);
    if(mac == 0) return;

    // look for matching source
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(mac != sources[i].mac) continue;
        sourceRx(sources + i, frame, flen);
        return;
    }

    // register new source if message was an announcement
    if(headerPTP->messageType != PTP2_MT_ANNOUNCE) return;
    // map message
    PTP2_ANNOUNCE *announce = (PTP2_ANNOUNCE *) (headerPTP + 1);
    uint32_t clkQual = announce->grandMasterClockQuality;
    // remove source if it's quality is too poor
    if(clkQual < 0x20 || clkQual > 0x31 || lutClkAccuracy[clkQual - 0x20] > PTP_MIN_ACCURACY) {
        for(int i = 0; i < PTP_MAX_SRCS; i++) {
            if(sources[i].mac == mac)
                sources[i].mac = 0;
        }
        return;
    }
    // activate as new source
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(sources[i].mac == 0) {
            sources[i].mac = mac;
            sources[i].rxLocal = 0;
            return;
        }
    }
}


static void runDelay(void *ref) {
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(sources[i].mac)
            sourceRequestDelay(sources + i);
    }
}

static void runSelect(void *ref) {
//    // select best clock
//    uint64_t now = CLK_MONO();
//    sourcePrimary = NULL;
//    for(int i = 0; i < cntSources; i++) {
//        if(((int64_t) (now - sources[i]->lastUpdate)) > (1ull << 32)) {
//            sources[i]->state = RPY_SD_ST_UNSELECTED;
//            continue;
//        }
//        if(sources[i]->usedOffset < 8 || sources[i]->usedDrift < 4) {
//            sources[i]->state = RPY_SD_ST_FALSETICKER;
//            continue;
//        }
//        if(sources[i]->freqSkew > PTP_MAX_SKEW) {
//            sources[i]->state = RPY_SD_ST_JITTERY;
//            continue;
//        }
//        sources[i]->state = RPY_SD_ST_SELECTABLE;
//        if(sourcePrimary == NULL) {
//            sourcePrimary = sources[i];
//            continue;
//        }
//        if(sources[i]->score < sourcePrimary->score) {
//            sourcePrimary = sources[i];
//        }
//    }
//
//    // sanity check source and check for update
//    PtpSource *source = sourcePrimary;
//    if(source == NULL) return;
//    source->state = RPY_SD_ST_SELECTED;
//    if(source->lastUpdate == lastUpdate) return;
//    lastUpdate = source->lastUpdate;
//
//    // set status
//    refId = source->id;
////    rootDelay = source->rootDelay + (uint32_t) (0x1p16f * source->delayMean);
////    rootDispersion = source->rootDispersion + (uint32_t) (0x1p16f * source->delayStdDev);
//
//    // update offset compensation
//    PLL_updateOffset(source->poll + 4, source->pollSample[source->samplePtr].offset, source->offsetMean);
//    // update frequency compensation
//    PLL_updateDrift(source->freqDrift);
}

void ptpApplyOffset(int64_t offset) {
//    for(int i = 0; i < cntSources; i++) {
//        PtpSource_applyOffset(sourceSlots + i, offset);
//    }
}


static void sourceDelay(PtpSource *src) {
    // wait for both timestamps
    if(!src->rxRemote) return;
    if(!src->txLocal) return;

    // compute delay using most recent sync
    uint64_t delay = src->txLocal + src->txRemote;
    delay -= src->rxLocal + src->rxRemote;
    src->delay = delay / 2;
}

static void sourceDelayTx(void *ref, uint8_t *frame, int flen) {
    PtpSource *src = (PtpSource *) ref;
    uint64_t stamps[3];
    NET_getTxTime(frame, stamps);
    src->txLocal = stamps[2];
    sourceDelay(src);
}

static void sourceRequestDelay(PtpSource *src) {
    if(!src->rxLocal) return;

    int txDesc = NET_getTxDesc();
    if(txDesc < 0) return;
    // allocate and clear frame buffer
    const int flen = PTP2_MIN_SIZE + sizeof(PTP2_TIMESTAMP);
    uint8_t *frame = NET_getTxBuff(txDesc);
    memset(frame, 0, flen);

    // map headers
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_PTP *headerPTP = (HEADER_PTP *) (headerEth + 1);
    PTP2_TIMESTAMP *origin = (PTP2_TIMESTAMP *) (headerPTP + 1);

    // IEEE 802.1AS
    headerEth->ethType = ETHTYPE_PTP;
    copyMAC(headerEth->macDst, gPtpMac);

    headerPTP->versionPTP = PTP2_VERSION;
    headerPTP->messageType = PTP2_MT_DELAY_REQ;
    headerPTP->messageLength = __builtin_bswap16(sizeof(HEADER_PTP) + sizeof(PTP2_TIMESTAMP));
    memcpy(headerPTP->sourceIdentity.identity, ptpClockId, sizeof(ptpClockId));
    headerPTP->sourceIdentity.portNumber = 0;
    headerPTP->domainNumber = PTP2_DOMAIN;
    headerPTP->logMessageInterval = 0;
    headerPTP->sequenceId = __builtin_bswap16(++src->seqDelay);

    // set timestamp
    toPtpTimestamp(CLK_TAI(), origin);

    // clear status flags
    src->rxRemote = 0;
    src->txLocal = 0;

    // transmit request
    NET_setTxCallback(txDesc, sourceDelayTx, src);
    NET_transmit(txDesc, flen);
}

static void sourceRx(PtpSource *src, uint8_t *frame, int flen) {
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_PTP *headerPTP = (HEADER_PTP *) (headerEth + 1);

    if(headerPTP->messageType == PTP2_MT_SYNC) {
        // process sync message (defer filter update until followup message)
        src->syncRate = (int8_t) headerPTP->logMessageInterval;
        src->seqSync = headerPTP->sequenceId;
        uint64_t stamps[3];
        NET_getRxTime(frame, stamps);
        src->rxLocal = stamps[2];
    }
    else if(headerPTP->messageType == PTP2_MT_FOLLOW_UP) {
        // process sync followup message
        if(src->seqSync == headerPTP->sequenceId) {
            sourceSync(src, (PTP2_TIMESTAMP *) (headerPTP + 1));
        }
    }
    else if(headerPTP->messageType == PTP2_MT_DELAY_RESP) {
        if(src->seqDelay == __builtin_bswap16(headerPTP->sequenceId)) {
            PTP2_DELAY_RESP *resp = (PTP2_DELAY_RESP *) (headerPTP + 1);
            src->rxRemote = fromPtpTimestamp(&(resp->receiveTimestamp));
            sourceDelay(src);
        }
    }
}

static void sourceSync(PtpSource *src, PTP2_TIMESTAMP *ts) {
    src->txRemote = fromPtpTimestamp(ts);

//    // advance sample buffer
//    incrFilter(src);
//    // set current sample
//    PtpPollSample *sample = src->pollSample + src->samplePtr;
//    // set compensated reference time
//    const uint64_t comp = src->syncRxStamps[1];
//    sample->comp = comp;
//    // set TAI reference time
//    const uint64_t tai = src->syncRxStamps[2];
//    sample->taiSkew = tai - comp;
//    // compute TAI offset
//    uint64_t offset = (fromPtpTimestamp(ts) - tai) + src->delay;
//    sample->offset = (int64_t) offset;
}



unsigned PTP_status(char *buffer) {
    char tmp[32];
    char *end = buffer;

    end = append(end, "MAC Address     Sync Delay\n");
    // display sources
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(sources[i].mac == 0) continue;
        end = macToStr(&(sources[i].mac), end);

        *(end++) = ' ';
        end += fmtFloat((float) sources[i].syncRate, 4, 0, end);

        *(end++) = ' ';
        tmp[fmtFloat(1e6f * 0x1p-32f * (float) sources[i].delay, 12, 3, tmp)] = 0;
        end = append(end, tmp);
        end = append(end, " us\n");
    }

    return end - buffer;
}
