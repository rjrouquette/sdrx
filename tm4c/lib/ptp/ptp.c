//
// Created by robert on 3/31/23.
//

#include <memory.h>
#include <math.h>
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

#define PTP_MAX_SAMPLES (32)
#define PTP_MAX_SRCS (8)
#define PTP_MIN_ACCURACY (250e-9f)


typedef struct PtpSample {
    uint64_t local;
    uint64_t remote;
} PtpSample;

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

static PtpSample samples[PTP_MAX_SAMPLES];
static PtpSource sources[PTP_MAX_SRCS];

static int ptrSamples;
static int cntSamples;

static float offsetDrift;
static float offsetMean;
static float offsetStdDev;

// source internal state functions
static void sourceDelay(PtpSource *src);
static void sourceDelayTx(void *ref, uint8_t *frame, int flen);
static void sourceRequestDelay(PtpSource *src);
static void sourceRx(PtpSource *src, uint8_t *frame, int flen);
static void sourceSync(PtpSource *src, PTP2_TIMESTAMP *ts);

static void runDelay(void *ref);
static void runMeasure(void *ref);

// chronyc request handler
static void chronycRequest(uint8_t *frame, int flen);

void PTP_init() {
    PLL_init();

    // set clock ID to MAC address
    getMAC(ptpClockId + 2);
    // update source delay at 2 Hz
    runSleep(1ull << 31, runDelay, NULL);
    // update offset measurement every second
    runSleep(1ull << 32, runMeasure, NULL);
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

static void runMeasure(void *ref) {
    if(cntSamples < 3) {
        cntSamples = 0;
        return;
    }

    float x[cntSamples];
    float y[cntSamples];

    // compute means
    uint64_t xOff = samples[(ptrSamples - 1) & (PTP_MAX_SAMPLES - 1)].local;
    float meanX = 0, meanY = 0;
    for(int i = 0; i < cntSamples; i++) {
        int j = (ptrSamples - i - 1) & (PTP_MAX_SAMPLES - 1);
        x[i] = toFloat((int64_t) (samples[j].local - xOff));
        y[i] = toFloat((int64_t) (samples[j].remote - samples[j].local));
        meanX += x[i];
        meanY += y[i];
    }
    meanX /= (float) cntSamples;
    meanY /= (float) cntSamples;

    float xx = 0, xy = 0;
    for(int i = 0; i < cntSamples; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        xx += a * a;
        xy += a * b;
    }
    float beta = (xx == 0) ? 0 : (xy / xx);

    // compute residual
    float res = 0;
    for(int i = 0; i < cntSamples; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        float z = b - beta * a;
        res += z * z;
    }
    res /=  (float) (cntSamples - 1);

    // remove outliers
    int cnt = 0;
    for(int i = 0; i < cntSamples; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        float z = b - beta * a;
        if(z * z < res) {
            x[cnt] = x[i];
            y[cnt] = y[i];
            ++cnt;
        }
    }

    // recompute means
    meanX = 0, meanY = 0;
    for(int i = 0; i < cnt; i++) {
        meanX += x[i];
        meanY += y[i];
    }
    meanX /= (float) cnt;
    meanY /= (float) cnt;

    if(cnt < 2) {
        offsetDrift = 0;
        offsetMean = meanY;
        offsetStdDev = 0;
        return;
    }

    // recompute beta
    xx = 0, xy = 0;
    for(int i = 0; i < cnt; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        xx += a * a;
        xy += a * b;
    }
    beta = (xx == 0) ? 0 : (xy / xx);

    // recompute residual
    res = 0;
    for(int i = 0; i < cnt; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        float z = b - beta * a;
        res += z * z;
    }
    res /=  (float) (cnt - 1);

    // compute final result
    offsetDrift = beta;
    offsetMean = meanY + (beta * (toFloat((int64_t) (CLK_TAI() - xOff)) - meanX));
    offsetStdDev = sqrtf(res);

    // update offset compensation
    PLL_updateOffset(offsetMean);
    // update frequency compensation
//    PLL_updateDrift(source->freqDrift);
}

void ptpApplyOffset(int64_t offset) {
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        sources[i].rxLocal += offset;
        sources[i].txLocal += offset;
    }
}


static void sourceDelay(PtpSource *src) {
    // wait for both timestamps
    if(!src->rxRemote) return;
    if(!src->txLocal) return;

    // compute delay using most recent sync
    uint64_t delay = src->rxLocal + src->rxRemote;
    delay -= src->txLocal + src->txRemote;
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
    // update remote transmit timestamp
    src->txRemote = fromPtpTimestamp(ts);

    // store sample
    samples[ptrSamples].local = src->rxLocal;
    samples[ptrSamples].remote = src->txRemote + src->delay;
    // advance sample pointer
    ptrSamples = (ptrSamples + 1) & (PTP_MAX_SAMPLES - 1);
    if(cntSamples < PTP_MAX_SAMPLES)
        ++cntSamples;
}



unsigned PTP_status(char *buffer) {
    char tmp[32];
    char *end = buffer;

    end = append(end, "MAC Address       Sync  Delay\n");
    // display sources
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(sources[i].mac == 0) continue;
        end = macToStr(&(sources[i].mac), end);

        *(end++) = ' ';
        end += fmtFloat((float) sources[i].syncRate, 4, 0, end);

        *(end++) = ' ';
        tmp[fmtFloat(1e6f * 0x1p-32f * (float) sources[i].delay, 8, 3, tmp)] = 0;
        end = append(end, tmp);
        end = append(end, " us\n");
    }

    end = append(end, "\noffset measurement:\n");

    tmp[fmtFloat(offsetDrift * 1e6f, 12, 3, tmp)] = 0;
    end = append(end, "  - drift: ");
    end = append(end, tmp);
    end = append(end, " ppm\n");

    tmp[fmtFloat(offsetMean * 1e6f, 12, 3, tmp)] = 0;
    end = append(end, "  - mean:  ");
    end = append(end, tmp);
    end = append(end, " us\n");

    tmp[fmtFloat(offsetStdDev * 1e6f, 12, 3, tmp)] = 0;
    end = append(end, "  - dev:   ");
    end = append(end, tmp);
    end = append(end, " us\n");

    return end - buffer;
}
