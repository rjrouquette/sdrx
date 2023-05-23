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
#include "../net/util.h"
#include "../run.h"
#include "common.h"
#include "pll.h"
#include "ptp.h"
#include "util.h"

#define PTP_HIST_DRIFT (16)
#define PTP_HIST_OFFSET (32)
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

static float ringDrift[PTP_HIST_DRIFT];
static PtpSample ringOffset[PTP_HIST_OFFSET];
static PtpSource sources[PTP_MAX_SRCS];

static int ptrDrift;
static int cntDrift;
static int ptrOffset;
static int cntOffset;

static uint64_t driftComp;
static uint64_t driftTai;
static float driftOff;
static int driftCount;
static float driftMean;
static float driftStdDev;

static uint64_t offsetComp;
static uint64_t offsetTai;
static int offsetCount;
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
    // remove source if its quality is too poor
    if(clkQual < 0x20 || clkQual > 0x31 || lutClkAccuracy[clkQual - 0x20] > PTP_MIN_ACCURACY) {
        for(int i = 0; i < PTP_MAX_SRCS; i++) {
            if(sources[i].mac == mac)
                memset(sources + i, 0, sizeof(PtpSource));
        }
        return;
    }
    // activate as new source
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(sources[i].mac == 0) {
            sources[i].mac = mac;
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

static int updateOffset() {
    if(cntOffset < 5) {
        cntOffset = 0;
        return 1;
    }

    float x[cntOffset];
    float y[cntOffset];

    uint32_t rem = 0;
    // compute compensated time
    uint64_t nowComp = CLK_MONO();
    nowComp += corrFrac(clkCompRate, nowComp - clkCompRef, &rem);
    nowComp += clkCompOffset;
    // compute TAI time
    uint64_t nowTai = nowComp;
    nowTai += corrFrac(clkTaiRate, nowTai - clkTaiRef, &rem);
    nowTai += clkTaiOffset;

    // compute means
    float meanX = 0, meanY = 0;
    for(int i = 0; i < cntOffset; i++) {
        int j = (ptrOffset - i - 1) & (PTP_HIST_OFFSET - 1);
        x[i] = toFloatU(nowTai - ringOffset[j].local);
        y[i] = toFloat((int64_t) (ringOffset[j].remote - ringOffset[j].local));
        meanX += x[i];
        meanY += y[i];
    }
    meanX /= (float) cntOffset;
    meanY /= (float) cntOffset;

    float xx = 0, xy = 0;
    for(int i = 0; i < cntOffset; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        xx += a * a;
        xy += a * b;
    }
    float beta = (xx <= 0) ? 0 : (xy / xx);

    // compute residual
    float res = 0;
    for(int i = 0; i < cntOffset; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        float z = b - beta * a;
        res += z * z;
    }
    res /=  (float) (cntOffset - 1);

    // remove outliers
    int cnt = 0;
    for(int i = 0; i < cntOffset; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        float z = b - beta * a;
        if(z * z <= res) {
            x[cnt] = x[i];
            y[cnt] = y[i];
            ++cnt;
        }
    }
    cntOffset = 0;
    if(cnt < 4) return 1;

    // recompute means
    meanX = 0, meanY = 0;
    for(int i = 0; i < cnt; i++) {
        meanX += x[i];
        meanY += y[i];
    }
    meanX /= (float) cnt;
    meanY /= (float) cnt;

    // recompute beta
    xx = 0, xy = 0;
    for(int i = 0; i < cnt; i++) {
        float a = x[i] - meanX;
        float b = y[i] - meanY;

        xx += a * a;
        xy += a * b;
    }
    beta = (xx <= 0) ? 0 : (xy / xx);

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
    offsetComp = nowComp;
    offsetTai = nowTai;
    offsetCount = cnt;
    offsetDrift = -beta;
    offsetMean = meanY - (beta * meanX);
    offsetStdDev = sqrtf(res);
    return 0;
}

static int updateDrift() {
    if(driftComp == 0) {
        driftComp = offsetComp;
        driftTai = offsetTai;
        driftOff = offsetMean;
        return 1;
    }

    // compute current drift
    uint64_t a = offsetComp - driftComp;
    uint64_t b = offsetTai - driftTai;
    float c = offsetMean - driftOff;
    float newDrift = (c + (0x1p-32f * ((float) (int32_t) (b - a)))) / toFloatU(a);
    // update state for next run
    driftComp = offsetComp;
    driftTai = offsetTai;
    driftOff = offsetMean;

    // add sample to ring buffer
    ringDrift[ptrDrift] = newDrift;
    // advance sample pointer
    ptrDrift = (ptrDrift + 1) & (PTP_HIST_DRIFT - 1);
    if(cntDrift < PTP_HIST_DRIFT)
        ++cntDrift;

    // wait for sufficient samples
    if(cntDrift < 8) return 1;

    float samples[cntDrift];
    for(int i = 0; i < cntDrift; i++) {
        int j = (ptrDrift - i - 1) & (PTP_HIST_DRIFT - 1);
        samples[i] = ringDrift[j];
    }

    int cnt;
    float mean, var;
    // compute mean and variance
    getMeanVar(cntDrift, samples, &mean, &var);
    // remove extrema
    float limit = var * 4;
    if(limit > 0) {
        cnt = 0;
        for (int i = 0; i < cntDrift; i++) {
            float diff = samples[i] - mean;
            if ((diff * diff) <= limit) {
                samples[cnt++] = samples[i];
            }
        }
        // recompute mean and variance
        if (cnt > 0)
            getMeanVar(cnt, samples, &mean, &var);
        else
            cnt = cntDrift;
    } else {
        cnt = cntDrift;
    }

    // update drift measurement
    driftCount = cnt;
    driftMean = mean;
    driftStdDev = sqrtf(var);
    return 0;
}

static void runMeasure(void *ref) {
    // update offset measurement
    if(updateOffset())
        return;

    // discard unstable results
    if(fabsf(offsetDrift) > 1e-3f) {
        if(fabsf(offsetDrift) / fabsf(offsetStdDev) < 100)
            return;
    }

    // update offset compensation
    PLL_updateOffset(offsetMean);

    // update drift measurement
    if(updateDrift())
        return;
    // update frequency compensation
    PLL_updateDrift(driftMean);
}

void ptpRestartOffset() {
    // reset timestamps for all sources
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        PtpSource *src = sources + i;
        src->rxLocal  = 0;
        src->rxRemote = 0;
        src->txLocal  = 0;
        src->txRemote = 0;
    }
    // reset offset filter
    cntOffset = 0;
    // reset drift filter
    driftComp = 0;
    cntDrift = 0;
}


static void sourceDelay(PtpSource *src) {
    // wait for all timestamps
    if(src->rxLocal  == 0) return;
    if(src->rxRemote == 0) return;
    if(src->txLocal  == 0) return;
    if(src->txRemote == 0) return;

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

    // clear delay timestamps
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
    // wait for valid RX timestamp
    if(src->rxLocal == 0) return;

    // update remote transmit timestamp
    src->txRemote = fromPtpTimestamp(ts);

    // add sample to ring buffer
    ringOffset[ptrOffset].local = src->rxLocal;
    ringOffset[ptrOffset].remote = src->txRemote + src->delay;
    // advance sample pointer
    ptrOffset = (ptrOffset + 1) & (PTP_HIST_OFFSET - 1);
    if(cntOffset < PTP_HIST_OFFSET)
        ++cntOffset;
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

    tmp[toDec(offsetCount, 8, ' ', tmp)] = 0;
    end = append(end, "  - used:  ");
    end = append(end, tmp);
    end = append(end, "\n");

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

    end = append(end, "\ndrift measurement:\n");

    tmp[toDec(driftCount, 8, ' ', tmp)] = 0;
    end = append(end, "  - used:  ");
    end = append(end, tmp);
    end = append(end, "\n");

    tmp[fmtFloat(driftMean * 1e6f, 12, 3, tmp)] = 0;
    end = append(end, "  - mean:  ");
    end = append(end, tmp);
    end = append(end, " ppm\n");

    tmp[fmtFloat(driftStdDev * 1e6f, 12, 3, tmp)] = 0;
    end = append(end, "  - dev:   ");
    end = append(end, tmp);
    end = append(end, " ppm\n");

    return end - buffer;
}
