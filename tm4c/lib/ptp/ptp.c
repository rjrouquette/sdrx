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

#define PTP_MAX_SRCS (8)
#define PTP_MIN_ACCURACY (250e-9f)

typedef struct PtpSource {
    uint64_t mac;
    uint64_t rxLocal;
    uint64_t rxRemote;
    uint64_t txLocal;
    uint64_t txRemote;
    uint32_t delay;
} PtpSource;


uint8_t ptpClockId[8];

static volatile uint64_t lastUpdate;
static volatile uint32_t refId;
static volatile float rootDelay;
static volatile float rootDispersion;

static PtpSource sources[PTP_MAX_SRCS];

// allocate PTP source
static void allocSource(uint8_t *frame, int flen);

static void runSelect(void *ref);

// chronyc request handler
static void chronycRequest(uint8_t *frame, int flen);

void PTP_init() {
    PLL_init();

    // set clock ID to MAC address
    getMAC(ptpClockId + 2);
    // update source selection at 16Hz
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
    // check if source is already active
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(sources[i].mac == mac) return;
    }
    // activate as new source
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(sources[i].mac == 0) {
            sources[i].mac = mac;
            return;
        }
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


unsigned PTP_status(char *buffer) {
    char tmp[32];
    char *end = buffer;

    end = append(end, "active sources:\n");
    // display sources
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        if(sources[i].mac == 0) continue;
        end = append(end, "- ");
        end = macToStr(&(sources[i].mac), end);

        tmp[fmtFloat(1e6f * 0x1p-32f * (float) sources[i].delay, 12, 3, tmp)] = 0;
        end = append(end, tmp);
        end = append(end, " us\n");
    }

    return end - buffer;
}
