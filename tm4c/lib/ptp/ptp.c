//
// Created by robert on 3/31/23.
//

#include <memory.h>
#include <math.h>
#include "../chrony/candm.h"
#include "../clk/comp.h"
#include "../clk/mono.h"
#include "../clk/tai.h"
#include "../clk/util.h"
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
#include "src.h"

#define PTP_MAX_SRCS (8)
#define PTP_MAX_SKEW (5e-6f)

static uint8_t clockId[8];

static volatile uint64_t lastUpdate;
static volatile uint32_t refId;
static volatile float rootDelay;
static volatile float rootDispersion;

static volatile int cntSources;
static PtpSource *sources[PTP_MAX_SRCS];
static PtpSource *sourcePrimary;
static PtpSource sourceSlots[PTP_MAX_SRCS];

// allocate PTP source
static void allocSource(uint8_t *frame, int flen);

static void runSelect(void *ref);

// chronyc request handler
static void chronycRequest(uint8_t *frame, int flen);

void PTP_init() {
    PLL_init();

    // set clock ID to MAC address
    getMAC(clockId + 2);
    // listen for chronyc status requests
    UDP_register(DEFAULT_CANDM_PORT, chronycRequest);
    // update source selection every second
    runSleep(1ull << 32, runSelect, NULL);
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

    uint32_t mac[2] = { 0, 0 };
    copyMAC(mac, headerEth->macSrc);

    // look for matching source
    for(int i = 0; i < cntSources; i++) {
        PtpSource *src = sources[i];
        if(src->mac[0] != mac[0]) continue;
        if(src->mac[1] != mac[1]) continue;
        PtpSource_process(src, frame, flen);
        return;
    }

    // register new source if message was an announcement
    if(headerPTP->messageType != PTP2_MT_ANNOUNCE) return;
    // limit number of sources
    if(cntSources >= PTP_MAX_SRCS) return;
    // allocate new source
    allocSource(frame, flen);
}

static void allocSource(uint8_t *frame, int flen) {
    for(int i = 0; i < PTP_MAX_SRCS; i++) {
        PtpSource *slot = sourceSlots + i;
        if(slot->mac[0] == 0) {
            // initialize peer record
            PtpSource_init(slot, frame, flen);
            // append to source list
            sources[cntSources++] = slot;
            // start source updates
            runSleep(1u << 31, (SchedulerCallback) PtpSource_run, slot);
            // return instance
            return;
        }
    }
}

static void runSelect(void *ref) {
    // select best clock
    uint64_t now = CLK_MONO();
    sourcePrimary = NULL;
    for(int i = 0; i < cntSources; i++) {
        if(((int64_t) (now - sources[i]->lastUpdate)) > (1ull << 32)) {
            sources[i]->state = RPY_SD_ST_UNSELECTED;
            continue;
        }
        if(sources[i]->usedOffset < 8 || sources[i]->usedDrift < 4) {
            sources[i]->state = RPY_SD_ST_FALSETICKER;
            continue;
        }
        if(sources[i]->freqSkew > PTP_MAX_SKEW) {
            sources[i]->state = RPY_SD_ST_JITTERY;
            continue;
        }
        sources[i]->state = RPY_SD_ST_SELECTABLE;
        if(sourcePrimary == NULL) {
            sourcePrimary = sources[i];
            continue;
        }
        if(sources[i]->score < sourcePrimary->score) {
            sourcePrimary = sources[i];
        }
    }

    // sanity check source and check for update
    PtpSource *source = sourcePrimary;
    if(source == NULL) return;
    source->state = RPY_SD_ST_SELECTED;
    if(source->lastUpdate == lastUpdate) return;
    lastUpdate = source->lastUpdate;

    // set status
    refId = source->id;
//    rootDelay = source->rootDelay + (uint32_t) (0x1p16f * source->delayMean);
//    rootDispersion = source->rootDispersion + (uint32_t) (0x1p16f * source->delayStdDev);

    // update offset compensation
    PLL_updateOffset(source->poll, source->pollSample[source->samplePtr].offset);
    // update frequency compensation
    PLL_updateDrift(source->poll, source->freqDrift);
}

void ptpApplyOffset(int64_t offset) {
    for(int i = 0; i < cntSources; i++) {
        PtpSource_applyOffset(sourceSlots + i, offset);
    }
}


// -----------------------------------------------------------------------------
// support for chronyc status queries ------------------------------------------
// -----------------------------------------------------------------------------

#define REQ_MIN_LEN (offsetof(CMD_Request, data.null.EOR))
#define REQ_MAX_LEN (sizeof(CMD_Request))

#define REP_LEN_NSOURCES (offsetof(CMD_Reply, data.n_sources.EOR))
#define REP_LEN_SOURCEDATA (offsetof(CMD_Reply, data.source_data.EOR))
#define REP_LEN_SOURCESTATS (offsetof(CMD_Reply, data.sourcestats.EOR))
#define REP_LEN_TRACKING (offsetof(CMD_Reply, data.tracking.EOR))

// replicate htons() function
__attribute__((always_inline))
static inline uint16_t htons(uint16_t value) { return __builtin_bswap16(value); }

// replicate htonl() function
__attribute__((always_inline))
static inline uint32_t htonl(uint32_t value) { return __builtin_bswap32(value); }

// replicate chronyc float format
static int32_t htonf(float value);

// convert 64-bit fixed point timestamp to chrony TimeSpec
static void toTimespec(uint64_t timestamp, volatile Timespec *ts) {
    union fixed_32_32 scratch;
    scratch.full = timestamp;
    // rough reduction from fraction to nanoseconds
    uint32_t temp = scratch.fpart;
    scratch.fpart -= temp >> 4;
    scratch.fpart -= temp >> 7;
    ts->tv_nsec = htonl(scratch.fpart >> 2);
    // set integer seconds
    ts->tv_sec_low = htonl(scratch.ipart);
    ts->tv_sec_high = 0;
}

// begin chronyc reply
static void chronycReply(CMD_Reply *cmdReply, const CMD_Request *cmdRequest);

// chronyc command handlers
static uint16_t chronycNSources(CMD_Reply *cmdReply, const CMD_Request *cmdRequest);
static uint16_t chronycSourceData(CMD_Reply *cmdReply, const CMD_Request *cmdRequest);
static uint16_t chronycSourceStats(CMD_Reply *cmdReply, const CMD_Request *cmdRequest);
static uint16_t chronycTracking(CMD_Reply *cmdReply, const CMD_Request *cmdRequest);

#define CHRONYC_HANDLER_CNT (5)
static const struct {
    uint16_t (*call) (CMD_Reply *cmdReply, const CMD_Request *cmdRequest);
    int len;
    uint16_t cmd;
} handlers[CHRONYC_HANDLER_CNT] = {
        { chronycNSources, REP_LEN_NSOURCES, REQ_N_SOURCES },
        { chronycSourceData, REP_LEN_SOURCEDATA, REQ_SOURCE_DATA },
        { chronycSourceStats, REP_LEN_SOURCESTATS, REQ_SOURCESTATS },
        { chronycTracking, REP_LEN_TRACKING, REQ_TRACKING }
};

static void chronycRequest(uint8_t *frame, int flen) {
    // drop invalid packets
    if(flen < (UDP_DATA_OFFSET + REQ_MIN_LEN)) return;
    if(flen > (UDP_DATA_OFFSET + REQ_MAX_LEN)) return;

    // map headers
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_IP4 *headerIP4 = (HEADER_IP4 *) (headerEth + 1);
    HEADER_UDP *headerUDP = (HEADER_UDP *) (headerIP4 + 1);
    CMD_Request *cmdRequest = (CMD_Request *) (headerUDP + 1);

    // drop invalid packets
    if(cmdRequest->pkt_type != PKT_TYPE_CMD_REQUEST) return;
    if(cmdRequest->pad1 != 0) return;
    if(cmdRequest->pad2 != 0) return;

    // drop packet if nack is not possible
    if(cmdRequest->version != PROTO_VERSION_NUMBER) {
        if(cmdRequest->version < PROTO_VERSION_MISMATCH_COMPAT_SERVER)
            return;
    }

    int txDesc = NET_getTxDesc();
    if(txDesc < 0) return;
    // allocate and clear frame buffer
    uint8_t *resp = NET_getTxBuff(txDesc);
    memset(resp, 0, UDP_DATA_OFFSET + sizeof(CMD_Reply));
    // return to sender
    memcpy(resp, frame, UDP_DATA_OFFSET);
    UDP_returnToSender(resp, ipAddress, DEFAULT_CANDM_PORT);

    // remap headers
    headerEth = (HEADER_ETH *) resp;
    headerIP4 = (HEADER_IP4 *) (headerEth + 1);
    headerUDP = (HEADER_UDP *) (headerIP4 + 1);
    CMD_Reply *cmdReply = (CMD_Reply *) (headerUDP + 1);

    // begin response
    chronycReply(cmdReply, cmdRequest);

    // nack bad protocol version
    if(cmdRequest->version != PROTO_VERSION_NUMBER) {
        cmdReply->status = htons(STT_BADPKTVERSION);
    } else {
        cmdReply->status = htons(STT_INVALID);
        const uint16_t cmd = htons(cmdRequest->command);
        const int len = flen - UDP_DATA_OFFSET;
        for(int i = 0; i < CHRONYC_HANDLER_CNT; i++) {
            if(handlers[i].cmd == cmd) {
                if(handlers[i].len == len)
                    cmdReply->status = (*(handlers[i].call))(cmdReply, cmdRequest);
                else
                    cmdReply->status = htons(STT_BADPKTLENGTH);
                break;
            }
        }
    }

    // finalize packet
    UDP_finalize(resp, flen);
    IPv4_finalize(resp, flen);
    // transmit packet
    NET_transmit(txDesc, flen);
}

static void chronycReply(CMD_Reply *cmdReply, const CMD_Request *cmdRequest) {
    cmdReply->version = PROTO_VERSION_NUMBER;
    cmdReply->pkt_type = PKT_TYPE_CMD_REPLY;
    cmdReply->command = cmdRequest->command;
    cmdReply->reply = htons(RPY_NULL);
    cmdReply->status = htons(STT_SUCCESS);
    cmdReply->sequence = cmdRequest->sequence;
}

static uint16_t chronycNSources(CMD_Reply *cmdReply, const CMD_Request *cmdRequest) {
    cmdReply->reply = htons(RPY_N_SOURCES);
    cmdReply->data.n_sources.n_sources = htonl(cntSources);
    return htons(STT_SUCCESS);
}

static uint16_t chronycSourceData(CMD_Reply *cmdReply, const CMD_Request *cmdRequest) {
    cmdReply->reply = htons(RPY_SOURCE_DATA);
    // locate source
    const uint32_t i = htonl(cmdRequest->data.source_data.index);
    if(i >= cntSources) return htons(STT_NOSUCHSOURCE);
    // sanity check
    PtpSource *source = sources[i];
    if(source == NULL) return htons(STT_NOSUCHSOURCE);

    // set source id
    cmdReply->data.source_data.mode = htons(RPY_SD_MD_REF);
    cmdReply->data.source_data.ip_addr.family = htons(IPADDR_INET4);
    cmdReply->data.source_data.ip_addr.addr.in4 = source->id;

    cmdReply->data.source_data.stratum = htons(1);
    cmdReply->data.source_data.reachability = htons(source->reach & 0xFF);
    cmdReply->data.source_data.orig_latest_meas.f = htonf(source->lastOffsetOrig);
    cmdReply->data.source_data.latest_meas.f = htonf(source->lastOffset);
    cmdReply->data.source_data.latest_meas_err.f = htonf(source->lastDelay);
    cmdReply->data.source_data.since_sample = htonl((CLK_MONO() - source->lastUpdate) >> 32);
    cmdReply->data.source_data.poll = (int16_t) htons(source->poll);
    cmdReply->data.source_data.state = htons(source->state);
    return htons(STT_SUCCESS);
}

static uint16_t chronycSourceStats(CMD_Reply *cmdReply, const CMD_Request *cmdRequest) {
    cmdReply->reply = htons(RPY_SOURCESTATS);

    const uint32_t i = htonl(cmdRequest->data.sourcestats.index);
    if(i >= cntSources) return htons(STT_NOSUCHSOURCE);
    // sanity check
    PtpSource *source = sources[i];
    if(source == NULL) return htons(STT_NOSUCHSOURCE);

    // set source id
    cmdReply->data.sourcestats.ref_id = source->id;
    cmdReply->data.sourcestats.ip_addr.family = htons(IPADDR_UNSPEC);
    cmdReply->data.sourcestats.ip_addr.addr.in4 = source->id;

    cmdReply->data.sourcestats.n_samples = htonl(source->sampleCount);
    cmdReply->data.sourcestats.n_runs = htonl(source->usedOffset);
    cmdReply->data.sourcestats.span_seconds = htonl(source->span);
    cmdReply->data.sourcestats.est_offset.f = htonf(source->offsetMean);
    cmdReply->data.sourcestats.sd.f = htonf(source->offsetStdDev);
    cmdReply->data.sourcestats.resid_freq_ppm.f = htonf(source->freqDrift * 1e6f);
    cmdReply->data.sourcestats.skew_ppm.f = htonf(source->freqSkew * 1e6f);
    return htons(STT_SUCCESS);
}

static uint16_t chronycTracking(CMD_Reply *cmdReply, const CMD_Request *cmdRequest) {
    cmdReply->reply = htons(RPY_TRACKING);
    cmdReply->data.tracking.ref_id = refId;
    cmdReply->data.tracking.stratum = 1;
    cmdReply->data.tracking.leap_status = 0;
    cmdReply->data.tracking.ip_addr.family = htons(IPADDR_UNSPEC);
    cmdReply->data.tracking.ip_addr.addr.in4 = refId;

    PtpSource *source = sourcePrimary;

    toTimespec(CLK_TAI_fromMono(lastUpdate) - clkTaiUtcOffset, &(cmdReply->data.tracking.ref_time));

    cmdReply->data.tracking.current_correction.f = htonf(PLL_offsetMean());
    cmdReply->data.tracking.last_offset.f = htonf(PLL_offsetLast());
    cmdReply->data.tracking.rms_offset.f = htonf(PLL_offsetRms());

    cmdReply->data.tracking.freq_ppm.f = htonf(-1e6f * (0x1p-32f * (float) CLK_COMP_getComp()));
    cmdReply->data.tracking.resid_freq_ppm.f = htonf(-1e6f * (0x1p-32f * (float) CLK_TAI_getTrim()));
    cmdReply->data.tracking.skew_ppm.f = htonf(1e6f * PLL_driftStdDev());

    cmdReply->data.tracking.root_delay.f = htonf(rootDelay);
    cmdReply->data.tracking.root_dispersion.f = htonf(rootDispersion);

    cmdReply->data.tracking.last_update_interval.f = htonf(source ? (0x1p-16f * (float) (1u << (16 + source->poll))) : 0);
    return htons(STT_SUCCESS);
}

/**
 * convert IEEE 754 to candm float format
 * @param value IEEE 754 single-precision float
 * @return candm float format
 */
static int32_t htonf(float value) {
    // decompose IEEE 754 single-precision float
    uint32_t raw = *(uint32_t*) &value;
    uint32_t coef = raw & ((1 << 23) - 1);
    int32_t exp = ((int32_t) (raw >> 23 & 0xFF)) - 127;
    uint32_t sign = (raw >> 31) & 1;

    // small values and NaNs
    if(exp < -65 || (exp == 128 && coef != 0))
        return 0;

    // large values
    if(exp > 61)
        return (int32_t) (sign ? 0x0000007Fu : 0xFFFFFF7Eu);

    // normal values
    coef |= 1 << 23;
    if(sign) coef = -coef;
    coef &= (1 << 25) - 1;
    return (int32_t) htonl(coef | ((exp + 2) << 25));
}
