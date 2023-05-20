//
// Created by robert on 3/31/23.
//

#include <memory.h>
#include <math.h>
#include "../../hw/emac.h"
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
#include "pll.h"
#include "ptp.h"
#include "src.h"

#define PTP_MAX_SRCS (8)

#define PTP2_MIN_SIZE (sizeof(HEADER_ETH) + sizeof(HEADER_PTP))
#define PTP2_VERSION (2)
#define PTP2_DOMAIN (1)


// PTP message types
enum PTP2_MTYPE {
    PTP2_MT_SYNC = 0x0,
    PTP2_MT_DELAY_REQ,
    PTP2_MT_PDELAY_REQ,
    PTP2_MT_PDELAY_RESP,

    PTP2_MT_FOLLOW_UP = 0x8,
    PTP2_MT_DELAY_RESP,
    PTP2_MT_PDELAY_FOLLOW_UP,
    PTP2_MT_ANNOUNCE,
    PTP2_MT_SIGNALING,
    PTP2_MT_MANAGEMENT
};

enum PTP2_CLK_CLASS {
    PTP2_CLK_CLASS_PRIMARY = 6,
    PTP2_CLK_CLASS_PRI_HOLD = 7,
    PTP2_CLK_CLASS_PRI_FAIL = 52
};

// PTP time source
enum PTP2_TSRC {
    PTP2_TSRC_ATOMIC = 0x10,
    PTP2_TSRC_GPS = 0x20,
    PTP2_TSRC_RADIO = 0x30,
    PTP2_TSRC_PTP = 0x40,
    PTP2_TSRC_NTP = 0x50,
    PTP2_TSRC_MANUAL = 0x60,
    PTP2_TSRC_OTHER = 0x90,
    PTP2_TSRC_INTERNAL = 0xA0
};

typedef struct PACKED PTP2_SRC_IDENT {
    uint8_t identity[8];
    uint16_t portNumber;
} PTP2_SRC_IDENT;
_Static_assert(sizeof(struct PTP2_SRC_IDENT) == 10, "PTP2_SRC_IDENT must be 10 bytes");

typedef struct PACKED PTP2_TIMESTAMP {
    uint16_t secondsHi;
    uint32_t secondsLo;
    uint32_t nanoseconds;
} PTP2_TIMESTAMP;
_Static_assert(sizeof(struct PTP2_TIMESTAMP) == 10, "PTP2_TIMESTAMP must be 10 bytes");

typedef struct PACKED HEADER_PTP {
    uint16_t messageType: 4;        // lower nibble
    uint16_t transportSpecific: 4;  // upper nibble
    uint16_t versionPTP: 4;         // lower nibble
    uint16_t reserved0: 4;          // upper nibble
    uint16_t messageLength;
    uint8_t domainNumber;
    uint8_t reserved1;
    uint16_t flags;
    uint64_t correctionField;
    uint32_t reserved2;
    PTP2_SRC_IDENT sourceIdentity;
    uint16_t sequenceId;
    uint8_t controlField;
    uint8_t logMessageInterval;
} HEADER_PTP;
_Static_assert(sizeof(struct HEADER_PTP) == 34, "HEADER_PTP must be 34 bytes");

typedef struct PACKED PTP2_ANNOUNCE {
    PTP2_TIMESTAMP originTimestamp;
    uint16_t currentUtcOffset;
    uint8_t reserved;
    uint8_t grandMasterPriority;
    uint32_t grandMasterClockQuality;
    uint8_t grandMasterPriority2;
    uint8_t grandMasterIdentity[8];
    uint16_t stepsRemoved;
    uint8_t timeSource;
} PTP2_ANNOUNCE;
_Static_assert(sizeof(struct PTP2_ANNOUNCE) == 30, "PTP2_ANNOUNCE must be 34 bytes");

typedef struct PACKED PTP2_DELAY_RESP {
    PTP2_TIMESTAMP receiveTimestamp;
    PTP2_SRC_IDENT requestingIdentity;
} PTP2_DELAY_RESP;
_Static_assert(sizeof(struct PTP2_DELAY_RESP) == 20, "PTP2_DELAY_RESP must be 34 bytes");

typedef struct PACKED PTP2_PDELAY_REQ {
    PTP2_TIMESTAMP receiveTimestamp;
    uint8_t reserved[10];
} PTP2_PDELAY_REQ;
_Static_assert(sizeof(struct PTP2_PDELAY_REQ) == 20, "PTP2_PDELAY_REQ must be 34 bytes");

typedef struct PACKED PTP2_PDELAY_RESP {
    PTP2_TIMESTAMP receiveTimestamp;
    PTP2_SRC_IDENT requestingIdentity;
} PTP2_PDELAY_RESP;
_Static_assert(sizeof(struct PTP2_PDELAY_RESP) == 20, "PTP2_PDELAY_RESP must be 34 bytes");

typedef struct PACKED PTP2_PDELAY_FOLLOW_UP {
    PTP2_TIMESTAMP responseTimestamp;
    PTP2_SRC_IDENT requestingIdentity;
} PTP2_PDELAY_FOLLOW_UP;
_Static_assert(sizeof(struct PTP2_PDELAY_RESP) == 20, "PTP2_PDELAY_FOLLOW_UP must be 34 bytes");


// lut for PTP clock accuracy codes
static float lutClkAccuracy[17] = {
        25e-9f, 100e-9f, 250e-9f, 1e-6f,
        2.5e-6f, 10e-6f, 25e-6f, 100e-6f,
        250e-6f, 1e-3f, 2.5e-3f, 10e-3f,
        25e-6f, 100e-3f, 250e-3f, 1.0f,
        10.0f
};

// IEEE 802.1AS broadcast MAC address (01:80:C2:00:00:0E)
static const uint8_t gPtpMac[6] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E };
static uint8_t clockId[8];
static volatile uint32_t seqId;

static volatile uint64_t lastUpdate;
static volatile uint32_t refId;
static volatile float rootDelay;
static volatile float rootDispersion;

static volatile int cntSources;
static PtpSource *sources[PTP_MAX_SRCS];
static PtpSource *sourcePrimary;
static PtpSource sourceSlots[PTP_MAX_SRCS];

static void toPtpTimestamp(uint64_t ts, PTP2_TIMESTAMP *tsPtp);
static uint32_t toPtpClkAccuracy(float rmsError);

// chronyc request handler
static void chronycRequest(uint8_t *frame, int flen);

void PTP_init() {
    PLL_init();

    // set clock ID to MAC address
    getMAC(clockId + 2);
    // IEEE 802.1AS multicast address
    EMAC_setMac(&(EMAC0.ADDR2), gPtpMac);
    // enable address matching
    EMAC0.ADDR2.HI.AE = 1;

    // listen for chronyc status requests
    UDP_register(DEFAULT_CANDM_PORT, chronycRequest);
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
    if(headerPTP->versionPTP != 2) return;
    // flip mac address for reply
    copyMAC(headerEth->macDst, headerEth->macSrc);

    // indicate time-server activity
    LED_act0();

    // TODO process inbound PTP messages
}

static void toPtpTimestamp(uint64_t ts, PTP2_TIMESTAMP *tsPtp) {
    union fixed_32_32 scratch;
    scratch.full = ts;
    // set seconds
    tsPtp->secondsHi = 0;
    tsPtp->secondsLo = __builtin_bswap32(scratch.ipart);
    // convert fraction to nanoseconds
    scratch.ipart = 0;
    scratch.full *= 1000000000u;
    tsPtp->nanoseconds = __builtin_bswap32(scratch.ipart);
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
    cmdReply->data.source_data.ip_addr.family = htons(IPADDR_UNSPEC);
    cmdReply->data.source_data.ip_addr.addr.in4 = source->id;

    cmdReply->data.source_data.stratum = htons(source->stratum);
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
