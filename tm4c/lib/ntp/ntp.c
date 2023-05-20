//
// Created by robert on 4/27/23.
//

#include <memory.h>
#include <math.h>

#include "../chrony/candm.h"
#include "../clk/tai.h"
#include "../clk/comp.h"
#include "../clk/mono.h"
#include "../clk/util.h"
#include "../led.h"
#include "../net.h"
#include "../net/dhcp.h"
#include "../net/dns.h"
#include "../net/eth.h"
#include "../run.h"

#include "common.h"
#include "gitversion.h"
#include "ntp.h"
#include "peer.h"
#include "ref.h"
#include "../ptp/pll.h"
#include "../net/util.h"

#define MAX_NTP_PEERS (8)
#define MAX_NTP_SRCS (9)

#define NTP_POOL_FQDN ("pool.ntp.org")
#define NTP_MAX_SKEW (50e-6f) // 50 ppm

#define DNS_UPDT_INTV (1ull << (32 + 4))    // every 16 seconds
#define SRC_UPDT_INTV (1ull << (32 - 4))    // 16 Hz

static NtpGPS srcGps;
static NtpPeer peerSlots[MAX_NTP_PEERS];
// allocate new peer
static NtpSource* ntpAllocPeer();
// deallocate peer
static void ntpRemovePeer(NtpSource *peer);

static NtpSource * sources[MAX_NTP_SRCS];
static NtpSource * selectedSource = NULL;
static volatile uint32_t cntSources = 0;
static volatile uint64_t lastUpdate = 0;

// aggregate NTP state
static volatile int leapIndicator;
static volatile int clockStratum = 16;
static volatile uint32_t refId;
static volatile uint32_t rootDelay;
static volatile uint32_t rootDispersion;


// hash table for interleaved timestamps
#define XLEAVE_SIZE (10)
static volatile struct TsXleave {
    uint64_t rxTime;
    uint64_t txTime;
} tsXleave[1u << XLEAVE_SIZE];

// hash function for interleaved timestamp table
static inline int hashXleave(uint32_t addr) {
    return (int) (((addr * 0xDE9DB139u) >> (32 - XLEAVE_SIZE)) & ((1u << XLEAVE_SIZE) - 1));
}


// request handlers
static void ntpRequest(uint8_t *frame, int flen);
static void chronycRequest(uint8_t *frame, int flen);
// response handlers
static void ntpResponse(uint8_t *frame, int flen);

// internal operations
static void runSelect(void *ref);
static void runDnsFill(void *ref);

// called by PLL for hard TAI adjustments
void ntpApplyOffset(int64_t offset) {
    for(int i = 0; i < cntSources; i++) {
        NtpSource *source = sources[i];
        if(source == NULL) continue;
        NtpSource_applyOffset(source, offset);
    }
}

void NTP_init() {
    PLL_init();

    UDP_register(NTP_PORT_SRV, ntpRequest);
    UDP_register(NTP_PORT_CLI, ntpResponse);
    // listen for crony status requests
    UDP_register(DEFAULT_CANDM_PORT, chronycRequest);

    // initialize empty peer records
    for(int i = 0; i < MAX_NTP_PEERS; i++)
        NtpPeer_init(peerSlots + i);

    // initialize GPS reference and register it as a source
    NtpGPS_init(&srcGps);
    sources[cntSources++] = (void *) &srcGps;
    // 16 Hz polling rate for sources
    runSleep(SRC_UPDT_INTV, srcGps.source.run, &srcGps);

    // update source selection at 16 Hz
    runSleep(SRC_UPDT_INTV, runSelect, NULL);
    // fill empty slots every 16 seconds
    runSleep(DNS_UPDT_INTV, runDnsFill, NULL);
}

uint32_t NTP_refId() {
    return refId;
}

static void ntpTxCallback(void *ref, uint8_t *frame, int flen) {
    // map headers
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_IP4 *headerIP4 = (HEADER_IP4 *) (headerEth + 1);
    HEADER_UDP *headerUDP = (HEADER_UDP *) (headerIP4 + 1);
    HEADER_NTP *headerNTP = (HEADER_NTP *) (headerUDP + 1);

    // retrieve hardware transmit time
    uint64_t stamps[3];
    NET_getTxTime(frame, stamps);
    const uint64_t txTime = (stamps[2] - clkTaiUtcOffset) + NTP_UTC_OFFSET;

    // record hardware transmit time
    uint32_t cell = hashXleave(headerIP4->dst);
    tsXleave[cell].rxTime = headerNTP->rxTime;
    tsXleave[cell].txTime = __builtin_bswap64(txTime);
}

// process client request
static void ntpRequest(uint8_t *frame, int flen) {
    // discard malformed packets
    if(flen < NTP4_SIZE) return;
    // map headers
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_IP4 *headerIP4 = (HEADER_IP4 *) (headerEth + 1);
    HEADER_UDP *headerUDP = (HEADER_UDP *) (headerIP4 + 1);
    HEADER_NTP *headerNTP = (HEADER_NTP *) (headerUDP + 1);

    // verify destination
    if(headerIP4->dst != ipAddress) return;
    // prevent loopback
    if(headerIP4->src == ipAddress) return;
    // filter non-client frames
    if(headerNTP->mode != NTP_MODE_CLI) return;
    // indicate time-server activity
    LED_act0();

    // retrieve rx time
    uint64_t stamps[3];
    NET_getRxTime(frame, stamps);
    // translate TAI timestamp into NTP domain
    const uint64_t rxTime = (stamps[2] - clkTaiUtcOffset) + NTP_UTC_OFFSET;

    // get TX descriptor
    const int txDesc = NET_getTxDesc();
    if(txDesc < 0) return;
    // set callback for precise TX timestamp
    NET_setTxCallback(txDesc, ntpTxCallback, NULL);
    // duplicate packet for sending
    uint8_t *temp = NET_getTxBuff(txDesc);
    memcpy(temp, frame, flen);
    frame = temp;

    // return the response directly to the sender
    UDP_returnToSender(frame, ipAddress, NTP_PORT_SRV);

    // remap headers
    headerEth = (HEADER_ETH *) frame;
    headerIP4 = (HEADER_IP4 *) (headerEth + 1);
    headerUDP = (HEADER_UDP *) (headerIP4 + 1);
    headerNTP = (HEADER_NTP *) (headerUDP + 1);

    // check for interleaved request
    int xleave = -1;
    const uint64_t orgTime = headerNTP->origTime;
    if(
            orgTime != 0 &&
            headerNTP->rxTime != headerNTP->txTime
            ) {
        // load TX timestamp if available
        int cell = hashXleave(headerIP4->dst);
        if(orgTime == tsXleave[cell].rxTime)
            xleave = cell;
    }

    // set type to server response
    headerNTP->mode = NTP_MODE_SRV;
    headerNTP->status = leapIndicator;
    // set stratum and precision
    headerNTP->stratum = clockStratum;
    headerNTP->precision = NTP_CLK_PREC;
    // set root delay
    headerNTP->rootDelay = __builtin_bswap32(rootDelay);
    // set root dispersion
    headerNTP->rootDispersion = __builtin_bswap32(rootDispersion);
    // set reference ID
    headerNTP->refID = refId;
    // set reference timestamp
    headerNTP->refTime = __builtin_bswap64(CLK_TAI_fromMono(lastUpdate) + NTP_UTC_OFFSET - clkTaiUtcOffset);
    // set origin and TX timestamps
    if(xleave < 0) {
        headerNTP->origTime = headerNTP->txTime;
        headerNTP->txTime = __builtin_bswap64(CLK_TAI() + NTP_UTC_OFFSET - clkTaiUtcOffset);
    } else {
        headerNTP->origTime = headerNTP->rxTime;
        headerNTP->txTime = tsXleave[xleave].txTime;
    }
    // set RX timestamp
    headerNTP->rxTime = __builtin_bswap64(rxTime);

    // finalize packet
    UDP_finalize(frame, flen);
    IPv4_finalize(frame, flen);
    // transmit packet
    NET_transmit(txDesc, flen);
}

// process peer response
static void ntpResponse(uint8_t *frame, int flen) {
    // discard malformed packets
    if(flen < NTP4_SIZE) return;
    // map headers
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_IP4 *headerIP4 = (HEADER_IP4 *) (headerEth + 1);
    HEADER_UDP *headerUDP = (HEADER_UDP *) (headerIP4 + 1);
    HEADER_NTP *headerNTP = (HEADER_NTP *) (headerUDP + 1);

    // verify destination
    if(isMyMAC(headerEth->macDst)) return;
    if(headerIP4->dst != ipAddress) return;
    // prevent loopback
    if(headerIP4->src == ipAddress) return;
    // filter non-server frames
    if(headerNTP->mode != NTP_MODE_SRV) return;
    // filter invalid frames
    if(headerNTP->origTime == 0) return;

    // locate recipient
    uint32_t srcAddr = headerIP4->src;
    for(uint32_t i = 0; i < cntSources; i++) {
        NtpSource *source = sources[i];

        // check for matching peer
        if(source == NULL) return;
        if(source->mode != RPY_SD_MD_CLIENT) continue;
        if(source->id != srcAddr) continue;

        // handoff frame to peer
        NtpPeer_recv(source, frame, flen);
        return;
    }
}

static void runSelect(void *ref) {
    // prune defunct sources
    for(int i = 0; i < cntSources; i++) {
        // reference sources are excluded
        if(sources[i]->mode == RPY_SD_MD_REF)
            continue;
        if(sources[i]->prune) {
            ntpRemovePeer(sources[i]);
            break;
        }
    }

    // select best clock
    selectedSource = NULL;
    for(int i = 0; i < cntSources; i++) {
        if(sources[i]->lost) {
            sources[i]->state = RPY_SD_ST_UNSELECTED;
            continue;
        }
        if(sources[i]->usedOffset < 8 || sources[i]->usedDrift < 4) {
            sources[i]->state = RPY_SD_ST_FALSETICKER;
            continue;
        }
        if(sources[i]->freqSkew > NTP_MAX_SKEW) {
            sources[i]->state = RPY_SD_ST_JITTERY;
            continue;
        }
        sources[i]->state = RPY_SD_ST_SELECTABLE;
        if(selectedSource == NULL) {
            selectedSource = sources[i];
            continue;
        }
        if(sources[i]->score < selectedSource->score) {
            selectedSource = sources[i];
        }
    }

    // sanity check source and check for update
    NtpSource *source = selectedSource;
    if(source == NULL) return;
    source->state = RPY_SD_ST_SELECTED;
    if(source->lastUpdate == lastUpdate) return;
    lastUpdate = source->lastUpdate;

    // set status
    refId = source->id;
    clockStratum = source->stratum + 1;
    leapIndicator = source->leap;
    rootDelay = source->rootDelay + (uint32_t) (0x1p16f * source->delayMean);
    rootDispersion = source->rootDispersion + (uint32_t) (0x1p16f * source->delayStdDev);

    // update offset compensation
    PLL_updateOffset(source->poll, source->pollSample[source->samplePtr].offset);
    // update frequency compensation
    PLL_updateDrift(source->poll, source->freqDrift);
}

static NtpSource* ntpAllocPeer() {
    for(int i = 0; i < MAX_NTP_PEERS; i++) {
        NtpPeer *slot = peerSlots + i;
        if(slot->source.id == 0) {
            // initialize peer record
            (*(slot->source.init))(slot);
            // append to source list
            sources[cntSources++] = (NtpSource *) slot;
            // start source updates
            runOnce((1u << 31), slot->source.run, slot);
            // return instance
            return (NtpSource *) slot;
        }
    }
    return NULL;
}

static void ntpRemovePeer(NtpSource *peer) {
    // deselect source if selected
    if(peer == selectedSource)
        selectedSource = NULL;

    // remove from task schedule
    runCancel((SchedulerCallback) peer->run, peer);
    // deregister peer
    peer->id = 0;
    uint32_t slot = -1u;
    for(uint32_t i = 0; i < cntSources; i++) {
        if(sources[i] == peer) slot = i;
    }
    if(slot > cntSources) return;
    // compact source list
    while(++slot < cntSources)
        sources[slot-1] = sources[slot];
    // clear unused slots
    while(slot < MAX_NTP_SRCS)
        sources[slot++] = NULL;
    // reduce source count
    --cntSources;
}

static void ntpDnsCallback(void *ref, uint32_t addr) {
    // ignore if slots are full
    if(cntSources >= MAX_NTP_SRCS) return;
    // ignore own address
    if(addr == ipAddress) return;
    // verify address is not currently in use
    for(uint32_t i = 0; i < cntSources; i++) {
        if(sources[i]->mode != RPY_SD_MD_CLIENT)
            continue;
        if(sources[i]->id == addr)
            return;
    }
    // attempt to add as new source
    NtpSource *newSource = ntpAllocPeer();
    if(newSource != NULL) {
        newSource->id = addr;
        if(!IPv4_testSubnet(ipSubnet, ipAddress, addr)) {
            // faster initial burst for local timeservers
            newSource->poll = 1;
            newSource->minPoll = 2;
            newSource->maxPoll = 4;
        }
    }
}

static void runDnsFill(void *ref) {
    if(cntSources < MAX_NTP_SRCS) {
        // fill with DHCP provided addresses
        uint32_t *addr;
        int cnt;
        DHCP_ntpAddr(&addr, &cnt);
        for (int i = 0; i < cnt; i++)
            ntpDnsCallback(NULL, addr[i]);

        // fill with servers from public ntp pool
        DNS_lookup(NTP_POOL_FQDN, ntpDnsCallback, NULL);
    }
}

