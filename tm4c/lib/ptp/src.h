//
// Created by robert on 4/27/23.
//

#ifndef GPSDO_SRC_H
#define GPSDO_SRC_H

#include <stdint.h>
#include <stdbool.h>

#define PTP_MAX_HISTORY (16)
#define PTP_MAX_STRAT (3)
#define PTP_MAX_DELAY (50e-3f)

struct PtpPollSample {
    int64_t offset;
    float delay;
    uint32_t taiSkew;
    uint64_t comp;
};
typedef struct PtpPollSample PtpPollSample;

struct PtpSource {
    // filter samples
    struct PtpPollSample pollSample[PTP_MAX_HISTORY];
    uint8_t samplePtr;
    uint8_t sampleCount;
    uint8_t usedOffset;
    uint8_t usedDrift;
    // time span
    int span;

    uint64_t lastUpdate;
    uint32_t id;
    uint32_t rootDelay;
    uint32_t rootDispersion;
    uint32_t rxCount;
    uint32_t rxValid;
    uint32_t txCount;
    uint16_t state;
    uint16_t reach;
    uint16_t stratum;
    int16_t poll;
    uint16_t pollCounter;
    int16_t minPoll;
    int16_t maxPoll;

    // last sample offset
    float lastOffset;
    float lastOffsetOrig;
    float lastDelay;
    // offset stats
    float offsetMean;
    float offsetStdDev;
    // delay stats
    float delayMean;
    float delayStdDev;
    // frequency stats
    float freqDrift;
    float freqSkew;
    // overall score
    float score;

    // status flags
    bool prune;
    bool lost;
};
typedef struct PtpSource PtpSource;

/**
 * Initialize source structure
 * @param this pointer to source structure
 */
void PtpSource_init(PtpSource *this);

/**
 * Perform internal state update for source
 * @param this pointer to source structure
 */
void PtpSource_run(PtpSource *this);

/**
 * Apply offset correction to samples
 * @param this pointer to source structure
 * @param offset correction to apply
 */
void PtpSource_applyOffset(PtpSource *this, int64_t offset);

#endif //GPSDO_SRC_H