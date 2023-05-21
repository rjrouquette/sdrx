//
// Created by robert on 4/27/23.
//

#ifndef GPSDO_SRC_H
#define GPSDO_SRC_H

#include <stdint.h>
#include <stdbool.h>

#define PTP_MAX_HISTORY (32)

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

    uint32_t mac[2];
    uint64_t delayTxStamps[3];
    uint64_t syncRxStamps[3];
    uint64_t lastUpdate;
    uint32_t syncDelay;
    uint32_t seqId;
    uint32_t id;
    uint32_t rootDelay;
    uint32_t rootDispersion;
    uint16_t syncSeq;
    uint16_t state;
    uint16_t reach;
    int16_t poll;

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

};
typedef struct PtpSource PtpSource;

/**
 * Initialize source structure
 * @param this pointer to source structure
 * @param frame frame buffer
 * @param flen length of frame
 */
void PtpSource_init(PtpSource *this, uint8_t *frame, int flen);

/**
 * Perform internal state update for source
 * @param this pointer to source structure
 */
void PtpSource_run(PtpSource *this);

/**
 * Process received frame
 * @param this pointer to source structure
 * @param frame frame buffer
 * @param flen length of frame
 */
void PtpSource_process(PtpSource *this, uint8_t *frame, int flen);

/**
 * Apply offset correction to samples
 * @param this pointer to source structure
 * @param offset correction to apply
 */
void PtpSource_applyOffset(PtpSource *this, int64_t offset);

#endif //GPSDO_SRC_H
