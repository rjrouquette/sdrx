//
// Created by robert on 4/27/23.
//

#include <math.h>
#include "../clk/mono.h"
#include "../clk/util.h"
#include "../net/eth.h"
#include "../net/util.h"
#include "common.h"
#include "src.h"
#include "../format.h"
#include "../net.h"

static void getMeanVar(int cnt, const float *v, float *mean, float *var);

void incrFilter(PtpSource *this) {
    this->samplePtr = (this->samplePtr + 1) & (PTP_MAX_HISTORY - 1);
    if(++this->sampleCount > PTP_MAX_HISTORY)
        this->sampleCount = PTP_MAX_HISTORY;
}

void updateFilter(PtpSource *this) {
    const int head = this->samplePtr;
    PtpPollSample *sample = this->pollSample + head;
    this->lastOffsetOrig = toFloat(sample->offset);
    this->lastOffset = this->lastOffsetOrig;
    this->lastDelay = sample->delay;

    // compute time spanned by samples
    int j = (head - (this->sampleCount - 1)) & (PTP_MAX_HISTORY - 1);
    this->span = (int) ((sample->comp - this->pollSample[j].comp) >> 32);

    // convert offsets to floats
    int index[PTP_MAX_HISTORY];
    float offset[PTP_MAX_HISTORY];
    float delay[PTP_MAX_HISTORY];
    int cnt = this->sampleCount;
    for (int i = 0; i < cnt; i++) {
        int k = (head - i) & (PTP_MAX_HISTORY - 1);
        index[i] = k;
        offset[i] = toFloat(this->pollSample[k].offset);
        delay[i] = this->pollSample[k].delay;
    }

    // compute mean and variance
    float mean, var;
    getMeanVar(this->sampleCount, delay, &mean, &var);
    this->delayMean = mean;
    this->delayStdDev = sqrtf(var);

    // compute mean and variance
    getMeanVar(this->sampleCount, offset, &mean, &var);
    // remove extrema
    float limit = var * 4;
    if(limit > 0) {
        j = 0;
        for (int i = 0; i < cnt; i++) {
            float diff = offset[i] - mean;
            if ((diff * diff) < limit) {
                index[j] = index[i];
                offset[j] = offset[i];
                ++j;
            }
        }
        cnt = j;
        // recompute mean and variance
        if (cnt > 0)
            getMeanVar(cnt, offset, &mean, &var);
    }
    // update offset stats
    this->usedOffset = cnt;
    this->offsetMean = mean;
    this->offsetStdDev = sqrtf(var);

    // analyse clock drift
    --cnt;
    float drift[cnt];
    for (int i = 0; i < cnt; i++) {
        PtpPollSample *current = this->pollSample + index[i];
        PtpPollSample *previous = this->pollSample + index[i+1];

        // all three deltas are required to isolate drift from offset adjustments
        uint32_t a = current->taiSkew - previous->taiSkew;
        uint32_t b = ((uint32_t) current->offset) - ((uint32_t) previous->offset);
        drift[i] = 0x1p-32f * ((float) (int32_t) (a + b)) / toFloatU(current->comp - previous->comp);
    }
    // compute mean and variance
    getMeanVar(cnt, drift, &mean, &var);
    // exclude outliers
    limit = var * 4;
    if(limit > 0) {
        j = 0;
        for (int i = 0; i < cnt; i++) {
            float diff = drift[i] - mean;
            if ((diff * diff) < limit) {
                drift[j] = drift[i];
                ++j;
            }
        }
        cnt = j;
        // recompute mean and variance
        if (cnt > 0)
            getMeanVar(cnt, drift, &mean, &var);
    }
    // set frequency status
    this->usedDrift = cnt;
    this->freqDrift = mean;
    this->freqSkew = sqrtf(var);
    // set overall score
    float score = fabsf(0x1p-16f * (float) this->rootDelay);
    score += 0x1p-16f * (float) this->rootDispersion;
    score += this->delayMean;
    score += this->delayStdDev;
    score += this->offsetStdDev;
    this->score = score;

    // set update time
    this->lastUpdate = CLK_MONO();
}

static void getMeanVar(const int cnt, const float *v, float *mean, float *var) {
    // return zeros if count is less than one
    if(cnt < 1) {
        *mean = 0;
        *var = 0;
        return;
    }

    // compute the mean
    float _mean = 0;
    for(int k = 0; k < cnt; k++)
        _mean += v[k];
    _mean /= (float) cnt;

    // compute the variance
    float _var = 0;
    for(int k = 0; k < cnt; k++) {
        float diff = v[k] - _mean;
        _var += diff * diff;
    }
    _var /= (float) (cnt - 1);

    // return result
    *mean = _mean;
    *var = _var;
}


void PtpSource_updateStatus(PtpSource *this) {
    // clear lost flag if peer was reached
    if(this->reach & 0xF)
        this->lost = false;

    // increment poll counter
    if(++this->pollCounter == 0)
        this->pollCounter = -1;

    // adjust poll interval
    if(
            this->sampleCount == PTP_MAX_HISTORY &&
            (this->reach & 0xFF) == 0xFF &&
            this->pollCounter >= 8 &&
            this->poll < this->maxPoll
    ) {
        // reset the counter
        this->pollCounter = 0;
        // increase poll interval (increase update rate)
        ++this->poll;
        // sanity check
        if(this->poll < this->minPoll)
            this->poll = this->minPoll;
    }
    else if(
            (this->reach & 0xF) == 0 &&
            this->pollCounter >= 4
    ) {
        // mark association as lost
        this->lost = true;
        // reset the counter
        this->pollCounter = 0;
        // decrease poll interval (increase update rate)
        if(this->poll > this->minPoll)
            --this->poll;
    }

    // mark source for pruning if it is unreachable
    this->prune = (this->reach == 0) && (this->pollCounter >= 16);
    // mark source for pruning if its stratum is too high
    this->prune |= this->stratum > PTP_MAX_STRAT;
    // mark source for pruning if its delay is too high
    this->prune |= (this->usedOffset > 7) && (this->delayMean > PTP_MAX_DELAY);
}

static void doSync(PtpSource *this, PTP2_TIMESTAMP *ts) {
    // advance sample buffer
    incrFilter(this);
    // set current sample
    PtpPollSample *sample = this->pollSample + this->samplePtr;
    // set compensated reference time
    const uint64_t comp = this->syncRxStamps[1];
    sample->comp = comp;
    // set TAI reference time
    const uint64_t tai = this->syncRxStamps[2];
    sample->taiSkew = tai - comp;
    // compute TAI offset
    uint64_t offset = (fromPtpTimestamp(ts) - tai) + this->syncDelay;
    sample->offset = (int64_t) offset;
    // record current delay
    sample->delay = (float) this->syncDelay;
    // update filter
    updateFilter(this);
}

void PtpSource_init(PtpSource *this, uint8_t *frame, int flen) {
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_PTP *headerPTP = (HEADER_PTP *) (headerEth + 1);
    PTP2_ANNOUNCE *announce = (PTP2_ANNOUNCE *) (headerPTP + 1);

    this->mac[1] = 0;
    copyMAC(this->mac, headerEth->macSrc);
    toHex(__builtin_bswap16(this->mac[1] & 0xFFFF), 4, '0', (char *) &(this->id));
}

void PtpSource_run(PtpSource *this) {

}

void PtpSource_process(PtpSource *this, uint8_t *frame, int flen) {
    HEADER_ETH *headerEth = (HEADER_ETH *) frame;
    HEADER_PTP *headerPTP = (HEADER_PTP *) (headerEth + 1);

    if(headerPTP->messageType == PTP2_MT_SYNC) {
        // process sync message (defer filter update until followup message)
        this->syncSeq = headerPTP->sequenceId;
        NET_getRxTime(frame, this->syncRxStamps);
    }
    else if(headerPTP->messageType == PTP2_MT_FOLLOW_UP) {
        // process sync followup message
        if(this->syncSeq == headerPTP->sequenceId) {
            doSync(this, (PTP2_TIMESTAMP *) (headerPTP + 1));
        }
    }
}

void PtpSource_applyOffset(PtpSource *this, int64_t offset) {
    for (int i = 0; i < PTP_MAX_HISTORY; i++) {
        this->pollSample[i].taiSkew += offset;
        this->pollSample[i].offset -= offset;
    }
}
