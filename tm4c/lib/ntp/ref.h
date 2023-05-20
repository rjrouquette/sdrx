//
// Created by robert on 4/27/23.
//

#ifndef GPSDO_REF_H
#define GPSDO_REF_H

#include "../ptp/src.h"

struct NtpGPS {
    NtpSource source;

    uint64_t lastPoll;
    uint64_t lastPps;
};
typedef struct NtpGPS NtpGPS;

/**
 * Initialize GPS structure
 * @param pObj pointer to GPS structure
 */
void NtpGPS_init(void *pObj);

#endif //GPSDO_REF_H