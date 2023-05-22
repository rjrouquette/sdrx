//
// Created by robert on 5/22/23.
//

#include "util.h"

void getMeanVar(const int cnt, const float *v, float *mean, float *var) {
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
