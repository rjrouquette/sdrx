//
// Created by robert on 4/27/23.
//

#ifndef GPSDO_TAI_H
#define GPSDO_TAI_H

#include <stdint.h>

extern volatile uint64_t clkTaiUtcOffset;

extern volatile uint64_t clkTaiOffset;
extern volatile uint64_t clkTaiRef;
extern volatile int32_t clkTaiRate;

/**
 * Returns the current value of the TAI clock
 * @return 64-bit fixed-point format (32.32)
 */
uint64_t CLK_TAI();

/**
 * Translate monotonic timestamp to TAI timestamp
 * @param ts timestamp to translate (32.32)
 * @return 64-bit fixed-point format (32.32)
 */
uint64_t CLK_TAI_fromMono(uint64_t ts);

/**
 * Adjust TAI reference time
 * @param seconds 64-bit fixed-point format (31.32)
 */
void CLK_TAI_adjust(int64_t seconds);

/**
 * Trim TAI clock frequency
 * @param trim new trim rate (0.31 fixed-point)
 */
void CLK_TAI_setTrim(int32_t trim);

/**
 * Get current trim rate for TAI clock frequency
 * @return current trim rate (0.31 fixed-point)
 */
int32_t CLK_TAI_getTrim();

#endif //GPSDO_TAI_H
