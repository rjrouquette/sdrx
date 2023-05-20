//
// Created by robert on 4/26/23.
//

#ifndef GPSDO_CLK_MONO_H
#define GPSDO_CLK_MONO_H

#include <stdint.h>
#include "../hw/timer.h"

#define CLK_FREQ (125000000)
#define TIMER_MONO (GPTM0)

// raw internal monotonic clock state
extern volatile uint32_t clkMonoInt;
extern volatile uint32_t clkMonoOff;
// timer tick offset between the ethernet clock and monotonic clock
extern volatile uint32_t clkMonoEth;
// timer tick capture of the PPS output
extern volatile uint32_t clkMonoPps;

/**
 * Returns the current value of the system clock (1s resolution)
 * @return Raw 32-bit count of 1s ticks
 */
uint32_t CLK_MONO_INT();

/**
 * Returns the current value of the system clock (~0.232ns resolution)
 * @return 64-bit fixed-point format (32.32)
 */
uint64_t CLK_MONO();

/**
 * Returns the raw value of the system clock timer
 */
#define CLK_MONO_RAW (TIMER_MONO.TAV.raw)


#endif //GPSDO_CLK_MONO_H
