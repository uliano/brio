/*
 * timestamp.hpp
 *
 * TimeStamp: a pure value type shared between the timebase driver (which
 * produces it - e.g. avrdx/ticker.hpp) and the formatting layer (which
 * prints it - util/print.hpp). It lives in util/ so that print.hpp never
 * has to include a target header: layering rule, util never depends on a
 * target.
 */

#pragma once

#include <stdint.h>

namespace brio {

/**
 * @struct TimeStamp
 * @brief High-precision timestamp: whole seconds + fractional ticks
 *
 * fraction = ticks / ticks_per_second. Unlike millis(), this representation
 * carries no jitter from the decimal-millisecond correction.
 */
struct TimeStamp {
    uint32_t seconds;  ///< Whole seconds elapsed (wraps after ~136 years)
    uint16_t ticks;    ///< Fractional second in ticks (0 .. ticks_per_second-1)
};

} // namespace brio
