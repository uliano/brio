/*
 * timestamp.hpp
 *
 * TimeStamp: a pure value type shared between the timebase driver (which
 * produces it - e.g. avrdx/ticker.hpp) and the formatting layer (which
 * prints it - util/print.hpp). It lives in util/ so that print.hpp never
 * has to include a target header: layering rule, util never depends on a
 * target.
 *
 * The fraction is in MILLISECONDS, not in ticks: a tick means something
 * different on every target (1/1024 s on AVR Dx, 1/1000 s on SysTick
 * targets), so a tick-based fraction would change meaning with the
 * silicon. Milliseconds are self-describing everywhere; the producing
 * driver converts with its own known rate. Sub-millisecond resolution is
 * deliberately NOT this type's job - raw Platform::now() ticks serve
 * fine-grained measurement.
 */

#pragma once

#include <stdint.h>

namespace brio {

/**
 * @struct TimeStamp
 * @brief Wall-clock style timestamp: whole seconds + millisecond fraction
 */
struct TimeStamp {
    uint32_t seconds;  ///< Whole seconds elapsed (wraps after ~136 years)
    uint16_t millis;   ///< Fractional second in milliseconds (0..999)
};

} // namespace brio
