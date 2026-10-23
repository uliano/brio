/*
 * timestamp.hpp
 *
 * TimeStamp: a time since the ticker's init() as whole seconds and a
 * millisecond fraction, a pure value type. A target's ticker produces
 * it and util/print.hpp prints it as "<seconds>.<millis>s" (12.045s).
 * The smallest use, a console's uptime command:
 *
 *     brio::TimeStamp ts;
 *     brio::Ticker::now(ts);
 *     brio::print(s, "uptime: ", ts, brio::crlf);
 *
 * The value is the caller's, filled in by now() and owned by whoever
 * holds it; nothing here waits, masks or keeps a pointer. Where now()
 * may be called from, and how it reads the two counters at one instant,
 * is each ticker's own comment. TimeStamp lives in util/ so that
 * print.hpp never has to include a target header (docs/design/overview.md,
 * "Layering: the strata": util never depends on a target).
 *
 * The fraction is in milliseconds, not in ticks: a tick means something
 * different on every target (1/1024 s on one, 1/1000 s on another), so
 * a tick-based fraction would change meaning with the silicon.
 * Milliseconds are self-describing everywhere; the producing driver
 * converts with its own known rate. Sub-millisecond resolution is
 * not this type's job: the kernel's own count is the platform's
 * P::now() in ticks (kernel/platform.hpp), and a finer measurement
 * reads a cycle ruler, a ticker's cycles() where it has one
 * (docs/design/benchmark.md, "The ruler").
 */

#pragma once

#include <stdint.h>

namespace brio {

/// Wall-clock style timestamp: whole seconds + millisecond fraction,
/// counted from the producing ticker's init().
struct TimeStamp {
    uint32_t seconds;  ///< Whole seconds elapsed (wraps after ~136 years)
    uint16_t millis;   ///< Fractional second in milliseconds (0..999)
};

} // namespace brio
