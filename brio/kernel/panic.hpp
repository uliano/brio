/*
 * panic.hpp
 *
 * The kernel's unrecoverable-failure facility: ONE hook, pluggable
 * reporters. panic() masks interrupts for good, writes the breadcrumb
 * into the platform's reset-surviving storage, executes break_here()
 * (halts right on the culprit when a debugger is attached, free NOP
 * otherwise - it does not consume the hardware breakpoint slots), and
 * hands over to the app-chosen Reporter for the manifestation:
 *
 *  - HaltReporter (here): the minimal default - fall into cli+loop;
 *  - LED blinkers / watchdog resetters are TARGET code, provided by the
 *    target strata or by the app (the kernel knows no LED);
 *  - "blink then reset", "reset now and report at next boot" compose
 *    freely: the breadcrumb is written BEFORE any reporter runs, so the
 *    information is safe whatever the manifestation does.
 *
 * At boot the app calls take_panic_record<P>(): a valid record (magic
 * match) is returned once and cleared - cross-check the reset-cause
 * register on the target for the full story.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "kernel/platform.hpp"

namespace brio {

enum class PanicCode : uint8_t {
    none = 0,
    queue_overflow = 1,   ///< context: producer-defined queue/AO id
    assert_failed = 2,
    kernel_fault = 3,
};

/// Marks a PanicRecord as valid ("brio" flavored, unlikely in cold RAM).
inline constexpr uint16_t panic_magic = 0xB210;

/// The minimal reporter: report() returns and panic() falls into its
/// final forever-loop, interrupts masked.
struct HaltReporter {
    static void report(PanicCode, uint8_t) {}
};

/// Unrecoverable failure: breadcrumb -> debugger hook -> reporter.
/// Never returns; interrupts stay masked from here on.
template <Platform P, typename Reporter = HaltReporter>
[[noreturn]] void panic(PanicCode code, uint8_t context = 0) {
    typename P::CriticalSection cs;  // masked for good: never destroyed
    P::panic_record() = PanicRecord{
        panic_magic, static_cast<uint8_t>(code), context};
    P::break_here();
    Reporter::report(code, context);
    for (;;) {
    }
}

/// Boot-side: fetch-and-clear the breadcrumb of a previous panic, if any.
template <Platform P>
std::optional<PanicRecord> take_panic_record() {
    PanicRecord& r = P::panic_record();
    if (r.magic != panic_magic) {
        return std::nullopt;
    }
    const PanicRecord copy = r;
    r.magic = 0;
    return copy;
}

} // namespace brio
