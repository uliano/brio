/*
 * panic.hpp
 *
 * panic(): what a program calls when it cannot continue. It never
 * returns. In order, it masks interrupts for good, writes a breadcrumb
 * (the code and a byte of context) into storage the platform keeps
 * across a reset, executes the platform's break_here(), and hands over
 * to a Reporter the application chooses:
 *
 *  - HaltReporter, the default defined here: report() returns and
 *    panic() spins forever with interrupts masked;
 *  - an LED blinker or a watchdog reset is target or application code,
 *    since the kernel knows no LED. A reporter that resets the chip is a
 *    `ResetReporter` in the reset header of the target strata that have
 *    one; the avrxt and ch32x035 strata have none, and a program there
 *    writes its own;
 *  - "blink then reset" and "reset now, report at next boot" compose
 *    freely, because the breadcrumb is written before any reporter runs
 *    and survives whatever the reporter does.
 *
 * What break_here() does with no debugger attached is the core's
 * business, not promised here (kernel/platform.hpp): a no-op on the AVR,
 * a HardFault on the Cortex-M families, a breakpoint exception on the
 * QingKe cores. A reporter that must run without a debugger is reached
 * only where the instruction returns; where it faults, the fault handler
 * is the path that runs, and the target's reset header says what it does
 * with the breadcrumb.
 *
 * At boot the application calls take_panic_record<P>(): a valid record
 * is returned once and cleared. The target's reset-cause register tells
 * the rest of the story.
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

/// Unrecoverable failure: breadcrumb, then break_here(), then the
/// reporter. Never returns; interrupts stay masked from here on. Any
/// context.
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

/// At boot: the breadcrumb of a previous panic, if any, returned once
/// and cleared. Empty when the stored record carries no valid magic.
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
