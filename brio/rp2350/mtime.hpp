/*
 * mtime.hpp
 *
 * The RISC-V platform timer (datasheet 3.1.8): a 64-bit counter in the
 * SIO block with one 64-bit comparator PER CORE, and the only piece of
 * this chip whose name says RISC-V while the hardware belongs to
 * everybody - the datasheet is explicit that it is "usable equally by
 * the Arm and RISC-V processors", and its comparator raises both the
 * hart's standard timer interrupt and the system interrupt line
 * SIO_IRQ_MTIMECMP that an NVIC can take.
 *
 * WHAT IT COUNTS is not cycles. MTIME_CTRL chooses between the system
 * clock and the TICK from the generator block of 8.5, and this file
 * takes the tick: with the generator dividing clk_ref down to one
 * microsecond, the counter reads MICROSECONDS and a change of clk_sys
 * does not move it. That is what makes it both a ruler a measurement can
 * trust across a rate change and, on the Hazard3 half, the kernel
 * timebase itself (rp2350/ticker.hpp).
 *
 * TWO CONSUMERS, ONE START. `start()` is idempotent and safe to call
 * from either of them: the ticker calls it before arming its comparator,
 * a program that only wants a ruler calls it and nothing else.
 *
 * DBGPAUSE IS CLEARED, both cores' bits. At reset the counter stops
 * while a core is halted in the debugger; a ruler that one core's
 * breakpoint can freeze for the other core is not a ruler, and the same
 * decision was taken on the RP2040's system timer for the same reason.
 *
 * THE READ AND THE WRITE ARE SEQUENCES, both 3.1.8's own: a 64-bit read
 * takes the high half, the low half and the high half again, and loops
 * if the two high reads differ; a comparator write puts all-ones in the
 * low half first, so that the pair is never briefly SMALLER than the
 * current time - which would raise an interrupt that nothing asked for.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "rp2350/clock.hpp"

namespace brio {

struct Mtime {
    Mtime() = delete;

    /// Start the counter on a one-microsecond tick divided out of
    /// clk_ref. False, and nothing started, when clk_ref is not a whole
    /// number of megahertz (the generator divides by a cycle COUNT, so a
    /// fractional microsecond has no honest setting) or when the
    /// generator did not start.
    template <typename C>
    static bool start(C) {
        const uint32_t ref_hz = C::ref_hz;
        if (ref_hz == 0u || (ref_hz % 1'000'000UL) != 0u) {
            return false;
        }
        if (!TickGenerator<TickConsumer::riscv>::start(ref_hz / 1'000'000UL)) {
            return false;
        }
        // EN set, FULLSPEED clear (count ticks, not system clock cycles),
        // both DBGPAUSE bits clear.
        SIO->MTIME_CTRL = SIO_MTIME_CTRL_EN_BITS;
        return true;
    }

    /// Is the counter enabled and its tick generator running?
    static bool running() {
        return (SIO->MTIME_CTRL & SIO_MTIME_CTRL_EN_BITS) != 0u &&
               TickGenerator<TickConsumer::riscv>::running();
    }

    /// The 64-bit counter, in microseconds, read against a rollover of
    /// its low half.
    static uint64_t now() {
        for (;;) {
            const uint32_t high = SIO->MTIMEH;
            const uint32_t low = SIO->MTIME;
            if (SIO->MTIMEH == high) {
                return (static_cast<uint64_t>(high) << 32) | low;
            }
        }
    }

    /// The low half alone: one bus read, and 71 minutes of range - what
    /// a span measured inside one program wants.
    static uint32_t micros() { return SIO->MTIME; }

    /// THIS CORE's comparator. The interrupt stands while the counter is
    /// greater than or equal to it.
    static void set_compare(uint64_t deadline) {
        SIO->MTIMECMP = 0xFFFFFFFFu;
        SIO->MTIMECMPH = static_cast<uint32_t>(deadline >> 32);
        SIO->MTIMECMP = static_cast<uint32_t>(deadline);
    }

    static uint64_t compare() {
        return (static_cast<uint64_t>(SIO->MTIMECMPH) << 32) | SIO->MTIMECMP;
    }

    /// Push the comparator out of reach: the interrupt cannot stand
    /// again until someone sets a deadline.
    static void disarm() { set_compare(0xFFFFFFFFFFFFFFFFull); }
};

} // namespace brio
