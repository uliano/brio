/*
 * timer.hpp
 *
 * The system timers (datasheet 12.8) - TWO of them here, TIMER0 and
 * TIMER1, where the RP2040 had one. Each is a 64-bit counter of
 * microseconds, monotonic for thousands of years, read through a latching
 * register pair or two raw halves, with four alarms that match the low 32
 * bits and raise one interrupt line each. Two instances exist so that two
 * security domains can each own a timebase; brio uses neither security
 * domain, so what the second one is here is a SECOND RULER - one a
 * measurement can stop, lock or take off the microsecond tick without
 * disturbing the one every other letter of a suite is judged against.
 *
 * WHAT IT COUNTS COMES FROM ANOTHER BLOCK. On the RP2040 the tick was the
 * watchdog's; here every consumer of a timebase has its own generator in
 * the TICKS block of 8.5 - one for each system timer, one for the
 * watchdog, one for each core's SysTick, one for the RISC-V platform
 * timer - and `TickGenerator<TickConsumer::timer0>` (rp2350/clock.hpp) is
 * this instance's. `init` starts it at clk_ref divided to one microsecond,
 * which with clk_ref on the crystal makes this THE CHIP'S RULER: a
 * microsecond wall clock independent of clk_sys. It shares the crystal
 * with everything else, so it judges ratios and arithmetic, never the
 * crystal itself.
 *
 * READING 64 BITS ON A 32-BIT BUS. TIMELR latches TIMEHR until TIMEHR is
 * read - correct for one reader, wrong the moment a second context (the
 * other core, a handler over the loop) reads TIMELR in between. `now()`
 * therefore reads the RAW pair: high, low, high again, repeat if the high
 * half moved. `now_low()` is the low half alone, enough for any span
 * under 71 minutes and what a bracket wants.
 *
 * ALARMS match the LOW 32 bits, so at most 2^32 us - about 71 minutes -
 * ahead. Writing ALARMn arms it (ARMED's bit set); the match clears ARMED
 * and raises INTR's bit, a level cleared by writing 1 to it; ARMED is
 * written with 1 to disarm early. The interrupt line is INTR gated by
 * INTE (INTF forces, INTS is the masked status). THE ALARM INDEX IS A
 * TEMPLATE PARAMETER and not an argument, because an alarm is claimed at
 * BUILD time: what makes alarm 2 of TIMER1 this program's is the app
 * binding the vector `isr_timer1_2`, a name the linker resolves. An index
 * the hardware has not got is then a compile error and not a silent write
 * into the next register.
 *
 * TWO REGISTERS THE RP2040 HAD NOT:
 *  - SOURCE (12.8.1.1) takes the counter OFF the tick and onto clk_sys
 *    cycles. It is not a timebase then - it moves with every rate change
 *    - but it is the finest counter this chip offers a program, and what
 *    a cycle-level measurement wants.
 *  - LOCKED disables every write to the block, and IT CANNOT BE CLEARED.
 *    The way back is the subsystem reset controller (rp2350/resets.hpp):
 *    `Resets::cycle(ResetBlock::timer1)` gives an unlocked block with its
 *    counter at zero, and re-running `init` starts it again.
 *
 * DBGPAUSE's two bits, SET AT RESET, stop the count while a debugger has
 * a core halted - and this chip has two cores per architecture: with the
 * bits set, a breakpoint on one core, or a fault taken by a core a probe
 * left debug-enabled, FREEZES THE RULER OF THE OTHER CORE, whose every
 * timed wait then never ends. So `init` CLEARS them, as the RP2040's
 * driver does and for the reason that chip measured: the timer counts
 * through a halt, a measurement across a halt is a measurement of the
 * halt, and a program that wants the datasheet's default asks for it with
 * debug_pause().
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/resets.hpp"

namespace brio {

/// What a system timer counts (SOURCE, 12.8.1.1).
enum class TimerSource : uint8_t {
    tick = 0,       ///< the microsecond tick of 8.5: a timebase
    sysclk = 1,     ///< clk_sys cycles: a cycle counter, not a timebase
};

/**
 * One system timer, TIMER0 or TIMER1.
 *
 * `Timer<0>` is what a program means by "the ruler" unless it says
 * otherwise; `Timer<1>` is the same hardware at another address, with
 * four interrupt lines of its own.
 */
template <uint8_t n>
struct Timer {
    static_assert(n < 2u,
                  "brio Timer: the RP2350 has two system timers, TIMER0 and TIMER1 "
                  "(datasheet 12.8) - the RP2040's single instance is Timer<0> here");
    Timer() = delete;

    static constexpr uint8_t alarm_count = 4;

    /// This instance's tick generator in the TICKS block (8.5).
    static constexpr TickConsumer tick_consumer =
        n == 0u ? TickConsumer::timer0 : TickConsumer::timer1;
    using Tick = TickGenerator<tick_consumer>;

    /// This instance's reset line.
    static constexpr uint32_t reset_block = n == 0u ? ResetBlock::timer0 : ResetBlock::timer1;

    /// The block from its reset state, DBGPAUSE cleared (the file header)
    /// and the tick started at one microsecond from clk_ref, which
    /// rp2350/clock.hpp's Clock puts on the crystal undivided: the
    /// divider is the crystal's megahertz.
    ///
    /// THE RESET IS CYCLED AND NOT MERELY RELEASED. A debugger's reset
    /// request on this chip resets the cores alone, so an image ordinarily
    /// starts on a previous life's peripherals - and this block has one
    /// bit, LOCKED, that refuses every write including its own clear. A
    /// release that found it set would leave `init` writing into a block
    /// that ignores it and reporting success. The price is the counter,
    /// which starts from zero, and an init means exactly that.
    ///
    /// False when the block did not come ready or the generator did not
    /// start. A clk_ref that is not a whole number of megahertz does not
    /// compile: the generator divides by a CYCLE COUNT, so a fractional
    /// microsecond has no honest setting.
    template <typename C>
    static bool init(C) {
        static_assert(C::ref_hz % 1'000'000UL == 0u,
                      "brio Timer: the 1 us tick divides clk_ref by a whole number of "
                      "cycles, so clk_ref must be a whole number of megahertz");
        if (!Resets::cycle(reset_block)) {
            return false;
        }
        source(TimerSource::tick);
        debug_pause(false, false);
        return Tick::start(C::ref_hz / 1'000'000UL);
    }

    // ---- the counter ---------------------------------------------------------

    /// Microseconds since the tick started, 64 bits, safe from any
    /// context (the raw-pair discipline in the file header).
    static uint64_t now() {
        uint32_t hi = block().TIMERAWH;
        for (;;) {
            const uint32_t lo = block().TIMERAWL;
            const uint32_t hi2 = block().TIMERAWH;
            if (hi == hi2) {
                return (static_cast<uint64_t>(hi) << 32) | lo;
            }
            hi = hi2;
        }
    }
    /// The low 32 bits alone: 71 minutes of unsigned span.
    static uint32_t now_low() { return block().TIMERAWL; }

    /// The latching read, ONE CONTEXT ONLY (the file header): TIMELR
    /// then TIMEHR.
    static uint64_t now_latched() {
        const uint32_t lo = block().TIMELR;
        const uint32_t hi = block().TIMEHR;
        return (static_cast<uint64_t>(hi) << 32) | lo;
    }

    // ---- the alarms ----------------------------------------------------------

    /// Arm alarm `a` to match the low 32 bits at `at`.
    template <uint8_t a>
    static void alarm(uint32_t at) {
        check_alarm<a>();
        alarm_reg<a>() = at;
    }
    /// Arm alarm `a` `us` microseconds from now.
    template <uint8_t a>
    static void alarm_in(uint32_t us) {
        check_alarm<a>();
        alarm_reg<a>() = now_low() + us;
    }
    /// The value the alarm will match: a plain read-back of ALARMa.
    template <uint8_t a>
    static uint32_t alarm_at() {
        check_alarm<a>();
        return alarm_reg<a>();
    }
    template <uint8_t a>
    static bool armed() {
        check_alarm<a>();
        return (block().ARMED & bit<a>()) != 0u;
    }
    /// Disarm before the match: ARMED is write-one-to-clear.
    template <uint8_t a>
    static void disarm() {
        check_alarm<a>();
        block().ARMED = bit<a>();
    }

    /// The alarm's interrupt enable (INTE), through the atomic aliases so
    /// that arming one line never disturbs another.
    template <uint8_t a>
    static void interrupt(bool on) {
        check_alarm<a>();
        if (on) {
            hw_set(block().INTE, bit<a>());
        } else {
            hw_clear(block().INTE, bit<a>());
        }
    }
    /// Raised (INTR), whether enabled or not.
    template <uint8_t a>
    static bool raised() {
        check_alarm<a>();
        return (block().INTR & bit<a>()) != 0u;
    }
    /// The masked status (INTS): what actually reaches the controller.
    template <uint8_t a>
    static bool pending() {
        check_alarm<a>();
        return (block().INTS & bit<a>()) != 0u;
    }
    /// Clear the raised flag: the handler's first act.
    template <uint8_t a>
    static void clear() {
        check_alarm<a>();
        block().INTR = bit<a>();
    }
    /// Raise it in software (INTF), for a test of the path.
    template <uint8_t a>
    static void force(bool on) {
        check_alarm<a>();
        if (on) {
            hw_set(block().INTF, bit<a>());
        } else {
            hw_clear(block().INTF, bit<a>());
        }
    }

    /// The interrupt line alarm `a` raises: TIMER0_IRQ_0..3 are lines
    /// 0..3 and TIMER1_IRQ_0..3 lines 4..7, one numbering for both
    /// architectures (3.8.4.2).
    template <uint8_t a>
    static constexpr IRQn_Type irq() {
        check_alarm<a>();
        return static_cast<IRQn_Type>(static_cast<int>(TIMER0_IRQ_0_IRQn) +
                                      static_cast<int>(alarm_count) * n + a);
    }

    // ---- pause, source, lock --------------------------------------------------

    /// Stop the count where it stands; false takes it up again from
    /// there, with no catching up.
    static void pause(bool on) { block().PAUSE = on ? TIMER_PAUSE_BITS : 0u; }
    static bool paused() { return (block().PAUSE & TIMER_PAUSE_BITS) != 0u; }

    /// Whether a core halted by a debugger stops the count (DBG0, DBG1):
    /// both set at reset, both cleared by init() (the file header).
    static void debug_pause(bool core0, bool core1) {
        block().DBGPAUSE = (core0 ? TIMER_DBGPAUSE_DBG0_BITS : 0u) |
                           (core1 ? TIMER_DBGPAUSE_DBG1_BITS : 0u);
    }
    /// The readback for one core. `core` is a RUN-TIME number because it
    /// is ordinarily `core_id()`; 0 reads DBG0 and anything else DBG1,
    /// this chip having two cores per architecture and no third bit.
    static bool debug_paused(uint8_t core) {
        const uint32_t bits = core == 0u ? TIMER_DBGPAUSE_DBG0_BITS : TIMER_DBGPAUSE_DBG1_BITS;
        return (block().DBGPAUSE & bits) != 0u;
    }

    /// What the counter counts (the file header): the microsecond tick,
    /// or clk_sys cycles. Switching does not reset the count - the unit
    /// under it changes and nothing else.
    static void source(TimerSource src) {
        block().SOURCE = static_cast<uint32_t>(src) & TIMER_SOURCE_BITS;
    }
    static TimerSource source() {
        return (block().SOURCE & TIMER_SOURCE_BITS) != 0u ? TimerSource::sysclk
                                                          : TimerSource::tick;
    }

    /// Whether writes to this block are refused (LOCKED).
    static bool locked() { return (block().LOCKED & TIMER_LOCKED_BITS) != 0u; }
    /// REFUSE EVERY FURTHER WRITE TO THIS TIMER, FOR GOOD. Reads go on
    /// working; no write does, the lock's own included. The one way back
    /// is the subsystem reset controller - `Resets::cycle(reset_block)`
    /// and `init` again - which costs the counter's value.
    static void lock() { block().LOCKED = TIMER_LOCKED_BITS; }

private:
    static constexpr uint32_t base = n == 0u ? TIMER0_BASE : TIMER1_BASE;
    static TIMER0_Type& block() { return *reinterpret_cast<TIMER0_Type*>(base); }

    template <uint8_t a>
    static constexpr void check_alarm() {
        static_assert(a < alarm_count,
                      "brio Timer: each system timer has four alarms, 0..3 "
                      "(datasheet 12.8.3)");
    }
    template <uint8_t a>
    static constexpr uint32_t bit() {
        return 1UL << a;
    }
    template <uint8_t a>
    static volatile uint32_t& alarm_reg() {
        return reg_at(base, TIMER_ALARM0_OFFSET + 4u * static_cast<uint32_t>(a));
    }
};

} // namespace brio
