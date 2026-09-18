/*
 * watchdog.hpp
 *
 * The watchdog block (datasheet 12.9), which on this chip is two things
 * under one name where the RP2040's was three:
 *
 *  - THE COUNTDOWN (12.9.3): a 24-bit counter loaded through LOAD,
 *    decremented on its tick, resetting whatever the three WDSEL
 *    registers select when it reaches zero, or at once on CTRL.TRIGGER.
 *    The pause bits hold it while a debugger has a core halted or is
 *    driving the bus fabric; REASON says whether the last watchdog reset
 *    was the countdown or the trigger.
 *  - EIGHT SCRATCH REGISTERS (12.9.5) that survive a system or a
 *    subsystem reset. THE BOOTROM READS SCRATCH4..7 at every boot for a
 *    magic word that redirects the boot into user code (5.2.4), and takes
 *    SCRATCH2/3 as that redirection's two parameters when it fires - so
 *    brio hands out SCRATCH0..3 and never writes the upper four, which is
 *    what leaves 2 and 3 "free for arbitrary user values" in 5.2.4's own
 *    words.
 *
 * THE TICK IS NOT THIS BLOCK'S ANY MORE (12.9.2). On the RP2040 the
 * watchdog owned a tick generator and lent it to the system timer; here
 * every consumer has its own generator in the TICKS block of 8.5, and
 * this one's is `TickGenerator<TickConsumer::watchdog>` (rp2350/clock.hpp)
 * - which is why `Watchdog::init` exists at all, and why a program can
 * run a timer with no watchdog or a watchdog with no timer.
 *
 * ERRATUM RP2040-E1 IS NOT THIS CHIP'S. The RP2040's counter decremented
 * twice per tick, so every LOAD was doubled and its reach halved. Here
 * LOAD is microseconds as written and the register's 0xffffff is about
 * 16.8 seconds, which is what 12.9's own LOAD description states. The
 * suite measures the decrement rate against the system timer rather than
 * trusting either sentence.
 *
 * WHAT A TIME-OUT RESETS is three registers, in three tiers (7.1), and
 * this driver writes ONE of them:
 *  - PSM_WDSEL, the SYSTEM tier: which stage of the power-on state
 *    machine the sequence restarts from. `start` and `force_reset` set it
 *    to everything but the two oscillators (`PsmStage::reboot`), so the
 *    chip comes back through the bootrom as a power-on would with the
 *    crystal still running. This is the tier brio reboots through.
 *  - RESETS_WDSEL, the SUBSYSTEM tier: which peripheral blocks go down
 *    with it. rp2350/resets.hpp's `Resets::watchdog_resets` writes it; a
 *    reboot through the PSM takes the whole reset controller down anyway.
 *  - POWMAN_WDSEL, the CHIP tier, which is new on this chip: a watchdog
 *    event can reset the power manager, the switched core domain or the
 *    chip. That register is password-protected (6.4) and belongs to the
 *    power chapter; nothing here writes POWMAN. The difference matters to
 *    a program and not only to a driver: a CHIP-level watchdog reset also
 *    clears the scratch registers below (7.2), where a system-level one
 *    is documented not to.
 *
 * ERRATUM RP2350-E19 IS LIVE ON STEPPING A2: a reboot hangs in the boot
 * path if any bit but FRCE_OFF.PROC1 is set in the power-on state
 * machine's hold register when it happens. Nothing in brio sets one, but
 * a previous life can (a debugger's scripts do), and the cost of being
 * sure is one store: every path here that can end in a reboot clears
 * FRCE_OFF except PROC1 first.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "rp2350/clock.hpp"
#include "rp2350/resets.hpp"

namespace brio {

/// Why the watchdog last fired (WATCHDOG.REASON).
///
/// It is read-only and it stands until a chip-level reset - OR until a
/// debugger warm-resets either core, which on this chip (and not on the
/// RP2040) CLEARS IT, so that code loaded under a probe after a time-out
/// does not keep seeing the time-out.
struct WatchdogReason {
    static constexpr uint32_t timer = WATCHDOG_REASON_TIMER_BITS;   ///< the countdown reached zero
    static constexpr uint32_t force = WATCHDOG_REASON_FORCE_BITS;   ///< CTRL.TRIGGER
};

/// The countdown, a monostate.
struct Watchdog {
    Watchdog() = delete;

    /// The longest time-out the 24-bit counter holds, in microseconds -
    /// about 16.8 s, and here it is the register's own range, the
    /// RP2040's halving being that chip's erratum and not this one's.
    static constexpr uint32_t max_timeout_us = WATCHDOG_LOAD_BITS;

    /// This block's tick generator in the TICKS block, for a program that
    /// wants to read or stop it: the countdown does not move while it is
    /// stopped.
    using Tick = TickGenerator<TickConsumer::watchdog>;

    /// Start this block's tick generator at one microsecond from clk_ref
    /// (8.5). Idempotent, and independent of the system timers' own
    /// generators. False when clk_ref is not a whole number of megahertz
    /// or the generator did not start.
    template <typename C>
    static bool init(C) {
        static_assert(C::ref_hz % 1'000'000UL == 0u,
                      "brio Watchdog: the 1 us tick divides clk_ref by a whole number of "
                      "cycles, so clk_ref must be a whole number of megahertz");
        return Tick::start(C::ref_hz / 1'000'000UL);
    }

    /// Start the countdown at `timeout_us` (longer than max_timeout_us is
    /// clamped) and select what a time-out resets: every stage of the
    /// power-on state machine but the two oscillators, so the chip
    /// reboots through the bootrom. `pause_on_debug` holds the count
    /// while a debugger has a core halted or is driving the bus fabric
    /// (CTRL.PAUSE_DBG0/1 and PAUSE_JTAG), the right default on a bench.
    /// The tick must be running (init()).
    static void start(uint32_t timeout_us, bool pause_on_debug = true) {
        hw_clear(WATCHDOG->CTRL, WATCHDOG_CTRL_ENABLE_BITS);
        arm_psm();
        constexpr uint32_t dbg = WATCHDOG_CTRL_PAUSE_DBG0_BITS | WATCHDOG_CTRL_PAUSE_DBG1_BITS |
                                 WATCHDOG_CTRL_PAUSE_JTAG_BITS;
        if (pause_on_debug) {
            hw_set(WATCHDOG->CTRL, dbg);
        } else {
            hw_clear(WATCHDOG->CTRL, dbg);
        }
        load_ = timeout_us > max_timeout_us ? max_timeout_us : timeout_us;
        WATCHDOG->LOAD = load_;
        hw_set(WATCHDOG->CTRL, WATCHDOG_CTRL_ENABLE_BITS);
    }

    /// Reload the countdown: the kick.
    static void kick() { WATCHDOG->LOAD = load_; }

    /// Stop the countdown. Unlike the other families' watchdogs this one
    /// is not one-way: ENABLE clears, and a stopped countdown stays
    /// stopped.
    static void stop() { hw_clear(WATCHDOG->CTRL, WATCHDOG_CTRL_ENABLE_BITS); }
    static bool running() { return (WATCHDOG->CTRL & WATCHDOG_CTRL_ENABLE_BITS) != 0u; }

    /// Microseconds left before the reset (CTRL.TIME), in the register's
    /// own units.
    static uint32_t remaining_us() { return WATCHDOG->CTRL & WATCHDOG_CTRL_TIME_BITS; }

    /// The time-out last written, after clamping: what kick() reloads.
    static uint32_t timeout_us() { return load_; }

    /// Which stages of the power-on state machine a watchdog event
    /// restarts from (PSM_WDSEL, through rp2350/resets.hpp). A program
    /// that wants a narrower reboot - one core, say - writes it after
    /// start(), which sets the whole-chip selection.
    static void system_resets(uint32_t stages) { Psm::watchdog_resets(stages); }
    static uint32_t system_resets() { return Psm::watchdog_resets(); }

    /// Reset NOW through the watchdog's path (CTRL.TRIGGER), REASON.FORCE
    /// at the next boot. The power-on state machine's selection is
    /// written here as start() writes it - everything but the two
    /// oscillators - so the trigger reboots the whole chip whether or not
    /// a countdown was ever started: this is the chip's one software
    /// reboot (rp2350/reset.hpp's Reset::software()).
    [[noreturn]] static void force_reset() {
        arm_psm();
        hw_set(WATCHDOG->CTRL, WATCHDOG_CTRL_TRIGGER_BITS);
        for (;;) {
        }
    }

    /// WATCHDOG.REASON: WatchdogReason bits, zero after any other reset.
    static uint32_t reason() { return WATCHDOG->REASON & WATCHDOG_REASON_BITS; }

private:
    /// The selection a reboot needs, and erratum RP2350-E19's guard with
    /// it (the file header): the hold register clear but for PROC1.
    static void arm_psm() {
        Psm::release(PsmStage::all & ~PsmStage::proc1);
        Psm::watchdog_resets(Psm::watchdog_resets() | PsmStage::reboot);
    }

    static inline uint32_t load_ = 0;
};

/// The four scratch words brio may use (SCRATCH0..3): storage the
/// datasheet says outlives a system or a subsystem reset and dies with
/// the RUN pin, the supply, or a watchdog event configured to reset the
/// CHIP (7.2). WHETHER IT OUTLIVES THE REBOOT THIS FILE MAKES IS NOT
/// SETTLED BY THE DOCUMENT: 7.3.1's table lists that reboot among the
/// chip-level causes and says every one of them resets this block,
/// SCRATCH0..7 included, where 7.2's note says the opposite. A program
/// that must carry a value across it carries a second copy in .noinit
/// until the bench says which is true (docs/rp2350/watchdog.md).
/// SCRATCH4..7 carry the bootrom's boot redirection and are not offered.
template <uint8_t n>
struct Scratch {
    static_assert(n < 4,
                  "brio Scratch: SCRATCH0..3 are the program's; SCRATCH4..7 carry the "
                  "bootrom's boot magic, entry point and stack pointer (datasheet 5.2.4)");
    Scratch() = delete;

    static volatile uint32_t& reg() {
        return reg_at(WATCHDOG_BASE, WATCHDOG_SCRATCH0_OFFSET + 4u * static_cast<uint32_t>(n));
    }
    static uint32_t read() { return reg(); }
    static void write(uint32_t v) { reg() = v; }
};

} // namespace brio
