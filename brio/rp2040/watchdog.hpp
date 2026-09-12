/*
 * watchdog.hpp
 *
 * The watchdog block (datasheet 4.7), which on this chip is three things
 * under one name:
 *
 *  - THE TICK GENERATOR (4.7.2): clk_ref divided by TICK.CYCLES into the
 *    nominal 1 us tick that the watchdog counts - AND THAT THE SYSTEM
 *    TIMER COUNTS (4.6.4: "the watchdog tick must be running for the
 *    timer to start counting"). Nothing starts it at boot but software,
 *    so `WatchdogTick::start` is the first act of rp2040/timer.hpp's
 *    init and the reason this file sits below it;
 *  - THE COUNTDOWN (4.7.3): a 24-bit counter loaded through LOAD,
 *    decremented on the tick, resetting through the power-on state
 *    machine and the reset controller (whatever their WDSEL registers
 *    select) when it reaches zero, or at once on CTRL.TRIGGER. Erratum
 *    RP2040-E1: it decrements TWICE per tick, so every value written to
 *    LOAD is doubled here and the chapter's 8.3 s maximum is the truth;
 *  - EIGHT SCRATCH REGISTERS (4.7.4) that survive every reset but a RUN
 *    pin or a power cycle: this chip's second reset-surviving store
 *    beside the .noinit SRAM. THE BOOTROM READS SCRATCH4..7 at every
 *    boot for a magic word that redirects the boot into user code
 *    (2.8.1.1), so brio hands out SCRATCH0..3 alone and never writes
 *    the upper four.
 *
 * What a watchdog reset resets is PSM_WDSEL's and RESETS_WDSEL's
 * business: `Watchdog::start` follows the SDK and selects everything
 * but the two oscillators in the PSM, so a time-out reboots through the
 * bootrom as a power-on would, the oscillators kept.
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

namespace brio {

/// The 1 us reference the watchdog and the system timer count.
struct WatchdogTick {
    WatchdogTick() = delete;

    /// Start the tick from clk_ref divided by `cycles` (1..511): 12 for
    /// a 12 MHz crystal on clk_ref. False when `cycles` does not fit
    /// TICK.CYCLES. Restarting rewrites the divider.
    static bool start(uint16_t cycles) {
        if (cycles == 0u || cycles > (WATCHDOG_TICK_CYCLES_BITS >> WATCHDOG_TICK_CYCLES_LSB)) {
            return false;
        }
        WATCHDOG->TICK = WATCHDOG_TICK_ENABLE_BITS | cycles;
        return true;
    }
    static void stop() { hw_clear(WATCHDOG->TICK, WATCHDOG_TICK_ENABLE_BITS); }
    static bool running() { return (WATCHDOG->TICK & WATCHDOG_TICK_RUNNING_BITS) != 0u; }
    static uint16_t cycles() {
        return static_cast<uint16_t>(WATCHDOG->TICK & WATCHDOG_TICK_CYCLES_BITS);
    }
    /// The divider's live count (TICK.COUNT), a debugging aid.
    static uint16_t count() {
        return static_cast<uint16_t>((WATCHDOG->TICK & WATCHDOG_TICK_COUNT_BITS) >>
                                     WATCHDOG_TICK_COUNT_LSB);
    }
};

/// Why the watchdog last fired (WATCHDOG.REASON), sticky until the next
/// chip-level reset.
struct WatchdogReason {
    static constexpr uint32_t timer = WATCHDOG_REASON_TIMER_BITS;   ///< the countdown reached zero
    static constexpr uint32_t force = WATCHDOG_REASON_FORCE_BITS;   ///< CTRL.TRIGGER
};

/// The countdown.
struct Watchdog {
    Watchdog() = delete;

    /// The longest time-out the 24-bit counter holds, in microseconds -
    /// halved by E1's double decrement.
    static constexpr uint32_t max_timeout_us = WATCHDOG_LOAD_BITS / 2u;

    /// Start the countdown at `timeout_us` (at most max_timeout_us:
    /// longer is clamped) and select what a time-out resets: the whole
    /// power-on state machine but the two oscillators, the SDK's choice,
    /// so the chip reboots through the bootrom. `pause_on_debug` holds
    /// the count while a core is halted by a debugger (CTRL.PAUSE_DBG0/1
    /// and PAUSE_JTAG), the right default on a bench. The tick must be
    /// running (WatchdogTick).
    static void start(uint32_t timeout_us, bool pause_on_debug = true) {
        hw_clear(WATCHDOG->CTRL, WATCHDOG_CTRL_ENABLE_BITS);
        hw_set(PSM->WDSEL, PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
        constexpr uint32_t dbg = WATCHDOG_CTRL_PAUSE_DBG0_BITS | WATCHDOG_CTRL_PAUSE_DBG1_BITS |
                                 WATCHDOG_CTRL_PAUSE_JTAG_BITS;
        if (pause_on_debug) { hw_set(WATCHDOG->CTRL, dbg); } else { hw_clear(WATCHDOG->CTRL, dbg); }
        uint32_t load = timeout_us > max_timeout_us ? WATCHDOG_LOAD_BITS : timeout_us * 2u;
        load_ = load;
        WATCHDOG->LOAD = load;
        hw_set(WATCHDOG->CTRL, WATCHDOG_CTRL_ENABLE_BITS);
    }

    /// Reload the countdown: the kick.
    static void kick() { WATCHDOG->LOAD = load_; }

    /// Stop the countdown. Unlike the other families' watchdogs this
    /// one is not one-way: ENABLE clears.
    static void stop() { hw_clear(WATCHDOG->CTRL, WATCHDOG_CTRL_ENABLE_BITS); }
    static bool running() { return (WATCHDOG->CTRL & WATCHDOG_CTRL_ENABLE_BITS) != 0u; }

    /// Microseconds left before the reset (CTRL.TIME, halved for E1).
    static uint32_t remaining_us() { return (WATCHDOG->CTRL & WATCHDOG_CTRL_TIME_BITS) / 2u; }

    /// Reset NOW through the watchdog's path (CTRL.TRIGGER), REASON.FORCE
    /// at the next boot. The power-on state machine's selection is
    /// written here as start() writes it - everything but the two
    /// oscillators - so the trigger reboots the whole chip whether or
    /// not a countdown was ever started: this is the chip's one
    /// software reboot (rp2040/reset.hpp's Reset::software()).
    [[noreturn]] static void force_reset() {
        hw_set(PSM->WDSEL, PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
        hw_set(WATCHDOG->CTRL, WATCHDOG_CTRL_TRIGGER_BITS);
        for (;;) {
        }
    }

    /// WATCHDOG.REASON: WatchdogReason bits, zero after any other reset.
    static uint32_t reason() { return WATCHDOG->REASON & WATCHDOG_REASON_BITS; }

private:
    static inline uint32_t load_ = 0;
};

/// The four scratch words brio may use (SCRATCH0..3): reset-surviving
/// storage that outlives a watchdog or a software reset and dies only
/// with the RUN pin or the supply. SCRATCH4..7 belong to the bootrom's
/// boot redirection and are not offered.
template <uint8_t n>
struct Scratch {
    static_assert(n < 4, "SCRATCH0..3 are brio's; SCRATCH4..7 carry the bootrom's boot magic");
    Scratch() = delete;

    static volatile uint32_t& reg() {
        return reg_at(WATCHDOG_BASE, WATCHDOG_SCRATCH0_OFFSET + 4u * n);
    }
    static uint32_t read() { return reg(); }
    static void write(uint32_t v) { reg() = v; }
};

} // namespace brio
