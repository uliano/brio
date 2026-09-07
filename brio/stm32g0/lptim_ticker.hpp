/*
 * lptim_ticker.hpp
 *
 * The TICKLESS kernel timebase of this family: kernel time counted on a
 * low-power timer clocked from the 32768 Hz LSE crystal, which keeps
 * counting through Stop 0 and Stop 1 (RM0444 26.5) - so kernel time
 * never stands still, nothing ever needs a resync, and the loop's next
 * deadline is placed in the timer's compare register instead of being
 * counted out in ticks. `LptimTicker<cfg>` is what stm32g0/platform.hpp's
 * `Stm32g0Platform<TB>` takes as its argument to become tickless: it is
 * `Tickless` (ticks, ticks_per_second, arm_wake), and the platform's
 * idle_until() is what calls arm_wake(). There is NO periodic interrupt
 * in such a program: the LPTIM raises one at the deadline (CMPM), one
 * per lap of its 16-bit counter to carry the high word (ARRM, every two
 * seconds), and nothing else.
 *
 * It is stm32g0/sleep.hpp's Stm32g0LptimTimedSleepSite promoted: that
 * site uses the same counter as an ALARM and a WITNESS beside a SysTick
 * ticker that stops in Stop; here the counter IS the timebase and the
 * two-clock arithmetic (elapsed-1, alarm+1, the phase of two readings)
 * disappears with the second clock. What a program gives up is one
 * LPTIM and its vector, bound by the app to this file's isr():
 *
 *     extern "C" void TIM6_DAC_LPTIM1_IRQHandler() { Tb::isr(); }
 *
 * (the G0B1's LPTIM1 shares its line with TIM6 and the DAC; isr()
 * returns the mask it served so a shared handler can tell "not mine").
 *
 * ## The counter runs at the crystal's rate; the tick is a shift
 *
 * The LPTIM counts the crystal UNDIVIDED, 32768 counts a second, and
 * the kernel tick is that count shifted right: `shift` 5 gives 1024
 * ticks a second - just over the SysTick programs' 1000, the AVR's own
 * rate, and a POWER OF TWO by construction, which is what makes
 * millis() two shifts and a multiply instead of a division the M0+ has
 * not got, and exact: 1000 ms per 1024 ticks, floor at every reading.
 * WHY NOT THE PRESCALER: every latency of this block scales with the
 * clock the COUNTER runs on, not the kernel clock - measured on the
 * bench (test_stm32_tickless letter x): a CMP write lands in 2..3
 * counts whatever the prescaler, 72..93 us at /1 and 2.0..2.8 ms at
 * /32, and a CMPM fires at the count edge AFTER equality. At /32 a
 * compare could not be placed closer than four milliseconds and a
 * masked wait for a completion cost three, which no console would
 * survive; at /1 every one of those numbers is a few tens of
 * microseconds and the deadline is placed to 30 us. The price is a lap
 * of two seconds instead of sixty-four - one ARRM interrupt every two
 * seconds, which a 32 kHz counter's own consumption dwarfs - and a
 * masked window that must stay under one second (below).
 * THE CRYSTAL BY DEFAULT, LSI FOR A BOARD WITHOUT ONE. On LSE the rate
 * is a power of two and every conversion is a shift. LSI - "32 kHz
 * nominal, 29.5..34 actual" across parts and temperature (DS13560
 * table 46), 32586 Hz measured on the bench die - has no rate a header
 * can know, so `LptimTickerConfig::source = lsi` takes the rate the
 * PROGRAM STATES in `lsi_hz`, and the DIRECTIONAL RULE decides what to
 * state: kernel time is exact by COUNT (a tick is a tick), what the
 * stated rate governs is the conversion of milliseconds into ticks, and
 * N ticks of a clock FASTER than stated are fewer real milliseconds
 * than asked - EARLY, which brio's time contract forbids. So a stated
 * rate must not sit BELOW the true one, the default is table 46's
 * ceiling (34000: never early on any part, up to 15 % late on a slow
 * one), and a program that measures its LSI (the rtc suite's TIM16
 * capture) states a tighter number. millis()/secs()/now() then divide
 * instead of shifting (two 32-bit divisions, no overflow, a few
 * microseconds - they are not the kernel's hot path; the kernel counts
 * ticks). Everything else - the lap, the compare rules, the floor - is
 * the same: they are properties of the counter, not of its clock.
 *
 * SYSTICK STAYS ON, INTERRUPT-LESS. armv6m/delay.hpp counts cycles on
 * SysTick's VAL and never needs its interrupt, so init() starts it as
 * armv6m/ticker.hpp's SysTickCounter (same reload rule, no TICKINT):
 * delay_us works by construction and a program on this timebase binds
 * NOTHING to SysTick_Handler. The Stop modes stop SysTick with HCLK as
 * before; a busy-wait is awake by definition.
 *
 * ## The compare, and the rules its arming follows
 *
 * The wake is CMP: CMPM fires at the count edge where CNT becomes
 * CMP + 1 (measured, letter x: exactly one count after equality at
 * every distance tried), on the same LSE edge ticks() reads - so a
 * compare at (the deadline's first count - 1) wakes AT the deadline,
 * never early, with no phase conversion and no second clock. What
 * makes the arming delicate is the register's own discipline, and two
 * errata rule out the obvious handshakes: 26.4.11 forbids a second CMP
 * write before the previous one's CMPOK ("unpredictable results"), and
 * ES0548 2.8.2 says "Flags must not be cleared outside the interrupt
 * subroutine" (a flag of a disabled interrupt cleared at the instant a
 * new event lands can leave the interrupt signal stuck high). A CMPOK
 * INTERRUPT would be the textbook handshake and is ruled out too: it
 * would wake every WFI a hundred microseconds after each arm - a Stop
 * exit per deadline. So:
 *
 *  1. A STORE IS ISSUED ONLY WHILE CMPOK IS CLEAR AND NO WRITE IS IN
 *     FLIGHT. The flag is the one fresh witness of a completed write:
 *     READ from thread mode (legal), CLEARED only by the handler -
 *     Lptim::isr() sweeps it as an unarmed flag, first, as 2.8.2's
 *     workaround orders. If the flag stands at arm time (a write landed
 *     and no LPTIM interrupt has run since), arm_wake() PENDS the LPTIM
 *     vector once and answers false: the platform returns without
 *     sleeping, the handler runs the moment interrupts are back and
 *     sweeps the flag, and the loop turns until the clear has crossed
 *     (the sweep's own count is remembered so the crossing is not
 *     mistaken for a new completion and the vector is not pended
 *     again). A write still in flight with the flag not yet up is the
 *     same answer: false, no masked wait, the loop turns until it
 *     lands - a masked wait of even 90 us would cost a console its
 *     bytes. The common path costs nothing: a deadline's own CMPM
 *     sweeps the completion of the store that placed it (letter e:
 *     534 CMPM for 534 deadlines in three seconds of three periodics,
 *     no deferral in the loop). The clear itself is visible on the
 *     APB side the moment the handler's ICR store returns (letter d:
 *     0 us), so the crossing guard never fires; it stays as the cheap
 *     insurance it is.
 *  2. THE LANDING RACE IS CLOSED BY A FLOOR: a compare must reach the
 *     register before the counter reaches it, and the write lands 2..3
 *     counts after the store whatever the phase of the count it was
 *     stored in - so the first count of a deadline must be at least SIX
 *     counts past the count `now` was read in (183 us at 32768 Hz). A
 *     deadline nearer than that is NOT slept for: arm_wake() answers
 *     false and the loop spins the few counts through process(), which
 *     fires it on time. Never early, never late by more than the loop's
 *     own turn; the alternative - placing it later - would be legal and
 *     worse.
 *  3. BEFORE A STOP THE STORE IS WAITED FOR. Whether an APB-to-kernel
 *     transfer still in flight completes with PCLK stopped is written
 *     nowhere in chapter 26, so with SLEEPDEEP set arm_wake() spends
 *     the 93 us on wait_cmp_ok() and the compare is in place before
 *     the machine stops (the timed site's own rule). In Sleep the APB
 *     keeps running and the store is left to land on its own. MEASURED
 *     (letter d): a store followed by the Stop 1 with NO wait lands all
 *     the same - the compare fires at its 292 ms on the RTC's wall,
 *     three runs of three - so the transfer completes on the kernel
 *     clock alone and this wait is insurance the silicon does not need;
 *     kept, because it costs 93 us per Stop round and the chapter
 *     promises nothing.
 *  4. A DEADLINE A LAP OR MORE AWAY IS NOT ARMED, AND THE COMPARE IS
 *     PARKED ON THE LAP: the ARRM two seconds out re-evaluates, and a
 *     low half-word would match a lap early. The register is MIRRORED:
 *     a deadline whose compare is already in the register stores
 *     nothing. Where the compare sits when nothing is armed MATTERS,
 *     because it matches once per lap wherever it is: parked at 0xFFFF
 *     - equal to ARR, which DOES match (measured, letter b) - its CMPM
 *     is raised at the counter's own wrap and served by the ARRM's
 *     interrupt, so a deadline-less Stop costs ONE wake per lap. The
 *     first version parked it half a lap out and every lap cost two
 *     (letter i: ten interrupts for a ten-second Stop, now five). The
 *     parking is done by arm_wake() for a far deadline and by park(),
 *     which the platform's idle_until(nullopt) calls, for none.
 *
 * ## ticks() inside a masked window
 *
 * The count is the high word (laps_, carried by the ARRM handler) over
 * the 16-bit CNT, read the classic way with one twist for the masked
 * case: with PRIMASK set the wrap happens but the handler does not
 * run, and ISR.ARRM standing with a LOW count says exactly that - one
 * lap more than laps_ knows. The read is bracketed (laps, ARRM, CNT,
 * ARRM, laps; retried unless both pairs agree) because the flag and
 * the counter reach the APB reader by different paths. Precondition,
 * stated: no masked window longer than half a lap (ONE SECOND), or a
 * high count under a standing ARRM reads as the previous lap. And CNT
 * itself is two agreeing reads (26.7.8, the asynchronous kernel clock)
 * - retried without bound here, since a timebase that answers
 * "nothing" is no timebase. The tick is the 32-bit count shifted, and
 * it wraps where the SysTick ticker's does: 2^32 ticks, 48 days at
 * 1024 Hz.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "stm32g0xx.h"

#include "stm32g0/device_tables.hpp"
#include "stm32g0/lptim.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/ticker.hpp"
#include "util/timestamp.hpp"

#if defined(LPTIM1_BASE)

namespace brio {

/// The counter's clock: the crystal, or the internal RC at a rate the
/// program states (the file header's directional rule).
enum class LptimTickerSource : uint8_t { lse, lsi };

/// Which LPTIM, which clock, and how far the count is shifted to make
/// the kernel tick. The default is LPTIM1 on LSE with a shift of 5:
/// 1024 ticks a second. `lsi_hz` is read only with `source = lsi`:
/// the rate to state is one NOT BELOW the true one, and the default is
/// DS13560 table 46's ceiling.
struct LptimTickerConfig {
    uint8_t instance = 1;
    uint8_t shift = 5;
    LptimTickerSource source = LptimTickerSource::lse;
    uint32_t lsi_hz = 34'000;
};

/// The counter's rate: the crystal's, or the stated LSI rate.
constexpr uint32_t lptim_ticker_count_hz(const LptimTickerConfig& c) {
    return c.source == LptimTickerSource::lse ? 32'768u : c.lsi_hz;
}

/// A shift that leaves at least 32 ticks a second, and - for LSI - a
/// stated rate inside table 46's own band (29.5..34 kHz): a number
/// outside it is not this oscillator.
constexpr bool lptim_ticker_config_valid(const LptimTickerConfig& c) {
    if (!lptim_present(c.instance) || c.shift > 10u) {
        return false;
    }
    if (c.source == LptimTickerSource::lsi) {
        return c.lsi_hz >= 29'500u && c.lsi_hz <= 34'000u;
    }
    return true;
}

/// The tick rate a configuration gives: the count rate >> shift.
constexpr uint32_t lptim_ticker_hz(const LptimTickerConfig& c) {
    return lptim_ticker_count_hz(c) >> c.shift;
}

template <LptimTickerConfig cfg = LptimTickerConfig{}>
class LptimTicker {
    static_assert(lptim_ticker_config_valid(cfg),
                  "brio LptimTicker: the LPTIM instance must exist on this part, the "
                  "shift must leave at least 32 ticks a second, and a stated LSI rate "
                  "must sit inside DS13560 table 46's 29.5..34 kHz");

public:
    LptimTicker() = delete;

    using L = Lptim<cfg.instance>;

    /// The counter's clock and rate (the crystal's, or the STATED LSI
    /// rate), and the tick's rate.
    static constexpr LptimTickerSource source = cfg.source;
    static constexpr bool on_crystal = source == LptimTickerSource::lse;
    static constexpr uint32_t count_hz = lptim_ticker_count_hz(cfg);
    static constexpr uint8_t shift = cfg.shift;
    static constexpr uint32_t ticks_per_second = lptim_ticker_hz(cfg);
    static constexpr uint32_t counts_per_tick = 1u << shift;

    /// One lap of the 16-bit counter, in counts and in ticks.
    static constexpr uint32_t lap_counts = 0x10000u;
    static constexpr uint32_t lap_ticks = lap_counts >> shift;

    /// Rule 2's floor: the first count of a deadline must be this many
    /// counts past the count `now` was read in (the write lands in 2..3,
    /// the phase adds up to one, one more for the margin).
    static constexpr uint32_t min_counts_ahead = 6;

    /// Where the compare is PARKED (rule 4) - at init, when nothing is
    /// armed, and when the deadline is a lap or more away: equal to ARR,
    /// so that its match is the counter's own wrap and the CMPM it
    /// raises is served by the same interrupt as the ARRM (measured:
    /// test_stm32_tickless letter b, a compare equal to ARR matches
    /// like any other; letter i, one interrupt per lap). Parked
    /// anywhere else it would be a SECOND wake every lap - the first
    /// version parked at 0x8000 and a ten-second Stop cost ten wakes
    /// where five would do.
    static constexpr uint16_t parked_cmp = 0xFFFF;

    /// The vector the app binds to isr() - shared on some parts
    /// (device_tables.hpp says with what).
    static constexpr IRQn_Type irq() { return L::irq(); }

    /**
     * Bring the timebase up: the LSE (through the RTC domain's gate and
     * DBP, and NOTHING ELSE of that domain - RTCSEL, the calendar and
     * the backup registers are not touched), the LPTIM on it undivided
     * as a free-running counter with ARRM and CMPM armed, the compare
     * parked, the wake line and the NVIC line open, the counter
     * started - and SysTick as the interrupt-less cycle counter delay_us
     * reads, from the same clock task the SysTick ticker would take.
     * Call once, after the clock init, before interrupts are enabled.
     *
     * False = the crystal never reported ready, or a step of the
     * chapter's own order was refused; a program then has no timebase
     * and the caller must look at the answer.
     */
    template <typename C>
    static bool init(C clock) {
        if constexpr (on_crystal) {
            RtcDomain::pwr_bus_clock(true);
            RtcDomain::unlock(true);
            RtcDomain::lse_enable(true);
            if (!RtcDomain::lse_wait_ready()) {
                return false;
            }
        } else {
            // LSI is RCC's own (5.2.6), no domain gate in the way; it
            // runs through a Stop for a kernel-clock consumer that asks
            // (the IWDG's precedent), which is what this counter is.
            Rcc::lsi_enable(true);
            if (!Rcc::lsi_wait_ready()) {
                return false;
            }
        }
        L::init();
        L::kernel_clock(on_crystal ? LptimClock::lse : LptimClock::lsi);
        laps_ = 0;
        write_pending_ = false;
        sweep_pending_ = false;
        swept_valid_ = false;
        deferrals_ = 0;
        floor_declines_ = 0;
        stores_ = 0;
        write_timeouts_ = 0;
        stop_waits_ = 0;
        if (!L::configure({.prescaler = LptimPrescaler::div1,
                           .interrupts = LptimFlag::arrm | LptimFlag::cmpm})) {
            return false;
        }
        L::enable();
        if (!L::set_arr(0xFFFFu) || !L::wait_arr_ok()) {
            return false;
        }
        // Parked BEFORE the counter starts: CMP comes out of reset at
        // zero and a counter started first matches it on its first
        // tick (the sleep site's finding, test_stm32_lptim letter g).
        if (!L::set_cmp(parked_cmp) || !L::wait_cmp_ok()) {
            return false;
        }
        cmp_reg_ = parked_cmp;
        if (!L::wake_line(true)) {
            return false;
        }
        Nvic::clear_pending(irq());
        Nvic::enable(irq());
        if (!L::start_continuous()) {
            return false;
        }
        if (!SysTickCounter::start(clock)) {
            return false;
        }
        // ARROK and CMPOK stand from the bring-up and only the handler
        // may sweep them (2.8.2): pend it, so the first thing that runs
        // once interrupts are enabled is that sweep.
        sweep_pending_ = true;
        Nvic::set_pending(irq());
        return true;
    }

    /// The 32-bit COUNT (32768 a second, wraps in 36 hours) - the raw
    /// reading, for the bench; exact with interrupts masked (see the
    /// header).
    static uint32_t count() {
        uint32_t hi;
        uint16_t lo;
        read_pair(hi, lo);
        return (hi << 16) | lo;
    }

    /// The kernel tick: the count shifted - the laps and the low half
    /// each shifted into place, so the tick keeps every lap bit the
    /// 32-bit count would drop and wraps at 2^32 ticks like every
    /// ticker (48 days at 1024 Hz).
    static uint32_t ticks() {
        uint32_t hi;
        uint16_t lo;
        read_pair(hi, lo);
        return tick_of(hi, lo);
    }

    /// Milliseconds: floor(ticks * 1000 / tps) - two shifts on the
    /// crystal (tps a power of two), two divisions on LSI (whole seconds
    /// first, so the product never overflows 32 bits).
    static uint32_t millis() {
        const uint32_t t = ticks();
        if constexpr (on_crystal) {
            return (t >> tick_shift) * 1000u + (((t & (ticks_per_second - 1u)) * 1000u) >> tick_shift);
        } else {
            return (t / ticks_per_second) * 1000u + ((t % ticks_per_second) * 1000u) / ticks_per_second;
        }
    }

    static uint32_t secs() {
        if constexpr (on_crystal) {
            return ticks() >> tick_shift;
        } else {
            return ticks() / ticks_per_second;
        }
    }

    /// TimeStamp from ONE reading: no guard needed, one word carries both.
    static void now(TimeStamp& out) {
        const uint32_t t = ticks();
        if constexpr (on_crystal) {
            out.seconds = t >> tick_shift;
            out.millis = static_cast<uint16_t>(((t & (ticks_per_second - 1u)) * 1000u) >> tick_shift);
        } else {
            out.seconds = t / ticks_per_second;
            out.millis = static_cast<uint16_t>(((t % ticks_per_second) * 1000u) / ticks_per_second);
        }
    }

    /**
     * Place the wake for `deadline` (absolute tick, strictly after
     * `now`, this timebase's own reading a moment ago), following the
     * rules of the header. Called with interrupts MASKED by
     * Stm32g0Platform::idle_until(). True: sleep. False: do not sleep
     * this turn - the deadline is too near to place (the loop spins it
     * through), a write is in flight, or a completion stands and the
     * vector has been pended to sweep it; the next turn asks again.
     */
    static bool arm_wake(uint32_t now, uint32_t deadline) {
        (void)now;   // re-read below: a tick edge may have passed since
        uint32_t hi;
        uint16_t lo;
        read_pair(hi, lo);
        const uint32_t c = (hi << 16) | lo;
        const int32_t distance = static_cast<int32_t>(deadline - tick_of(hi, lo));
        if (distance <= 0) {
            return false;   // due now: the loop's next process() fires it
        }
        if (static_cast<uint32_t>(distance) >= lap_ticks) {
            return store(parked_cmp);   // rule 4: the lap's own wake re-evaluates
        }
        // The count where the tick becomes `deadline`, and the compare
        // one below it (CMPM fires at the edge AFTER equality).
        const uint32_t target =
            (c & ~(counts_per_tick - 1u)) + (static_cast<uint32_t>(distance) << shift);
        if (target - c < min_counts_ahead) {
            floor_declines_ = floor_declines_ + 1u;
            return false;   // rule 2: too near to place - spin it through
        }
        return store(static_cast<uint16_t>(target - 1u));
    }

    /**
     * Nothing armed at all: the compare PARKED where its match is the
     * lap's own edge (rule 4), so that a sleep with no deadline costs
     * one interrupt per lap and not two. Same answer as arm_wake's:
     * true = sleep, false = the loop turns once (a completion being
     * swept). The platform's idle_until(nullopt) calls it.
     */
    static bool park() { return store(parked_cmp); }

private:
    /**
     * The one store path (rules 1 and 3, and the mirror): `cmp` into
     * the register unless it is already there, only with CMPOK clear
     * and nothing in flight, waited for when a Stop is armed.
     */
    static bool store(uint16_t cmp) {
        if (cmp == cmp_reg_) {
            return true;    // the mirror: already in the register
        }
        if (L::cmp_ok()) {
            // Rule 1: a completion stands - the flag may be READ from
            // here, only the handler may clear it. Whatever was in
            // flight has landed; pend the sweep once, and while it is
            // pending, or its clear is still crossing, just decline.
            write_pending_ = false;
            deferrals_ = deferrals_ + 1u;
            if (!read_sweep_pending()) {
                const uint16_t since = static_cast<uint16_t>(L::count_raw() - swept_at_);
                if (!read_swept_valid() || since >= 4u) {
                    sweep_pending_ = true;
                    Nvic::set_pending(irq());
                }
            }
            return false;
        }
        if (read_write_pending()) {
            return false;   // in flight (26.4.11), the flag not up yet: the loop turns
        }
        (void)L::set_cmp(cmp);
        cmp_reg_ = cmp;
        write_pending_ = true;
        stores_ = stores_ + 1u;
        if ((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0u) {
            // Rule 3: in place before the machine stops.
            stop_waits_ = stop_waits_ + 1u;
            if (!L::wait_cmp_ok()) {
                write_timeouts_ = write_timeouts_ + 1u;
                write_pending_ = false;
                return false;
            }
        }
        return true;
    }

public:
    /**
     * The ISR body: the resource's ordered clear (unarmed flags first -
     * CMPOK among them, which is why it is read BEFORE the sweep), the
     * lap carry, and the notes rule 1 lives on. Returns what the
     * resource served (0 for a software-pended sweep, which a shared
     * handler reads as "not mine").
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t pending = L::status();
        const uint32_t served = L::isr();
        if ((pending & LptimFlag::cmpok) != 0u) {
            write_pending_ = false;
            swept_at_ = L::count_raw();
            swept_valid_ = true;
        }
        sweep_pending_ = false;
        if ((served & LptimFlag::arrm) != 0u) {
            laps_ = laps_ + 1u;
        }
        return served;
    }

    // ---- readbacks, for the bench and for a curious app -------------------

    static uint32_t laps() { return read_laps(); }
    static uint16_t cmp_reg() { return cmp_reg_; }
    static bool write_pending() { return read_write_pending(); }
    /// Arms declined because a completion stood unswept (rule 1).
    static uint32_t deferrals() { return deferrals_; }
    /// Arms declined because the deadline was too near to place (rule 2).
    static uint32_t floor_declines() { return floor_declines_; }
    /// Compare stores issued.
    static uint32_t stores() { return stores_; }
    /// Waits for CMPOK that never came (a dead LPTIM; never seen).
    static uint32_t write_timeouts() { return write_timeouts_; }
    /// Stores waited for because a Stop was armed (rule 3).
    static uint32_t stop_waits() { return stop_waits_; }

private:
    /// log2(tps) on the crystal; unused on LSI (its tps is not a power of two).
    static constexpr uint8_t tick_shift = static_cast<uint8_t>(15u - shift);

    static uint32_t read_laps() { return *const_cast<const volatile uint32_t*>(&laps_); }
    static bool read_write_pending() { return *const_cast<const volatile bool*>(&write_pending_); }
    static bool read_sweep_pending() { return *const_cast<const volatile bool*>(&sweep_pending_); }
    static bool read_swept_valid() { return *const_cast<const volatile bool*>(&swept_valid_); }

    /// The bracketed read of the header: laps, ARRM, CNT, ARRM, laps -
    /// retried unless both pairs agree - with the pending-ARRM
    /// correction folded into `hi`.
    static void read_pair(uint32_t& hi, uint16_t& lo) {
        for (;;) {
            const uint32_t hi0 = read_laps();
            const bool wrapped0 = (L::status() & LptimFlag::arrm) != 0u;
            const uint16_t l = count_agreed();
            const bool wrapped1 = (L::status() & LptimFlag::arrm) != 0u;
            const uint32_t hi1 = read_laps();
            if (hi0 != hi1 || wrapped0 != wrapped1) {
                continue;   // a lap or its handler went by: read again
            }
            hi = (wrapped0 && l < 0x8000u) ? hi0 + 1u : hi0;
            lo = l;
            return;
        }
    }

    static uint32_t tick_of(uint32_t hi, uint16_t lo) {
        return (hi << (16u - shift)) | (static_cast<uint32_t>(lo) >> shift);
    }

    /// Two agreeing reads of CNT, retried without bound (see the header).
    static uint16_t count_agreed() {
        for (;;) {
            const std::optional<uint16_t> v = L::count();
            if (v.has_value()) {
                return *v;
            }
        }
    }

    static inline uint32_t laps_ = 0;           // written by isr(), read everywhere
    static inline bool write_pending_ = false;  // set by arm_wake(), cleared by isr()
    static inline bool sweep_pending_ = false;  // set with the pend, cleared by isr()
    static inline bool swept_valid_ = false;
    static inline uint16_t swept_at_ = 0;       // CNT when the handler swept CMPOK
    static inline uint16_t cmp_reg_ = parked_cmp;
    static inline uint32_t deferrals_ = 0;
    static inline uint32_t floor_declines_ = 0;
    static inline uint32_t stores_ = 0;
    static inline uint32_t write_timeouts_ = 0;
    static inline uint32_t stop_waits_ = 0;
};

} // namespace brio

#endif // LPTIM1_BASE
