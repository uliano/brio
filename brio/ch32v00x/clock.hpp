/*
 * clock.hpp
 *
 * The main clock of the CH32V00x: where HCLK comes from and what its
 * rate is, as ONE compile-time truth every driver of this stratum
 * derives from (docs/design/clock.md; no F_CPU here either).
 *
 * THE TREE, IN ONE PARAGRAPH (RM ch. 3). The root is either the 24 MHz
 * internal RC (HSI, on out of reset) or the PLL, which on this family
 * has no ratio to choose: it doubles its source, so HSI gives exactly
 * 48 MHz. SYSCLK is one of those two, and HCLK is SYSCLK divided by
 * HPRE - whose reset value is /3, which is why the chip wakes up at 8
 * MHz. There is no APB prescaler on this family: the peripheral buses
 * run at HCLK, so `pclk_hz` is `hz` and a driver that asks either gets
 * the same number.
 *
 * WHAT init() ORDERS. The flash wait states first (RM 18.3.1: 0 waits
 * to 15 MHz, 1 to 24, 2 to 48), then the divider, then - for a PLL
 * rate - the PLL and the switch, each wait bounded so a dead
 * oscillator returns false instead of hanging the boot. Raising the
 * latency before raising the rate is the safe order, and this runs at
 * boot from the reset clock where the latency can only go up.
 *
 * THE RESOURCE UNDER THE TASKS. `Rcc` is the block itself: the roots
 * (HSI and its trim, the LSI that the watchdogs and the AWU count on,
 * the PLL), the switch and the divider as they stand, the MCO output
 * on PC4, the system clock monitor (SCM: SYSCM_EN and its failure
 * flag, the one clock supervisor this family offers a program without
 * a crystal - the CSS proper watches the HSE), and the per-peripheral
 * clock enables and resets on the three buses. Every configuring
 * driver of the stratum opens its own enable through it.
 *
 * THE RUNTIME REGIME. `DynamicClock<Boot, Users...>` is the AVR's
 * shape and the simplest of the four families': one root (Boot's, the
 * HSI or the PLL), one divider (HPRE), and set<hz>() / set(hz) that
 * name the NEW RATE, fan it out to the users synchronously and in
 * list order, and then move the divider - with the flash wait states
 * raised before a rise and lowered after a fall. The discrete-rate
 * surface (rate_count, rate_hz, rate_index) is what a delay table
 * indexes by.
 *
 * WHAT IS NOT HERE. HSE (the crystal input this package bonds on
 * PA1/PA2) and the LSI as a SYSCLK root are named in ClockSource so
 * that asking for one is a compile error with an explanation rather
 * than a wrong clock, and they are built when a board needs them; the
 * CSS, which watches an HSE, waits with it. A root switch at run time
 * (PLL on and off) is not a DynamicClock rate: two clocks of different
 * roots are two programs, and the divider is what a running program
 * changes.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "util/clock.hpp"

namespace brio {

/// Where SYSCLK comes from.
enum class ClockSource : uint8_t {
    internal,   ///< HSI, the 24 MHz internal RC
    pll,        ///< the PLL, which doubles its source (HSI: 48 MHz)
    crystal,    ///< HSE with a crystal on PA1/PA2 - not implemented yet
    external,   ///< HSE in bypass - not implemented yet
    lsi,        ///< the 128 kHz internal RC as SYSCLK - not implemented yet
};

inline constexpr uint32_t hsi_hz = 24'000'000UL;
inline constexpr uint32_t sysclk_max_hz = 48'000'000UL;

/// RM 3.4.2, HPRE[3:0]: codes 0..7 divide by 1..8, codes 8..15 divide
/// by 2, 4, 8, 16, 32, 64, 128, 256. The two halves overlap on 2, 4 and
/// 8; hpre_for() below picks the low code there, so a divider has one
/// spelling.
constexpr uint32_t hpre_divider(uint8_t code) {
    return code < 8u ? static_cast<uint32_t>(code) + 1u
                     : 2UL << (code - 8u);
}

/// The HPRE code that divides `src_hz` to exactly `hz`, or 0xFF when no
/// divider does.
constexpr uint8_t hpre_for(uint32_t src_hz, uint32_t hz) {
    for (uint8_t code = 0; code < 16u; ++code) {
        const uint32_t div = hpre_divider(code);
        if (hz != 0u && src_hz / div == hz && src_hz % div == 0u) {
            return code;
        }
    }
    return 0xFF;
}

/// How long a root or a switch may take before init() gives up. Bounded
/// spins, not while(1): a boot that cannot reach its rate must return
/// and let the program say so, and on a 48 MHz core this many turns is
/// far more than the tens of microseconds the PLL needs.
inline constexpr uint32_t clock_timeout_turns = 100'000UL;

/// Which bus a peripheral's enable and reset bits sit on (RM 3.4.5..3.4.8).
enum class Bus : uint8_t { hb, pb2, pb1 };

/**
 * The RCC block, monostate: the roots, the switch, the divider, the
 * output, the monitor, and the peripheral gates. Verbs read and write
 * the registers as they stand; the policy (which rate, which root) is
 * the tasks' below.
 */
struct Rcc {
    Rcc() = delete;

    // ---- HSI: the 24 MHz internal RC, on out of reset ----------------------
    static bool hsi_on() { return (rcc()->CTLR & rcc_hsion) != 0u; }
    static bool hsi_ready() { return (rcc()->CTLR & rcc_hsirdy) != 0u; }
    static void hsi(bool on) {
        if (on) { rcc()->CTLR |= rcc_hsion; } else { rcc()->CTLR &= ~rcc_hsion; }
    }
    /// The factory calibration the hardware loaded (read-only).
    static uint8_t hsi_calibration() { return static_cast<uint8_t>((rcc()->CTLR & rcc_hsical_mask) >> 8); }
    /// The user trim, 0..31, 16 the centre; each step nudges the rate.
    static uint8_t hsi_trim() { return static_cast<uint8_t>((rcc()->CTLR & rcc_hsitrim_mask) >> 3); }
    static void hsi_trim(uint8_t value) {
        rcc()->CTLR = (rcc()->CTLR & ~rcc_hsitrim_mask) | (static_cast<uint32_t>(value & 0x1Fu) << 3);
    }

    // ---- LSI: the 128 kHz internal RC (the IWDG's and the AWU's) ----------
    static bool lsi_on() { return (rcc()->RSTSCKR & rcc_lsion) != 0u; }
    static bool lsi_ready() { return (rcc()->RSTSCKR & rcc_lsirdy) != 0u; }
    /// The register shares the reset flags and RMVF; this touches LSION
    /// alone. Ready takes a few LSI cycles; three more after a stop
    /// before the ready bit means anything again (RM 3.4.9).
    static void lsi(bool on) {
        if (on) { rcc()->RSTSCKR |= rcc_lsion; } else { rcc()->RSTSCKR &= ~rcc_lsion; }
    }

    // ---- PLL: the doubler --------------------------------------------------
    static bool pll_on() { return (rcc()->CTLR & rcc_pllon) != 0u; }
    static bool pll_ready() { return (rcc()->CTLR & rcc_pllrdy) != 0u; }

    // ---- the switch and the divider as they stand ------------------------
    static uint32_t sysclk_source() { return rcc()->CFGR0 & rcc_sws_mask; }
    static uint8_t hpre_code() { return static_cast<uint8_t>((rcc()->CFGR0 & rcc_hpre_mask) >> 4); }
    static void hpre(uint8_t code) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_hpre_mask) | (static_cast<uint32_t>(code & 0xFu) << 4);
    }

    // ---- MCO: a clock on PC4, for a counter at the desk --------------------
    /// `source` is one of rcc_mco_sysclk / _hsi / _hse / _pll, or 0 for
    /// off. The pad is the caller's to hand over (Pin<'C', 4>::function()).
    static void mco(uint32_t source) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_mco_mask) | (source & rcc_mco_mask);
    }
    static uint32_t mco() { return rcc()->CFGR0 & rcc_mco_mask; }

    // ---- SCM: the system clock monitor (RM 3.3.7) --------------------------
    /// With SYSCM_EN set, a system clock failure raises SYSCLK_FAILIF,
    /// brakes TIM1, and interrupts through the RCC line if enabled.
    static void monitor(bool on) {
        if (on) { rcc()->CTLR |= rcc_syscm_en; } else { rcc()->CTLR &= ~rcc_syscm_en; }
    }
    static bool monitor() { return (rcc()->CTLR & rcc_syscm_en) != 0u; }
    static bool clock_failed() { return (rcc()->RSTSCKR & rcc_sysclk_failif) != 0u; }
    /// Write 0 to clear (RW0).
    static void clear_clock_failed() { rcc()->RSTSCKR &= ~rcc_sysclk_failif; }
    static void failure_interrupt(bool on) {
        if (on) { rcc()->INTR |= rcc_sysclk_failie; } else { rcc()->INTR &= ~rcc_sysclk_failie; }
    }

    // ---- the peripheral gates --------------------------------------------
    static void clock(Bus bus, uint32_t mask, bool on) {
        volatile uint32_t& reg = bus == Bus::hb ? rcc()->HBPCENR
                               : bus == Bus::pb2 ? rcc()->PB2PCENR : rcc()->PB1PCENR;
        if (on) { reg |= mask; } else { reg &= ~mask; }
    }
    static bool clock(Bus bus, uint32_t mask) {
        const uint32_t reg = bus == Bus::hb ? rcc()->HBPCENR
                           : bus == Bus::pb2 ? rcc()->PB2PCENR : rcc()->PB1PCENR;
        return (reg & mask) == mask;
    }
    /// Pulse a peripheral's reset line: on, then off. The HB bus has no
    /// reset register on this family.
    static void reset(Bus bus, uint32_t mask) {
        volatile uint32_t& reg = bus == Bus::pb2 ? rcc()->PB2PRSTR : rcc()->PB1PRSTR;
        reg |= mask;
        reg &= ~mask;
    }
};

/**
 * The static main clock: `hz` is the ONE compile-time truth about HCLK.
 *
 *   using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
 *   constexpr SysClock clock;
 *   SysClock::init();                // first thing in main()
 *   Serial::init(clock, 115200);     // drivers ask the tag
 */
template <ClockSource src, uint32_t target_hz>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t hz = target_hz;        ///< HCLK
    static constexpr uint32_t pclk_hz = target_hz;   ///< PB1/PB2 = HCLK here
    static constexpr bool is_static = true;

    /// SYSCLK before HPRE: the root's own rate.
    static constexpr uint32_t sysclk_hz = (src == ClockSource::pll) ? 2u * hsi_hz : hsi_hz;
    static constexpr uint8_t hpre_code = hpre_for(sysclk_hz, target_hz);

    static_assert(src == ClockSource::internal || src == ClockSource::pll,
                  "brio Clock: only ClockSource::internal (HSI) and ClockSource::pll "
                  "(HSI x2) are implemented on the CH32V00x - HSE and LSI arrive with "
                  "their first board");
    static_assert(hpre_code != 0xFF,
                  "brio Clock: this rate is not the root divided by an HPRE divider "
                  "(1..8, then 16, 32, 64, 128, 256) - HSI is 24 MHz, the PLL 48 MHz");
    static_assert(target_hz <= sysclk_max_hz, "HCLK must not exceed 48 MHz");

    /**
     * Take the clock to `hz`. Returns false if the PLL never locks or
     * the switch never takes - the caller decides what to say about a
     * boot that stayed on the reset clock.
     */
    /// After a Standby the hardware has switched SYSCLK to the HSI and
    /// turned the PLL off (RM 2.3.3): the tree is put back by running
    /// init() again, which is what a sleep site calls on its way out.
    static bool restore() { return init(); }

    static bool init() {
        // Wait states first: at boot the core runs at the reset rate (8
        // MHz, zero waits), so this can only raise them, which is the
        // safe direction.
        flash_ctl()->ACTLR = flash_latency_for(target_hz);

        uint32_t cfgr = rcc()->CFGR0;
        cfgr &= ~rcc_hpre_mask;
        cfgr |= static_cast<uint32_t>(hpre_code) << 4;
        rcc()->CFGR0 = cfgr;

        if constexpr (src == ClockSource::internal) {
            // HSI is on out of reset; a program that stopped it is
            // asking for it back here.
            rcc()->CTLR |= rcc_hsion;
            if (!wait_for([] { return (rcc()->CTLR & rcc_hsirdy) != 0u; })) {
                return false;
            }
            cfgr = rcc()->CFGR0;
            cfgr = (cfgr & ~rcc_sw_mask) | rcc_sw_hsi;
            rcc()->CFGR0 = cfgr;
            return wait_for([] { return (rcc()->CFGR0 & rcc_sws_mask) == rcc_sws_hsi; });
        } else {
            // PLLSRC = 0 is HSI; RM 3.3.4 wants the source chosen
            // BEFORE the PLL is turned on, and it cannot be changed
            // while the PLL runs.
            rcc()->CTLR &= ~rcc_pllon;
            rcc()->CFGR0 &= ~rcc_pllsrc;
            rcc()->CTLR |= rcc_pllon;
            if (!wait_for([] { return (rcc()->CTLR & rcc_pllrdy) != 0u; })) {
                return false;
            }
            cfgr = rcc()->CFGR0;
            cfgr = (cfgr & ~rcc_sw_mask) | rcc_sw_pll;
            rcc()->CFGR0 = cfgr;
            return wait_for([] { return (rcc()->CFGR0 & rcc_sws_mask) == rcc_sws_pll; });
        }
    }

private:
    template <typename Pred>
    static bool wait_for(Pred done) {
        for (uint32_t turn = 0; turn < clock_timeout_turns; ++turn) {
            if (done()) {
                return true;
            }
        }
        return false;
    }
};

/**
 * The runtime regime. Boot is a static Clock<...> naming the root and
 * its undivided rate; set<hz>() / set(hz) name the NEW RATE (the app
 * speaks Hz - the HPRE code that produces it is this silicon's detail,
 * resolved by hpre_for; a rate no divider reaches is a compile error /
 * a false), fan it out to Users (each a ClockUser, checked where the
 * list is written) in list order, synchronously, and THEN move the
 * divider - so a user can drain what it has in flight at the old rate
 * before adopting the new one. The flash wait states are raised before
 * a rise and lowered after a fall, so the array is never read with
 * fewer waits than its rate needs. Call set() only when nothing that
 * depends on the rate is mid-transfer.
 *
 *   using Boot = brio::Clock<brio::ClockSource::pll, 48'000'000>;
 *   using SysClock = brio::DynamicClock<Boot, brio::Ticker, Serial>;
 *   constexpr SysClock clock;
 *   SysClock::init();                 // Boot's init: 48 MHz
 *   SysClock::set<6'000'000>();       // the users rebased, then HPRE /8
 */
template <typename Boot, ClockUser... Users>
struct DynamicClock {
    static constexpr ClockSource source = Boot::source;
    static constexpr uint32_t source_hz = Boot::sysclk_hz;
    static constexpr bool is_static = false;
    static_assert(Boot::hpre_code == 0,
                  "DynamicClock: give Boot the root's undivided rate (HPRE 1); the "
                  "divider is what set() changes");

    static uint32_t hz() { return hz_; }

    /// The DISCRETE-RATE surface: this clock only ever runs at source_hz
    /// divided by one of the sixteen HPRE codes, so per-rate arithmetic
    /// can be expanded per index at compile time, and the one runtime
    /// fact is WHICH code is current. (Three rates have two codes; the
    /// low one is the spelling hpre_for() picks.)
    static constexpr uint8_t rate_count = 16;
    static constexpr uint32_t rate_hz(uint8_t i) { return source_hz / hpre_divider(i); }
    static uint8_t rate_index() { return idx_; }

    template <typename U>
    static constexpr bool rebases = (std::same_as<U, Users> || ...);

    static bool init() {
        const bool ok = Boot::init();
        hz_ = Boot::hz;
        idx_ = 0;
        return ok;
    }

    /// After a Standby: the root back up (Boot's init, which also puts
    /// HPRE at 1 and the wait states at the root's), then the CURRENT
    /// rate's divider and wait states again. The users are told nothing:
    /// their rate never changed, only the silicon forgot it.
    static bool restore() {
        const bool ok = Boot::init();
        flash_ctl()->ACTLR = flash_latency_for(Boot::hz);
        Rcc::hpre(idx_);
        flash_ctl()->ACTLR = flash_latency_for(hz_);
        return ok;
    }

    static constexpr bool can_run_at(uint32_t hz) { return hpre_for(source_hz, hz) != 0xFF; }

    template <uint32_t hz>
    static void set() {
        static_assert(can_run_at(hz),
                      "DynamicClock: this rate is not the root divided by an HPRE "
                      "divider (1..8, then 16, 32, 64, 128, 256)");
        apply(hpre_for(source_hz, hz), hz);
    }

    static bool set(uint32_t hz) {
        const uint8_t code = hpre_for(source_hz, hz);
        if (code == 0xFF) {
            return false;
        }
        apply(code, hz);
        return true;
    }

private:
    static void apply(uint8_t code, uint32_t next) {
        (Users::rebase(next), ...);
        const bool rising = next > hz_;
        if (rising) {
            flash_ctl()->ACTLR = flash_latency_for(next);
        }
        Rcc::hpre(code);
        if (!rising) {
            flash_ctl()->ACTLR = flash_latency_for(next);
        }
        hz_ = next;
        idx_ = code;
    }

    static inline uint32_t hz_ = Boot::hz;
    static inline uint8_t idx_ = 0;
};

} // namespace brio
