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
 * WHAT IS NOT HERE. HSE (the crystal input this package bonds on
 * PA1/PA2) and the 128 kHz LSI are named in ClockSource so that asking
 * for one is a compile error with an explanation rather than a wrong
 * clock, and they are built when a board needs them. There is no
 * DynamicClock on this family yet; the ClockUser contract that a
 * dynamic clock needs is already what the ticker and the USART
 * implement, so the day it arrives nothing above has to change.
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

} // namespace brio
