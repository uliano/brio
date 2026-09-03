/*
 * clock.hpp
 *
 * The STM32G0 clock tree (RCC, RM0444 ch. 5) in the two strata every
 * brio target uses (docs/design/clock.md):
 *
 *  RESOURCES - monostates, thin typed views with the discipline built in:
 *    Rcc      the reset and clock control block: the HSI16 root and its
 *             HSISYS divider, the PLL (configure, start, lock), the
 *             SYSCLK switch with its status readback, the AHB/APB
 *             prescalers, the PERIPHERAL CLOCK ENABLES (one bit per
 *             peripheral in IOPENR/AHBENR/APBENR1/APBENR2) and the
 *             per-peripheral kernel-clock multiplexers (CCIPR)
 *    (`Pwr`, the whole of chapter 4, lives in stm32g0/pwr.hpp: the
 *    task below reads PWR_CR1.VOS from it, because the flash latency
 *    table is indexed by the voltage range, and opens the PWR block's
 *    bus clock on the way. One chapter, one owner.)
 *
 *  TASK - what an application names:
 *    Clock<source, hz>   the static main clock: ONE constexpr truth `hz`
 *             every driver derives from (no F_CPU in this build, exactly
 *             as on the other two targets); init() composes the
 *             resources and reports whether the requested root runs.
 *
 * THE THIRD CLOCK MODEL, and what crosses the util contract. The AVR
 * has one prescaler on one main clock; the SAM has a generator per
 * peripheral; this family has SHARED PRESCALERS (HPRE for AHB, PPRE for
 * APB) below one SYSCLK, an ENABLE BIT per peripheral that gates its
 * bus clock (a peripheral whose bit is clear does not even answer
 * register reads - 5.2.17), and a KERNEL-CLOCK multiplexer for the few
 * peripherals that may run off something other than their bus (USART1..3,
 * I2C1, ADC, LPTIM, RTC). What crosses the contract is unchanged:
 * `clock_hz(clock)` is SYSCLK = HCLK, and this first cut PINS HPRE and
 * PPRE at 1 so that PCLK == HCLK == hz and one number serves every
 * driver - stated as `Clock::pclk_hz` beside `hz`, so that a driver on
 * APB asks for the rate that is really its own and the day the
 * prescalers move nothing above it has to change. A task that divides
 * the buses is a legitimate future member of this file, not a change to
 * util/.
 *
 * SCOPE. Two roots are built: `internal` = HSI16 through the HSISYS
 * divider (16, 8, 4, 2, 1, 0.5, 0.25, 0.125 MHz) and `pll` = HSI16
 * through the PLL's R output, which is how the part reaches its 64 MHz
 * ceiling (an exact ratio is searched at compile time; an unreachable
 * rate is a compile error naming the rule). HSE (crystal or bypass),
 * LSI, LSE, HSI48 and the P/Q outputs are DECLARED in the enum and
 * refused, and the DynamicClock question is not opened - the samc
 * position, for the same reason: which root SYSCLK takes at run time and
 * who is told is a design decision, not a side effect of a bring-up.
 *
 * Facts that shape the code (RM0444 5.2, 5.4, 3.3.4, 4.1.4; ES0548 on
 * silicon rev Z, DBGMCU_IDCODE 0x10016467):
 *  - out of reset the device runs HSI16 undivided as HSISYS = SYSCLK =
 *    HCLK = PCLK at 16 MHz, VCORE Range 1, FLASH_ACR.LATENCY 0;
 *  - table 13 ties the wait states to HCLK per voltage range, and 3.3.4
 *    orders them BEFORE a rise (with a readback) and AFTER a fall;
 *  - the PLL input after /M must sit in 2.66..16 MHz, the VCO in
 *    96..344 MHz, PLLRCLK <= 64 MHz, N in 8..86, R in 2..8 (5.4.4);
 *    the PLL is configured only while stopped (5.2.4);
 *  - 5.2.7: a SYSCLK switch takes effect only when the target is ready,
 *    and SWS reports which source is in force - init() waits for it;
 *  - 5.2.17: a peripheral's enable bit takes two clock cycles to act;
 *    every bus-enable verb below reads the register back, which is the
 *    stall that covers it;
 *  - ES0548 2.2.4: with HSIDIV != 0 the part cannot enter Stop and
 *    peripherals with clock-request capability cannot wake it - a
 *    divided `internal` rate is therefore a stated caveat for the
 *    future sleep site, not a refusal here.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/device_tables.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/pwr.hpp"
#include "util/clock.hpp"

namespace brio {

// =============================================================================
// Resources
// =============================================================================

/// RCC_CFGR.SW / SWS: what SYSCLK may come from (5.4.3).
enum class SysclkSource : uint8_t {
    hsisys = 0,
    hse = 1,
    pllrclk = 2,
    lsi = 3,
    lse = 4,
};

/// A PLL setting: fVCO = fIN x N / M, fPLLR = fVCO / R (5.4.4). Only the
/// R output is driven by this stratum (it is the one SYSCLK can take).
struct PllConfig {
    uint8_t m = 1;   ///< input divider 1..8
    uint8_t n = 8;   ///< VCO multiplier 8..86
    uint8_t r = 2;   ///< R output divider 2..8
};

inline constexpr uint32_t hsi16_hz = 16'000'000UL;
inline constexpr uint32_t sysclk_max_hz = 64'000'000UL;
inline constexpr uint32_t pll_in_min_hz = 2'660'000UL;
inline constexpr uint32_t pll_in_max_hz = 16'000'000UL;
inline constexpr uint32_t pll_vco_min_hz = 96'000'000UL;
inline constexpr uint32_t pll_vco_max_hz = 344'000'000UL;

/// The chapter's limits on a setting, from the HSI16 input.
constexpr bool pll_config_valid(const PllConfig& c) {
    if (c.m < 1 || c.m > 8 || c.n < 8 || c.n > 86 || c.r < 2 || c.r > 8) {
        return false;
    }
    const uint32_t in = hsi16_hz / c.m;
    if (hsi16_hz % c.m != 0u || in < pll_in_min_hz || in > pll_in_max_hz) {
        return false;
    }
    const uint32_t vco = in * c.n;
    if (vco < pll_vco_min_hz || vco > pll_vco_max_hz) {
        return false;
    }
    return vco / c.r <= sysclk_max_hz;
}

/// PLLRCLK a setting produces from HSI16.
constexpr uint32_t pll_output_hz(const PllConfig& c) {
    return (hsi16_hz / c.m) * c.n / c.r;
}

/// The EXACT setting for `hz` from HSI16, or m = 0 when none exists.
/// Deterministic: the smallest M first (an undivided input is the
/// cleanest), then the smallest R, then the smallest N - so 64 MHz is
/// M 1, N 8, R 2 (VCO 128 MHz).
constexpr PllConfig pll_config_for(uint32_t hz) {
    for (uint8_t m = 1; m <= 8; ++m) {
        for (uint8_t r = 2; r <= 8; ++r) {
            for (uint8_t n = 8; n <= 86; ++n) {
                const PllConfig c{m, n, r};
                if (pll_config_valid(c) && pll_output_hz(c) == hz &&
                    ((hsi16_hz / m) * n) % r == 0u) {
                    return c;
                }
            }
        }
    }
    return {0, 0, 0};
}

/// RCC_CR.HSIDIV code for an HSISYS rate, 0xFF when HSI16 cannot produce
/// it: 16 MHz divided by 2^k, k = 0..7.
constexpr uint8_t hsidiv_for(uint32_t hz) {
    for (uint8_t k = 0; k < 8; ++k) {
        if ((hsi16_hz >> k) == hz) {
            return k;
        }
    }
    return 0xFF;
}

/**
 * The RCC block. Every verb is a register-level fact of chapter 5; the
 * ordering rules (latency first, PLL stopped before reconfiguring, wait
 * for READY, wait for SWS) are the TASK's job below, so that a resource
 * verb never does two things.
 */
struct Rcc {
    Rcc() = delete;

    static constexpr uint32_t ready_spins = 100'000u;

    // ---- HSI16 and HSISYS ---------------------------------------------------
    static void hsi_enable(bool on) {
        RCC->CR = on ? (RCC->CR | RCC_CR_HSION) : (RCC->CR & ~RCC_CR_HSION);
    }
    static bool hsi_ready() { return (RCC->CR & RCC_CR_HSIRDY) != 0u; }
    static bool hsi_wait_ready() {
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if (hsi_ready()) {
                return true;
            }
        }
        return false;
    }
    /// HSIDIV code 0..7 (divide by 2^code). Legal while running from
    /// HSISYS: the divider output is glitch-free by the chapter's
    /// silence, and ST's own drivers change it under a running core.
    static void hsi_div(uint8_t code) {
        RCC->CR = (RCC->CR & ~RCC_CR_HSIDIV_Msk) |
                  ((static_cast<uint32_t>(code) << RCC_CR_HSIDIV_Pos) & RCC_CR_HSIDIV_Msk);
    }
    static uint8_t hsi_div() {
        return static_cast<uint8_t>((RCC->CR & RCC_CR_HSIDIV_Msk) >> RCC_CR_HSIDIV_Pos);
    }

    // ---- the PLL ---------------------------------------------------------------
    static void pll_enable(bool on) {
        RCC->CR = on ? (RCC->CR | RCC_CR_PLLON) : (RCC->CR & ~RCC_CR_PLLON);
    }
    static bool pll_ready() { return (RCC->CR & RCC_CR_PLLRDY) != 0u; }
    static bool pll_wait(bool ready) {
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if (pll_ready() == ready) {
                return true;
            }
        }
        return false;
    }
    /// Write the whole PLLCFGR for the HSI16 source with the R output
    /// enabled and P/Q off. Refused (false, nothing written) while the
    /// PLL runs - 5.4.4 says the fields can be written only when it is
    /// disabled - and for a setting outside the chapter's limits.
    static bool pll_configure(const PllConfig& c) {
        if (!pll_config_valid(c) || (RCC->CR & RCC_CR_PLLON) != 0u) {
            return false;
        }
        RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI |
                       (static_cast<uint32_t>(c.m - 1u) << RCC_PLLCFGR_PLLM_Pos) |
                       (static_cast<uint32_t>(c.n) << RCC_PLLCFGR_PLLN_Pos) |
                       (static_cast<uint32_t>(c.r - 1u) << RCC_PLLCFGR_PLLR_Pos) |
                       RCC_PLLCFGR_PLLREN;
        return true;
    }

    // ---- SYSCLK and the bus prescalers ------------------------------------------
    static void sysclk_select(SysclkSource s) {
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) |
                    (static_cast<uint32_t>(s) << RCC_CFGR_SW_Pos);
    }
    static SysclkSource sysclk_status() {
        return static_cast<SysclkSource>((RCC->CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos);
    }
    static bool sysclk_wait(SysclkSource s) {
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if (sysclk_status() == s) {
                return true;
            }
        }
        return false;
    }
    /// HPRE and PPRE written to their divide-by-1 codes (0xxx / 0xx).
    static void bus_prescalers_unity() {
        RCC->CFGR = RCC->CFGR & ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE_Msk);
    }
    static bool bus_prescalers_are_unity() {
        return (RCC->CFGR & (RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE_Msk)) == 0u;
    }

    // ---- LSI, the low-speed RC (5.2.6, 5.2.14) --------------------------------
    //
    // It lives in RCC_CSR, a register whose TOP BYTE belongs to the
    // reset chapter (stm32g0/reset.hpp's flags and RMVF); both owners
    // read-modify-write and neither touches the other's bits. LSI is
    // the IWDG's clock and one of the RTC's, and nothing else here uses
    // it - it is not offered as a SYSCLK root (the task's enum refuses
    // that, see the file header).
    //
    /**
     * RCC_CR.HSIKERON (5.4.1): keep HSI16 running for a KERNEL-CLOCK
     * consumer even when the system does not need it - through a Stop
     * mode, and in Run when SYSCLK is on something else.
     *
     * IT IS NOT THE SAME MECHANISM as a peripheral's own clock request.
     * 33.5.21: a USART whose kernel clock is gated in Stop asks for it
     * back on the falling edge of its RX line (usart_ker_ck_req) and
     * releases it again if the wake-up event is not verified - the
     * oscillator is started ON DEMAND and for as long as the frame
     * lasts. HSIKERON instead keeps it running unconditionally, which
     * costs current and buys latency (no startup time in the path). A
     * wake from Stop works with the request alone; this bit is the
     * escape for a consumer that cannot afford the start-up, and for
     * measuring the difference.
     */
    static void hsi_kernel_request(bool on) {
        RCC->CR = on ? (RCC->CR | RCC_CR_HSIKERON) : (RCC->CR & ~RCC_CR_HSIKERON);
    }
    static bool hsi_kernel_request() { return (RCC->CR & RCC_CR_HSIKERON) != 0u; }

    // 5.2.14: starting the IWDG FORCES LSI on whatever LSION says, and
    // it "cannot be disabled" afterwards - so lsi_ready() standing with
    // lsi_enabled() clear is the witness that something else (the IWDG,
    // the RTC, the CSS on LSE) is asking for it.
    static void lsi_enable(bool on) {
        RCC->CSR = on ? (RCC->CSR | RCC_CSR_LSION) : (RCC->CSR & ~RCC_CSR_LSION);
    }
    static bool lsi_enabled() { return (RCC->CSR & RCC_CSR_LSION) != 0u; }
    static bool lsi_ready() { return (RCC->CSR & RCC_CSR_LSIRDY) != 0u; }
    /// TIM16_TISEL's code for LSI on TI1 (25.6.18). The timer owns the
    /// multiplexer, this block owns the signal - the stratum's rule
    /// again, and what lets a suite weigh this oscillator with no wire.
    static constexpr uint8_t lsi_tim16_ti1_code = 1;

    static bool lsi_wait_ready() {
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if (lsi_ready()) {
                return true;
            }
        }
        return false;
    }

    // ---- peripheral clock enables (5.2.17) ----------------------------------------
    // Each verb reads its register back after the store: that read is
    // the two-cycle stall the chapter asks the software to account for,
    // and it is the same trick ST's own drivers use.
    static void io_clock(char port, bool on) {
        const uint32_t mask = gpio_port_clock_mask(port);
        RCC->IOPENR = on ? (RCC->IOPENR | mask) : (RCC->IOPENR & ~mask);
        (void)RCC->IOPENR;
    }
    static bool io_clock(char port) {
        return (RCC->IOPENR & gpio_port_clock_mask(port)) != 0u;
    }
    static void ahb_clock(uint32_t mask, bool on) {
        RCC->AHBENR = on ? (RCC->AHBENR | mask) : (RCC->AHBENR & ~mask);
        (void)RCC->AHBENR;
    }
    static void apb1_clock(uint32_t mask, bool on) {
        RCC->APBENR1 = on ? (RCC->APBENR1 | mask) : (RCC->APBENR1 & ~mask);
        (void)RCC->APBENR1;
    }
    static bool apb1_clock(uint32_t mask) { return (RCC->APBENR1 & mask) == mask; }
    static void apb2_clock(uint32_t mask, bool on) {
        RCC->APBENR2 = on ? (RCC->APBENR2 | mask) : (RCC->APBENR2 & ~mask);
        (void)RCC->APBENR2;
    }
    static bool apb2_clock(uint32_t mask) { return (RCC->APBENR2 & mask) == mask; }

    // ---- peripheral resets (5.4.15, 5.4.16) ---------------------------------
    // RCC_APBRSTRx holds a peripheral in reset while its bit stands, so a
    // reset is a PULSE and not a store: set, read back (the same
    // two-cycle stall the enables pay), clear. This is the STM32's
    // equivalent of the SAM's CTRLA.SWRST and the only way a driver gets
    // a peripheral to a documented state without writing every register
    // of it by hand - which is what a driver over a block with three
    // dozen registers would otherwise have to promise.
    static void apb1_reset(uint32_t mask) {
        RCC->APBRSTR1 |= mask;
        (void)RCC->APBRSTR1;
        RCC->APBRSTR1 &= ~mask;
        (void)RCC->APBRSTR1;
    }
    static void apb2_reset(uint32_t mask) {
        RCC->APBRSTR2 |= mask;
        (void)RCC->APBRSTR2;
        RCC->APBRSTR2 &= ~mask;
        (void)RCC->APBRSTR2;
    }

    // ---- kernel-clock multiplexers (5.4.21) ---------------------------------------
    /// A two-bit CCIPR field at `pos`: the codes are the field's own
    /// (USARTnSEL: 00 PCLK, 01 SYSCLK, 10 HSI16, 11 LSE).
    static void kernel_clock(uint8_t pos, uint8_t code) {
        RCC->CCIPR = (RCC->CCIPR & ~(0x3u << pos)) |
                     ((static_cast<uint32_t>(code) & 0x3u) << pos);
    }
    static uint8_t kernel_clock(uint8_t pos) {
        return static_cast<uint8_t>((RCC->CCIPR >> pos) & 0x3u);
    }
};

// =============================================================================
// Tasks
// =============================================================================

/// Where SYSCLK comes from. `internal` and `pll` are implemented; the
/// rest name the tree's other roots so the vocabulary does not change
/// under an application when they are built, and so asking for one
/// today is a compile error with an explanation instead of a wrong clock.
enum class ClockSource : uint8_t {
    internal,   ///< HSI16 through the HSISYS divider
    pll,        ///< HSI16 x PLL, the R output (the road to 64 MHz)
    crystal,    ///< HSE with a crystal (the Nucleo-64 ships without one)
    external,   ///< HSE in bypass mode (the ST-LINK's MCO reaches it via solder bridges)
    lsi,        ///< the ~32 kHz internal RC as SYSCLK
    lse,        ///< the 32.768 kHz crystal as SYSCLK
};

/**
 * The static main clock: `hz` is the ONE compile-time truth about SYSCLK
 * that every driver of this target derives from.
 *
 *   using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
 *   constexpr SysClock clock;
 *   SysClock::init();                // first thing in main()
 *   Serial::init(clock, 115200);     // drivers ask the tag
 *
 * `hz` is SYSCLK = HCLK; `pclk_hz` is the APB rate, equal to `hz`
 * because this task pins both prescalers at 1 (see the file header).
 */
template <ClockSource src, uint32_t src_hz>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t hz = src_hz;        ///< SYSCLK = HCLK (HPRE = 1)
    static constexpr uint32_t pclk_hz = src_hz;   ///< PCLK (PPRE = 1)
    static constexpr bool is_static = true;

    static_assert(src == ClockSource::internal || src == ClockSource::pll,
                  "brio Clock: only ClockSource::internal (HSI16 / HSIDIV) and "
                  "ClockSource::pll (HSI16 x PLL -> PLLRCLK) are implemented on the "
                  "STM32G0 today - HSE, LSI and LSE arrive with their first consumer");
    static_assert(src != ClockSource::internal || hsidiv_for(src_hz) != 0xFF,
                  "brio Clock: an HSISYS rate is 16 MHz divided by a power of two "
                  "(16, 8, 4, 2, 1, 0.5, 0.25, 0.125 MHz); anything else wants the PLL");
    static_assert(src != ClockSource::pll || pll_config_for(src_hz).m != 0,
                  "brio Clock: no exact PLL ratio reaches this rate from HSI16 - the "
                  "input after /M must sit in 2.66..16 MHz, the VCO in 96..344 MHz, "
                  "R in 2..8, and PLLRCLK must not exceed 64 MHz (RM0444 5.4.4)");
    static_assert(src_hz <= sysclk_max_hz, "SYSCLK must not exceed 64 MHz");

    /// The setting `hz` needs (meaningful for the source it belongs to).
    static constexpr uint8_t hsidiv = hsidiv_for(src_hz);
    static constexpr PllConfig pll = pll_config_for(src_hz);

    /// Bring SYSCLK to `hz`. Returns false when a root did not report
    /// ready, the switch did not take, or the wait states did not land -
    /// the caller then knows the rate is NOT the one `hz` claims. Call
    /// first in main(), before any driver init.
    ///
    /// The whole sequence is RE-STATED rather than assumed: the part
    /// boots on HSI16 undivided, but a debugger or a bootloader may have
    /// left anything behind, and `hz` is a promise.
    static bool init() {
        // This stratum's latency table is the Range 1 column; a core left
        // in Range 2 by someone else would be under-waited at 64 MHz. PWR
        // is an APB peripheral with an enable bit of its own (APBENR1.PWREN,
        // clear at reset), and 5.2.17 says a clockless peripheral's
        // registers are not readable - the bench read the right reset
        // value through the closed gate once, which is luck and not a
        // contract, so the gate is opened first and left open (the sleep
        // site will want it anyway).
        Pwr::bus_clock(true);
        if (Pwr::range() != 1u) {
            return false;
        }

        // Wait states BEFORE a rise, AFTER a fall (3.3.4).
        constexpr uint8_t ws = FlashWaitStates::for_hz(hz);
        const bool raising = ws > FlashWaitStates::get();
        bool ok = true;
        if (raising) {
            ok = FlashWaitStates::set(ws);
        }

        Rcc::hsi_enable(true);
        ok = Rcc::hsi_wait_ready() && ok;

        if constexpr (src == ClockSource::internal) {
            // Park SYSCLK on HSISYS first (it already is, out of reset),
            // then set the divider under it, then put the PLL away.
            Rcc::sysclk_select(SysclkSource::hsisys);
            ok = Rcc::sysclk_wait(SysclkSource::hsisys) && ok;
            Rcc::hsi_div(hsidiv);
            Rcc::pll_enable(false);
        } else {
            // The PLL is configured only while stopped (5.2.4); if SYSCLK
            // is on it from a previous life, step off first.
            if (Rcc::sysclk_status() == SysclkSource::pllrclk) {
                Rcc::hsi_div(0);
                Rcc::sysclk_select(SysclkSource::hsisys);
                ok = Rcc::sysclk_wait(SysclkSource::hsisys) && ok;
            }
            Rcc::pll_enable(false);
            ok = Rcc::pll_wait(false) && ok;
            ok = Rcc::pll_configure(pll) && ok;
            Rcc::pll_enable(true);
            ok = Rcc::pll_wait(true) && ok;
            Rcc::sysclk_select(SysclkSource::pllrclk);
            ok = Rcc::sysclk_wait(SysclkSource::pllrclk) && ok;
        }

        Rcc::bus_prescalers_unity();

        if (!raising) {
            ok = FlashWaitStates::set(ws) && ok;
        }
        return ok;
    }
};

} // namespace brio
