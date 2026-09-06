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
 *  TASKS - what an application names:
 *    Clock<source, hz, regime>   the static main clock: ONE constexpr
 *             truth `hz` every driver derives from (no F_CPU in this
 *             build, exactly as on the other two targets); init()
 *             composes the resources and reports whether the requested
 *             root runs. `regime` is the VOLTAGE SIDE of the rate - the
 *             VCORE range and the regulator (PowerRegime below) - Range 1
 *             by default, which is what every program before the dynamic
 *             clock ran in.
 *    DynamicClock<Rates<...>, Users...>   the runtime regime: a PACK of
 *             static Clock tasks the program may run at, the first the
 *             boot rate, set<hz>() / set(hz) / set_index() switching
 *             between them under the running program after fanning the
 *             new rate out to Users (util/clock.hpp's ClockUser contract,
 *             the avrdx precedent) - and restore(), the verb the sleep
 *             site calls when a Stop has dropped SYSCLK to HSISYS.
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
 * refused.
 *
 * THE DYNAMIC CLOCK, AND WHY A RATE IS A TUPLE HERE (docs/design/
 * clock.md, "The other targets"). On the AVR a dynamic clock's rate is
 * the boot rate over one prescaler, so its discrete set is an array the
 * type indexes; on this family a rate is (the SYSCLK root and its rate,
 * the VCORE range, the regulator) - 64 MHz on the PLL in Range 1, 16 MHz
 * on HSISYS, 2 MHz on HSISYS/8 in Range 2 on the low-power regulator -
 * and the reachable rates come from two disjoint families with no single
 * prescaler to index. So the discrete set is an EXPLICIT PACK the program
 * names, each member a static Clock task with its regime, and the switch
 * is DIRECTION-AWARE around the flash latency and the regulator:
 *   rising:  leave low-power run (REGLPF clear), the range UP (VOSF
 *            clear), the fan-out, then the rate's own init() - wait
 *            states up, then the root;
 *   falling: the fan-out, the rate's init() - the root, then wait states
 *            down - the range DOWN, and low-power run LAST, because 4.3.2
 *            wants SYSCLK at or below 2 MHz before it.
 * A rate in Range 2 takes its wait states from table 13's Range 2 column
 * inside its own init(), which is why the range is safe to lower after
 * it: the stricter column is already in force. What a Stop leaves behind
 * (HSISYS, the PLL off, HSIDIV and LPR kept) is put back by restore() -
 * the CURRENT rate, no fan-out, the sleep site's verb.
 *
 * THE FAN-OUT IS SMALL BY CONSTRUCTION: RCC_CCIPR takes a peripheral OFF
 * SYSCLK (a USART on HSI16 or LSE, the LPTIM on the crystal, the ADC on
 * HSI16, the RTC and the IWDG never on it), and a tickless program's
 * kernel timebase is off it too. What follows SYSCLK and says so with a
 * rebase(): the Uart on PCLK/SYSCLK (a no-op on HSI16/LSE), the Adc in a
 * PCLK mode (a divider that keeps its clock in range), and SysTick as
 * BasicTicker or SysTickCounter (a new reload). The timers take a static
 * clock only (tim.hpp says why), the FDCAN and the WWDG take the bus
 * rate as a NUMBER.
 *
 * Facts that shape the code (RM0444 5.2, 5.4, 3.3.4, 4.1.4; ES0548 on
 * silicon rev Z, DBGMCU_IDCODE 0x10016467):
 *  - out of reset the device runs HSI16 undivided as HSISYS = SYSCLK =
 *    HCLK = PCLK at 16 MHz, VCORE Range 1, FLASH_ACR.LATENCY 0;
 *  - table 13 ties the wait states to HCLK per voltage range, and 3.3.4
 *    orders them BEFORE a rise (with a readback) and AFTER a fall;
 *  - 4.1.4: Range 2 serves up to 16 MHz; going up is VOS then VOSF then
 *    the wait states then the frequency, going down is the reverse;
 *    4.3.2: low-power run wants SYSCLK <= 2 MHz first and REGLPF clear
 *    before any rise after leaving it;
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
 *    divided `internal` rate is therefore a stated caveat for the sleep
 *    site (Pwr::stop_hsidiv_hazard() is the predicate), not a refusal.
 */

#pragma once

#include <stdint.h>

#include <concepts>

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

    // ---- the clock output (5.2.16) --------------------------------------------
    //
    // RCC_CFGR.MCOSEL / MCOPRE: one of the tree's clocks, prescaled by a
    // power of two, as a SIGNAL. It reaches a pad through that pad's
    // alternate function (PA8's AF0 is MCO on every part - the pin's
    // job, not this file's) and, with no pad at all, the timers' input
    // multiplexers: TIM2/TIM3's ETRSEL and TIM14/16/17's TISEL name
    // "MCO" among their sources, which is what makes an internal clock
    // COUNTABLE by a timer that is not on it - HSI16/64 into TIM2's ETR
    // is a 4 us wall that does not move with SYSCLK (test_stm32_clock's
    // instrument). MCO2 (the G0B1/G0C1's second output) is not built.
    /// The codes common to every header of the pack; 8..11 (PLLP, PLLQ,
    /// RTCCLK, RTC_WAKEUP) are the G0B1 class's own and pass as literals.
    static bool mco(uint8_t source_code, uint8_t log2_div) {
        const uint32_t sel = static_cast<uint32_t>(source_code) << RCC_CFGR_MCOSEL_Pos;
        const uint32_t pre = static_cast<uint32_t>(log2_div) << RCC_CFGR_MCOPRE_Pos;
        if ((sel & ~RCC_CFGR_MCOSEL_Msk) != 0u || (pre & ~RCC_CFGR_MCOPRE_Msk) != 0u) {
            return false;
        }
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCOSEL_Msk | RCC_CFGR_MCOPRE_Msk)) | sel | pre;
        return true;
    }
    static void mco_off() { RCC->CFGR = RCC->CFGR & ~(RCC_CFGR_MCOSEL_Msk | RCC_CFGR_MCOPRE_Msk); }
    static constexpr uint8_t mco_off_code = 0;
    static constexpr uint8_t mco_sysclk_code = 1;
    static constexpr uint8_t mco_hsi16_code = 3;
    static constexpr uint8_t mco_hse_code = 4;
    static constexpr uint8_t mco_pllr_code = 5;
    static constexpr uint8_t mco_lsi_code = 6;
    static constexpr uint8_t mco_lse_code = 7;

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

    /// The SAME two-bit field engine over RCC_CCIPR2 (5.4.22), the
    /// SECOND independent-clock register: the I2S, USB and FDCAN
    /// multiplexers live there and not in CCIPR. It is a separate verb
    /// and not a wider `kernel_clock` because it is a separate REGISTER,
    /// and because the register is a struct member only the G0B1/G0C1
    /// header declares - the reserve hands back a null pointer on the
    /// parts without it (rcc_ccipr2(), the flash_ecc2r() precedent), so
    /// this file still compiles on every header of the pack.
    ///
    /// False when the device has no CCIPR2 at all, with NOTHING written.
    static bool kernel_clock2(uint8_t pos, uint8_t code) {
        volatile uint32_t* reg = rcc_ccipr2();
        if (reg == nullptr || pos > 30u) {
            return false;
        }
        *reg = (*reg & ~(0x3u << pos)) |
               ((static_cast<uint32_t>(code) & 0x3u) << pos);
        return true;
    }
    static uint8_t kernel_clock2(uint8_t pos) {
        volatile uint32_t* reg = rcc_ccipr2();
        if (reg == nullptr || pos > 30u) {
            return 0xFF;
        }
        return static_cast<uint8_t>((*reg >> pos) & 0x3u);
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

/// The voltage side of a rate: which VCORE range the main regulator
/// holds (4.1.4) and whether the low-power regulator supplies VCORE
/// instead (4.3.2). Range 1 is the reset state and serves every rate to
/// 64 MHz; Range 2 serves up to 16 MHz with one more wait state at the
/// top of its column and less current; low-power run is Range 2 with
/// the main regulator off, legal at or below 2 MHz - the one lever the
/// other two families have not got, and the reason this target's
/// dynamic clock exists at all.
enum class PowerRegime : uint8_t {
    range1,          ///< VOS = Range 1, main regulator (the reset state)
    range2,          ///< VOS = Range 2, main regulator: SYSCLK <= 16 MHz
    low_power_run,   ///< Range 2 and PWR_CR1.LPR: SYSCLK <= 2 MHz
};

inline constexpr uint32_t range2_max_hz = 16'000'000UL;
inline constexpr uint32_t low_power_run_max_hz = 2'000'000UL;

/// Table 13's column for a regime: the Range 2 column serves both
/// Range 2 regimes (the low-power regulator's VCORE is the Range 2
/// level, 4.3.2).
constexpr uint8_t flash_wait_states_for(PowerRegime regime, uint32_t hz) {
    return regime == PowerRegime::range1 ? FlashWaitStates::for_hz(hz)
                                         : FlashWaitStates::for_hz_range2(hz);
}

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
 *
 * `regime` is the rate's voltage side (PowerRegime): Range 1 by default,
 * and init() then REFUSES a core it finds in Range 2 rather than raising
 * it, because a range change is a sequence around the wait states that
 * belongs to whoever ordered the rates - DynamicClock below, or the
 * program. A Range 2 rate's init() is the whole falling sequence of
 * 4.1.4 on its own: the wait states from the Range 2 column (the
 * stricter one, so it is legal to set them in either range), the root,
 * then the range down and, for `low_power_run`, LPR last; it first
 * LEAVES low-power run if a previous life set it, because every step of
 * that sequence is priced for the main regulator. So a static Range 2
 * clock stands alone as a boot clock, and is also exactly the body a
 * dynamic switch runs.
 */
template <ClockSource src, uint32_t src_hz, PowerRegime regime = PowerRegime::range1>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t hz = src_hz;        ///< SYSCLK = HCLK (HPRE = 1)
    static constexpr uint32_t pclk_hz = src_hz;   ///< PCLK (PPRE = 1)
    static constexpr bool is_static = true;
    static constexpr PowerRegime power_regime = regime;
    /// The RCC_CFGR.SWS value this task's root reports once in force.
    static constexpr SysclkSource sysclk_source =
        src == ClockSource::pll ? SysclkSource::pllrclk : SysclkSource::hsisys;

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
    static_assert(regime == PowerRegime::range1 || src_hz <= range2_max_hz,
                  "brio Clock: VCORE Range 2 serves SYSCLK up to 16 MHz (RM0444 4.1.4) "
                  "- a faster rate is a Range 1 rate");
    static_assert(regime != PowerRegime::low_power_run || src_hz <= low_power_run_max_hz,
                  "brio Clock: low-power run wants SYSCLK at or below 2 MHz (RM0444 4.3.2)");

    /// The setting `hz` needs (meaningful for the source it belongs to).
    static constexpr uint8_t hsidiv = hsidiv_for(src_hz);
    static constexpr PllConfig pll = pll_config_for(src_hz);
    /// Table 13's wait states for `hz` in this regime's column.
    static constexpr uint8_t wait_states = flash_wait_states_for(regime, src_hz);

    /// Bring SYSCLK to `hz`. Returns false when a root did not report
    /// ready, the switch did not take, or the wait states did not land -
    /// the caller then knows the rate is NOT the one `hz` claims. Call
    /// first in main(), before any driver init.
    ///
    /// The whole sequence is RE-STATED rather than assumed: the part
    /// boots on HSI16 undivided, but a debugger or a bootloader may have
    /// left anything behind, and `hz` is a promise.
    static bool init() {
        // PWR is an APB peripheral with an enable bit of its own
        // (APBENR1.PWREN, clear at reset), and 5.2.17 says a clockless
        // peripheral's registers are not readable - the bench read the
        // right reset value through the closed gate once, which is luck
        // and not a contract, so the gate is opened first and left open
        // (the sleep site wants it anyway).
        Pwr::bus_clock(true);
        bool ok = true;
        if constexpr (regime == PowerRegime::range1) {
            // This regime's latency table is the Range 1 column; a core
            // left in Range 2 by someone else would be under-waited at
            // 64 MHz, and raising the range is the orderer's job (the
            // class comment) - refused, nothing written.
            if (Pwr::range() != 1u) {
                return false;
            }
        } else {
            // A Range 2 rate: leave the low-power regulator first if a
            // previous life is on it (4.3.2's exit - LPR clear, REGLPF
            // clear - is legal at any rate; its 2 MHz limit is the
            // ENTRY's, taken last below). The range itself is lowered
            // after the root, once the Range 2 wait states are in force.
            if (Pwr::low_power_run()) {
                ok = Pwr::low_power_run(false);
            }
        }

        // Wait states BEFORE a rise, AFTER a fall (3.3.4).
        constexpr uint8_t ws = wait_states;
        const bool raising = ws > FlashWaitStates::get();
        if (raising) {
            ok = FlashWaitStates::set(ws) && ok;
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

        if constexpr (regime != PowerRegime::range1) {
            // 4.1.4's falling order: frequency, wait states (the Range 2
            // column is already in force), then VOS - and the low-power
            // regulator last of all, at a rate 4.3.2 allows it (the
            // static_assert above). Pwr::range() waits VOSF out; entering
            // LPR has no readback to wait for (REGLPF rises, and it is
            // the EXIT that waits on it).
            if (Pwr::range() != 2u) {
                ok = Pwr::range(2) && ok;
            }
            if constexpr (regime == PowerRegime::low_power_run) {
                ok = Pwr::low_power_run(true) && ok;
            }
        }
        return ok;
    }
};

/// The pack of rates a DynamicClock may run at: static Clock tasks, the
/// FIRST the boot rate. A type list and nothing else - the AVR's
/// discrete set is an array over one prescaler, this family's is the
/// program's own choice of tuples (the file header).
template <typename... Rs>
struct Rates {
    static constexpr uint8_t count = sizeof...(Rs);
};

/**
 * The runtime regime: `Rates<R0, R1, ...>` names the rates - each a
 * static `Clock<source, hz, regime>`, R0 the boot rate - and Users the
 * drivers a switch fans the new rate out to (each a ClockUser: `static
 * void rebase(uint32_t hz)`, checked by the concept where the list is
 * written) IN LIST ORDER, synchronously, BEFORE the rate changes - so a
 * user can drain what it has in flight at the old rate. Then the target
 * rate's own init() runs, with this file's direction-aware steps around
 * it (the file header): a rise leaves low-power run and raises the
 * range FIRST, a fall lets the rate's init() lower them LAST.
 *
 *   using Fast = brio::Clock<brio::ClockSource::pll, 64'000'000>;
 *   using Mid  = brio::Clock<brio::ClockSource::internal, 16'000'000,
 *                            brio::PowerRegime::range2>;
 *   using Slow = brio::Clock<brio::ClockSource::internal, 2'000'000,
 *                            brio::PowerRegime::low_power_run>;
 *   using SysClock = brio::DynamicClock<brio::Rates<Fast, Mid, Slow>,
 *                                       brio::SysTickCounter, Link, brio::Adc>;
 *   constexpr SysClock clock;
 *   SysClock::init();                 // Fast's init: 64 MHz, Range 1
 *   SysClock::set<2'000'000>();       // the users rebased, then Slow
 *
 * The discrete-rate surface (docs/design/clock.md) is the pack's:
 * rate_count, rate_hz(i), rate_index() - what armv6m/delay.hpp
 * dispatches on so that no division runs at wait time. Two rates may
 * share an hz (16 MHz in Range 1 and in Range 2 are different tuples):
 * set<hz>() and set(hz) take the FIRST that matches, set_index<i>() /
 * set_index(i) name one exactly.
 *
 * Call set() only when nothing that depends on the rate is
 * mid-transfer (a bus transaction in flight is the caller's problem -
 * in an AO system, ask the bus AOs first). Main context only.
 *
 * WHAT A STOP DOES TO THIS: the part comes out of Stop 0/1 on HSISYS
 * with the PLL off, HSIDIV kept, LPR kept (pwr.hpp fact 4). restore()
 * re-runs the CURRENT rate's init() with no fan-out - the users were
 * configured for that rate and it is that rate that comes back - and
 * is what stm32g0/sleep.hpp's site calls first thing after a wake; a
 * rate already in force (SWS says so: every HSISYS rate, since HSIDIV
 * survives) costs one register read.
 */
template <typename RateList, ClockUser... Users>
struct DynamicClock;

template <typename... Rs, ClockUser... Users>
struct DynamicClock<Rates<Rs...>, Users...> {
    static constexpr bool is_static = false;
    static constexpr uint8_t rate_count = sizeof...(Rs);
    static_assert(rate_count >= 1, "brio DynamicClock: at least the boot rate");
    static_assert(rate_count <= 16, "brio DynamicClock: sixteen rates is more than any program needs");
    static_assert((Rs::is_static && ...), "brio DynamicClock: every rate is a static Clock task");
#if defined(F_CPU)
    static_assert(false, "brio DynamicClock: F_CPU must not be defined with a runtime clock");
#endif

    /// SYSCLK = HCLK now, and PCLK, which equals it (the prescalers are
    /// pinned at 1 by every rate's init()).
    static uint32_t hz() { return hz_; }
    static uint32_t pclk_hz() { return hz_; }

    /// The discrete-rate surface: the pack, by position.
    static constexpr uint32_t rate_hz(uint8_t i) { return rate_hz_[i]; }
    static uint8_t rate_index() { return idx_; }
    static constexpr PowerRegime rate_regime(uint8_t i) { return rate_regime_[i]; }
    static constexpr SysclkSource rate_source(uint8_t i) { return rate_source_[i]; }
    /// The regime in force now, by the mirror (the silicon's own
    /// readbacks are Pwr::range() and Pwr::low_power_run()).
    static PowerRegime power_regime() { return rate_regime_[idx_]; }

    /// Is U one of the users that set() rebases? Drivers assert this in
    /// init(clock): a clocked driver forgotten in the list would keep
    /// running at the old rate in silence - a compile error instead.
    template <typename U>
    static constexpr bool rebases = (std::same_as<U, Users> || ...);

    /// The position of the first rate at `hz`, or rate_count when none.
    static constexpr uint8_t index_of(uint32_t hz) {
        for (uint8_t i = 0; i < rate_count; ++i) {
            if (rate_hz_[i] == hz) {
                return i;
            }
        }
        return rate_count;
    }
    static constexpr bool can_run_at(uint32_t hz) { return index_of(hz) < rate_count; }

    /// The boot rate (the pack's first), no fan-out: nothing is
    /// initialized yet. See Clock::init for the return.
    static bool init() { return enter(0, false); }

    /// Switch to a rate known at compile time (checked: a rate the pack
    /// does not name does not compile).
    template <uint32_t hz>
    static bool set() {
        static_assert(can_run_at(hz), "brio DynamicClock: this rate is not in the Rates pack");
        return enter(index_of(hz), true);
    }
    /// Switch to a rate chosen at run time; false (nothing changed) when
    /// the pack has no such rate.
    static bool set(uint32_t hz) {
        const uint8_t i = index_of(hz);
        if (i >= rate_count) {
            return false;
        }
        return enter(i, true);
    }
    /// The same by position, for a program whose rates share an hz.
    template <uint8_t i>
    static bool set_index() {
        static_assert(i < rate_count, "brio DynamicClock: no such rate");
        return enter(i, true);
    }
    static bool set_index(uint8_t i) {
        if (i >= rate_count) {
            return false;
        }
        return enter(i, true);
    }

    /// Put the CURRENT rate back after a Stop dropped SYSCLK to HSISYS -
    /// no fan-out (the users hold the divisors for exactly this rate).
    /// True at once when SWS already reports the rate's root, and true
    /// with nothing done while a set() is in progress in thread mode
    /// (this verb is legal from an ISR - the wake's own, if a program
    /// wants the rate back before the first AO runs - and a switch
    /// parks SYSCLK on HSISYS for a moment on its way to the PLL).
    static bool restore() {
        if (switching_ || Rcc::sysclk_status() == rate_source_[idx_]) {
            return true;
        }
        return enter(idx_, false);
    }
    /// A set() is between its first and its last store.
    static bool switching() { return switching_; }

private:
    static constexpr uint32_t rate_hz_[rate_count] = {Rs::hz...};
    static constexpr PowerRegime rate_regime_[rate_count] = {Rs::power_regime...};
    static constexpr SysclkSource rate_source_[rate_count] = {Rs::sysclk_source...};

    /// The switch, for one rate of the pack, in the order the file
    /// header states. The mirror is written BEFORE the rate's init()
    /// so that a user rebased for the new rate and a delay_us
    /// dispatching on the index agree from the first instruction after
    /// the switch; a failed init() returns false and the mirror then
    /// names what was ASKED, which is Clock::init's own contract.
    template <typename R>
    static bool enter_rate(uint8_t i, bool fan_out) {
        switching_ = true;
        bool ok = true;
        if constexpr (R::power_regime == PowerRegime::range1) {
            // Rising into Range 1: the regulator before anything - LPR
            // off and REGLPF clear (4.3.2), then VOS 1 and VOSF clear
            // (4.1.4) - because R::init() refuses a core in Range 2.
            if (Pwr::low_power_run()) {
                ok = Pwr::low_power_run(false);
            }
            if (Pwr::range() != 1u) {
                ok = Pwr::range(1) && ok;
            }
        }
        // (A Range 2 rate's init() leaves LPR itself and lowers the
        // range last - nothing to do here in that direction.)
        if (fan_out) {
            (Users::rebase(R::hz), ...);
        }
        hz_ = R::hz;
        idx_ = i;
        ok = R::init() && ok;
        if constexpr (R::sysclk_source == SysclkSource::pllrclk) {
            // HSISYS IS NOT LEFT DIVIDED BEHIND A PLL RATE. A PLL rate
            // takes HSI16 undivided and its init() has no reason to
            // touch HSIDIV, so a program that came up the ladder from
            // HSISYS/8 would keep the divider - harmless to the rate,
            // measured (test_stm32_clock letter d, the first version),
            // and NOT harmless to what a Stop lands on (HSISYS at 2 MHz
            // instead of 16, and ES0548 2.2.4's wake hazard along with
            // it). SYSCLK is on the PLL here, so the write is free.
            Rcc::hsi_div(0);
        }
        switching_ = false;
        return ok;
    }

    /// Dispatch by position: a fold over the pack that stops at the
    /// i-th member (no table, no RAM).
    static bool enter(uint8_t i, bool fan_out) {
        bool ok = false;
        uint8_t k = 0;
        ((k == i ? (ok = enter_rate<Rs>(k, fan_out), true) : (++k, false)) || ...);
        return ok;
    }

    static inline uint32_t hz_ = rate_hz_[0];
    static inline uint8_t idx_ = 0;
    static inline volatile bool switching_ = false;
};

} // namespace brio
