/*
 * clock.hpp
 *
 * The STM32F4 clock tree (RCC, RM0090 ch. 6/7, RM0390 ch. 6, RM0383
 * ch. 6) in the two strata every brio target uses (docs/design/clock.md):
 *
 *  RESOURCES - monostates, thin typed views with the discipline built in:
 *    Rcc      the reset and clock control block: the HSI root, the HSE
 *             root (crystal or bypass), the main PLL (configure, start,
 *             lock), the SYSCLK switch with its status readback, the
 *             AHB/APB prescalers, the PERIPHERAL CLOCK ENABLES (one bit
 *             per peripheral in AHB1ENR/AHB2ENR/AHB3ENR/APB1ENR/APB2ENR)
 *             and resets, the two MCO outputs, and the AUDIO PLL with the
 *             I2S clock selector that is the I2S chapter's kernel clock
 *    (`Pwr` - the regulator scale and the over-drive pair the task
 *    below sequences - lives in stm32f4/pwr.hpp; the flash latency in
 *    stm32f4/flash.hpp. One chapter, one owner.)
 *
 *  TASKS - what an application names:
 *    Clock<source, hz, hse_hz, hse_mode>   the static main clock: ONE
 *             constexpr truth `hz` every driver derives from (there is
 *             no F_CPU in this build); init() composes the resources
 *             and reports whether the requested root runs.
 *    Rates<R0, R1, ...> + DynamicClock<Rates<...>, Users...>   the
 *             runtime regime: a PACK of static Clock tasks, R0 the boot
 *             rate, `set<hz>()`/`set(hz)` fanning the new rate out to the
 *             listed ClockUsers and then switching, `restore()` after a
 *             Stop. Its own header comment carries the ordering rules.
 *
 * THE CLOCK MODEL: THE STM32G0'S, WITH THE PRESCALERS NO LONGER PINNED.
 * This family has one SYSCLK, SHARED PRESCALERS below it (HPRE for AHB,
 * PPRE1 and PPRE2 for the two APBs) and an ENABLE BIT per peripheral
 * that gates its bus clock - the STM32G0's model. What the G0 stratum
 * could do and this one cannot is pin every prescaler at 1: the APBs
 * have CEILINGS (45 and 90 MHz on the F42x/F43x and F446, 42 and 84 on
 * the F405 class, 50 and 100 on the F411 - the reserve's ladder) and a
 * core at 180 MHz cannot feed them undivided. So `hz` is SYSCLK = HCLK
 * (HPRE stays 1), and beside it the task states `pclk1_hz` and
 * `pclk2_hz`, each the largest power-of-two division of HCLK under its
 * bus's ceiling - and a driver on an APB asks `apb_hz(clock, on_apb2)`
 * for the rate that is really its own (the USART's divisor divides its
 * bus clock, not SYSCLK). What crosses the util contract is unchanged:
 * `clock_hz(clock)` is SYSCLK = HCLK. The timers' kernel clock (twice
 * the APB clock when the APB is divided, RM0090 7.3.3's TIMPRE rule)
 * is the timer chapter's to state.
 *
 * THREE THINGS ON THE WAY UP, IN THE CHAPTER'S ORDER. Raising this
 * family above its 16 MHz reset rate is a sequence the manuals spell
 * out and this task follows literally (RM0090 5.1.4, 3.5.1, 7.3.3):
 *   1. the REGULATOR SCALE (PWR_CR.VOS) for the target rate, written
 *      while the PLL is off and SYSCLK is HSI or HSE - the reset state;
 *   2. the PLL configured and started; where the rate needs OVER-DRIVE
 *      (above 168 MHz on the parts that have it, or above 144 at scale 2)
 *      the over-drive pair is sequenced now, between the PLL's start and
 *      the switch to it, with no peripheral clock yet enabled;
 *   3. the FLASH LATENCY for the target HCLK, read back; the ART
 *      accelerator on; the APB prescalers for the target rate; the PLL
 *      waited for; the switch, and SWS read back.
 * `Clock::init()` itself sequences no fall: it runs once from the reset
 * state, and every rate a program names is a rise from 16 MHz. THE FALL
 * IS `DynamicClock`'S, and on this family it is one move and not a
 * mirror image - a switch parks SYSCLK on the HSI first, because neither
 * the PLL nor the regulator scale nor the over-drive bits may be written
 * with the PLL running; after the park every rate is a rise. Its own
 * header comment says the rest.
 *
 * THE PLL RATIO IS SEARCHED AT COMPILE TIME under the chapter's limits
 * (RM0090 7.3.2): the input divided by M must land in 1..2 MHz (2 MHz
 * "to limit PLL jitter" - the search takes the smallest M, which is the
 * largest input the window allows), the VCO in 100..432 MHz, N in
 * 50..432, P in {2, 4, 6, 8}; an EXACT ratio or a compile error naming
 * the rule. Q, the 48 MHz output for USB OTG FS, SDIO and the RNG, is
 * the smallest divider that keeps VCO / Q at or below 48 MHz, and
 * `usb_hz` says what came out: 48 MHz exactly at 168 or 96 MHz, 45 MHz
 * at 180 (a USB program picks 168; the chapter's own choice).
 *
 * WHAT IS BUILT: `hsi` (HSI undivided, 16 MHz - the reset state, the
 * one rate every header runs at with no ladder read), `hse` (the
 * crystal or the bypassed clock undivided), `pll_hsi` and `pll_hse` (the
 * PLL from either root). LSI, LSE, the SAI PLL and the AHB prescaler are
 * DECLARED and refused.
 *
 * THE AUDIO PLL IS NOT A SYSCLK ROOT and is therefore not a Clock task's
 * business: PLLI2S feeds the I2S kernel clock alone, off the same root
 * the main PLL uses, so its verbs sit on `Rcc` (the block that owns
 * PLLI2SCFGR and the I2S source selector) and stm32f4/spi.hpp is what
 * calls them. Where the header declares no PLLI2SON there is no audio PLL
 * and every one of those verbs answers false.
 *
 * BOARD FACTS the apps of this project state: 8 MHz from the ST-LINK's
 * MCO into HSE in bypass on the Nucleo-F446RE (UM1724 7.9.1, board
 * revision C-02 and up), an 8 MHz crystal on HSE on the STM32F429I-DISC1
 * (UM1670 7.12.1: X3 fitted, the MCO route's SB18 open - a bypass there
 * never sees HSERDY, measured), a 25 MHz crystal on the F411 black pill.
 *
 * Facts that shape the code (RM0090 7.2, 7.3; RM0383 6.2, 6.3):
 *  - out of reset the device runs HSI at 16 MHz as SYSCLK = HCLK =
 *    PCLK1 = PCLK2, FLASH_ACR.LATENCY 0, the accelerator off;
 *  - the HSE takes a crystal of 4..26 MHz or a bypassed clock of 1..50
 *    MHz (the datasheets' tables), HSEON then HSERDY; HSEBYP is written
 *    before HSEON;
 *  - PLLCFGR may be written only while the PLL is off (7.3.2); PLLON
 *    then PLLRDY;
 *  - 7.3.3: a SYSCLK switch takes effect only when the target is ready,
 *    and SWS reports which source is in force - init() waits for it;
 *    the APB prescaler cautions are the ceilings above;
 *  - 7.3.10 and its twins: a peripheral's enable bit needs a dummy read
 *    before the peripheral is used (ES0206 2.2.7, "Delay after an RCC
 *    peripheral clock enabling"); every bus-enable verb below reads the
 *    register back, which is that access.
 */
#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/device_tables.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/pwr.hpp"
#include "util/clock.hpp"

namespace brio {

// ---- vocabulary -----------------------------------------------------------------

/// RCC_CFGR.SW / SWS encodings (7.3.3).
enum class SysclkSource : uint8_t { hsi = 0, hse = 1, pll = 2 };

/// The main PLL's fields (7.3.2): the input divider M (2..63), the VCO
/// multiplier N (50..432), the SYSCLK divider P (2, 4, 6, 8) and the
/// 48 MHz-domain divider Q (2..15). `m == 0` means "no configuration".
struct PllConfig {
    uint8_t m = 0;
    uint16_t n = 0;
    uint8_t p = 0;
    uint8_t q = 0;
    bool from_hse = false;
};

inline constexpr uint32_t pll_input_min_hz = 1'000'000u;
inline constexpr uint32_t pll_input_max_hz = 2'000'000u;
inline constexpr uint32_t pll_vco_min_hz = 100'000'000u;
inline constexpr uint32_t pll_vco_max_hz = 432'000'000u;
inline constexpr uint32_t pll_q_domain_max_hz = 48'000'000u;

/// The exact PLL ratio for `out_hz` from a `src_hz` root, or a config
/// with m == 0 when none exists under the limits. Smallest M first
/// (the largest legal input), then P ascending - the first exact N wins.
constexpr PllConfig pll_config_for(uint32_t src_hz, uint32_t out_hz, bool from_hse) {
    for (uint8_t m = 2; m <= 63; ++m) {
        if (src_hz % m != 0u) {
            continue;
        }
        const uint32_t in = src_hz / m;
        if (in < pll_input_min_hz || in > pll_input_max_hz) {
            continue;
        }
        for (uint8_t p = 2; p <= 8; p = static_cast<uint8_t>(p + 2)) {
            const uint32_t vco = out_hz * p;
            if (vco < pll_vco_min_hz || vco > pll_vco_max_hz) {
                continue;
            }
            if (vco % in != 0u) {
                continue;
            }
            const uint32_t n = vco / in;
            if (n < 50u || n > 432u) {
                continue;
            }
            uint8_t q = 2;
            while (q < 15 && vco / q > pll_q_domain_max_hz) {
                ++q;
            }
            return PllConfig{m, static_cast<uint16_t>(n), p, q, from_hse};
        }
    }
    return PllConfig{};
}

/// The audio PLL's fields (RM0090 7.3.23, RM0390 6.3.23, RM0383 6.3.22).
/// `m` is the input divider ON THE PARTS THAT HAVE ONE OF THEIR OWN
/// (`plli2s_has_m()`); where they have not, the audio PLL divides the MAIN
/// PLL's M and this field stays 0. `n` is the VCO multiplier (50..432), `r`
/// the I2S output divider (2..7); `q` (2..15) and `p` (2, 4, 6, 8) are the
/// other two outputs, written only where the header declares the field.
struct PllI2sConfig {
    uint8_t m = 0;
    uint16_t n = 0;
    uint8_t r = 0;
    uint8_t q = 0;
    uint8_t p = 0;
};

inline constexpr uint32_t plli2s_r_min = 2;
inline constexpr uint32_t plli2s_r_max = 7;

/// The R output a configuration produces from a `src_hz` root, with
/// `main_m` the main PLL's input divider for the parts whose audio PLL has
/// no M of its own. 0 when the configuration is empty.
constexpr uint32_t plli2s_r_hz(uint32_t src_hz, const PllI2sConfig& c, uint8_t main_m) {
    const uint8_t m = c.m != 0u ? c.m : main_m;
    if (m == 0u || c.n == 0u || c.r == 0u) {
        return 0u;
    }
    return (src_hz / m) * c.n / c.r;
}

/// The exact audio-PLL ratio for an R output of `out_hz`, under the main
/// PLL's own limits (the input 1..2 MHz, the VCO 100..432 MHz, N 50..432).
/// `m_fixed` names the input divider where the part has no PLLI2SM (pass
/// the main PLL's M); 0 lets the search choose one. `n == 0` when no exact
/// ratio exists - the caller then states a rate this root cannot make.
constexpr PllI2sConfig plli2s_config_for(uint32_t src_hz, uint32_t out_hz, uint8_t m_fixed = 0) {
    const uint8_t m_lo = m_fixed != 0u ? m_fixed : 2u;
    const uint8_t m_hi = m_fixed != 0u ? m_fixed : 63u;
    for (uint8_t m = m_lo; m <= m_hi; ++m) {
        if (m == 0u || src_hz % m != 0u) {
            continue;
        }
        const uint32_t in = src_hz / m;
        if (in < pll_input_min_hz || in > pll_input_max_hz) {
            continue;
        }
        for (uint8_t r = 2; r <= 7; ++r) {
            const uint32_t vco = out_hz * r;
            if (vco < pll_vco_min_hz || vco > pll_vco_max_hz || vco % in != 0u) {
                continue;
            }
            const uint32_t n = vco / in;
            if (n < 50u || n > 432u) {
                continue;
            }
            uint8_t q = 2;
            while (q < 15 && vco / q > pll_q_domain_max_hz) {
                ++q;
            }
            return PllI2sConfig{m_fixed != 0u ? uint8_t{0} : m, static_cast<uint16_t>(n), r, q, 2};
        }
    }
    return PllI2sConfig{};
}

/// What the I2S kernel clock may be taken from. Which of the four a part
/// offers is `i2s_clock_select()`'s (stm32f4/device_tables.hpp): the parts
/// with RCC_CFGR.I2SSRC have the first two, the parts with the
/// RCC_DCKCFGR pair have all four.
enum class I2sSource : uint8_t {
    plli2s_r = 0,   ///< the audio PLL's R output
    ckin = 1,       ///< the I2S_CKIN pad, an external audio clock
    pll_r = 2,      ///< the MAIN PLL's R output
    root = 3,       ///< HSI or HSE, whichever PLLCFGR.PLLSRC names
};

/// The smallest power-of-two APB division (1, 2, 4, 8, 16) that keeps
/// `hclk / div` at or below `ceiling`; 0 when none does.
constexpr uint8_t apb_divider_for(uint32_t hclk, uint32_t ceiling) {
    for (uint8_t div = 1; div <= 16; div = static_cast<uint8_t>(div * 2)) {
        if (hclk / div <= ceiling) {
            return div;
        }
    }
    return 0;
}

/// RCC_CFGR.PPREx encoding of a divider (7.3.3): 0xx for 1, 100 for 2,
/// 101 for 4, 110 for 8, 111 for 16.
constexpr uint8_t ppre_code(uint8_t div) {
    switch (div) {
        case 1: return 0;
        case 2: return 4;
        case 4: return 5;
        case 8: return 6;
        case 16: return 7;
        default: return 0;
    }
}

/// The regulator scale `hclk` needs on this part - the lowest scale
/// whose ceiling covers it, over-drive counted where the part has it -
/// and whether over-drive is needed. `scale == scale3` with `known ==
/// false` means the ladder does not reach it.
struct RateRegime {
    bool known = false;
    VoltageScale scale = VoltageScale::scale3;
    bool over_drive = false;
};

constexpr RateRegime regime_for(uint32_t hclk) {
    constexpr SysclkLadder l = sysclk_ladder();
    RateRegime r{};
    if (!l.known) {
        return r;
    }
    if (l.scale3_hz != 0u && hclk <= l.scale3_hz) {
        return RateRegime{true, VoltageScale::scale3, false};
    }
    if (hclk <= l.scale2_hz) {
        return RateRegime{true, VoltageScale::scale2, false};
    }
    if (hclk <= l.scale1_hz) {
        return RateRegime{true, VoltageScale::scale1, false};
    }
    if (l.od_scale1_hz != 0u && hclk <= l.od_scale1_hz) {
        return RateRegime{true, VoltageScale::scale1, true};
    }
    return r;
}

// ---- the RCC resource -----------------------------------------------------------

/// The reset and clock control block as a monostate resource.
struct Rcc {
    Rcc() = delete;

    /// Bounded waits on a ready flag: the HSI is ready in microseconds,
    /// a crystal in up to a couple of milliseconds, the PLL in hundreds
    /// of microseconds; a million iterations at the 16 MHz reset rate is
    /// hundreds of milliseconds - a hardware fault, not a slow start.
    static constexpr uint32_t ready_spins = 1'000'000u;

    // ---- HSI (7.3.1) --------------------------------------------------------------
    static void hsi_enable(bool on) { bit(RCC->CR, RCC_CR_HSION, on); }
    static bool hsi_ready() { return (RCC->CR & RCC_CR_HSIRDY) != 0u; }
    static bool hsi_wait_ready() { return wait(RCC->CR, RCC_CR_HSIRDY, true); }

    // ---- HSE (7.3.1): crystal or bypassed clock ---------------------------------
    /// HSEBYP is written before HSEON (7.3.1: bypass "can be written
    /// only if the HSE oscillator is disabled").
    static void hse_enable(bool on, bool bypass) {
        if (on) {
            bit(RCC->CR, RCC_CR_HSEBYP, bypass);
            RCC->CR |= RCC_CR_HSEON;
        } else {
            RCC->CR &= ~RCC_CR_HSEON;
        }
    }
    static bool hse_ready() { return (RCC->CR & RCC_CR_HSERDY) != 0u; }
    static bool hse_wait_ready() { return wait(RCC->CR, RCC_CR_HSERDY, true); }
    static bool hse_bypassed() { return (RCC->CR & RCC_CR_HSEBYP) != 0u; }

    /// The clock security system on the HSE (7.2.7): an HSE failure then
    /// switches SYSCLK to HSI and raises the CSS interrupt on the NMI.
    static void css(bool on) { bit(RCC->CR, RCC_CR_CSSON, on); }

    // ---- LSI (7.2.8, 7.3.21): the low-speed RC, 32 kHz nominal -----------------
    //
    // Not a SYSCLK root and not a Clock task's business: this oscillator
    // is what the independent watchdog counts and what the RTC may run
    // on, so the verbs live here (the block's owner) and the chapters
    // above call them. IT SITS IN RCC_CSR, whose top eight bits are the
    // RESET FLAGS (stm32f4/reset.hpp): every write here is a
    // read-modify-write that leaves them alone, which is safe because
    // RMVF reads as zero and writing zero to it has no effect.
    //
    // LSIRDY is not a witness of LSION alone: a started IWDG forces the
    // oscillator on (7.2.9) and so does an RTC whose clock select names
    // it, and neither sets LSION.
    static void lsi_enable(bool on) { bit(RCC->CSR, RCC_CSR_LSION, on); }
    static bool lsi_enabled() { return (RCC->CSR & RCC_CSR_LSION) != 0u; }
    static bool lsi_ready() { return (RCC->CSR & RCC_CSR_LSIRDY) != 0u; }
    static bool lsi_wait_ready() { return wait(RCC->CSR, RCC_CSR_LSIRDY, true); }

    // ---- the main PLL (7.3.2) -------------------------------------------------------
    static void pll_enable(bool on) { bit(RCC->CR, RCC_CR_PLLON, on); }
    static bool pll_ready() { return (RCC->CR & RCC_CR_PLLRDY) != 0u; }
    static bool pll_wait(bool ready) { return wait(RCC->CR, RCC_CR_PLLRDY, ready); }

    /// Write PLLCFGR from a configuration. Refused (false, nothing
    /// written) while the PLL is on - 7.3.2 allows the write only then -
    /// or when the config is empty. The R field, where the header has
    /// one (the F446's I2S/SAI/SPDIF divider), is written at its reset
    /// value 2; the reserved top bits elsewhere are kept.
    static bool pll_configure(const PllConfig& c) {
        if (c.m == 0u || (RCC->CR & RCC_CR_PLLON) != 0u) {
            return false;
        }
        uint32_t v = (static_cast<uint32_t>(c.m) << RCC_PLLCFGR_PLLM_Pos) |
                     (static_cast<uint32_t>(c.n) << RCC_PLLCFGR_PLLN_Pos) |
                     (static_cast<uint32_t>((c.p >> 1) - 1u) << RCC_PLLCFGR_PLLP_Pos) |
                     (static_cast<uint32_t>(c.q) << RCC_PLLCFGR_PLLQ_Pos) |
                     (c.from_hse ? RCC_PLLCFGR_PLLSRC_HSE : 0u);
#if defined(RCC_PLLCFGR_PLLR)
        v |= 2u << RCC_PLLCFGR_PLLR_Pos;
#else
        v |= RCC->PLLCFGR & 0xF0000000u;
#endif
        RCC->PLLCFGR = v;
        return true;
    }

    // ---- the audio PLL (7.3.23) and the I2S clock select (7.3.3, 6.3.24) -------------
    //
    // A SECOND PLL off the SAME root as the main one (PLLCFGR.PLLSRC picks
    // HSI or HSE for both), whose R output is what an I2S divides. It is
    // the I2S chapter's clock and nothing else's here, so the verbs sit
    // with the block that owns the registers and stm32f4/spi.hpp calls
    // them. Absent on the parts whose header declares no PLLI2SON, where
    // every verb below answers false and writes nothing.
    static void plli2s_enable(bool on) {
#if defined(RCC_CR_PLLI2SON)
        bit(RCC->CR, RCC_CR_PLLI2SON, on);
#else
        (void)on;
#endif
    }
    static bool plli2s_ready() {
#if defined(RCC_CR_PLLI2SRDY)
        return (RCC->CR & RCC_CR_PLLI2SRDY) != 0u;
#else
        return false;
#endif
    }
    static bool plli2s_wait(bool ready) {
#if defined(RCC_CR_PLLI2SRDY)
        return wait(RCC->CR, RCC_CR_PLLI2SRDY, ready);
#else
        return !ready;
#endif
    }

    /// Write PLLI2SCFGR from a configuration. Refused (false, nothing
    /// written) while the audio PLL is on - the chapter allows the write
    /// only then - when the configuration is empty, and when the part has
    /// an input divider of its own but the caller named none. The fields
    /// the header does not declare are not written: on the F42x/F43x class
    /// there is no M (the main PLL's is used) and no P.
    static bool plli2s_configure(const PllI2sConfig& c) {
#if defined(RCC_CR_PLLI2SON)
        if (c.n == 0u || c.r == 0u || (RCC->CR & RCC_CR_PLLI2SON) != 0u) {
            return false;
        }
        if (plli2s_has_m() != (c.m != 0u)) {
            return false;
        }
        uint32_t v = (static_cast<uint32_t>(c.n) << RCC_PLLI2SCFGR_PLLI2SN_Pos) |
                     (static_cast<uint32_t>(c.r) << RCC_PLLI2SCFGR_PLLI2SR_Pos);
#if defined(RCC_PLLI2SCFGR_PLLI2SM_Pos)
        v |= static_cast<uint32_t>(c.m) << RCC_PLLI2SCFGR_PLLI2SM_Pos;
#endif
#if defined(RCC_PLLI2SCFGR_PLLI2SQ_Pos)
        v |= static_cast<uint32_t>(c.q == 0u ? 2u : c.q) << RCC_PLLI2SCFGR_PLLI2SQ_Pos;
#endif
#if defined(RCC_PLLI2SCFGR_PLLI2SP_Pos)
        v |= static_cast<uint32_t>(((c.p == 0u ? 2u : c.p) >> 1) - 1u) << RCC_PLLI2SCFGR_PLLI2SP_Pos;
#endif
        RCC->PLLI2SCFGR = v;
        return true;
#else
        (void)c;
        return false;
#endif
    }

    /// What PLLI2SCFGR holds now (m stays 0 where the part has no PLLI2SM).
    static PllI2sConfig plli2s_config() {
        PllI2sConfig c{};
#if defined(RCC_CR_PLLI2SON)
        const uint32_t v = RCC->PLLI2SCFGR;
        c.n = static_cast<uint16_t>((v & RCC_PLLI2SCFGR_PLLI2SN_Msk) >> RCC_PLLI2SCFGR_PLLI2SN_Pos);
        c.r = static_cast<uint8_t>((v & RCC_PLLI2SCFGR_PLLI2SR_Msk) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
#if defined(RCC_PLLI2SCFGR_PLLI2SM_Pos)
        c.m = static_cast<uint8_t>((v & RCC_PLLI2SCFGR_PLLI2SM_Msk) >> RCC_PLLI2SCFGR_PLLI2SM_Pos);
#endif
#if defined(RCC_PLLI2SCFGR_PLLI2SQ_Pos)
        c.q = static_cast<uint8_t>((v & RCC_PLLI2SCFGR_PLLI2SQ_Msk) >> RCC_PLLI2SCFGR_PLLI2SQ_Pos);
#endif
#if defined(RCC_PLLI2SCFGR_PLLI2SP_Pos)
        c.p = static_cast<uint8_t>(
            (((v & RCC_PLLI2SCFGR_PLLI2SP_Msk) >> RCC_PLLI2SCFGR_PLLI2SP_Pos) + 1u) * 2u);
#endif
#endif
        return c;
    }

    /// The main PLL's own input divider, for the parts whose audio PLL
    /// shares it.
    static uint8_t pll_m() {
        return static_cast<uint8_t>((RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos);
    }

    /// Point an I2S kernel clock at a source. `on_apb2` names WHICH
    /// selector on a part with the RCC_DCKCFGR pair, where the numbering
    /// is by BUS: I2S1SRC serves the APB1 instances and I2S2SRC the APB2
    /// ones. On a part with RCC_CFGR.I2SSRC there is one selector for
    /// every instance and the argument is ignored; only `plli2s_r` and
    /// `ckin` exist there, and the other two are refused. Refused whole
    /// where the part's selector is one this stratum has not read
    /// (`I2sClockSelect::dckcfgr_one`, the F410's four codes) or has none.
    static bool i2s_source(I2sSource s, bool on_apb2 = false) {
#if defined(RCC_DCKCFGR_I2S1SRC)
        const uint32_t code = static_cast<uint32_t>(s);
        if (on_apb2) {
            RCC->DCKCFGR = (RCC->DCKCFGR & ~RCC_DCKCFGR_I2S2SRC_Msk) |
                           (code << RCC_DCKCFGR_I2S2SRC_Pos);
        } else {
            RCC->DCKCFGR = (RCC->DCKCFGR & ~RCC_DCKCFGR_I2S1SRC_Msk) |
                           (code << RCC_DCKCFGR_I2S1SRC_Pos);
        }
        return true;
#elif defined(RCC_CFGR_I2SSRC)
        (void)on_apb2;
        if (s != I2sSource::plli2s_r && s != I2sSource::ckin) {
            return false;
        }
        bit(RCC->CFGR, RCC_CFGR_I2SSRC, s == I2sSource::ckin);
        return true;
#else
        (void)s;
        (void)on_apb2;
        return false;
#endif
    }

    static I2sSource i2s_source(bool on_apb2 = false) {
#if defined(RCC_DCKCFGR_I2S1SRC)
        const uint32_t v = RCC->DCKCFGR;
        const uint32_t code = on_apb2 ? ((v & RCC_DCKCFGR_I2S2SRC_Msk) >> RCC_DCKCFGR_I2S2SRC_Pos)
                                      : ((v & RCC_DCKCFGR_I2S1SRC_Msk) >> RCC_DCKCFGR_I2S1SRC_Pos);
        return static_cast<I2sSource>(code);
#elif defined(RCC_CFGR_I2SSRC)
        (void)on_apb2;
        return (RCC->CFGR & RCC_CFGR_I2SSRC) != 0u ? I2sSource::ckin : I2sSource::plli2s_r;
#else
        (void)on_apb2;
        return I2sSource::plli2s_r;
#endif
    }

    /**
     * The FMPI2C1 kernel-clock multiplexer, on the parts that carry that
     * block: RCC_DCKCFGR2's FMPI2C1SEL (RM0390 6.3.27, where ST spells the
     * field "I2C4 kernel clock source selection" - I2C4 and FMPI2C1 being
     * one peripheral under two names, ES0298 2.12.10).
     *
     * `code` is the field's own: 0 the peripheral's APB clock, 1 SYSCLK, 2
     * the HSI, 3 the APB clock again ("same as 00"). Two bits, so no code
     * is refused here; what a driver may name is its own enum's business
     * (stm32f4/fmpi2c.hpp's FmpI2cClock). False - nothing written - on a
     * part with no such peripheral.
     */
    static bool fmpi2c_kernel_clock(uint8_t code) {
#if defined(RCC_DCKCFGR2_FMPI2C1SEL)
        RCC->DCKCFGR2 = (RCC->DCKCFGR2 & ~RCC_DCKCFGR2_FMPI2C1SEL_Msk) |
                        ((static_cast<uint32_t>(code) & 0x3u) << RCC_DCKCFGR2_FMPI2C1SEL_Pos);
        return true;
#else
        (void)code;
        return false;
#endif
    }

    static uint8_t fmpi2c_kernel_clock() {
#if defined(RCC_DCKCFGR2_FMPI2C1SEL)
        return static_cast<uint8_t>((RCC->DCKCFGR2 & RCC_DCKCFGR2_FMPI2C1SEL_Msk) >>
                                    RCC_DCKCFGR2_FMPI2C1SEL_Pos);
#else
        return 0;
#endif
    }

    // ---- SYSCLK (7.3.3) --------------------------------------------------------------
    static void sysclk_select(SysclkSource s) {
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (static_cast<uint32_t>(s) << RCC_CFGR_SW_Pos);
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

    // ---- the bus prescalers (7.3.3) ----------------------------------------------------
    /// HPRE at 1 (0xxx), PPRE1 and PPRE2 from their dividers. A divider
    /// ppre_code() does not know is written as 1.
    static void bus_prescalers(uint8_t apb1_div, uint8_t apb2_div) {
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk)) |
                    (static_cast<uint32_t>(ppre_code(apb1_div)) << RCC_CFGR_PPRE1_Pos) |
                    (static_cast<uint32_t>(ppre_code(apb2_div)) << RCC_CFGR_PPRE2_Pos);
    }
    static uint8_t apb1_divider() { return divider_of((RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos); }
    static uint8_t apb2_divider() { return divider_of((RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos); }
    static bool ahb_undivided() { return (RCC->CFGR & RCC_CFGR_HPRE_3) == 0u; }

    // ---- RCC_DCKCFGR.TIMPRE (7.3.24) -----------------------------------------------
    /// THE TIMERS' OWN PRESCALER RULE, and the one thing in this block
    /// that is not the timers' chapter's: a timer counts HCLK when its APB
    /// prescaler is 1, and TWICE its APB clock when it is divided - unless
    /// this bit is set, and then it counts HCLK at a prescaler of 1 OR 2
    /// and four times PCLK beyond. Clear out of reset, and absent
    /// altogether on the F405/F407/F415/F417 headers, where the reader
    /// answers false and the setter writes nothing. What the bit means for
    /// a given instance is stm32f4/tim.hpp's `tim_clock_hz()`.
    static void timpre(bool on) {
#if defined(RCC_DCKCFGR_TIMPRE)
        bit(RCC->DCKCFGR, RCC_DCKCFGR_TIMPRE, on);
#else
        (void)on;
#endif
    }
    static bool timpre() {
#if defined(RCC_DCKCFGR_TIMPRE)
        return (RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) != 0u;
#else
        return false;
#endif
    }

    // ---- MCO (7.3.3): the two clock outputs -------------------------------------------
    /// MCO1 on PA8: 00 HSI, 01 LSE, 10 HSE, 11 PLL; the prescaler 1..5
    /// (codes 0xx = 1, 100..111 = 2..5). The pad is the caller's.
    static bool mco1(uint8_t source_code, uint8_t div) {
        if (source_code > 3u || div == 0u || div > 5u) {
            return false;
        }
        const uint32_t pre = div == 1u ? 0u : (4u + (div - 2u));
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO1_Msk | RCC_CFGR_MCO1PRE_Msk)) |
                    (static_cast<uint32_t>(source_code) << RCC_CFGR_MCO1_Pos) |
                    (pre << RCC_CFGR_MCO1PRE_Pos);
        return true;
    }
    /// MCO2 on PC9: 00 SYSCLK, 01 PLLI2S, 10 HSE, 11 PLL; the same
    /// prescaler. The F410/F412 class has MCO1 alone, and the header
    /// says so by declaring no MCO2 field.
#if defined(RCC_CFGR_MCO2)
    static bool mco2(uint8_t source_code, uint8_t div) {
        if (source_code > 3u || div == 0u || div > 5u) {
            return false;
        }
        const uint32_t pre = div == 1u ? 0u : (4u + (div - 2u));
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO2_Msk | RCC_CFGR_MCO2PRE_Msk)) |
                    (static_cast<uint32_t>(source_code) << RCC_CFGR_MCO2_Pos) |
                    (pre << RCC_CFGR_MCO2PRE_Pos);
        return true;
    }
#endif
    static constexpr uint8_t mco1_hsi_code = 0;
    static constexpr uint8_t mco1_lse_code = 1;
    static constexpr uint8_t mco1_hse_code = 2;
    static constexpr uint8_t mco1_pll_code = 3;
    static constexpr uint8_t mco2_sysclk_code = 0;
    static constexpr uint8_t mco2_hse_code = 2;
    static constexpr uint8_t mco2_pll_code = 3;

    // ---- peripheral clock enables (7.3.10 .. 7.3.14) and resets (7.3.5 .. 7.3.9) ------
    /// Each enable verb reads its register back: the dummy access ES0206
    /// 2.2.7 asks for between the enable and the first register write.
    static void io_clock(char port, bool on) {
        bit(RCC->AHB1ENR, gpio_port_clock_mask(port), on);
        (void)RCC->AHB1ENR;
    }
    static bool io_clock(char port) {
        const uint32_t m = gpio_port_clock_mask(port);
        return m != 0u && (RCC->AHB1ENR & m) == m;
    }
    static void ahb1_clock(uint32_t mask, bool on) { bit(RCC->AHB1ENR, mask, on); (void)RCC->AHB1ENR; }
    static bool ahb1_clock(uint32_t mask) { return (RCC->AHB1ENR & mask) == mask; }
    // AHB2 and AHB3 exist where the header declares an enable bit on
    // them (the F410 has neither bus; the F401/F411 have AHB2 for the
    // OTG FS alone; AHB3 carries the external memory controller and, on
    // the F446, the QUADSPI): the guards ask the header, as the reserve's
    // rule wants a presence question asked.
#if defined(RCC_AHB2ENR_OTGFSEN) || defined(RCC_AHB2ENR_DCMIEN) || defined(RCC_AHB2ENR_RNGEN)
    static void ahb2_clock(uint32_t mask, bool on) { bit(RCC->AHB2ENR, mask, on); (void)RCC->AHB2ENR; }
    static bool ahb2_clock(uint32_t mask) { return (RCC->AHB2ENR & mask) == mask; }
    static void ahb2_reset(uint32_t mask) { RCC->AHB2RSTR |= mask; RCC->AHB2RSTR &= ~mask; }
#endif
#if defined(RCC_AHB3ENR_FMCEN) || defined(RCC_AHB3ENR_FSMCEN) || defined(RCC_AHB3ENR_QSPIEN)
    static void ahb3_clock(uint32_t mask, bool on) { bit(RCC->AHB3ENR, mask, on); (void)RCC->AHB3ENR; }
    static bool ahb3_clock(uint32_t mask) { return (RCC->AHB3ENR & mask) == mask; }
    static void ahb3_reset(uint32_t mask) { RCC->AHB3RSTR |= mask; RCC->AHB3RSTR &= ~mask; }
#endif
    static void apb1_clock(uint32_t mask, bool on) { bit(RCC->APB1ENR, mask, on); (void)RCC->APB1ENR; }
    static bool apb1_clock(uint32_t mask) { return (RCC->APB1ENR & mask) == mask; }
    static void apb2_clock(uint32_t mask, bool on) { bit(RCC->APB2ENR, mask, on); (void)RCC->APB2ENR; }
    static bool apb2_clock(uint32_t mask) { return (RCC->APB2ENR & mask) == mask; }

    /// Pulse a peripheral's reset line: every register of the block to
    /// its reset value, the bus clock untouched.
    static void ahb1_reset(uint32_t mask) { RCC->AHB1RSTR |= mask; RCC->AHB1RSTR &= ~mask; }
    static void apb1_reset(uint32_t mask) { RCC->APB1RSTR |= mask; RCC->APB1RSTR &= ~mask; }
    static void apb2_reset(uint32_t mask) { RCC->APB2RSTR |= mask; RCC->APB2RSTR &= ~mask; }

private:
    static void bit(volatile uint32_t& r, uint32_t mask, bool on) {
        if (on) {
            r |= mask;
        } else {
            r &= ~mask;
        }
    }
    static bool wait(volatile uint32_t& r, uint32_t mask, bool set) {
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if (((r & mask) != 0u) == set) {
                return true;
            }
        }
        return false;
    }
    static uint8_t divider_of(uint32_t code) {
        return code < 4u ? 1u : static_cast<uint8_t>(2u << (code - 4u));
    }
};

// ---- the clock TASK -------------------------------------------------------------------

/// The root a Clock task runs SYSCLK from.
enum class ClockSource : uint8_t {
    hsi,        ///< HSI undivided: 16 MHz, the reset state
    hse,        ///< the HSE root undivided (crystal or bypass)
    pll_hsi,    ///< the main PLL fed by HSI
    pll_hse,    ///< the main PLL fed by HSE
};

/// How the HSE pins are used: a crystal between OSC_IN and OSC_OUT, or
/// an external clock into OSC_IN (HSEBYP).
enum class HseMode : uint8_t { crystal, bypass };

inline constexpr uint32_t hsi_hz = 16'000'000u;
inline constexpr uint32_t hse_crystal_min_hz = 4'000'000u;
inline constexpr uint32_t hse_crystal_max_hz = 26'000'000u;
inline constexpr uint32_t hse_bypass_min_hz = 1'000'000u;
inline constexpr uint32_t hse_bypass_max_hz = 50'000'000u;

/// THE APB DIVIDER RULE, in one place: the smallest power-of-two division
/// that keeps a bus under its ceiling, and 1 at or below the 16 MHz reset
/// rate (where no ladder is consulted, so a part whose ceilings the
/// reserve does not know still runs). `Clock` folds it into `apb1_div` and
/// `apb2_div`; a driver rebased by the dynamic clock below calls it with
/// the NEW rate, because the RCC still holds the old divider at that
/// moment.
constexpr uint8_t apb_divider_at(uint32_t hclk, bool on_apb2) {
    if (hclk <= hsi_hz) {
        return 1;
    }
    constexpr SysclkLadder l = sysclk_ladder();
    return apb_divider_for(hclk, on_apb2 ? l.apb2_max_hz : l.apb1_max_hz);
}

/// The bus rate that follows from it. Zero where no divider keeps the bus
/// in range, which `Clock`'s own static_assert refuses at compile time.
constexpr uint32_t apb_hz_at(uint32_t hclk, bool on_apb2) {
    const uint8_t div = apb_divider_at(hclk, on_apb2);
    return div == 0u ? 0u : hclk / div;
}

/**
 * Clock<source, hz, hse_hz, hse_mode>: the static main clock.
 *
 *   using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000,
 *                                8'000'000, brio::HseMode::bypass>;
 *   constexpr SysClock clock;
 *   SysClock::init();                 // false: a root did not come up
 *
 * `hz` is SYSCLK = HCLK; `pclk1_hz` and `pclk2_hz` the two APB rates the
 * task derives; `usb_hz` what the PLL's Q output gives (0 without the
 * PLL). All constexpr, so every divisor built from them folds.
 */
template <ClockSource src, uint32_t sys_hz, uint32_t hse_hz = 0, HseMode hse_mode = HseMode::crystal>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t hz = sys_hz;
    static constexpr bool is_static = true;

    static constexpr bool uses_hse = src == ClockSource::hse || src == ClockSource::pll_hse;
    static constexpr bool uses_pll = src == ClockSource::pll_hsi || src == ClockSource::pll_hse;
    static constexpr uint32_t root_hz = uses_hse ? hse_hz : hsi_hz;

    static_assert(!uses_hse || hse_hz != 0u,
                  "brio Clock: an HSE root needs its rate as the third argument");
    static_assert(!uses_hse || hse_mode == HseMode::bypass ||
                      (hse_hz >= hse_crystal_min_hz && hse_hz <= hse_crystal_max_hz),
                  "brio Clock: an HSE crystal is 4..26 MHz on this family (the datasheets)");
    static_assert(!uses_hse || hse_mode == HseMode::crystal ||
                      (hse_hz >= hse_bypass_min_hz && hse_hz <= hse_bypass_max_hz),
                  "brio Clock: a bypassed HSE clock is 1..50 MHz on this family (the datasheets)");
    static_assert(src != ClockSource::hsi || sys_hz == hsi_hz,
                  "brio Clock: the hsi root runs 16 MHz undivided - the AHB prescaler is "
                  "declared and not built");
    static_assert(src != ClockSource::hse || sys_hz == hse_hz,
                  "brio Clock: the hse root runs the HSE rate undivided");

    static constexpr PllConfig pll = uses_pll ? pll_config_for(root_hz, sys_hz, src == ClockSource::pll_hse)
                                              : PllConfig{};
    static_assert(!uses_pll || pll.m != 0u,
                  "brio Clock: no exact PLL ratio for this rate (RM0090 7.3.2: input / M "
                  "in 1..2 MHz, VCO in 100..432 MHz, N in 50..432, P in {2, 4, 6, 8})");

    /// The Q output (USB OTG FS, SDIO, RNG): 48 MHz when the ratio has it.
    static constexpr uint32_t usb_hz = uses_pll ? (root_hz / pll.m) * pll.n / pll.q : 0u;

    /// A rate at or below the 16 MHz reset rate runs as the reset state
    /// does - scale untouched, zero wait states, undivided buses - on any
    /// root and any header; only a rate above it needs the part's ladder.
    static constexpr bool needs_ladder = sys_hz > hsi_hz;

    static constexpr RateRegime regime = regime_for(sys_hz);
    static_assert(!needs_ladder || regime.known,
                  "brio Clock: this rate is above what the reserve's ladder knows for this "
                  "part (stm32f4/device_tables.hpp: the ladders read are the F405, F42x/F43x, "
                  "F446 and F411 classes'), or above the part's ceiling");
    static constexpr uint8_t wait_states = needs_ladder ? flash_wait_states_for(sys_hz) : 0u;
    static_assert(wait_states != 0xFFu, "brio Clock: no wait-state band for this rate");

    static constexpr SysclkLadder ladder = sysclk_ladder();
    static constexpr uint8_t apb1_div = apb_divider_at(sys_hz, false);
    static constexpr uint8_t apb2_div = apb_divider_at(sys_hz, true);
    static_assert(apb1_div != 0u && apb2_div != 0u, "brio Clock: no APB divider keeps the buses in range");

    /// The two APB rates, the ones a peripheral's divisor divides.
    static constexpr uint32_t pclk1_hz = sys_hz / apb1_div;
    static constexpr uint32_t pclk2_hz = sys_hz / apb2_div;

    static constexpr SysclkSource sysclk_source =
        src == ClockSource::hsi ? SysclkSource::hsi
                                : (src == ClockSource::hse ? SysclkSource::hse : SysclkSource::pll);

    /// Bring the tree to the named rate, from the reset state. False, and
    /// the tree left where it stopped, when a root does not come up or a
    /// readback does not agree.
    static bool init() {
        if constexpr (uses_hse) {
            Rcc::hse_enable(true, hse_mode == HseMode::bypass);
            if (!Rcc::hse_wait_ready()) {
                return false;
            }
        }
        if constexpr (src == ClockSource::hsi) {
            // The reset state: HSI on, SYSCLK = HSI, 0 wait states. The
            // accelerator is switched on all the same: it costs nothing
            // and a program that measures a rate deserves the same flash
            // behaviour at every one.
            FlashAccel::enable_all();
            return Rcc::sysclk_status() == SysclkSource::hsi;
        }
        if constexpr (uses_pll) {
            // 1. the regulator scale, PLL off, SYSCLK on HSI (kept at its
            // reset value for a rate the reset state already serves).
            if constexpr (needs_ladder) {
                if (!Pwr::scale(regime.scale)) {
                    return false;
                }
            }
            // 2. the PLL.
            if (!Rcc::pll_configure(pll)) {
                return false;
            }
            Rcc::pll_enable(true);
            if constexpr (regime.over_drive) {
                if (!Pwr::over_drive_enter()) {
                    return false;
                }
            }
        }
        // 3. the flash first, then the buses, then the switch.
        if (!FlashWaitStates::set(wait_states)) {
            return false;
        }
        FlashAccel::enable_all();
        Rcc::bus_prescalers(apb1_div, apb2_div);
        if constexpr (uses_pll) {
            if (!Rcc::pll_wait(true)) {
                return false;
            }
        }
        Rcc::sysclk_select(sysclk_source);
        return Rcc::sysclk_wait(sysclk_source);
    }
};

/// The rate a peripheral on an APB really runs at: pclk2 for the APB2
/// instances, pclk1 for the rest. Folds for a static clock.
template <typename C>
constexpr uint32_t apb_hz(C, bool on_apb2) {
    if constexpr (C::is_static) {
        return on_apb2 ? C::pclk2_hz : C::pclk1_hz;
    } else {
        return on_apb2 ? C::pclk2_hz() : C::pclk1_hz();
    }
}

// ---- the dynamic clock ---------------------------------------------------------------

/// The pack of rates a DynamicClock may run at: static Clock tasks, the
/// FIRST the boot rate. A type list and nothing else - the discrete set on
/// this family is the program's own choice of tuples (below).
template <typename... Rs>
struct Rates {
    static constexpr uint8_t count = sizeof...(Rs);
};

/**
 * The runtime regime: `Rates<R0, R1, ...>` names the rates - each a static
 * `Clock<source, hz, hse_hz, hse_mode>`, R0 the boot rate - and Users the
 * drivers a switch fans the new rate out to (each a ClockUser: `static void
 * rebase(uint32_t hz)`, checked by the concept where the list is written)
 * IN LIST ORDER, synchronously, BEFORE anything moves - so a user can drain
 * what it has in flight at the OLD rate and program itself for the new one.
 *
 *   using Top  = brio::Clock<brio::ClockSource::pll_hse, 180'000'000,
 *                            8'000'000, brio::HseMode::bypass>;
 *   using Mid  = brio::Clock<brio::ClockSource::pll_hse,  84'000'000,
 *                            8'000'000, brio::HseMode::bypass>;
 *   using Boot = brio::Clock<brio::ClockSource::hsi, 16'000'000>;
 *   using SysClock = brio::DynamicClock<brio::Rates<Top, Mid, Boot>,
 *                                       brio::Ticker, Serial>;
 *   constexpr SysClock clock;
 *   SysClock::init();                  // Top's init: 180 MHz in over-drive
 *   SysClock::set<16'000'000>();       // the users rebased, then the HSI
 *
 * A RATE IS A TUPLE HERE TOO, and on this family it has five members, not
 * the STM32G0's three: the SYSCLK root (with the PLL ratio the search
 * folded), the REGULATOR SCALE and whether the rate needs OVER-DRIVE, the
 * FLASH LATENCY, and THE TWO APB PRESCALERS that keep PCLK1 under 45 MHz
 * and PCLK2 under 90. Every member is a constant of the `Clock` task, so
 * naming the rate names the tuple and nothing has to be repeated.
 *
 * EVERY SWITCH PASSES THROUGH 16 MHz ON THE HSI, and that is not a detour
 * but this family's arithmetic. PLLCFGR "can be written only when the PLL
 * is disabled" (7.3.2), the regulator scale "can be modified only when the
 * PLL is OFF and the HSI or HSE clock source is selected as system clock"
 * (5.1.4), and ODEN and ODSWEN are writable only "when the HSI or HSE is
 * selected as system clock". So a switch between two PLL rates has to park
 * on a root, and the park is what collapses the manuals' two orderings
 * (3.5.1: the flash latency FIRST on the way up, LAST on the way down;
 * 5.1.4: the scale up before the rate and down after it) into ONE order -
 * after the park the machine is at the family's lowest rate, so everything
 * that follows is a RISE and the rising order is the only one needed. The
 * park costs the HSI's own settling and the two switch waits; the suite
 * prints what it costs.
 *
 * The discrete-rate surface (docs/design/clock.md) is the pack's:
 * rate_count, rate_hz(i), rate_index() - what cortexm/delay.hpp dispatches
 * on so that no division runs at wait time. Two rates may share an hz (the
 * same 84 MHz from the HSI's PLL and from the HSE's are different tuples):
 * set<hz>() and set(hz) take the FIRST that matches, set_index<i>() /
 * set_index(i) name one exactly.
 *
 * Call set() only when nothing that depends on the rate is mid-transfer (a
 * bus transaction in flight is the caller's problem - in an AO system, ask
 * the bus AOs first). Main context only.
 *
 * WHAT A STOP DOES TO THIS: the part comes out of Stop on the HSI with the
 * PLL off, the HSE off, the regulator back at scale 3 and over-drive
 * disabled (pwr.hpp fact 2), while the flash latency, the prescalers and
 * every peripheral register survive. restore() re-runs the CURRENT rate's
 * entry with no fan-out - the users were configured for that rate and it is
 * that rate that comes back - and is what stm32f4/sleep.hpp's site calls
 * first thing after a wake; a rate already in force (SWS says so, which is
 * every HSI rate) costs one register read.
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

    /// SYSCLK = HCLK now, and the two APB rates beside it - the ones a
    /// peripheral's divisor really divides (`apb_hz(clock, on_apb2)`).
    static uint32_t hz() { return hz_; }
    static uint32_t pclk1_hz() { return rate_pclk1_hz_[idx_]; }
    static uint32_t pclk2_hz() { return rate_pclk2_hz_[idx_]; }
    /// The PLL's Q output at the rate in force; 0 on a rate without a PLL.
    static uint32_t usb_hz() { return rate_usb_hz_[idx_]; }

    /// The discrete-rate surface: the pack, by position.
    static constexpr uint32_t rate_hz(uint8_t i) { return rate_hz_[i]; }
    static uint8_t rate_index() { return idx_; }
    static constexpr uint32_t rate_pclk1_hz(uint8_t i) { return rate_pclk1_hz_[i]; }
    static constexpr uint32_t rate_pclk2_hz(uint8_t i) { return rate_pclk2_hz_[i]; }
    static constexpr SysclkSource rate_source(uint8_t i) { return rate_source_[i]; }
    static constexpr RateRegime rate_regime(uint8_t i) { return rate_regime_[i]; }
    static constexpr uint8_t rate_wait_states(uint8_t i) { return rate_ws_[i]; }

    /// The regulator tuple in force now, by the mirror (the silicon's own
    /// readbacks are Pwr::scale() and Pwr::over_drive_active()).
    static RateRegime regime() { return rate_regime_[idx_]; }

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

    /// Put the CURRENT rate back after a Stop dropped SYSCLK to the HSI -
    /// no fan-out (the users hold the divisors for exactly this rate).
    /// True at once when SWS already reports the rate's root, and true with
    /// nothing done while a set() is in progress in thread mode (this verb
    /// is legal from an ISR - the wake's own, if a program wants the rate
    /// back before the first AO runs - and a switch parks SYSCLK on the HSI
    /// for a moment on its way to the PLL).
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
    static constexpr uint32_t rate_pclk1_hz_[rate_count] = {Rs::pclk1_hz...};
    static constexpr uint32_t rate_pclk2_hz_[rate_count] = {Rs::pclk2_hz...};
    static constexpr uint32_t rate_usb_hz_[rate_count] = {Rs::usb_hz...};
    static constexpr SysclkSource rate_source_[rate_count] = {Rs::sysclk_source...};
    static constexpr RateRegime rate_regime_[rate_count] = {Rs::regime...};
    static constexpr uint8_t rate_ws_[rate_count] = {Rs::wait_states...};

    /**
     * The switch, for one rate of the pack, in the order the file header
     * states. The mirror is written BEFORE the rate's init() so that a user
     * rebased for the new rate and a delay_us dispatching on the index
     * agree from the first instruction after the switch; a failed init()
     * returns false and the mirror then names what was ASKED, which is
     * Clock::init's own contract.
     */
    template <typename R>
    static bool enter_rate(uint8_t i, bool fan_out) {
        switching_ = true;
        // The fan-out is FIRST and at the old rate: a user drains what it
        // has in flight on the clock that is still running, and programs
        // its divisor from the rate it is handed (apb_hz_at() derives the
        // new bus rate arithmetically - the RCC still holds the old
        // prescalers here, and reading them would be reading the past).
        if (fan_out) {
            (Users::rebase(R::hz), ...);
        }
        hz_ = R::hz;
        idx_ = i;

        // THE PARK: the HSI as SYSCLK, the PLL off, over-drive left. Every
        // store that follows is refused by the silicon without it (the file
        // header), and after it the machine is at 16 MHz - so the rest is a
        // rise whichever direction the caller asked for.
        Rcc::hsi_enable(true);
        bool ok = Rcc::hsi_wait_ready();
        Rcc::sysclk_select(SysclkSource::hsi);
        ok = Rcc::sysclk_wait(SysclkSource::hsi) && ok;
        Rcc::pll_enable(false);
        ok = Rcc::pll_wait(false) && ok;
        ok = Pwr::over_drive_exit() && ok;

        ok = R::init() && ok;

        if constexpr (R::source == ClockSource::hsi) {
            // The HSI rate's own init() is a readback and the accelerator:
            // it has no reason to touch the flash or the buses, because a
            // STATIC Clock at 16 MHz starts from a machine that is already
            // there. Coming DOWN to it, both are still set for the rate
            // left behind - and the prescalers matter, because the users
            // were just rebased on the arithmetic below.
            ok = FlashWaitStates::set(R::wait_states) && ok;
            Rcc::bus_prescalers(R::apb1_div, R::apb2_div);
        }
        switching_ = false;
        return ok;
    }

    /// Dispatch by position: a fold over the pack that stops at the i-th
    /// member (no table, no RAM).
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
