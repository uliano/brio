/*
 * clock.hpp
 *
 * The clock tree of the CH32V203 (RM ch. 3, with the pieces of ch. 33
 * that belong to it): the roots, the PLL, the switch, the four
 * prescalers, the clock output, the security system and the peripheral
 * gates - and the two tasks that take the tree to a rate, one stated at
 * compile time and one chosen from a pack at run time.
 *
 * WHAT MAKES THIS TREE ITS OWN. Four things, none of them the F1's
 * whose register names it borrows:
 *
 *  - THE PLL'S INPUT DIVIDER FOR THE HSI IS IN ANOTHER BLOCK. RCC_CFGR0
 *    says only WHICH root feeds the PLL; whether the HSI arrives whole
 *    or halved is EXTEN_CTR.HSIPRE (RM 33.2.1), a register the RCC
 *    chapter never mentions. Out of reset it is 0 - the PLL sees 4 MHz,
 *    not 8 - so a program that only ever wrote RCC registers would find
 *    every rate half of what it asked for. This file owns that bit.
 *  - PB1 RUNS AT HALF OF HCLK ABOVE 72 MHz, AND THAT IS A MEASUREMENT,
 *    NOT A HABIT. The datasheet's block diagram rates both peripheral
 *    buses at the core's own ceiling of 144 MHz, and this file believed
 *    it - until the USB device controller, which lives on PB1, answered
 *    every packet of an enumeration with a packet-memory overflow and
 *    stored none of them (sixteen PMAOVR in one attempt, the data never
 *    reaching the buffer). WCH's own clock code sets PPRE1 = /2 at
 *    EVERY rate it offers - 48, 56, 72, 96, 120 and 144 MHz, from the
 *    HSI or a crystal - and only the bare-HSE path leaves it undivided;
 *    the datasheet's own consumption tables are measured with
 *    PCLK1 = HCLK/2. So PB1 is capped at `pb1_max_hz` here and
 *    `pclk1_hz` says so; where the real ceiling of that bus lies,
 *    between that cap and the datasheet's 144, is in the document's gap
 *    list with what would answer it. PB2 keeps the whole rate, as it
 *    does in the vendor's ladder.
 *  - THERE ARE NO FLASH WAIT STATES TO PROGRAM. The flash chapter's
 *    register table (RM 32.4) has no latency field at all: the array is
 *    divided into a zero-wait region and a non-zero-wait one by the
 *    part, and FLASH_CTLR.SCKMOD chooses whether the flash is accessed
 *    at the system clock or at half of it, defaulting to half. So
 *    nothing here touches the flash controller. What the chapter DOES
 *    ask for is at the other end: above `flash_safe_sysclk_hz` an ERASE
 *    or a PROGRAM wants HCLK halved around it (32.1), which is the NVM
 *    driver's business and is named here only because it is a statement
 *    about a RATE.
 *  - THE DEVICE CLASS DECIDES PART OF THE ARITHMETIC. One manual covers
 *    four families, and two of its clock fields mean different things
 *    per class - PLLXTPRE divides the HSE by 1 or 2 on the CH32V20x_D6
 *    and by 4 or 8 on the D8, and USBPRE's fourth code (the PLL over
 *    five) exists only on the D8 - so both are read from the part's own
 *    table (device::pll_hse_div, device::has_usb_pre_div5) and never
 *    spelled here. The PLL's input and output ranges are part facts for
 *    the same reason.
 *
 * THE ORDER OF init() IS THE SILICON'S, NOT A HABIT: it parks SYSCLK on
 * the HSI before it touches anything else. The PLL's own fields are
 * writable only while it is off, and it will not go off while it is the
 * system clock, so a tree that is already running the PLL swallows every
 * write without a flag - see init().
 *
 * THE USB CLOCK IS PART OF THE TREE. USBPRE divides the PLL by 1, 2 or 3
 * and the device controller needs exactly 48 MHz, so `usb_hz` here is
 * a compile-time answer - 48 MHz when the PLL runs at 48, 96 or 144, and
 * zero otherwise - and init() programs the divider BEFORE any USB gate
 * is opened, which is the order 3.4.2 asks for. A USB driver
 * static_asserts on it rather than discovering at run time that its
 * frames are the wrong length.
 *
 * THE ADC'S PRESCALER IS PART OF THE TREE TOO, and it is the one place
 * where a legal SYSCLK leaves a peripheral out of specification: ADCPRE
 * divides PCLK2 by 2, 4, 6 or 8 and the converter is rated for
 * `adc_max_hz`, so any PCLK2 above eight times that has no code which
 * keeps the ADC in range. init() programs the slowest useful divider and
 * `adc_in_spec` says whether it was enough; the refusal is the ADC
 * driver's to make, because a program that never converts is not wrong
 * to run at the family's ceiling.
 *
 * WHAT A DYNAMIC CLOCK HANDS ITS USERS is HCLK, once, before the switch
 * - never a bus rate. A driver on PB1 derives its own with
 * pclk1_hz_at(hclk), the same arithmetic the static task folds at
 * compile time, because the RCC still holds the OLD prescalers at that
 * moment and reading them would be reading the past.
 */

#pragma once

#include <stdint.h>

#include <concepts>

#include "ch32v203/device.hpp"
#include "ch32v203/pin.hpp"
#include "util/clock.hpp"

namespace brio {

/// Where SYSCLK comes from, as a program names it.
enum class ClockSource : uint8_t {
    internal,   ///< HSI, the 8 MHz internal RC
    pll,        ///< the PLL, fed by the HSI or by a crystal named beside it
    crystal,    ///< HSE with a crystal on OSC_IN/OSC_OUT, its rate the third parameter
    external,   ///< HSE in bypass: a clock into OSC_IN, its rate the third parameter
};

/// Where SYSCLK comes from, as RCC_CFGR0's SW and SWS say it (3.4.2).
enum class SysclkSource : uint8_t { hsi = 0, hse = 1, pll = 2 };

/// What the clock output pad may carry (3.3.5.5). The codes above
/// pll_div2 name PLL2, PLL3 and the Ethernet oscillator, which belong
/// to the D8C classes of other families and are not spelled here; 00xx
/// is no clock at all.
enum class McoSource : uint8_t { none = 0, sysclk = 4, hsi = 5, hse = 6, pll_div2 = 7 };

/// What the USB device controller must be fed, whatever the core runs at.
inline constexpr uint32_t usb_required_hz = 48'000'000UL;
/// What PB1 is allowed to run at here, and why (the file header).
inline constexpr uint32_t pb1_max_hz = 72'000'000UL;
/// The converter's own ceiling (RM 3.4.2's note on ADCPRE, and the
/// datasheet's fADC).
inline constexpr uint32_t adc_max_hz = 14'000'000UL;
/// The rate above which a flash ERASE or PROGRAM asks for HCLK halved
/// around it (RM 32.1). Nothing in this file acts on it - the NVM
/// driver does - but it is a statement about a rate and belongs beside
/// the others.
inline constexpr uint32_t flash_safe_sysclk_hz = 120'000'000UL;

/// RM 3.4.2, HPRE[3:0]: codes 0..7 leave SYSCLK undivided, codes 8..15
/// divide by 2, 4, 8, 16, 64, 128, 256, 512 - the ladder SKIPS 32, which
/// is the one trap in this field.
constexpr uint32_t hpre_divider(uint8_t code) {
    switch (code) {
        case 8:  return 2;
        case 9:  return 4;
        case 10: return 8;
        case 11: return 16;
        case 12: return 64;
        case 13: return 128;
        case 14: return 256;
        case 15: return 512;
        default: return 1;
    }
}

/// The HPRE code that divides `src_hz` to exactly `hz`, or 0xFF when no
/// divider does. Code 0 is the undivided one.
constexpr uint8_t hpre_for(uint32_t src_hz, uint32_t hz) {
    if (hz == 0u) {
        return 0xFF;
    }
    if (src_hz == hz) {
        return 0;
    }
    for (uint8_t code = 8; code < 16u; ++code) {
        const uint32_t div = hpre_divider(code);
        if (src_hz / div == hz && src_hz % div == 0u) {
            return code;
        }
    }
    return 0xFF;
}

/// RM 3.4.2, PPRE1[2:0] and PPRE2[2:0], one encoding for both buses:
/// 0xx leaves HCLK undivided, 100..111 divide by 2, 4, 8, 16.
constexpr uint32_t ppre_divider(uint8_t code) {
    return (code & 0x4u) == 0u ? 1u : (2u << (code & 0x3u));
}

/// The smallest PPRE divider that keeps `hclk` at or under `max_hz`, as
/// a code; the /16 code when even that is not enough, which no rate of
/// this family reaches.
constexpr uint8_t ppre_for(uint32_t hclk, uint32_t max_hz) {
    if (hclk <= max_hz) {
        return 0;
    }
    for (uint8_t code = 4; code < 8u; ++code) {
        if (hclk / ppre_divider(code) <= max_hz) {
            return code;
        }
    }
    return 7;
}

/// RM 3.4.2, ADCPRE[1:0]: PCLK2 divided by 2, 4, 6 or 8.
constexpr uint32_t adcpre_divider(uint8_t code) {
    return 2u + 2u * static_cast<uint32_t>(code & 0x3u);
}

/// The smallest ADC divider that keeps the converter at or under
/// `max_hz`, as a code - and the slowest code when none does, which is
/// what a PCLK2 above eight times the ceiling leaves (the file header).
constexpr uint8_t adcpre_for(uint32_t pclk2, uint32_t max_hz) {
    for (uint8_t code = 0; code < 4u; ++code) {
        if (pclk2 / adcpre_divider(code) <= max_hz) {
            return code;
        }
    }
    return 3;
}

/// RM 3.4.2, USBPRE[1:0]: the PLL divided by 1, 2 or 3 - and by 5 on
/// the classes whose PLL reaches 240 MHz (device::has_usb_pre_div5).
constexpr uint8_t usbpre_code_for(uint8_t div) {
    return div == 1u ? 0u : div == 2u ? 1u : div == 3u ? 2u : div == 5u ? 3u : 0xFF;
}
constexpr uint32_t usbpre_divider(uint8_t code) {
    return code == 0u ? 1u : code == 1u ? 2u : code == 2u ? 3u : 5u;
}
/// Whether a divider is one THIS PART's USB prescaler can make.
constexpr bool usbpre_exists(uint8_t div) {
    return div == 1u || div == 2u || div == 3u || (div == 5u && device::has_usb_pre_div5);
}

/// What the PLL can multiply by, in code order: x2..x16 and then x18.
constexpr bool pll_mul_exists(uint32_t mul) {
    return (mul >= 2u && mul <= 16u) || mul == 18u;
}

/// The two bus rates that follow from an HCLK under this stratum's rule
/// (the file header). A driver rebased by a dynamic clock derives its
/// own bus rate with these, because it is handed HCLK and the RCC still
/// holds the prescalers of the rate being left.
constexpr uint32_t pclk1_hz_at(uint32_t hclk) {
    return hclk / ppre_divider(ppre_for(hclk, pb1_max_hz));
}
constexpr uint32_t pclk2_hz_at(uint32_t hclk) { return hclk; }

/// What a TIMER on that bus counts: the peripheral clock when its
/// prescaler is 1, twice it otherwise (RM 3.3.1, figure 3-3). The
/// timers are not this file's chapter; the rule is, because it is a
/// property of the tree.
constexpr uint32_t timclk_hz_at(uint32_t pclk, uint8_t ppre_code) {
    return ppre_divider(ppre_code) == 1u ? pclk : pclk * 2u;
}

/// How long a root or a switch may take before a verb gives up. Bounded
/// spins, not while(1): a boot that cannot reach its rate must return
/// and let the program say so.
inline constexpr uint32_t clock_timeout_turns = 100'000UL;

/// Which bus a peripheral's enable and reset bits sit on (RM 3.4.5..3.4.8).
enum class Bus : uint8_t { hb, pb2, pb1 };

/**
 * The RCC block, monostate: the roots, the switch, the prescalers, the
 * output, the security system, the ready interrupts and the peripheral
 * gates. Verbs read and write the registers as they stand; the policy -
 * which rate, which root - is the task's below.
 *
 * What is NOT here: RCC_BDCTLR, the backup domain's own register (the
 * LSE, the RTC's clock select and RTCEN), because it is write-protected
 * by PWR_CTLR.DBP and belongs with the RTC; and RCC_RSTSCKR's reset
 * flags, which are reset.hpp's - this file owns only the two LSI bits
 * that share that register.
 */
struct Rcc {
    Rcc() = delete;

    // ---- the peripheral gates --------------------------------------------
    static void enable(Bus bus, uint32_t mask) {
        switch (bus) {
            case Bus::hb:  rcc()->HBPCENR |= mask; break;
            case Bus::pb2: rcc()->PB2PCENR |= mask; break;
            case Bus::pb1: rcc()->PB1PCENR |= mask; break;
        }
    }

    static void disable(Bus bus, uint32_t mask) {
        switch (bus) {
            case Bus::hb:  rcc()->HBPCENR &= ~mask; break;
            case Bus::pb2: rcc()->PB2PCENR &= ~mask; break;
            case Bus::pb1: rcc()->PB1PCENR &= ~mask; break;
        }
    }

    static bool enabled(Bus bus, uint32_t mask) {
        switch (bus) {
            case Bus::hb:  return (rcc()->HBPCENR & mask) == mask;
            case Bus::pb2: return (rcc()->PB2PCENR & mask) == mask;
            case Bus::pb1: return (rcc()->PB1PCENR & mask) == mask;
        }
        return false;
    }

    /// Pulse a peripheral's reset line: held, then released, which is
    /// how a driver puts its block back to its reset state without
    /// touching anyone else's.
    static void reset(Bus bus, uint32_t mask) {
        switch (bus) {
            case Bus::hb:  rcc()->HBRSTR |= mask;  rcc()->HBRSTR &= ~mask;  break;
            case Bus::pb2: rcc()->PB2PRSTR |= mask; rcc()->PB2PRSTR &= ~mask; break;
            case Bus::pb1: rcc()->PB1PRSTR |= mask; rcc()->PB1PRSTR &= ~mask; break;
        }
    }

    // ---- the high-speed roots ---------------------------------------------
    static bool hsi_ready() { return (rcc()->CTLR & rcc_hsirdy) != 0u; }

    /// Start or stop the HSI and RETURN AT ONCE. The pair enable() +
    /// ready() is what a program times a ramp with; hsi_start() below is
    /// the bounded wait most callers want.
    static void hsi_enable(bool on) {
        if (on) {
            rcc()->CTLR |= rcc_hsion;
        } else {
            rcc()->CTLR &= ~rcc_hsion;
        }
    }

    /// Start the 8 MHz HSI and wait a bounded time for it. It is on out
    /// of reset, so this is usually a readback - but a program that
    /// stopped it, or a warm start, must be able to get it back: it is
    /// the root everything else is configured from (see Clock::init).
    static bool hsi_start() {
        hsi_enable(true);
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->CTLR & rcc_hsirdy) != 0u) {
                return true;
            }
        }
        return false;
    }

    /// Stop the HSI. The chapter recommends against it (3.3.2) and the
    /// hardware starts it again by itself on a wake from Stop or
    /// Standby and on a clock-security failure, so this is a verb for a
    /// program that runs off the HSE and wants the RC quiet - never a
    /// step in reaching a rate.
    static void hsi_stop() { hsi_enable(false); }

    /// The HSI's user trim, five bits around a centre of 16, ADDED to
    /// the factory calibration (3.4.1): about 20 kHz a step.
    static void hsi_trim(uint8_t trim) {
        rcc()->CTLR = (rcc()->CTLR & ~rcc_hsitrim_mask) |
                      ((static_cast<uint32_t>(trim) & 0x1Fu) << 3);
    }
    static uint8_t hsi_trim() {
        return static_cast<uint8_t>((rcc()->CTLR & rcc_hsitrim_mask) >> 3);
    }
    static uint8_t hsi_calibration() {
        return static_cast<uint8_t>((rcc()->CTLR & rcc_hsical_mask) >> 8);
    }

    static bool hse_ready() { return (rcc()->CTLR & rcc_hserdy) != 0u; }

    /// Start the HSE, as a crystal or as an external clock, and RETURN
    /// AT ONCE - the verb a program times the oscillator's ramp with.
    /// HSEBYP may only be written with HSEON clear, which is why the
    /// order here is not negotiable.
    static void hse_enable(bool bypass) {
        rcc()->CTLR &= ~rcc_hseon;
        if (bypass) {
            rcc()->CTLR |= rcc_hsebyp;
        } else {
            rcc()->CTLR &= ~rcc_hsebyp;
        }
        rcc()->CTLR |= rcc_hseon;
    }

    /// The same, waiting a bounded time for HSERDY.
    static bool hse_start(bool bypass) {
        hse_enable(bypass);
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->CTLR & rcc_hserdy) != 0u) {
                return true;
            }
        }
        return false;
    }

    static void hse_stop() { rcc()->CTLR &= ~(rcc_hseon | rcc_hsebyp); }

    /// Whether the HSE keeps oscillating in a low-power mode
    /// (EXTEN_CTR.HSEPLP, RM 33.2.1). The bit belongs to the
    /// CH32V20x_D8 class; on every other part of this family the verb
    /// writes nothing and the getter answers false, because a register
    /// with other live bits in it must not be stirred for a bit that is
    /// not there.
    static void hse_in_low_power(bool keep) {
        if constexpr (device::is_d8_class) {
            exten_update(keep ? exten_hseplp : 0u, exten_hseplp);
        } else {
            (void)keep;
        }
    }
    static bool hse_in_low_power() {
        if constexpr (device::is_d8_class) {
            return (exten()->CTR & exten_hseplp) != 0u;
        } else {
            return false;
        }
    }

    // ---- the clock security system ----------------------------------------
    /// With the HSE as a root, its failure switches SYSCLK back to the
    /// HSI, stops the HSE and the PLL, brakes the advanced-control
    /// timer and raises the NON-MASKABLE interrupt (3.3.6). The
    /// detector is armed by hardware when HSERDY rises and disarmed
    /// when the HSE stops, so this bit is only meaningful with the
    /// oscillator running.
    static void clock_monitor(bool on) {
        if (on) {
            rcc()->CTLR |= rcc_csson;
        } else {
            rcc()->CTLR &= ~rcc_csson;
        }
    }
    static bool clock_monitor() { return (rcc()->CTLR & rcc_csson) != 0u; }
    /// Whether the detector has fired since the flag was last cleared.
    static bool clock_failed() { return (rcc()->INTR & rcc_cssf) != 0u; }

    /**
     * The non-maskable interrupt's body: the vector an app binds for
     * the clock security system (index 2 of this table). True when the
     * CSS was the reason - the flag stood and has been cleared - and
     * false when something else took the vector, which is the caller's
     * to handle.
     *
     * By the time this runs the silicon has ALREADY changed the tree:
     * SYSCLK is the HSI, the HSE and the PLL are off, and every divisor
     * in the program was computed for a rate that is gone. The body
     * clears the flag and says so; what to do about the rate - run
     * degraded, restore(), reset - is the program's decision and not a
     * driver's.
     */
    [[gnu::always_inline]] static bool css_isr() {
        if ((rcc()->INTR & rcc_cssf) == 0u) {
            return false;
        }
        clear_interrupt_flags(rcc_cssf);
        return true;
    }

    // ---- the ready interrupts (RCC_INTR, 3.4.3) ---------------------------
    /// The flags that stand (rcc_lsirdyf ... rcc_pllrdyf, rcc_cssf).
    static uint32_t interrupt_flags() { return rcc()->INTR & rcc_intr_flags; }

    /// Clear the flags in `mask`. The flags are read-only and their
    /// clear bits write-only sixteen places above them, so this rewrites
    /// the enables as they stand and disarms nothing.
    static void clear_interrupt_flags(uint32_t mask) {
        rcc()->INTR = (rcc()->INTR & rcc_intr_enables) | ((mask & rcc_intr_flags) << 16);
    }

    /// Arm exactly the ready interrupts in `mask` (rcc_lsirdyie ...
    /// rcc_pllrdyie) and disarm the rest. The clock security system is
    /// NOT one of them: it has no enable bit and no line of this
    /// controller - it is the non-maskable interrupt.
    static void ready_interrupts(uint32_t mask) {
        rcc()->INTR = mask & rcc_intr_enables;
    }
    static uint32_t ready_interrupts() { return rcc()->INTR & rcc_intr_enables; }

    /// The RCC vector's body: the ready flags that stood, cleared and
    /// handed back for the program to read. The security system's flag
    /// is left alone here - it arrives on the non-maskable vector.
    [[gnu::always_inline]] static uint32_t ready_isr() {
        const uint32_t f = rcc()->INTR & (rcc_intr_flags & ~rcc_cssf);
        clear_interrupt_flags(f);
        return f;
    }

    // ---- the low-speed root -----------------------------------------------
    /// The LSI: the independent watchdog's root, one of the RTC's, and
    /// the auto-wake unit's. Its two bits live with the reset flags in
    /// RCC_RSTSCKR (3.4.10) - a read-modify-write is safe there because
    /// every reset flag is read-only and RMVF reads back zero. Its RATE
    /// is nominal: device::lsi_min_hz .. device::lsi_max_hz.
    static void lsi_enable(bool on) {
        if (on) {
            rcc()->RSTSCKR |= rcc_lsion;
        } else {
            rcc()->RSTSCKR &= ~rcc_lsion;
        }
    }
    static bool lsi_start() {
        lsi_enable(true);
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->RSTSCKR & rcc_lsirdy) != 0u) {
                return true;
            }
        }
        return false;
    }
    static void lsi_stop() { lsi_enable(false); }
    static bool lsi_ready() { return (rcc()->RSTSCKR & rcc_lsirdy) != 0u; }

    // ---- the PLL ----------------------------------------------------------
    /// Configure and start the PLL: `from_hse` picks the root, `divided`
    /// asks for the root's rate divided on the way in (EXTEN's HSIPRE
    /// for the HSI, RCC's PLLXTPRE for the HSE - two registers, one
    /// meaning, which is why this verb takes one flag; WHAT the divider
    /// is on the HSE path is the part's, device::pll_hse_div), `mul` is
    /// the multiplier. The PLL must be OFF for any of it to stick, and
    /// it will not go off while it is the system clock.
    static bool pll_start(bool from_hse, bool divided, uint32_t mul) {
        rcc()->CTLR &= ~rcc_pllon;

        if (from_hse) {
            uint32_t cfgr = rcc()->CFGR0 | rcc_pllsrc;
            if (divided) {
                cfgr |= rcc_pllxtpre;
            } else {
                cfgr &= ~rcc_pllxtpre;
            }
            rcc()->CFGR0 = cfgr;
        } else {
            rcc()->CFGR0 &= ~rcc_pllsrc;
            // The HSI's own divider lives in EXTEN, not in RCC (see the
            // file header), and its sense is the opposite of PLLXTPRE's:
            // the bit SET is the whole clock.
            exten_update(divided ? 0u : exten_hsipre, exten_hsipre);
        }

        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_pllmul_mask) |
                       (rcc_pllmul_code(mul) << rcc_pllmul_shift);

        rcc()->CTLR |= rcc_pllon;
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->CTLR & rcc_pllrdy) != 0u) {
                return true;
            }
        }
        return false;
    }

    static void pll_stop() { rcc()->CTLR &= ~rcc_pllon; }
    static bool pll_ready() { return (rcc()->CTLR & rcc_pllrdy) != 0u; }
    /// What the PLL is configured from and by, as the registers stand.
    static bool pll_from_hse() { return (rcc()->CFGR0 & rcc_pllsrc) != 0u; }
    static bool pll_input_divided() {
        return pll_from_hse() ? (rcc()->CFGR0 & rcc_pllxtpre) != 0u
                              : (exten()->CTR & exten_hsipre) == 0u;
    }
    static uint32_t pll_multiplier() {
        return rcc_pllmul_of((rcc()->CFGR0 & rcc_pllmul_mask) >> rcc_pllmul_shift);
    }

    // ---- the switch and the prescalers ------------------------------------
    /// Point SYSCLK at a root and wait until the hardware confirms it in
    /// SWS. The two fields are the whole switch: there is no order to
    /// respect beyond the root being ready.
    static bool sysclk_select(SysclkSource src) {
        const uint32_t sw = static_cast<uint32_t>(src);
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_sw_mask) | sw;
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if (((rcc()->CFGR0 & rcc_sws_mask) >> 2) == sw) {
                return true;
            }
        }
        return false;
    }

    /// Which root SYSCLK actually runs on (SWS, set by hardware).
    static SysclkSource sysclk_status() {
        return static_cast<SysclkSource>((rcc()->CFGR0 & rcc_sws_mask) >> 2);
    }

    static void prescalers(uint8_t hpre_code, uint8_t ppre1_code, uint8_t ppre2_code) {
        uint32_t cfgr = rcc()->CFGR0;
        cfgr &= ~(rcc_hpre_mask | rcc_ppre1_mask | rcc_ppre2_mask);
        cfgr |= static_cast<uint32_t>(hpre_code) << 4;
        cfgr |= static_cast<uint32_t>(ppre1_code) << rcc_ppre1_shift;
        cfgr |= static_cast<uint32_t>(ppre2_code) << rcc_ppre2_shift;
        rcc()->CFGR0 = cfgr;
    }

    static uint8_t hpre_code() {
        return static_cast<uint8_t>((rcc()->CFGR0 & rcc_hpre_mask) >> 4);
    }
    static uint8_t ppre1_code() {
        return static_cast<uint8_t>((rcc()->CFGR0 & rcc_ppre1_mask) >> rcc_ppre1_shift);
    }
    static uint8_t ppre2_code() {
        return static_cast<uint8_t>((rcc()->CFGR0 & rcc_ppre2_mask) >> rcc_ppre2_shift);
    }

    /// The ADC's own divider off PCLK2 (2, 4, 6 or 8 by code), and the
    /// duty-cycle bit beside it: with ADCDUTY set the converter's clock
    /// spends longer low, which is a knob for a slow sample and not a
    /// rate. The second duty bit of that register belongs to other
    /// classes and is not spelled here.
    static void adc_prescaler(uint8_t code) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_adcpre_mask) |
                       ((static_cast<uint32_t>(code) & 0x3u) << rcc_adcpre_shift);
    }
    static uint8_t adc_prescaler() {
        return static_cast<uint8_t>((rcc()->CFGR0 & rcc_adcpre_mask) >> rcc_adcpre_shift);
    }
    static void adc_duty_extended(bool on) {
        if (on) {
            rcc()->CFGR0 |= rcc_adcduty;
        } else {
            rcc()->CFGR0 &= ~rcc_adcduty;
        }
    }
    static bool adc_duty_extended() { return (rcc()->CFGR0 & rcc_adcduty) != 0u; }

    /// The Ethernet transceiver's own divider off HCLK, on the one part
    /// of this family that has a MAC (device::has_ethernet). The block
    /// itself is not this stratum's; the divider is, because it sits in
    /// this register.
    static void eth_prescaler(bool halved) {
        if constexpr (device::has_ethernet) {
            if (halved) {
                rcc()->CFGR0 |= rcc_ethpre;
            } else {
                rcc()->CFGR0 &= ~rcc_ethpre;
            }
        } else {
            (void)halved;
        }
    }

    /// The PLL's divider on the way to the USB blocks. 3.4.2 requires it
    /// to be written before the USB clock gates are opened. False, and
    /// nothing written, for a divider this part's field cannot make.
    static bool usb_prescaler(uint8_t div) {
        if (!usbpre_exists(div)) {
            return false;
        }
        const uint32_t code = usbpre_code_for(div);
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_usbpre_mask) | (code << rcc_usbpre_shift);
        return true;
    }
    static uint8_t usb_prescaler() {
        return static_cast<uint8_t>((rcc()->CFGR0 & rcc_usbpre_mask) >> rcc_usbpre_shift);
    }

    /// The clock output's SOURCE. The pad is the caller's (Mco below
    /// claims it): this verb only says what the multiplexer carries.
    static void mco(McoSource src) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_mco_mask) |
                       (static_cast<uint32_t>(src) << 24);
    }
    static McoSource mco() {
        const uint32_t code = (rcc()->CFGR0 & rcc_mco_mask) >> 24;
        // 00xx is no clock: four codes for one meaning.
        return code < 4u ? McoSource::none : static_cast<McoSource>(code);
    }

private:
    /// A read-modify-write of EXTEN_CTR that cannot clear a lock-up flag
    /// the program has not read: LKUPRST is write-one-to-clear, so it is
    /// masked out of every store this file makes.
    static void exten_update(uint32_t set, uint32_t mask) {
        const uint32_t ctr = exten()->CTR & ~(exten_lkuprst | mask);
        exten()->CTR = ctr | (set & mask);
    }
};

/**
 * The clock output on PA8 as a task: the pad claimed and the
 * multiplexer set in one verb, so a program cannot half-do it.
 *
 *   brio::Mco::init(brio::McoSource::hsi);      // the RC on the pad
 *   brio::Mco::off();                           // the pad released
 *
 * THE PAD IS THE LIMIT, not the multiplexer: PA8 is an ordinary GPIO in
 * alternate push-pull, so what comes out of it at the family's ceiling
 * is a measurement of the pad and not of the tree (3.3.5.5 also warns
 * that the output may truncate cycles at a source switch). The verb
 * takes the pad's speed class for that reason and defaults to the
 * fastest.
 *
 * TWO PACKAGES OF THIS SERIES DO NOT BRING PA8 OUT (the CH32V203F6 and
 * the CH32V203G6), so `has_pad` is a part fact and init() ANSWERS FALSE
 * there rather than refusing to compile: the multiplexer still exists
 * on those parts and `Rcc::mco()` still writes it - what is missing is
 * the way out of the package.
 */
struct Mco {
    Mco() = delete;

    /// Whether this package brings the output pad out at all.
    static constexpr bool has_pad = (device::port_pins('A') & (1u << 8)) != 0u;

    /// Claim the pad and put `src` on it. False, and nothing done, on a
    /// package with no such pad.
    ///
    /// The pad is named through its PORT and not as Pin<'A', 8>, for
    /// usb.hpp's reason: a Pin naming a pad is formed where it is
    /// WRITTEN, and a header of this stratum must compile on every part
    /// - including the two whose package has not got this one.
    static bool init(McoSource src, PinSpeed speed = PinSpeed::fast) {
        if (!has_pad) {
            return false;
        }
        Port<'A'>::configure(8, pin_nibble(PinMode::alternate, PinDrive::push_pull, speed));
        Rcc::mco(src);
        return true;
    }

    /// Stop driving: the multiplexer to no clock, the pad back to a
    /// floating input.
    static void off() {
        Rcc::mco(McoSource::none);
        if (has_pad) {
            Port<'A'>::configure(8, pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast));
        }
    }

    static McoSource source() { return Rcc::mco(); }
};

/**
 * The clock as a TASK: a rate stated at compile time, reached once at
 * boot and true for the life of the program.
 *
 * `Clock<ClockSource::pll, 144'000'000>` is the HSI whole into the PLL
 * times eighteen; `Clock<ClockSource::internal, 8'000'000>` is the reset
 * clock named out loud; `Clock<ClockSource::crystal, 24'000'000,
 * 24'000'000>` is a crystal straight through. The ratio search is
 * constexpr and a rate this tree cannot make exactly is a COMPILE error,
 * never a rounded one - the ONE rate truth every driver reads
 * (`clock_hz(clock)`), so there is no F_CPU here either.
 */
template <ClockSource src, uint32_t target_hz, uint32_t xtal_hz = 0>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t hz = target_hz;        ///< HCLK
    static constexpr bool is_static = true;

    /// The HSE's part in this clock: SYSCLK itself, or the PLL's source
    /// when a crystal rate is named beside ClockSource::pll.
    static constexpr bool uses_hse = src == ClockSource::crystal || src == ClockSource::external ||
                                     (src == ClockSource::pll && xtal_hz != 0u);
    static constexpr bool hse_bypass = src == ClockSource::external;
    static constexpr uint32_t crystal_hz = xtal_hz;
    /// The root before the PLL: the HSI, or the HSE at the rate named.
    static constexpr uint32_t root_hz = uses_hse ? xtal_hz : hsi_hz;

    /// THE PLL SEARCH. The root reaches the PLL through a divider the
    /// PART decides - 1 or 2 from the HSI, device::pll_hse_div from the
    /// HSE, where the CH32V20x_D8's 32 MHz oscillator arrives divided by
    /// four or by eight and never whole - and the multiplier is x2..x16
    /// or x18. The FIRST divider is preferred where both reach the
    /// target, which keeps the multiplier small and is what the 8 MHz
    /// HSI needs to reach this family's ceiling (8 x 18).
    static constexpr uint32_t pll_div_undivided = uses_hse ? device::pll_hse_div[0] : 1u;
    static constexpr uint32_t pll_div_divided   = uses_hse ? device::pll_hse_div[1] : 2u;

    static constexpr bool pll_input_divided = [] -> bool {
        if (root_hz % pll_div_undivided != 0u) {
            return true;
        }
        for (uint32_t mul = 2; mul <= 18u; ++mul) {
            if (pll_mul_exists(mul) && (root_hz / pll_div_undivided) * mul == target_hz) {
                return false;
            }
        }
        return true;
    }();
    static constexpr uint32_t pll_in_div = pll_input_divided ? pll_div_divided : pll_div_undivided;
    static constexpr uint32_t pll_in_hz = root_hz / pll_in_div;
    static constexpr uint32_t pll_mul = [] -> uint32_t {
        for (uint32_t mul = 2; mul <= 18u; ++mul) {
            if (pll_mul_exists(mul) && pll_in_hz * mul == target_hz) {
                return mul;
            }
        }
        return 0u;
    }();

    /// SYSCLK before HPRE.
    static constexpr uint32_t sysclk_hz = (src == ClockSource::pll) ? pll_in_hz * pll_mul : root_hz;
    static constexpr uint8_t hpre_code = hpre_for(sysclk_hz, target_hz);
    static constexpr SysclkSource sysclk_source = src == ClockSource::pll ? SysclkSource::pll
                                                  : uses_hse             ? SysclkSource::hse
                                                                         : SysclkSource::hsi;

    /// PB1 is capped and PB2 is not (the file header). The two are
    /// separate names because a driver must ask for ITS bus: a USART on
    /// PB1 and one on PB2 do not divide the same number.
    static constexpr uint8_t ppre1_code = ppre_for(target_hz, pb1_max_hz);
    static constexpr uint8_t ppre2_code = 0x0;   ///< /1
    static constexpr uint32_t pclk1_hz = target_hz / ppre_divider(ppre1_code);
    static constexpr uint32_t pclk2_hz = target_hz;
    /// What a timer on each bus counts - twice the bus rate wherever the
    /// bus is divided.
    static constexpr uint32_t timclk1_hz = timclk_hz_at(pclk1_hz, ppre1_code);
    static constexpr uint32_t timclk2_hz = timclk_hz_at(pclk2_hz, ppre2_code);

    /// The USB blocks want 48 MHz and take the PLL divided by 1, 2 or 3:
    /// this is what they would get, and zero when this tree cannot feed
    /// them at all.
    static constexpr uint8_t usb_divider = [] -> uint8_t {
        if (src != ClockSource::pll) {
            return 0;
        }
        for (uint8_t div = 1; div <= 5u; ++div) {
            if (usbpre_exists(div) && sysclk_hz % div == 0u &&
                sysclk_hz / div == usb_required_hz) {
                return div;
            }
        }
        return 0;
    }();
    static constexpr uint32_t usb_hz = usb_divider != 0u ? usb_required_hz : 0u;

    /// What the ADC would be fed, and whether that is within its rating.
    /// A PCLK2 above eight times adc_max_hz leaves no code that is: the
    /// tree still runs, and the converter's driver is what refuses (the
    /// file header).
    static constexpr uint8_t adc_code = adcpre_for(pclk2_hz, adc_max_hz);
    static constexpr uint32_t adc_hz = pclk2_hz / adcpre_divider(adc_code);
    static constexpr bool adc_in_spec = adc_hz <= adc_max_hz;

    /// Whether a flash erase or program at this rate must halve HCLK
    /// around itself (RM 32.1). Nothing here acts on it.
    static constexpr bool flash_needs_halving = target_hz > flash_safe_sysclk_hz;

    static_assert(src != ClockSource::internal || xtal_hz == 0u,
                  "brio Clock: the HSI has no crystal rate to name");
    static_assert((src != ClockSource::crystal && src != ClockSource::external) || xtal_hz != 0u,
                  "brio Clock: a crystal or an external clock is named with its rate, the third parameter");
    static_assert(!uses_hse || device::has_hse_pins,
                  "brio Clock: this part's package brings out no OSC_IN/OSC_OUT pad, so it has no "
                  "HSE in either form - the HSI and the PLL on it are its whole tree "
                  "(parts/<part>.hpp)");
    static_assert(!uses_hse || (xtal_hz >= device::hse_min_hz && xtal_hz <= device::hse_max_hz),
                  "brio Clock: this part's HSE oscillator takes device::hse_min_hz to hse_max_hz");
    static_assert(src != ClockSource::pll || pll_mul != 0u,
                  "brio Clock: no PLL multiplier (x2..x16, x18) reaches this rate from the root "
                  "through either input divider this part offers - the HSI is 8 MHz");
    static_assert(src != ClockSource::pll ||
                      (pll_in_hz >= device::pll_in_min_hz && pll_in_hz <= device::pll_in_max_hz),
                  "brio Clock: the PLL's input is outside the range this part states "
                  "(device::pll_in_min_hz .. pll_in_max_hz)");
    static_assert(src != ClockSource::pll ||
                      (sysclk_hz >= device::pll_out_min_hz && sysclk_hz <= device::pll_out_max_hz),
                  "brio Clock: the PLL's output is outside the range this part states "
                  "(device::pll_out_min_hz .. pll_out_max_hz)");
    static_assert(hpre_code != 0xFF,
                  "brio Clock: this HCLK is not SYSCLK divided by an HPRE divider "
                  "(1, then 2, 4, 8, 16, 64, 128, 256, 512 - the ladder skips 32)");
    static_assert(target_hz <= device::sysclk_max_hz, "brio Clock: HCLK must not exceed this part's ceiling");

    /**
     * Take the clock to `hz`. Returns false if a root never starts, the
     * PLL never locks or the switch never takes - the caller decides
     * what to say about a boot that stayed on the reset clock.
     *
     * The order is the tree's: the roots first, then the prescalers (so
     * no bus is ever briefly over its ceiling), then the switch.
     */
    static bool init() {
        // PARK ON THE HSI FIRST, AND THIS IS NOT A FORMALITY. PLLMUL,
        // PLLSRC and PLLXTPRE are writable only while the PLL is OFF,
        // and the PLL refuses to stop while it is the system clock - so
        // a tree that arrives here with the PLL already running takes
        // every one of those writes in SILENCE and keeps the rate it
        // had. That is not a rare state: a debugger's reset that spares
        // the peripherals leaves it (measured on this bench - the core
        // stayed on the previous image's PLL rate while the divisor of
        // every driver was computed for this one), and so does a second
        // call to init(). The HSI is on out of reset and needs no
        // waiting on a cold boot, which is what makes the park cheap.
        if (!Rcc::hsi_start()) {
            return false;
        }
        if (!Rcc::sysclk_select(SysclkSource::hsi)) {
            return false;
        }
        Rcc::pll_stop();

        // The prescalers next, while HCLK is the HSI's 8 MHz: no bus is
        // ever briefly over its ceiling this way.
        Rcc::prescalers(hpre_code, ppre1_code, ppre2_code);
        Rcc::adc_prescaler(adc_code);

        if constexpr (uses_hse) {
            if (!Rcc::hse_start(hse_bypass)) {
                return false;
            }
        }

        if constexpr (src == ClockSource::pll) {
            if (!Rcc::pll_start(uses_hse, pll_input_divided, pll_mul)) {
                return false;
            }
            if constexpr (usb_divider != 0u) {
                Rcc::usb_prescaler(usb_divider);
            }
        }

        return Rcc::sysclk_select(sysclk_source);
    }

    /// After a deep sleep the hardware comes back on the HSI with the
    /// PLL off: the tree is put back by running init() again, which is
    /// what a sleep site will call on its way out.
    static bool restore() { return init(); }
};

// ---- the dynamic clock -----------------------------------------------------

/// The pack of rates a DynamicClock may run at: static Clock tasks, the
/// FIRST the boot rate. A type list and nothing else - the discrete set
/// on this family is the program's own choice of tuples (below).
template <typename... Rs>
struct Rates {
    static constexpr uint8_t count = sizeof...(Rs);
};

/**
 * The runtime regime: `Rates<R0, R1, ...>` names the rates - each a
 * static `Clock<source, hz, xtal_hz>`, R0 the boot rate - and Users the
 * drivers a switch fans the new rate out to (each a ClockUser: `static
 * void rebase(uint32_t hz)`, checked by the concept where the list is
 * written) IN LIST ORDER, synchronously, BEFORE anything moves - so a
 * user can drain what it has in flight at the OLD rate and program
 * itself for the new one.
 *
 *   using Top  = brio::Clock<brio::ClockSource::pll, 144'000'000>;
 *   using Mid  = brio::Clock<brio::ClockSource::pll,  48'000'000>;
 *   using Boot = brio::Clock<brio::ClockSource::internal, 8'000'000>;
 *   using SysClock = brio::DynamicClock<brio::Rates<Top, Mid, Boot>,
 *                                       brio::Ticker, Serial>;
 *   constexpr SysClock clock;
 *   SysClock::init();                  // Top's init: 144 MHz
 *   SysClock::set<8'000'000>();        // the users rebased, then the HSI
 *
 * A RATE IS A TUPLE: the SYSCLK root with the PLL ratio the search
 * folded, HPRE, both peripheral prescalers, the ADC's divider and the
 * USB one. Every member is a constant of the `Clock` task, so naming
 * the rate names the tuple and nothing has to be repeated.
 *
 * EVERY SWITCH PARKS ON THE HSI, and that is not a detour but this
 * family's arithmetic: PLLMUL, PLLSRC and PLLXTPRE take a write only
 * with the PLL off, and the PLL will not stop while it is SYSCLK (the
 * file header). The static task's init() already parks for exactly that
 * reason, so a switch here IS that init() - one order serving both
 * directions, whichever way the rate moves.
 *
 * The discrete-rate surface (docs/design/clock.md) is the pack's:
 * rate_count, rate_hz(i), rate_index() - what ch32v203/delay.hpp
 * dispatches on so that no division runs at wait time. Two rates may
 * share an hz (48 MHz from the HSI's PLL and from a crystal's are
 * different tuples): set<hz>() and set(hz) take the FIRST that matches,
 * set_index<i>() / set_index(i) name one exactly.
 *
 * Call set() only when nothing that depends on the rate is mid-transfer
 * (a bus transaction in flight is the caller's problem - in an AO
 * system, ask the bus AOs first). Main context only.
 *
 * WHAT A STOP DOES TO THIS: the part comes out of Stop and Standby on
 * the HSI with the PLL off (RM 3.3.1's note), while the prescalers and
 * every peripheral register survive. restore() re-runs the CURRENT
 * rate's entry with no fan-out - the users were configured for that
 * rate and it is that rate that comes back - and is what a sleep site
 * calls first thing after a wake; a rate already in force (SWS says so,
 * which is every HSI rate) costs one register read.
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

    /// HCLK now, and the two bus rates beside it - the ones a
    /// peripheral's divisor really divides.
    static uint32_t hz() { return hz_; }
    static uint32_t pclk1_hz() { return rate_pclk1_hz_[idx_]; }
    static uint32_t pclk2_hz() { return rate_pclk2_hz_[idx_]; }
    /// What the USB blocks are fed at the rate in force; 0 on a rate
    /// that cannot feed them.
    static uint32_t usb_hz() { return rate_usb_hz_[idx_]; }
    /// What the ADC is fed, and whether the rate keeps it in range.
    static uint32_t adc_hz() { return rate_adc_hz_[idx_]; }
    static bool adc_in_spec() { return rate_adc_ok_[idx_]; }

    /// The discrete-rate surface: the pack, by position.
    static constexpr uint32_t rate_hz(uint8_t i) { return rate_hz_[i]; }
    static uint8_t rate_index() { return idx_; }
    static constexpr uint32_t rate_pclk1_hz(uint8_t i) { return rate_pclk1_hz_[i]; }
    static constexpr uint32_t rate_pclk2_hz(uint8_t i) { return rate_pclk2_hz_[i]; }
    static constexpr uint32_t rate_usb_hz(uint8_t i) { return rate_usb_hz_[i]; }
    static constexpr SysclkSource rate_source(uint8_t i) { return rate_source_[i]; }

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

    /// Put the CURRENT rate back after a sleep dropped SYSCLK to the HSI
    /// - no fan-out (the users hold the divisors for exactly this rate).
    /// True at once when SWS already reports the rate's root.
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
    static constexpr uint32_t rate_adc_hz_[rate_count] = {Rs::adc_hz...};
    static constexpr bool rate_adc_ok_[rate_count] = {Rs::adc_in_spec...};
    static constexpr SysclkSource rate_source_[rate_count] = {Rs::sysclk_source...};

    /**
     * The switch, for one rate of the pack. The mirror is written BEFORE
     * the rate's init() so that a user rebased for the new rate and a
     * delay_us dispatching on the index agree from the first instruction
     * after the switch; a failed init() returns false and the mirror
     * then names what was ASKED, which is Clock::init's own contract.
     */
    template <typename R>
    static bool enter_rate(uint8_t i, bool fan_out) {
        switching_ = true;
        // The fan-out is FIRST and at the old rate: a user drains what
        // it has in flight on the clock that is still running, and
        // programs its divisor from the HCLK it is handed (pclk1_hz_at()
        // derives the new bus rate arithmetically - the RCC still holds
        // the old prescalers here, and reading them would be reading the
        // past).
        if (fan_out) {
            (Users::rebase(R::hz), ...);
        }
        hz_ = R::hz;
        idx_ = i;
        // R::init() parks on the HSI itself, which is the whole of this
        // family's switching discipline (the file header).
        const bool ok = R::init();
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
    static inline bool switching_ = false;
};

} // namespace brio
