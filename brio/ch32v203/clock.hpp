/*
 * clock.hpp
 *
 * The clock tree of the CH32V203 (RM ch. 3): the roots, the PLL, the
 * switch, the three prescalers and the peripheral gates - and the task
 * that takes the tree to a rate stated at compile time.
 *
 * WHAT MAKES THIS TREE ITS OWN. Three things, none of them the F1's
 * whose register names it borrows:
 *
 *  - THE PLL'S INPUT DIVIDER FOR THE HSI IS IN ANOTHER BLOCK. RCC_CFGR0
 *    says only WHICH root feeds the PLL; whether the HSI arrives whole
 *    or halved is EXTEN_CTR.HSIPRE (RM 33.2.1), a register the RCC
 *    chapter never mentions. Out of reset it is 0 - the PLL sees 4 MHz,
 *    not 8 - so a program that only ever wrote RCC registers would find
 *    every rate half of what it asked for. This file owns that bit.
 *  - PB1 RUNS AT HALF OF HCLK, AND THAT IS A MEASUREMENT, NOT A HABIT.
 *    The datasheet's block diagram rates both peripheral buses at the
 *    core's own ceiling of 144 MHz, and this file believed it - until
 *    the USB device controller, which lives on PB1, answered every
 *    packet of an enumeration with a packet-memory overflow and stored
 *    none of them (sixteen PMAOVR in one attempt, the data never
 *    reaching the buffer). WCH's own clock code sets PPRE1 = /2 at
 *    EVERY rate it offers - 48, 56, 72, 96, 120 and 144 MHz, from the
 *    HSI or a crystal - and only the bare-HSE path leaves it undivided.
 *    So PB1 is halved here too and pclk1_hz says so; where the real
 *    ceiling of that bus lies, between 72 MHz and the datasheet's 144,
 *    is not known and is in the document's gap list. PB2 keeps the
 *    whole rate, as it does in the vendor's ladder.
 *  - THERE ARE NO FLASH WAIT STATES TO PROGRAM. The array is divided
 *    into a zero-wait region and a non-zero-wait one by the part, not
 *    by a LATENCY field (RM ch. 32 has none), so nothing here touches
 *    the flash controller. What the chapter DOES ask for is at the
 *    other end: above 120 MHz a flash ERASE or PROGRAM wants the system
 *    clock halved around it (32.1), which is the NVM driver's business
 *    and not this file's.
 *
 * THE ORDER OF init() IS THE SILICON'S, NOT A HABIT: it parks SYSCLK on
 * the HSI before it touches anything else. The PLL's own fields are
 * writable only while it is off, and it will not go off while it is the
 * system clock, so a tree that is already running the PLL swallows every
 * write without a flag - see init().
 *
 * THE USB CLOCK IS PART OF THE TREE. USBPRE divides the PLL by 1, 2 or
 * 3 and the device controller needs exactly 48 MHz, so `usb_hz` here is
 * a compile-time answer - 48 MHz when the PLL runs at 48, 96 or 144, and
 * zero otherwise - and init() programs the divider BEFORE any USB gate
 * is opened, which is the order 3.4.2 asks for. A USB driver
 * static_asserts on it rather than discovering at run time that its
 * frames are the wrong length.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/device.hpp"
#include "util/clock.hpp"

namespace brio {

/// Where SYSCLK comes from.
enum class ClockSource : uint8_t {
    internal,   ///< HSI, the 8 MHz internal RC
    pll,        ///< the PLL, fed by the HSI or by a crystal named beside it
    crystal,    ///< HSE with a crystal on OSC_IN/OSC_OUT, its rate the third parameter
    external,   ///< HSE in bypass: a clock into OSC_IN, its rate the third parameter
};

/// The ceiling this family is rated for; the part's own is device::.
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
/// What the USB device controller must be fed, whatever the core runs at.
inline constexpr uint32_t usb_required_hz = 48'000'000UL;

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

/// What the PLL can multiply by, in code order: x2..x16 and then x18.
constexpr bool pll_mul_exists(uint32_t mul) {
    return (mul >= 2u && mul <= 16u) || mul == 18u;
}

/// How long a root or a switch may take before init() gives up. Bounded
/// spins, not while(1): a boot that cannot reach its rate must return
/// and let the program say so.
inline constexpr uint32_t clock_timeout_turns = 100'000UL;

/// Which bus a peripheral's enable and reset bits sit on (RM 3.4.5..3.4.8).
enum class Bus : uint8_t { hb, pb2, pb1 };

/**
 * The RCC block, monostate: the roots, the switch, the prescalers and
 * the peripheral gates. Verbs read and write the registers as they
 * stand; the policy - which rate, which root - is the task's below.
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

    // ---- the roots --------------------------------------------------------
    static bool hsi_ready() { return (rcc()->CTLR & rcc_hsirdy) != 0u; }

    /// Start the 8 MHz HSI and wait a bounded time for it. It is on out
    /// of reset, so this is usually a readback - but a program that
    /// stopped it, or a warm start, must be able to get it back: it is
    /// the root everything else is configured from (see Clock::init).
    static bool hsi_start() {
        rcc()->CTLR |= rcc_hsion;
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->CTLR & rcc_hsirdy) != 0u) {
                return true;
            }
        }
        return false;
    }

    /// The HSI's user trim, five bits around a centre of 16 (3.4.1).
    static void hsi_trim(uint8_t trim) {
        rcc()->CTLR = (rcc()->CTLR & ~rcc_hsitrim_mask) |
                      ((static_cast<uint32_t>(trim) & 0x1Fu) << 3);
    }
    static uint8_t hsi_calibration() {
        return static_cast<uint8_t>((rcc()->CTLR & rcc_hsical_mask) >> 8);
    }

    /// Start the HSE, as a crystal or as an external clock, and wait a
    /// bounded time for it. HSEBYP may only be written with HSEON clear,
    /// which is why the order here is not negotiable.
    static bool hse_start(bool bypass) {
        rcc()->CTLR &= ~rcc_hseon;
        if (bypass) {
            rcc()->CTLR |= rcc_hsebyp;
        } else {
            rcc()->CTLR &= ~rcc_hsebyp;
        }
        rcc()->CTLR |= rcc_hseon;
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->CTLR & rcc_hserdy) != 0u) {
                return true;
            }
        }
        return false;
    }

    static void hse_stop() { rcc()->CTLR &= ~(rcc_hseon | rcc_hsebyp); }

    /// The clock security system: with the HSE as a root, its failure
    /// switches SYSCLK back to the HSI and raises the non-maskable
    /// interrupt (3.3.4). Only meaningful once the HSE is running.
    static void clock_monitor(bool on) {
        if (on) {
            rcc()->CTLR |= rcc_csson;
        } else {
            rcc()->CTLR &= ~rcc_csson;
        }
    }

    /// The 40 kHz LSI: the watchdog's root and one of the RTC's.
    static bool lsi_start() {
        rcc()->RSTSCKR |= rcc_lsion;
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->RSTSCKR & rcc_lsirdy) != 0u) {
                return true;
            }
        }
        return false;
    }
    static void lsi_stop() { rcc()->RSTSCKR &= ~rcc_lsion; }

    // ---- the PLL ----------------------------------------------------------
    /// Configure and start the PLL: `from_hse` picks the root, `halved`
    /// asks for the root's rate divided by two on the way in (EXTEN's
    /// HSIPRE for the HSI, RCC's PLLXTPRE for the HSE - two registers,
    /// one meaning, which is why this verb takes one flag), `mul` is the
    /// multiplier. The PLL must be OFF for any of it to stick.
    static bool pll_start(bool from_hse, bool halved, uint32_t mul) {
        rcc()->CTLR &= ~rcc_pllon;

        if (from_hse) {
            uint32_t cfgr = rcc()->CFGR0 | rcc_pllsrc;
            if (halved) {
                cfgr |= rcc_pllxtpre;
            } else {
                cfgr &= ~rcc_pllxtpre;
            }
            rcc()->CFGR0 = cfgr;
        } else {
            rcc()->CFGR0 &= ~rcc_pllsrc;
            // The HSI's own divider lives in EXTEN, not in RCC (see the
            // file header). LKUPRST is write-one-to-clear: it is masked
            // out so this read-modify-write cannot clear a lock-up flag
            // the program has not read yet.
            const uint32_t ctr = exten()->CTR & ~exten_lkuprst;
            exten()->CTR = halved ? (ctr & ~exten_hsipre) : (ctr | exten_hsipre);
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

    // ---- the switch and the prescalers ------------------------------------
    /// Point SYSCLK at a root and wait until the hardware confirms it in
    /// SWS. The two fields are the whole switch: there is no order to
    /// respect beyond the root being ready.
    static bool sysclk_select(uint32_t sw_bits, uint32_t sws_bits) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_sw_mask) | sw_bits;
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->CFGR0 & rcc_sws_mask) == sws_bits) {
                return true;
            }
        }
        return false;
    }

    static void prescalers(uint8_t hpre_code, uint8_t ppre1_code, uint8_t ppre2_code) {
        uint32_t cfgr = rcc()->CFGR0;
        cfgr &= ~(rcc_hpre_mask | rcc_ppre1_mask | rcc_ppre2_mask);
        cfgr |= static_cast<uint32_t>(hpre_code) << 4;
        cfgr |= static_cast<uint32_t>(ppre1_code) << rcc_ppre1_shift;
        cfgr |= static_cast<uint32_t>(ppre2_code) << rcc_ppre2_shift;
        rcc()->CFGR0 = cfgr;
    }

    /// The PLL's divider on the way to the USB blocks: 1, 2 or 3.
    /// 3.4.2 requires it to be written before the USB clock gates are
    /// opened.
    static void usb_prescaler(uint8_t div) {
        const uint32_t code = div == 1u ? 0UL : div == 2u ? 1UL : 2UL;
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_usbpre_mask) | (code << rcc_usbpre_shift);
    }

    /// The clock output pad: one of the roots on PA8, for a counter or a
    /// scope to judge this file's arithmetic.
    static void mco(uint32_t source_bits) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_mco_mask) | source_bits;
    }
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

    /// THE PLL SEARCH. The input may be the root or half of it, and the
    /// multiplier is x2..x16 or x18; the whole root is preferred where
    /// both reach the target, which keeps the multiplier small and is
    /// what the 8 MHz HSI needs to reach this family's ceiling (8 x 18).
    static constexpr uint32_t pll_target = target_hz;
    static constexpr bool pll_halved = [] -> bool {
        for (uint32_t mul = 2; mul <= 18u; ++mul) {
            if (pll_mul_exists(mul) && root_hz * mul == pll_target) {
                return false;
            }
        }
        return true;
    }();
    static constexpr uint32_t pll_in_hz = pll_halved ? root_hz / 2u : root_hz;
    static constexpr uint32_t pll_mul = [] -> uint32_t {
        for (uint32_t mul = 2; mul <= 18u; ++mul) {
            if (pll_mul_exists(mul) && pll_in_hz * mul == pll_target) {
                return mul;
            }
        }
        return 0u;
    }();

    /// SYSCLK before HPRE.
    static constexpr uint32_t sysclk_hz = (src == ClockSource::pll) ? pll_in_hz * pll_mul : root_hz;
    static constexpr uint8_t hpre_code = hpre_for(sysclk_hz, target_hz);

    /// PB1 is halved and PB2 is not (see the file header). The two are
    /// separate names because a driver must ask for ITS bus: a USART on
    /// PB1 and one on PB2 do not divide the same number.
    /// PB1 is capped at 72 MHz: undivided while HCLK is at or below
    /// that, halved above it.
    static constexpr bool pb1_halved = target_hz > 72'000'000UL;
    static constexpr uint8_t ppre1_code = pb1_halved ? 0x4 : 0x0;
    static constexpr uint8_t ppre2_code = 0x0;   ///< /1
    static constexpr uint32_t pclk1_hz = pb1_halved ? target_hz / 2u : target_hz;
    static constexpr uint32_t pclk2_hz = target_hz;

    /// The USB blocks want 48 MHz and take the PLL divided by 1, 2 or 3:
    /// this is what they would get, and zero when this tree cannot feed
    /// them at all.
    static constexpr uint8_t usb_divider = [] -> uint8_t {
        if (src != ClockSource::pll) {
            return 0;
        }
        for (uint8_t div = 1; div <= 3u; ++div) {
            if (sysclk_hz / div == usb_required_hz && sysclk_hz % div == 0u) {
                return div;
            }
        }
        return 0;
    }();
    static constexpr uint32_t usb_hz = usb_divider != 0u ? usb_required_hz : 0u;

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
                  "brio Clock: no PLL multiplier (x2..x16, x18) reaches this rate from the root, "
                  "whole or halved - the HSI is 8 MHz");
    static_assert(hpre_code != 0xFF,
                  "brio Clock: this HCLK is not SYSCLK divided by an HPRE divider "
                  "(1, then 2, 4, 8, 16, 64, 128, 256, 512 - the ladder skips 32)");
    static_assert(sysclk_hz <= sysclk_max_hz, "brio Clock: SYSCLK must not exceed 144 MHz");
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
        if (!Rcc::sysclk_select(rcc_sw_hsi, rcc_sws_hsi)) {
            return false;
        }
        Rcc::pll_stop();

        // The prescalers next, while HCLK is the HSI's 8 MHz: no bus is
        // ever briefly over its ceiling this way.
        Rcc::prescalers(hpre_code, ppre1_code, ppre2_code);

        if constexpr (uses_hse) {
            if (!Rcc::hse_start(hse_bypass)) {
                return false;
            }
        }

        if constexpr (src == ClockSource::pll) {
            if (!Rcc::pll_start(uses_hse, pll_halved, pll_mul)) {
                return false;
            }
            if constexpr (usb_divider != 0u) {
                Rcc::usb_prescaler(usb_divider);
            }
        }

        if constexpr (src == ClockSource::pll) {
            return Rcc::sysclk_select(rcc_sw_pll, rcc_sws_pll);
        } else if constexpr (uses_hse) {
            return Rcc::sysclk_select(rcc_sw_hse, rcc_sws_hse);
        } else {
            return Rcc::sysclk_select(rcc_sw_hsi, rcc_sws_hsi);
        }
    }

    /// After a deep sleep the hardware comes back on the HSI with the
    /// PLL off: the tree is put back by running init() again, which is
    /// what a sleep site will call on its way out.
    static bool restore() { return init(); }
};

} // namespace brio
