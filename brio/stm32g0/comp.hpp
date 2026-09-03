/*
 * comp.hpp
 *
 * The STM32G0's analog comparators (RM0444 ch. 18): `Comp<n>`, one per
 * instance, where n = 1..3 - and how many exist is the DEVICE HEADER's
 * answer, not a list here (stm32g0/device_tables.hpp): the G0B1/G0C1
 * carry three, the G071 class two, and the G031 class none at all, so
 * `Comp<3>` is a compile error on a G071 and `Comp<1>` is one on a G031.
 *
 * WHAT A COMPARATOR IS HERE. One CSR and nothing else - a rail-to-rail
 * comparator whose PLUS input is one of three pads (or nothing), whose
 * MINUS input is one of three pads, one of the two DAC channels, or
 * VREFINT and its quarter, half and three-quarter taps, with
 * programmable hysteresis, two speeds, an inverting polarity, a blanking
 * window taken from a timer's compare output, and an output that goes to
 * a pad, to a timer input and to an EXTI line all at once.
 *
 * FOUR FACTS THAT SHAPE THIS FILE.
 *
 * 1. THE ONLY ANALOG SOURCE INSIDE THE CHIP REACHES THE MINUS INPUT.
 *    Tables 93, 95 and 97 give the plus input three PADS and "open" -
 *    there is no internal signal among them - while tables 94, 96 and 98
 *    give the minus input the DAC's two channels and four VREFINT taps.
 *    So a threshold can be produced with no wire and a SIGNAL cannot: a
 *    comparator measurement on a board with nothing attached to a plus
 *    pad can exercise every multiplexer, polarity, blanking and event
 *    path, and cannot walk a threshold. That is a fact about the
 *    silicon, and it is stated here so no suite pretends otherwise.
 *
 * 2. THE INPUT TABLES ARE PER INSTANCE AND THE PADS ARE NOT A PATTERN.
 *    COMP1's plus inputs are PC5/PB2/PA1, COMP2's PB4/PB6/PA3, COMP3's
 *    PB0/PC1/PE7 - and PE7/PE8 exist only on a part that bonds port E.
 *    `positive_pin()` / `negative_pin()` publish the tables as DATA and
 *    `config_valid()` refuses a selection whose pad this device has not
 *    got, which is this stratum's per-package gate with no hand-kept
 *    list behind it.
 *
 * 3. LOCK IS ONE-WAY UNTIL A RESET. 18.3.4: setting it makes the WHOLE
 *    register read-only including the lock bit itself, and only an MCU
 *    reset clears it. The verb is offered because functional-safety
 *    applications are what the bit is for, and it is spelled `lock()` so
 *    nobody reaches it by accident; every configuring verb here refuses
 *    while it stands rather than storing into a register the silicon
 *    ignores.
 *
 * 4. THERE IS NO INTERRUPT OF ITS OWN. 18.5: the output goes to an EXTI
 *    line - 17, 18 and 20, all three CONFIGURABLE - and the vector is
 *    the ADC's, shared by all three comparators (table 61). This driver
 *    PUBLISHES its line number and does not include stm32g0/exti.hpp:
 *    the fabric driver owns the fabric and the peripheral owns its own
 *    vocabulary (the samc EVSYS ruling, kept). The chapter's own
 *    sequence is to configure the EXTI line FIRST and enable the
 *    comparator last, so the enable's own transient does not arrive on
 *    an unconfigured line.
 *
 * THE CLOCK. 18.3.3 says two things that read as contradictory - "there
 * is no clock enable control bit provided in the RCC controller" and
 * "reset and clock enable bits are common for COMP and SYSCFG" - and the
 * address map settles it: COMPn_CSR lives INSIDE the SYSCFG block
 * (COMP1_BASE = SYSCFG_BASE + 0x200), so RCC_APBENR2.SYSCFGEN is what
 * makes the register accessible at all, while the comparator's own
 * polarity and output logic keep working without it (18.3.3's
 * "Important" paragraph, which is what makes a comparator useful in Stop
 * mode). `init()` opens that gate and `release()` does not close it -
 * SYSCFG is shared.
 *
 * ERRATA: ES0548 Rev 3 has NO item touching the comparators on either
 * silicon revision.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// COMPx_CSR.INPSEL - which of the instance's three pads drives the plus
/// input, or none (tables 93, 95, 97).
enum class CompPositive : uint8_t { input0 = 0, input1 = 1, input2 = 2, open = 3 };

/// COMPx_CSR.INMSEL - the minus input (tables 94, 96, 98). The three
/// pad codes are named for the code and not for the pad, because WHICH
/// pad they are differs per instance; `Comp<n>::negative_pin()` is what
/// says.
enum class CompNegative : uint8_t {
    vrefint_quarter = 0,
    vrefint_half = 1,
    vrefint_three_quarters = 2,
    vrefint = 3,
    dac_channel1 = 4,
    dac_channel2 = 5,
    input6 = 6,
    input7 = 7,
    input8 = 8,
};

/// COMPx_CSR.HYST (18.3.6). DS13560 table 68 gives the typical values:
/// 10, 20 and 30 mV.
enum class CompHysteresis : uint8_t { none = 0, low = 1, medium = 2, high = 3 };

/// COMPx_CSR.PWRMODE (18.3.8). The two other codes are Reserved.
enum class CompPower : uint8_t { high_speed = 0, medium_speed = 1 };

/// COMPx_CSR.BLANKSEL, which is a MASK and not a selector: 18.6.1 lists
/// one bit per source and a caller may OR them. The blanking window is
/// the complement of the chosen signal ANDed with the output (18.3.7).
struct CompBlank {
    static constexpr uint8_t none = 0;
    static constexpr uint8_t tim1_oc4 = 1u << 0;
    static constexpr uint8_t tim1_oc5 = 1u << 1;
    static constexpr uint8_t tim2_oc3 = 1u << 2;
    static constexpr uint8_t tim3_oc3 = 1u << 3;
    static constexpr uint8_t tim15_oc2 = 1u << 4;
    static constexpr uint8_t all = 0x1F;
};

/// A pad from one of the input tables: the port letter and the pin, or
/// `valid` false where the code selects something that is not a pad (or
/// a pad this device does not bond).
struct CompPad {
    char port = 0;
    uint8_t pin = 0;
    bool valid = false;
};

/// Table 93 / 95 / 97: the plus input's three pads, by instance.
constexpr CompPad comp_positive_pin(uint8_t n, CompPositive sel) {
    CompPad p{};
    switch (n) {
        case 1:
            switch (sel) {
                case CompPositive::input0: p = {'C', 5, true}; break;
                case CompPositive::input1: p = {'B', 2, true}; break;
                case CompPositive::input2: p = {'A', 1, true}; break;
                default: return {};
            }
            break;
        case 2:
            switch (sel) {
                case CompPositive::input0: p = {'B', 4, true}; break;
                case CompPositive::input1: p = {'B', 6, true}; break;
                case CompPositive::input2: p = {'A', 3, true}; break;
                default: return {};
            }
            break;
        case 3:
            switch (sel) {
                case CompPositive::input0: p = {'B', 0, true}; break;
                case CompPositive::input1: p = {'C', 1, true}; break;
                case CompPositive::input2: p = {'E', 7, true}; break;
                default: return {};
            }
            break;
        default: return {};
    }
    // A pad on a port this device does not bond is not an input.
    return gpio_port_present(p.port) ? p : CompPad{};
}

/// Table 94 / 96 / 98: the minus input's three PAD codes, by instance.
/// Every other code is an internal source and answers `valid` false.
constexpr CompPad comp_negative_pin(uint8_t n, CompNegative sel) {
    CompPad p{};
    switch (n) {
        case 1:
            switch (sel) {
                case CompNegative::input6: p = {'B', 1, true}; break;
                case CompNegative::input7: p = {'C', 4, true}; break;
                case CompNegative::input8: p = {'A', 0, true}; break;
                default: return {};
            }
            break;
        case 2:
            switch (sel) {
                case CompNegative::input6: p = {'B', 3, true}; break;
                case CompNegative::input7: p = {'B', 7, true}; break;
                case CompNegative::input8: p = {'A', 2, true}; break;
                default: return {};
            }
            break;
        case 3:
            switch (sel) {
                case CompNegative::input6: p = {'B', 2, true}; break;
                case CompNegative::input7: p = {'C', 0, true}; break;
                case CompNegative::input8: p = {'E', 8, true}; break;
                default: return {};
            }
            break;
        default: return {};
    }
    return gpio_port_present(p.port) ? p : CompPad{};
}

/**
 * Whose plus input COMPn borrows when WINMODE is set - and the answer is
 * NOT "the next one up". 18.6.1's three register descriptions say it
 * one instance at a time: COMP1 takes COMP2_INP, COMP2 takes COMP1_INP,
 * COMP3 takes COMP2_INP. So the two pairs 18.3.5 names (COMP1+COMP2 and
 * COMP2+COMP3) are reached from opposite directions, and a driver that
 * assumed n+1 would refuse a legal COMP2 window and allow an impossible
 * one. 0 where this device has no such partner.
 */
constexpr uint8_t comp_window_partner(uint8_t n) {
    const uint8_t partner = n == 2u ? 1u : 2u;
    return (n >= 1u && n <= 3u && comp_present(n) && comp_present(partner)) ? partner : 0u;
}

/// What one comparator is configured with.
struct CompConfig {
    CompPositive positive = CompPositive::open;
    CompNegative negative = CompNegative::vrefint_half;
    CompHysteresis hysteresis = CompHysteresis::none;
    CompPower power = CompPower::high_speed;
    /// POLARITY: the output (and VALUE, and the EXTI line) inverted.
    bool inverted = false;
    /// WINMODE: take the PARTNER's plus input instead of this instance's
    /// own, which is how a window comparator shares one pad (18.3.5).
    /// WHICH partner is not a pattern - see comp_window_partner().
    bool window_input = false;
    /// WINOUT: report VALUE xor the neighbour's VALUE - the window's own
    /// "inside or outside" answer (18.6.1).
    bool window_output = false;
    /// BLANKSEL, a mask of CompBlank constants.
    uint8_t blanking = CompBlank::none;
};

constexpr bool comp_config_valid(uint8_t n, const CompConfig& c) {
    if (!comp_present(n)) {
        return false;
    }
    if (c.positive != CompPositive::open && !comp_positive_pin(n, c.positive).valid) {
        return false;   // a pad this device does not bond
    }
    switch (c.negative) {
        case CompNegative::input6:
        case CompNegative::input7:
        case CompNegative::input8:
            if (!comp_negative_pin(n, c.negative).valid) {
                return false;
            }
            break;
        case CompNegative::vrefint_quarter:
        case CompNegative::vrefint_half:
        case CompNegative::vrefint_three_quarters:
        case CompNegative::vrefint:
            break;
        case CompNegative::dac_channel1:
        case CompNegative::dac_channel2:
            if (!dac_present()) {
                return false;   // no DAC on this part, so no such threshold
            }
            break;
        default: return false;
    }
    if (c.power != CompPower::high_speed && c.power != CompPower::medium_speed) {
        return false;   // 18.6.1: the other two PWRMODE codes are Reserved
    }
    if ((c.blanking & ~static_cast<uint8_t>(CompBlank::all)) != 0u) {
        return false;
    }
    // WINMODE borrows a PARTICULAR instance's plus input, and which one
    // is per instance (comp_window_partner): a device without that
    // partner cannot spell the window.
    if (c.window_input && comp_window_partner(n) == 0u) {
        return false;
    }
    return true;
}

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
class Comp {
public:
    static_assert(comp_present(n),
                  "brio Comp: this device has no comparator of that number (18.1: the "
                  "third is the G0B1/G0C1's alone and the G031 class has none)");

    Comp() = delete;

    static constexpr uint8_t index = n;

    /// The EXTI line this comparator's output raises - a CONFIGURABLE
    /// line, so a sense must be chosen before anything is pending
    /// (stm32g0/exti.hpp). Published, not used: this file includes no
    /// EXTI driver.
    static constexpr uint8_t exti_line = comp_exti_line(n);

    /// The NVIC line, which is the ADC's and is shared by all three
    /// comparators (table 61).
    static constexpr IRQn_Type irq() { return comp_irq(); }

    /// Whose plus input this instance borrows under WINMODE, 0 where
    /// this device has no such partner (comp_window_partner's note: the
    /// mapping is not n + 1).
    static constexpr uint8_t window_partner = comp_window_partner(n);

    static COMP_TypeDef& regs() { return *reinterpret_cast<COMP_TypeDef*>(comp_base(n)); }

    static constexpr CompPad positive_pin(CompPositive s) { return comp_positive_pin(n, s); }
    static constexpr CompPad negative_pin(CompNegative s) { return comp_negative_pin(n, s); }
    static constexpr bool config_valid(const CompConfig& c) { return comp_config_valid(n, c); }

    /**
     * Open the gate the registers live behind: COMPn_CSR sits inside the
     * SYSCFG block, so RCC_APBENR2.SYSCFGEN is what makes it readable and
     * writable (this file's header). Nothing is closed again by
     * `release()` - SYSCFG is shared with the EXTI's own multiplexer and
     * with everything else in that corner.
     */
    static void init() {
        Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN, true);
    }

    static bool bus_clock() { return Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN); }

    // ---- configuration --------------------------------------------------------

    /**
     * Write the whole CSR from `c`, leaving the comparator DISABLED.
     *
     * REFUSES while LOCK stands (18.3.4 makes the register read-only,
     * and a store that lands nowhere is worse than a false), and refuses
     * a configuration this device cannot have.
     */
    static bool configure(const CompConfig& c) {
        if (locked() || !comp_config_valid(n, c)) {
            return false;
        }
        regs().CSR = csr_word(c);
        return true;
    }

    /// Put the input pads this configuration names into analog mode -
    /// 18.3.2's requirement, and the pads' own reset state.
    ///
    /// It is a RUNTIME loop over a port letter because the input tables
    /// are runtime data (a `Pin<'A',1>` cannot be formed from a value),
    /// which is why this verb reaches GPIO's registers here instead of
    /// through stm32g0/pin.hpp's per-pin face.
    static bool claim_inputs(const CompConfig& c) {
        if (!comp_config_valid(n, c)) {
            return false;
        }
        analog_pad(comp_positive_pin(n, c.positive));
        analog_pad(comp_negative_pin(n, c.negative));
        return true;
    }

    /// EN. 18.5's sequence puts this LAST, after the EXTI line is
    /// configured and armed. Refuses while locked.
    static bool enable(bool on) {
        if (locked()) {
            return false;
        }
        regs().CSR = on ? (regs().CSR | COMP_CSR_EN) : (regs().CSR & ~COMP_CSR_EN);
        return true;
    }

    static bool enabled() { return (regs().CSR & COMP_CSR_EN) != 0u; }

    /**
     * VALUE: the output AFTER the polarity selector and the blanking
     * (18.6.1's own wording and figure 68), which is the whole reason
     * this is not "is the plus input above the minus one".
     */
    static bool value() { return (regs().CSR & COMP_CSR_VALUE) != 0u; }

    static CompPositive positive() {
        return static_cast<CompPositive>((regs().CSR & COMP_CSR_INPSEL_Msk) >>
                                         COMP_CSR_INPSEL_Pos);
    }
    static CompNegative negative() {
        return static_cast<CompNegative>((regs().CSR & COMP_CSR_INMSEL_Msk) >>
                                         COMP_CSR_INMSEL_Pos);
    }
    static CompHysteresis hysteresis() {
        return static_cast<CompHysteresis>((regs().CSR & COMP_CSR_HYST_Msk) >>
                                           COMP_CSR_HYST_Pos);
    }
    static CompPower power() {
        return static_cast<CompPower>((regs().CSR & COMP_CSR_PWRMODE_Msk) >>
                                      COMP_CSR_PWRMODE_Pos);
    }
    static bool inverted() { return (regs().CSR & COMP_CSR_POLARITY) != 0u; }
    static bool window_input() { return (regs().CSR & COMP_CSR_WINMODE) != 0u; }
    static bool window_output() { return (regs().CSR & COMP_CSR_WINOUT) != 0u; }
    static uint8_t blanking() {
        return static_cast<uint8_t>((regs().CSR & COMP_CSR_BLANKING_Msk) >>
                                    COMP_CSR_BLANKING_Pos);
    }

    // ---- the lock (18.3.4) ----------------------------------------------------

    static bool locked() { return (regs().CSR & COMP_CSR_LOCK) != 0u; }

    /**
     * Make the whole register read-only until the next MCU RESET - the
     * lock bit included. There is no unlock and there is no undo: this is
     * offered because 18.3.4's safety case is real, and it is a separate
     * verb from `configure()` for exactly that reason.
     */
    static void lock() { regs().CSR = regs().CSR | COMP_CSR_LOCK; }

    /// Everything this file turned on: the comparator off and its CSR
    /// back to the reset value. Refused - and reported - while locked,
    /// because there is nothing a driver can do about it.
    static bool release() {
        if (locked()) {
            return false;
        }
        regs().CSR = 0;
        return true;
    }

private:
    static constexpr uint32_t csr_word(const CompConfig& c) {
        uint32_t w = 0;
        w |= static_cast<uint32_t>(c.negative) << COMP_CSR_INMSEL_Pos;
        w |= static_cast<uint32_t>(c.positive) << COMP_CSR_INPSEL_Pos;
        if (c.window_input) w |= COMP_CSR_WINMODE;
        if (c.window_output) w |= COMP_CSR_WINOUT;
        if (c.inverted) w |= COMP_CSR_POLARITY;
        w |= static_cast<uint32_t>(c.hysteresis) << COMP_CSR_HYST_Pos;
        w |= static_cast<uint32_t>(c.power) << COMP_CSR_PWRMODE_Pos;
        w |= static_cast<uint32_t>(c.blanking) << COMP_CSR_BLANKING_Pos;
        return w;
    }

    /// MODER = analog for one pad named by letter and number. The port's
    /// clock is opened first, as every configuring verb of
    /// stm32g0/pin.hpp does (5.2.17: a clockless port ignores writes in
    /// silence).
    static void analog_pad(const CompPad& p) {
        if (!p.valid) {
            return;
        }
        Rcc::io_clock(p.port, true);
        GPIO_TypeDef& g = *reinterpret_cast<GPIO_TypeDef*>(gpio_port_base(p.port));
        const uint32_t shift = 2u * p.pin;
        g.MODER = g.MODER | (0x3UL << shift);              // analog = 0b11
        g.PUPDR = g.PUPDR & ~(0x3UL << shift);             // 7.3.1: no pull in analog
    }
};

} // namespace brio
