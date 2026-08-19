/*
 * ac.hpp
 *
 * The AVR DA/DB analog comparators (AC, DS40002247B ch. 32) as brio
 * sees them - see docs/avrdx/ac.md:
 *
 *  RESOURCE - Ac<n>, n = 0..2: a config struct owns the inputs (one of
 *  four positive pins, one of three negative pins or DACREF), the DACREF
 *  code, hysteresis, power profile, inversion, initial value, output
 *  pin and route, run-standby, interrupt sense; init<cfg>() / init(cfg)
 *  write it (and turn the chosen pins into analog inputs). Verbs:
 *  state(), dacref(code) / threshold_mv(mv), the interrupt enable and
 *  the ISR body cmp(), the OUT event generator, the window pairing.
 *  The reference of DACREF is the comparators' shared VREF.ACREF: the
 *  config names it (`Ref`, vref.hpp) like the DAC's config does.
 *
 *  TASKS - what an application names:
 *    Threshold<Ac>     "the input crossed V": DACREF from millivolts,
 *                      an interrupt per crossing direction
 *    Window<Ac, Ac>    "the input is above / inside / below": two
 *                      comparators on one pin, WINSEL, the state
 *
 * Facts that shape the code (32.3, 39.17; errata F has no AC item):
 *  - V_DACREF = DACREF x V_ACREF / 256 (8-bit); offset +-5 mV typ.,
 *    hysteresis 0 / 10 / 25 / 50 mV, response 85 / 250 / 460 ns by
 *    power profile; the AC and (if internal) the reference take their
 *    start-up after enable, a new input pin or reference wants settling
 *    - INITVAL holds the output meanwhile;
 *  - inputs on the 48-pin part: AINP0 PD2 (all), AINP1 PE0/PD3/PD4,
 *    AINP2 PE2/PD4/PE1, AINP3 PD6 (all: the DAC pin), AINN0 PD3/PD5/PD7,
 *    AINN1 PD0 (all), AINN2 PD7 (all) for AC0/AC1/AC2; the pins' digital
 *    input buffer is disabled by init();
 *  - the output: CMPSTATE in STATUS, the OUT event (level, async, 0x20
 *    + n), the pin (OUTEN: PA7 default - CLKOUT's pin - or ALT1 PC6, one
 *    route bit per instance in PORTMUX.ACROUTEA), the CCL input menu
 *    (ACn OUT on LUT input n), one interrupt per instance (CMP) with
 *    sense both / falling / rising;
 *  - window mode: ACn pairs with ACn+1 or ACn+2 (WINSEL), both MUXPOS on
 *    the same pin, the partner holds the UPPER limit on its negative
 *    input and ACn the lower; WINSTATE reads above/inside/below and
 *    INTMODE picks which state (above/inside/below/outside) raises the
 *    interrupt and the event (CMPSTATE then means "matches");
 *  - RUNSTDBY keeps it alive in standby, pin, event and interrupt
 *    included, without CLK_PER.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/vref.hpp"

namespace brio {

// ---- the knobs (32.5) -------------------------------------------------------------

enum class AcPos : uint8_t {
    ainp0 = AC_MUXPOS_AINP0_gc, ainp1 = AC_MUXPOS_AINP1_gc,
    ainp2 = AC_MUXPOS_AINP2_gc, ainp3 = AC_MUXPOS_AINP3_gc,
};
enum class AcNeg : uint8_t {
    ainn0 = AC_MUXNEG_AINN0_gc, ainn1 = AC_MUXNEG_AINN1_gc,
    ainn2 = AC_MUXNEG_AINN2_gc, dacref = AC_MUXNEG_DACREF_gc,
};
enum class AcHysteresis : uint8_t {
    none = AC_HYSMODE_NONE_gc, small = AC_HYSMODE_SMALL_gc,       ///< ~10 mV
    medium = AC_HYSMODE_MEDIUM_gc, large = AC_HYSMODE_LARGE_gc,   ///< ~25 / ~50 mV
};
enum class AcPower : uint8_t {
    fast = AC_POWER_PROFILE0_gc,      ///< ~85 ns, most current
    medium = AC_POWER_PROFILE1_gc,    ///< ~250 ns, 17 uA
    low = AC_POWER_PROFILE2_gc,       ///< ~460 ns
};
/// Normal-mode interrupt sense (INTMODE).
enum class AcSense : uint8_t {
    both = AC_INTMODE_NORMAL_BOTHEDGE_gc,
    falling = AC_INTMODE_NORMAL_NEGEDGE_gc,   ///< positive input goes below negative
    rising = AC_INTMODE_NORMAL_POSEDGE_gc,    ///< positive input goes above negative
};
/// Window-mode interrupt/event condition (INTMODE with WINSEL).
enum class AcWindowSense : uint8_t {
    above = AC_INTMODE_WINDOW_ABOVE_gc, inside = AC_INTMODE_WINDOW_INSIDE_gc,
    below = AC_INTMODE_WINDOW_BELOW_gc, outside = AC_INTMODE_WINDOW_OUTSIDE_gc,
};
enum class AcWindowState : uint8_t { above = 0, inside = 1, below = 2 };

struct AcConfig {
    AcPos positive = AcPos::ainp0;
    AcNeg negative = AcNeg::dacref;
    Ref reference = Ref::v1024;        ///< VREF.ACREF, for DACREF
    bool reference_always_on = false;
    uint8_t dacref = 128;              ///< V = dacref x ref / 256
    AcHysteresis hysteresis = AcHysteresis::none;
    AcPower power = AcPower::fast;
    bool invert = false;
    bool init_high = false;            ///< INITVAL: the output until the AC is ready
    bool output_pin = false;           ///< OUTEN: PA7 (or PC6 with alt_pin), driven as output
    bool alt_pin = false;
    bool run_standby = false;
    AcSense sense = AcSense::both;
};

/// The DACREF code nearest to `mv` for a reference of `ref_mv`
/// millivolts (255 at and above the reference).
constexpr uint8_t ac_dacref_code(uint16_t mv, uint16_t ref_mv) {
    if (ref_mv == 0) return 0;
    const uint32_t c = (static_cast<uint32_t>(mv) * 256u + ref_mv / 2) / ref_mv;
    return c > 255 ? 255 : static_cast<uint8_t>(c);
}
constexpr uint16_t ac_dacref_mv(uint8_t code, uint16_t ref_mv) {
    return static_cast<uint16_t>((static_cast<uint32_t>(code) * ref_mv + 128) / 256);
}

/// The pin (port letter, pin number) of an input on this package
/// (DS40002247B ch. 3, 48-pin column). {0, 0} = none.
struct AcPin { char port; uint8_t pin; };
constexpr AcPin ac_pos_pin(uint8_t n, AcPos p) {
    switch (p) {
        case AcPos::ainp0: return {'D', 2};
        case AcPos::ainp1: return n == 0 ? AcPin{'E', 0} : n == 1 ? AcPin{'D', 3} : AcPin{'D', 4};
        case AcPos::ainp2: return n == 0 ? AcPin{'E', 2} : n == 1 ? AcPin{'D', 4} : AcPin{'E', 1};
        case AcPos::ainp3: return {'D', 6};
    }
    return {0, 0};
}
constexpr AcPin ac_neg_pin(uint8_t n, AcNeg p) {
    switch (p) {
        case AcNeg::ainn0: return n == 0 ? AcPin{'D', 3} : n == 1 ? AcPin{'D', 5} : AcPin{'D', 7};
        case AcNeg::ainn1: return {'D', 0};
        case AcNeg::ainn2: return {'D', 7};
        case AcNeg::dacref: return {0, 0};
    }
    return {0, 0};
}

// ---- the resource ---------------------------------------------------------------

template <uint8_t n>
class Ac {
    static_assert(n <= 2, "AVR DA/DB: AC0..AC2");

public:
    Ac() = delete;

    static constexpr uint8_t index = n;
    using OutDefault = Pin<'A', 7>;
    using OutAlt = Pin<'C', 6>;
    using OutEvent = EvAcOut<n>;      ///< generator: the comparator output level

    template <AcConfig cfg>
    static void init() { init(cfg); }

    /// Disables, sets the shared reference, turns the chosen pins into
    /// analog inputs, writes MUX/DACREF/sense/route, enables. The
    /// interrupt stays off (enable_interrupt).
    static void init(const AcConfig& cfg) {
        auto& a = regs();
        a.CTRLA = 0;
        a.INTCTRL = 0;
        if (cfg.negative == AcNeg::dacref) Vref::ac(cfg.reference, cfg.reference_always_on);
        analog_input(ac_pos_pin(n, cfg.positive));
        analog_input(ac_neg_pin(n, cfg.negative));
        a.MUXCTRL = static_cast<uint8_t>(
            static_cast<uint8_t>(cfg.positive) | static_cast<uint8_t>(cfg.negative) |
            (cfg.invert ? AC_INVERT_bm : 0) | (cfg.init_high ? AC_INITVAL_bm : 0));
        a.DACREF = cfg.dacref;
        a.CTRLB = AC_WINSEL_DISABLED_gc;
        a.INTCTRL = static_cast<uint8_t>(cfg.sense);        // sense without enable
        route(cfg.alt_pin);
        if (cfg.output_pin) {
            if (cfg.alt_pin) OutAlt::output(); else OutDefault::output();
        }
        a.STATUS = AC_CMPIF_bm;
        a.CTRLA = static_cast<uint8_t>(
            AC_ENABLE_bm | static_cast<uint8_t>(cfg.hysteresis) | static_cast<uint8_t>(cfg.power) |
            (cfg.output_pin ? AC_OUTEN_bm : 0) | (cfg.run_standby ? AC_RUNSTDBY_bm : 0));
    }

    static void enable() { regs().CTRLA |= AC_ENABLE_bm; }
    static void disable() { regs().CTRLA &= static_cast<uint8_t>(~AC_ENABLE_bm); }

    /// The comparator output (after INVERT): positive above negative.
    static bool state() { return (regs().STATUS & AC_CMPSTATE_bm) != 0; }

    /// DACREF as a code, or as millivolts of the reference named at init.
    static void dacref(uint8_t code) { regs().DACREF = code; }
    static void threshold_mv(uint16_t mv, uint16_t ref_mv) { regs().DACREF = ac_dacref_code(mv, ref_mv); }

    // ---- interrupt ----------------------------------------------------------
    static void sense(AcSense s) { regs().INTCTRL = static_cast<uint8_t>((regs().INTCTRL & AC_CMP_bm) | static_cast<uint8_t>(s)); }
    static void enable_interrupt(bool on) {
        if (on) regs().INTCTRL |= AC_CMP_bm; else regs().INTCTRL &= static_cast<uint8_t>(~AC_CMP_bm);
    }
    static bool flag() { return (regs().STATUS & AC_CMPIF_bm) != 0; }
    static void clear_flag() { regs().STATUS = AC_CMPIF_bm; }
    /// ISR body for ACn_AC_vect: the state now, the flag cleared.
    [[gnu::always_inline]] static bool cmp() {
        regs().STATUS = AC_CMPIF_bm;
        return state();
    }

    // ---- window -------------------------------------------------------------
    /// Pair this comparator (the LOWER limit on its negative input) with
    /// AC n+1 or n+2 (the UPPER limit on its negative input, the same
    /// positive pin): the interrupt/event fires for `when`.
    template <uint8_t partner>
    static void window(AcWindowSense when) {
        static_assert(partner == n + 1 || partner == n + 2, "window partner: ACn+1 or ACn+2");
        static_assert(partner <= 2, "no such comparator");
        regs().INTCTRL = static_cast<uint8_t>((regs().INTCTRL & AC_CMP_bm) | static_cast<uint8_t>(when));
        regs().CTRLB = partner == n + 1 ? AC_WINSEL_UPSEL1_gc : AC_WINSEL_UPSEL2_gc;
    }
    static void window_off() { regs().CTRLB = AC_WINSEL_DISABLED_gc; }
    static AcWindowState window_state() { return static_cast<AcWindowState>(regs().STATUS >> 6); }

    static constexpr AC_t& regs() {
        if constexpr (n == 0) return AC0;
        else if constexpr (n == 1) return AC1;
        else return AC2;
    }

private:
    static void route(bool alt) {
        constexpr uint8_t bit = static_cast<uint8_t>(1u << n);
        if (alt) PORTMUX.ACROUTEA |= bit; else PORTMUX.ACROUTEA &= static_cast<uint8_t>(~bit);
    }
    /// Digital input buffer off on an analog input pin (nothing for {0,0}).
    static void analog_input(AcPin p) {
        if (p.port == 0) return;
        volatile uint8_t& ctrl = pinctrl_of(p.port, p.pin);
        ctrl = static_cast<uint8_t>((ctrl & ~PORT_ISC_gm) | PORT_ISC_INPUT_DISABLE_gc);
    }
};

// ---- tasks ----------------------------------------------------------------------

/// Threshold<Ac>: "the input on `pin` crossed `mv`" - DACREF from
/// millivolts of the reference, hysteresis, an interrupt on the chosen
/// crossing(s). ISR body: crossed() returns the new state (true =
/// above). The event OutEvent carries the level for the rest of the
/// chip (a CCL, a timer capture on the crossing).
template <typename A>
struct Threshold {
    Threshold() = delete;

    static bool init(AcPos pin, uint16_t mv, Ref ref = Ref::v1024, uint16_t ref_known_mv = 0,
                     AcHysteresis hyst = AcHysteresis::medium, AcSense sense = AcSense::both,
                     bool interrupt = true) {
        ref_mv_ = ref_mv(ref, ref_known_mv);
        if (ref_mv_ == 0 || mv > ref_mv_) return false;
        A::init({.positive = pin, .negative = AcNeg::dacref, .reference = ref,
                 .dacref = ac_dacref_code(mv, ref_mv_), .hysteresis = hyst, .sense = sense});
        A::enable_interrupt(interrupt);
        return true;
    }
    /// Move the threshold (settling applies).
    static void set_mv(uint16_t mv) { A::threshold_mv(mv, ref_mv_); }
    static uint16_t mv() { return ac_dacref_mv(A::regs().DACREF, ref_mv_); }
    static bool above() { return A::state(); }
    [[gnu::always_inline]] static bool crossed() { return A::cmp(); }

private:
    static inline uint16_t ref_mv_ = 0;
};

/// Window<Lower, Upper>: the input on `pin` against two DACREF levels -
/// Lower (ACn) holds the lower limit, Upper (ACn+1 or ACn+2) the upper;
/// state() reads above / inside / below; the interrupt (on Lower's
/// vector) and Lower's event fire for `when`. ISR body: changed().
template <typename Lower, typename Upper>
struct Window {
    Window() = delete;
    static_assert(Upper::index == Lower::index + 1 || Upper::index == Lower::index + 2,
                  "Window: the upper comparator is ACn+1 or ACn+2 of the lower");

    static bool init(AcPos pin, uint16_t low_mv, uint16_t high_mv, AcWindowSense when,
                     Ref ref = Ref::v1024, uint16_t ref_known_mv = 0,
                     AcHysteresis hyst = AcHysteresis::medium, bool interrupt = true) {
        const uint16_t r = ref_mv(ref, ref_known_mv);
        if (r == 0 || high_mv > r || low_mv > high_mv) return false;
        Upper::init({.positive = pin, .negative = AcNeg::dacref, .reference = ref,
                     .dacref = ac_dacref_code(high_mv, r), .hysteresis = hyst});
        Lower::init({.positive = pin, .negative = AcNeg::dacref, .reference = ref,
                     .dacref = ac_dacref_code(low_mv, r), .hysteresis = hyst});
        Lower::template window<Upper::index>(when);
        Lower::enable_interrupt(interrupt);
        return true;
    }
    static AcWindowState state() { return Lower::window_state(); }
    [[gnu::always_inline]] static AcWindowState changed() {
        Lower::clear_flag();
        return Lower::window_state();
    }
};

} // namespace brio
