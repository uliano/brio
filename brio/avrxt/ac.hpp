/*
 * ac.hpp
 *
 * The analog comparators (AC) of the AVR DA/DB (DS40002247B ch. 32) and
 * the AVR EA (DS40002443A ch. 30) as brio sees them - one IP, the EA's
 * with two instances, a fifth positive and a fourth negative pin, two
 * power profiles and the window pairing ACn+1 only - see
 * docs/avrxt/ac.md:
 *
 *  RESOURCE - Ac<n>, n = 0..2 (0..1 on the EA): a config struct owns the
 *  inputs (one of four positive pins - five on the EA - one of three
 *  negative pins - four on the EA - or DACREF), the DACREF
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
 * Facts that shape the code (DS40002247B 32.3, 39.17; DS40002443A 30.3,
 * 35.18; no AC silicon erratum on either - the DB errata document's
 * clarification 3.7.4 adds tDACREF = 25 us, the EA's DS80001048D 3.8
 * fills table 35-27):
 *  - V_DACREF = DACREF x V_ACREF / 256 (8-bit); offset +-5 mV typ.,
 *    hysteresis 0 / 10 / 25 / 50 mV, response 85 / 250 / 460 ns by
 *    power profile on the DB; offset +-10 typ. and +-30 mV max,
 *    hysteresis 0 / 10 / 20 / 30 mV, response 50 ns rising and 150 ns
 *    falling at profile 0 on the EA; the AC and (if internal) the reference take their
 *    start-up after enable, a new input pin or reference wants settling
 *    - INITVAL holds the output meanwhile;
 *  - inputs on the 48-pin part: AINP0 PD2 (all), AINP1 PE0/PD3/PD4,
 *    AINP2 PE2/PD4/PE1, AINP3 PD6 (all: the DAC pin), AINN0 PD3/PD5/PD7,
 *    AINN1 PD0 (all), AINN2 PD7 (all) for AC0/AC1/AC2; on the EA also
 *    AINP4 PC3 and AINN3 PC2 (both instances, every package); the pins'
 *    digital input buffer is disabled by init();
 *  - the output: CMPSTATE in STATUS, the OUT event (level, async, 0x20
 *    + n), the pin (OUTEN: PA7 default - CLKOUT's pin - or ALT1 PC6, one
 *    route bit per instance in PORTMUX.ACROUTEA), the CCL input menu
 *    (ACn OUT on LUT input n; input 2 carries AC1 on the EA, which has no
 *    AC2 - ccl.hpp), one interrupt per instance (CMP) with
 *    sense both / falling / rising;
 *  - window mode: ACn pairs with ACn+1 or ACn+2 (WINSEL; ACn+1 only on
 *    the EA, which has no UPSEL2), both MUXPOS on
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

#include "avrxt/device_tables.hpp"
#include "avrxt/evsys.hpp"
#include "avrxt/pin.hpp"
#include "avrxt/vref.hpp"
#include "util/analog.hpp"

namespace brio {

// ---- the knobs (32.5) -------------------------------------------------------------

/// AINP4 and AINN3 carry the device tables' probe of the header's
/// enumerator: 0xFF where the part has no such pin (the AVR DA/DB), and a
/// config naming one is refused there.
enum class AcPos : uint8_t {
    ainp0 = AC_MUXPOS_AINP0_gc, ainp1 = AC_MUXPOS_AINP1_gc,
    ainp2 = AC_MUXPOS_AINP2_gc, ainp3 = AC_MUXPOS_AINP3_gc,
    ainp4 = ac_ainp4_code,
};
enum class AcNeg : uint8_t {
    ainn0 = AC_MUXNEG_AINN0_gc, ainn1 = AC_MUXNEG_AINN1_gc,
    ainn2 = AC_MUXNEG_AINN2_gc, dacref = AC_MUXNEG_DACREF_gc,
    ainn3 = ac_ainn3_code,
};
enum class AcHysteresis : uint8_t {
    none = AC_HYSMODE_NONE_gc, small = AC_HYSMODE_SMALL_gc,       ///< ~10 mV
    medium = AC_HYSMODE_MEDIUM_gc, large = AC_HYSMODE_LARGE_gc,   ///< ~25 / ~50 mV
};
/// The third profile's code is the device tables' probe of the header's
/// enumerator (0xFF where the part has no third profile, and a config
/// asking for it is refused there).
enum class AcPower : uint8_t {
    fast = AC_POWER_PROFILE0_gc,      ///< ~85 ns, most current
    medium = AC_POWER_PROFILE1_gc,    ///< ~250 ns, 17 uA
    low = ac_power_profile2_code,     ///< ~460 ns
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
/// millivolts (255 at and above the reference). Thin names over the
/// one home of counts<->mV arithmetic (util/analog.hpp, host-tested);
/// the DACREF is an 8-bit divider, steps = 256.
constexpr uint8_t ac_dacref_code(uint16_t mv, uint16_t ref_mv) {
    return static_cast<uint8_t>(dac_code(mv, 256, ref_mv));
}
constexpr uint16_t ac_dacref_mv(uint8_t code, uint16_t ref_mv) {
    return dac_mv(code, 256, ref_mv);
}

/// The pin (port letter, pin number) of an input on this package
/// (DS40002247B ch. 3, 48-pin column). {0, 0} = none.
struct AcPin { char port; uint8_t pin; };
constexpr AcPin ac_pos_pin(uint8_t n, AcPos p) {
    if constexpr (ac_ainp4_code != 0xFF) {
        if (p == AcPos::ainp4) return {'C', 3};
    }
    switch (p) {
        case AcPos::ainp0: return {'D', 2};
        case AcPos::ainp1: return n == 0 ? AcPin{'E', 0} : n == 1 ? AcPin{'D', 3} : AcPin{'D', 4};
        case AcPos::ainp2: return n == 0 ? AcPin{'E', 2} : n == 1 ? AcPin{'D', 4} : AcPin{'E', 1};
        case AcPos::ainp3: return {'D', 6};
        default: break;                   // AINP4 above, where the part has it
    }
    return {0, 0};
}
constexpr AcPin ac_neg_pin(uint8_t n, AcNeg p) {
    if constexpr (ac_ainn3_code != 0xFF) {
        if (p == AcNeg::ainn3) return {'C', 2};
    }
    switch (p) {
        case AcNeg::ainn0: return n == 0 ? AcPin{'D', 3} : n == 1 ? AcPin{'D', 5} : AcPin{'D', 7};
        case AcNeg::ainn1: return {'D', 0};
        case AcNeg::ainn2: return {'D', 7};
        case AcNeg::dacref: return {0, 0};
        default: break;                   // AINN3 above, where the part has it
    }
    return {0, 0};
}

/// Are the selected inputs' pins on a port this package bonds? The
/// PORTE positives (AC0 AINP1/AINP2, AC2 AINP2) do not exist on
/// 28/32-pin parts and init refuses them. Pin-level absences within a
/// bonded port (PD0 = AINN1 on DB 28/32, PC6 = the ALT1 output on
/// 28/32) await the device tables - the table above is the 48/64-pin
/// column, identical on the three dies. AINP4 and AINN3 map to no pin on
/// a part without them (the DA/DB), so the same test refuses them.
constexpr bool ac_config_valid(uint8_t n, const AcConfig& c) {
    if (c.power == AcPower::low && !has_ac_profile2) return false;
    if (!port_exists(ac_pos_pin(n, c.positive).port)) return false;
    if (c.negative != AcNeg::dacref &&
        !port_exists(ac_neg_pin(n, c.negative).port)) return false;
    return true;
}

// ---- the resource ---------------------------------------------------------------

template <uint8_t n>
class Ac {
    static_assert(n < ac_count, "no such analog comparator on this part (AVR DA/DB: AC0..AC2, AVR EA: AC0, AC1)");

public:
    Ac() = delete;

    static constexpr uint8_t index = n;
    using OutDefault = Pin<'A', 7>;
    using OutAlt = Pin<'C', 6>;
    using OutEvent = EvAcOut<n>;      ///< generator: the comparator output level

    template <AcConfig cfg>
    static void init() {
        static_assert(ac_config_valid(n, cfg),
                      "this package does not bond the selected input's pin "
                      "(the PORTE positives exist on 48/64-pin parts only), or this "
                      "part has no third power profile for AcPower::low, or no "
                      "AINP4/AINN3 (the AVR EA's PC3/PC2)");
        (void)init(cfg);
    }

    /// Disables, sets the shared reference, turns the chosen pins into
    /// analog inputs, writes MUX/DACREF/sense/route, enables. The
    /// interrupt stays off (enable_interrupt). False, touching
    /// nothing, when an input's pin is not bonded on this package.
    static bool init(const AcConfig& cfg) {
        if (!ac_config_valid(n, cfg)) return false;
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
        return true;
    }

    static void enable() { regs().CTRLA |= AC_ENABLE_bm; }
    static void disable() { regs().CTRLA &= static_cast<uint8_t>(~AC_ENABLE_bm); }

    /// The comparator output, after INVERT (without it: positive above
    /// negative; with it: the opposite).
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
        static_assert(partner < ac_count, "no such comparator");
        static_assert(partner == n + 1 || has_ac_upsel2,
                      "this part's window pairs a comparator with ACn+1 only (no UPSEL2)");
        regs().INTCTRL = static_cast<uint8_t>((regs().INTCTRL & AC_CMP_bm) | static_cast<uint8_t>(when));
        regs().CTRLB = partner == n + 1 ? static_cast<uint8_t>(AC_WINSEL_UPSEL1_gc)
                                        : ac_winsel_upsel2_code;
    }
    static void window_off() { regs().CTRLB = AC_WINSEL_DISABLED_gc; }
    static AcWindowState window_state() { return static_cast<AcWindowState>(regs().STATUS >> 6); }

    static constexpr AC_t& regs() {
        if constexpr (n == 0) return AC0;
#if defined(AC2)
        else if constexpr (n == 1) return AC1;
        else return AC2;
#else
        else return AC1;
#endif
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
        if (!A::init({.positive = pin, .negative = AcNeg::dacref, .reference = ref,
                      .dacref = ac_dacref_code(mv, ref_mv_), .hysteresis = hyst, .sense = sense}))
            return false;
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
        (void)Upper::init({.positive = pin, .negative = AcNeg::dacref, .reference = ref,
                     .dacref = ac_dacref_code(high_mv, r), .hysteresis = hyst});
        (void)Lower::init({.positive = pin, .negative = AcNeg::dacref, .reference = ref,
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
