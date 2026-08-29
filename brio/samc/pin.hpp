/*
 * pin.hpp
 *
 * The SAM C21 I/O pins (PORT, DS60001479M ch. 28), register-level, in
 * the same two faces avrdx/pin.hpp offers:
 *
 *  Port<'B'>       the port RESOURCE - one GROUP of the PORT peripheral:
 *                  32-bit mask operations on DIR/OUT/IN and the
 *                  multi-pin configuration engine (WRCONFIG);
 *  Pin<'B', 23>    the per-pin face: direction and value, configure()
 *                  as one PINCFG store, the peripheral-function handoff
 *                  (PMUX), a PwmChannel (max 1) and a PinRef factory.
 *
 *   using Led = brio::Pin<'B', 23>;   // PB23
 *   Led::output();
 *   Led::toggle();
 *   Led::input(brio::PinPull::up);
 *   Rx::function(brio::PinFunction::d, {.input_enable = true});  // SERCOM
 *
 * TWO FACTS THAT DIFFER FROM AVR and shape everything below.
 *
 * 1. THE INPUT BUFFER IS OFF BY DEFAULT. PINCFG.INEN gates it, and IN
 *    reads 0 for a pin whose buffer is disabled - including a pin this
 *    program is driving. So input() turns it on, and output() turns it on
 *    too: read() then means the same thing here as on every other brio
 *    target. A pin whose microamps matter parks it with
 *    configure({}) (all of PINCFG cleared).
 * 2. THERE IS NO PIN INTERRUPT IN PORT. The edge/level senses AVR keeps
 *    in PINnCTRL are a separate peripheral here (EIC, ch. 26), reached
 *    through PMUX function A. Hence no PinSense vocabulary and no
 *    take_flags() in this file: an EIC driver will own them, and
 *    inventing half of one here would be a promise with no code behind
 *    it. What PORT does have is the OPPOSITE direction: it is an event
 *    USER (EVCTRL, 28.6.4), four inputs per group that set, clear,
 *    toggle or FOLLOW one pin each - and the follow ("out") action is
 *    the only one that survives a standby, which is what makes a pad
 *    move while the CPU is stopped.
 *
 * ONE PACKAGE FACT, EXHAUSTIVELY. This family has exactly TWO port
 * groups, A and B, on every variant (E/G/J alike): the device header's
 * PORT_GROUPS says 2 and there is no third group anywhere in the pack.
 * port_exists() below reads that number rather than listing letters, so
 * it stays true if a relative ever grows a group C. Which PINS of an
 * existing group are bonded on a given package is a FINER question - a
 * device-table job, open here exactly as it is open on AVR.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "util/pwm_channel.hpp"

namespace brio {

/// Whether this device bonds out the port group of a letter at all.
/// PORT_GROUPS is the device header's own count (the authority), so no
/// #ifdef ladder is needed and none can go stale. Group-level only: a
/// pin missing from an EXISTING group is the per-package device tables'
/// job, not built for any target yet.
constexpr bool port_exists(char p) {
    const int group = p - 'A';
    return group >= 0 && group < static_cast<int>(PORT_GROUPS);
}

/// Runtime pin descriptor: lets a pin chosen at compile time travel
/// inside a request event (the CS/DC of a SPI transaction, asserted by
/// the bus AO and not by its client). A null PinRef (the default) means
/// "no such pin": set/clear are no-ops, so an optional pin costs one
/// branch. Build one with Pin<...>::ref().
struct PinRef {
    port_group_registers_t* group = nullptr;
    uint32_t mask = 0;

    void set() const {
        if (group != nullptr) {
            group->PORT_OUTSET = mask;
        }
    }
    void clear() const {
        if (group != nullptr) {
            group->PORT_OUTCLR = mask;
        }
    }
    constexpr bool valid() const { return group != nullptr; }
};

// ---- pin configuration vocabulary (28.8.13, 28.8.14) ------------------------

/// The internal pull. PULLEN switches it on; its DIRECTION is the pin's
/// OUT bit while the pin is an input (28.6.3.2), which is why the pull
/// is one enum here and not a bool: the two registers must agree, and
/// only the driver can guarantee that they do.
enum class PinPull : uint8_t { none, up, down };

/// PMUX function codes A..I (28.8.13). Which function a given pad
/// actually offers is a table of the pin, published in the datasheet's
/// I/O multiplexing chapter and in the device header's PIN_<pad><fn>_*
/// symbols - a peripheral driver's route table names it, this enum only
/// spells the selector.
enum class PinFunction : uint8_t {
    a = PORT_PMUX_PMUXE_A_Val,
    b = PORT_PMUX_PMUXE_B_Val,
    c = PORT_PMUX_PMUXE_C_Val,
    d = PORT_PMUX_PMUXE_D_Val,
    e = PORT_PMUX_PMUXE_E_Val,
    f = PORT_PMUX_PMUXE_F_Val,
    g = PORT_PMUX_PMUXE_G_Val,
    h = PORT_PMUX_PMUXE_H_Val,
    i = PORT_PMUX_PMUXE_I_Val,
};

// ---- the PORT as an event USER (28.6.4, EVCTRL 28.8.12) ---------------------

/// EVCTRL.EVACTm: what an incoming event does to the pin it names.
///
/// `out` is the one action with NO clock in its path - 28.6.4 says the
/// output event "is sent to the pin without any internal latency",
/// where set/clear/toggle "will be executed up to three clock cycles
/// after a rising edge" - and it is therefore the ONLY action the same
/// section says survives a standby: "In Standby mode, only the Out
/// action is possible, and the Set, Clear, and Toggle actions are not
/// available."
enum class PortEventAction : uint8_t {
    out = PORT_EVCTRL_EVACT0_OUT_Val,      ///< the pin FOLLOWS the event level
    set = PORT_EVCTRL_EVACT0_SET_Val,      ///< set OUT on a rising event edge
    clear = PORT_EVCTRL_EVACT0_CLR_Val,    ///< clear OUT on a rising event edge
    toggle = PORT_EVCTRL_EVACT0_TGL_Val,   ///< toggle OUT on a rising event edge
};

/// One of the four event inputs of one PORT group: which pin of the
/// group it addresses (PIDm), what it does (EVACTm), and whether it is
/// listening at all (PORTEIm). Each input addresses exactly one pin;
/// one pin may be addressed by up to four inputs, and table 28-3 is the
/// priority when several act at once.
struct PortEventConfig {
    uint8_t pin = 0;                                 ///< PIDm, 0..31 within the group
    PortEventAction action = PortEventAction::out;   ///< EVACTm
    bool enable = false;                             ///< PORTEIm
};

/// PID is five bits, so only the pin number can be out of range.
constexpr bool port_event_config_valid(const PortEventConfig& c) {
    return c.pin < 32u;
}

/// The whole PINCFG as one value. PMUXEN is NOT here: it is decided by
/// WHICH verb is called (configure() gives the pad to PORT, function()
/// hands it to a peripheral), never by a flag a caller could set
/// inconsistently with the PMUX nibble.
struct PinConfig {
    bool input_enable = false;   ///< INEN: without it IN always reads 0
    PinPull pull = PinPull::none;
    bool strong_drive = false;   ///< DRVSTR: the high-drive output stage
};

constexpr uint8_t pin_cfg_byte(const PinConfig& c, bool peripheral) {
    return static_cast<uint8_t>((peripheral ? PORT_PINCFG_PMUXEN_Msk : 0u) |
                                (c.input_enable ? PORT_PINCFG_INEN_Msk : 0u) |
                                (c.pull != PinPull::none ? PORT_PINCFG_PULLEN_Msk : 0u) |
                                (c.strong_drive ? PORT_PINCFG_DRVSTR_Msk : 0u));
}

// ---- the port resource ------------------------------------------------------

/// Port<'B'>: one GROUP of the PORT peripheral - the mask operations a
/// single pin cannot express (a whole bus of select lines moving in one
/// store) and the multi-pin configuration engine. Pin<letter, n> is the
/// per-pin face and delegates its register access here.
template <char L>
struct Port {
    Port() = delete;
    static_assert(port_exists(L), "this port group does not exist on this device");

    static constexpr char letter = L;
    static constexpr uint8_t group = static_cast<uint8_t>(L - 'A');

    static port_group_registers_t& regs() { return PORT_REGS->GROUP[group]; }

    // Mask operations (bit n = pin n of this group).
    static uint32_t in() { return regs().PORT_IN; }
    static uint32_t dir() { return regs().PORT_DIR; }
    static uint32_t out() { return regs().PORT_OUT; }
    static void dir_set(uint32_t m) { regs().PORT_DIRSET = m; }
    static void dir_clear(uint32_t m) { regs().PORT_DIRCLR = m; }
    static void dir_toggle(uint32_t m) { regs().PORT_DIRTGL = m; }
    static void out_set(uint32_t m) { regs().PORT_OUTSET = m; }
    static void out_clear(uint32_t m) { regs().PORT_OUTCLR = m; }
    static void out_toggle(uint32_t m) { regs().PORT_OUTTGL = m; }

    /// Multi-pin configuration (WRCONFIG, 28.8.11): one PINCFG (and, for
    /// function(), one PMUX nibble) into every pin of `pins` through
    /// WRCONFIG - the SAM analog of AVR's PINCTRLUPD engine. WRCONFIG's
    /// pin mask is 16 bits wide with a half-word selector, so a mask
    /// spanning both halves costs the two stores it must.
    static void configure_mask(uint32_t pins, const PinConfig& cfg) {
        write_config(pins, pin_cfg_byte(cfg, false), 0, false);
        apply_pull(pins, cfg.pull);
    }

    /// The same, handing the pins to a peripheral function.
    static void function_mask(uint32_t pins, PinFunction fn, const PinConfig& cfg) {
        write_config(pins, pin_cfg_byte(cfg, true), static_cast<uint8_t>(fn), true);
        apply_pull(pins, cfg.pull);
    }

    // ---- the group as an event USER (28.6.4) --------------------------------
    //
    // Four inputs per group, each addressing one pin. THE EVSYS USER
    // INDICES ARE PUBLISHED HERE and the wiring is the caller's, per the
    // rule samc/evsys.hpp states: that driver owns the fabric, every
    // peripheral owns its own vocabulary. Table 29-3 marks all four of
    // these users ASYNCHRONOUS PATH ONLY - an obligation on the caller's
    // EventChannelConfig, which this file cannot see and does not
    // pretend to check.

    static constexpr uint8_t event_input_count = PORT_EV_NUM;

    /// The EVSYS user index of event input `m` of THIS group. The four
    /// PORT users are a property of the peripheral and not of the group:
    /// the header declares exactly four, so both groups' EVCTRL
    /// registers are reached through the same four users, and which
    /// group a user acts on is decided by which group's EVCTRL has
    /// PORTEIm set.
    static constexpr uint8_t event_user(uint8_t m) {
        return static_cast<uint8_t>(EVENT_ID_USER_PORT_EV_0 + m);
    }

    static uint32_t evctrl() { return regs().PORT_EVCTRL; }

    /// Read back input `m`.
    static PortEventConfig event_config(uint8_t m) {
        if (m >= event_input_count) {
            return {};
        }
        const uint32_t shift = static_cast<uint32_t>(m) * 8u;
        const uint32_t field = (evctrl() >> shift) & 0xFFu;
        return PortEventConfig{
            .pin = static_cast<uint8_t>(field & 0x1Fu),
            .action = static_cast<PortEventAction>((field >> 5) & 0x3u),
            .enable = (field & 0x80u) != 0u,
        };
    }

    /// Write input `m`, leaving the other three alone. False - and
    /// nothing written - for an input index or a pin number outside the
    /// register.
    static bool configure_event(uint8_t m, const PortEventConfig& c) {
        if (m >= event_input_count || !port_event_config_valid(c)) {
            return false;
        }
        const uint32_t shift = static_cast<uint32_t>(m) * 8u;
        const uint32_t field = static_cast<uint32_t>(c.pin) |
                               (static_cast<uint32_t>(c.action) << 5) |
                               (c.enable ? 0x80u : 0u);
        regs().PORT_EVCTRL =
            (evctrl() & ~(0xFFu << shift)) | (field << shift);
        return true;
    }

    /// The compile-time twin, so a pin number outside the group is a
    /// compile error rather than a false at run time.
    template <uint8_t m, PortEventConfig c>
    static bool configure_event() {
        static_assert(m < event_input_count,
                      "brio Port: this family has four PORT event inputs per group "
                      "(EVCTRL, 28.8.12)");
        static_assert(port_event_config_valid(c),
                      "brio Port: EVCTRL.PIDm is five bits - a PORT group has 32 pins");
        return configure_event(m, c);
    }

    /// Stop listening on all four inputs of this group.
    static void release_events() { regs().PORT_EVCTRL = 0; }

private:
    /// The pull direction lives in OUT, so it is written only when a
    /// pull is actually asked for: an unconditional store would change
    /// the level of every pin of the mask that happens to be an output.
    static void apply_pull(uint32_t pins, PinPull pull) {
        if (pull == PinPull::up) {
            out_set(pins);
        } else if (pull == PinPull::down) {
            out_clear(pins);
        }
    }

    /// WRPMUX is set only when a function is actually being selected:
    /// configure_mask() leaves the stale PMUX nibbles alone, exactly as
    /// Pin::configure() does, so the two verbs disturb the same state.
    static void write_config(uint32_t pins, uint8_t cfg, uint8_t pmux, bool with_pmux) {
        const uint32_t common = (static_cast<uint32_t>(cfg) << 16) |
                                PORT_WRCONFIG_PMUX(pmux) |
                                PORT_WRCONFIG_WRPINCFG_Msk |
                                (with_pmux ? PORT_WRCONFIG_WRPMUX_Msk : 0u);
        if ((pins & 0xFFFFu) != 0u) {
            regs().PORT_WRCONFIG = common | PORT_WRCONFIG_PINMASK(pins & 0xFFFFu);
        }
        if ((pins >> 16) != 0u) {
            regs().PORT_WRCONFIG =
                common | PORT_WRCONFIG_HWSEL_Msk | PORT_WRCONFIG_PINMASK(pins >> 16);
        }
    }

    // The PINCFG bits sit at the same offsets in WRCONFIG's upper half
    // (PMUXEN 16, INEN 17, PULLEN 18, DRVSTR 22) as in PINCFG itself
    // (0, 1, 2, 6) - which is why write_config() can shift the byte
    // whole instead of taking the register apart field by field.
    static_assert(PORT_WRCONFIG_PMUXEN_Pos == PORT_PINCFG_PMUXEN_Pos + 16);
    static_assert(PORT_WRCONFIG_INEN_Pos == PORT_PINCFG_INEN_Pos + 16);
    static_assert(PORT_WRCONFIG_PULLEN_Pos == PORT_PINCFG_PULLEN_Pos + 16);
    static_assert(PORT_WRCONFIG_DRVSTR_Pos == PORT_PINCFG_DRVSTR_Pos + 16);
};

// ---- the pin ----------------------------------------------------------------

template <char PortLetter, uint8_t PinNum>
struct Pin {
    static_assert(port_exists(PortLetter), "this port group does not exist on this device");
    static_assert(PinNum < 32, "a PORT group has 32 pins");

    static constexpr char port_letter = PortLetter;
    static constexpr uint8_t pin_number = PinNum;
    static constexpr uint32_t mask = 1u << PinNum;

    static port_group_registers_t& port() { return Port<PortLetter>::regs(); }

    /// This pin's PINCFG byte.
    static volatile uint8_t& pincfg() { return port().PORT_PINCFG[PinNum]; }

    /// Runtime descriptor of this pin (for request events - see PinRef).
    static PinRef ref() { return {&port(), mask}; }

    // A pin is the degenerate PwmChannel (max = 1: any non-zero duty is
    // "on"), so generic actuators written over that concept drive bare
    // pins and timer channels with the same code.
    static constexpr uint16_t max = 1;
    static void duty(uint16_t v) { if (v) set(); else clear(); }

    // Basic I/O. The single-bit registers make every one of these a
    // plain store: no read-modify-write, nothing to guard against a
    // handler touching another pin of the same group.
    static void toggle() { port().PORT_OUTTGL = mask; }
    static void set()    { port().PORT_OUTSET = mask; }
    static void clear()  { port().PORT_OUTCLR = mask; }
    static bool read()   { return (port().PORT_IN & mask) != 0u; }
    static bool is_output() { return (port().PORT_DIR & mask) != 0u; }

    /// Drive the pin. The input buffer is enabled with it, so read()
    /// reports what the pad is actually at - see the file header.
    static void output() {
        input_enable(true);
        port().PORT_DIRSET = mask;
    }

    /// Release the pin, buffer on, optionally with an internal pull.
    static void input(PinPull p = PinPull::none) {
        port().PORT_DIRCLR = mask;
        pull(p);
        input_enable(true);
    }

    /// The whole PINCFG in ONE store, the pad staying under PORT
    /// control (PMUXEN cleared). configure({}) is the "give this pin
    /// back, buffer off, no pull" reset.
    static void configure(const PinConfig& cfg) {
        pincfg() = pin_cfg_byte(cfg, false);
        this_pull(cfg.pull);
    }

    /// Hand the pad to a peripheral: PMUX nibble + PMUXEN, plus the rest
    /// of PINCFG in the same breath (a peripheral INPUT still needs
    /// INEN - the mux does not turn the buffer on).
    static void function(PinFunction fn, const PinConfig& cfg = {}) {
        volatile uint8_t& reg = port().PORT_PMUX[PinNum / 2];
        const uint8_t nibble = static_cast<uint8_t>(fn);
        if constexpr ((PinNum & 1u) == 0u) {
            reg = static_cast<uint8_t>((reg & ~PORT_PMUX_PMUXE_Msk) |
                                       PORT_PMUX_PMUXE(nibble));
        } else {
            reg = static_cast<uint8_t>((reg & ~PORT_PMUX_PMUXO_Msk) |
                                       PORT_PMUX_PMUXO(nibble));
        }
        pincfg() = pin_cfg_byte(cfg, true);
        this_pull(cfg.pull);
    }

    /// Take the pad back from its peripheral function (PMUXEN only:
    /// the stale PMUX nibble is harmless and re-stated by the next
    /// function() call).
    static void release() {
        pincfg() = static_cast<uint8_t>(pincfg() & ~PORT_PINCFG_PMUXEN_Msk);
    }
    static bool has_function() { return (pincfg() & PORT_PINCFG_PMUXEN_Msk) != 0u; }

    /// INEN alone, keeping the rest of PINCFG.
    static void input_enable(bool on) {
        const uint8_t v = pincfg();
        pincfg() = static_cast<uint8_t>(on ? (v | PORT_PINCFG_INEN_Msk)
                                           : (v & ~PORT_PINCFG_INEN_Msk));
    }

    /// The internal pull alone: PULLEN plus the OUT bit that gives it a
    /// direction. Only effective while the pin is an input.
    static void pull(PinPull p) {
        const uint8_t v = pincfg();
        pincfg() = static_cast<uint8_t>(p == PinPull::none
                                            ? (v & ~PORT_PINCFG_PULLEN_Msk)
                                            : (v | PORT_PINCFG_PULLEN_Msk));
        this_pull(p);
    }

    /// DRVSTR alone: the stronger output driver.
    static void strong_drive(bool on) {
        const uint8_t v = pincfg();
        pincfg() = static_cast<uint8_t>(on ? (v | PORT_PINCFG_DRVSTR_Msk)
                                           : (v & ~PORT_PINCFG_DRVSTR_Msk));
    }

private:
    static void this_pull(PinPull p) {
        if (p == PinPull::up) {
            set();
        } else if (p == PinPull::down) {
            clear();
        }
    }
};

static_assert(PwmChannel<Pin<'A', 0>>);

} // namespace brio
