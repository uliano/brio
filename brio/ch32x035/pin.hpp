/*
 * pin.hpp
 *
 * Compile-time GPIO for the CH32X035: `Pin<'A', 2>` is a type, its port
 * and mask fold to constants, and every verb is a single register access
 * - or two, where the port's high byte has registers of its own.
 *
 * THE ENCODING IS THE F1's NIBBLE, CUT DOWN (RM 8.3.1.1). Four bits per
 * pin, CNF above MODE, and MODE is two bits - but on this series they
 * carry NO SPEED: 00 is input and 01, 10 and 11 are all "output mode".
 * CNF under an input is 00 analog, 01 floating, 10 pulled; under an
 * output it is 00 push-pull and 10 alternate push-pull ("I2C automatic
 * open drain"), and the manual gives no code for an open-drain output at
 * all - WCH's own library has no such mode for this series either. So a
 * pin here has no speed and no drive to choose, and the verbs below take
 * neither.
 *
 * TWENTY-FOUR PINS A PORT, OVER THREE CONFIGURATION REGISTERS. CFGLR
 * holds pins 0..7, CFGHR 8..15 and CFGXR 16..23; OUTDR, INDR and BCR are
 * 24 bits wide, but BSHR sets and resets pins 0..15 alone and BSXR does
 * the same for 16..23 (8.3.1.5, 8.3.1.9). A mask spanning both halves is
 * therefore TWO stores - each one atomic against a handler on another pin
 * of the same port, the pair not.
 *
 * CFGHR IS NEVER READ. WCH's peripheral library, on a die whose chip
 * identifier word (0x1FFFF704, device.hpp's chip_id_word()) has zero in
 * bits 7:4, does not read GPIOx_CFGHR: it keeps a copy of each port's in
 * RAM, starting from the reset value 0x44444444, and writes the register
 * WHOLE from that copy. The reference manual says nothing about it. This
 * file does the same on every die, which is correct whether or not the
 * register reads back: pins 8..15 are configured through a copy
 * (`Port::cfghr_copy()`), and `Port::cfghr_register()` is the raw read a
 * suite compares against it. The contract that follows: every store to
 * CFGHR goes through this file.
 *
 * PULLS HAVE NO REGISTER. An input with CNF = 10 is pulled, and the
 * DIRECTION comes from that pin's bit in OUTDR: 1 pulls up, 0 pulls down.
 * Every pad has the pull-up; only PA0..PA15, PC16 and PC17 have the
 * pull-down (8's opening, device.hpp's gpio_pull_down_pads), and a
 * pull-down asked of any other pad is refused. PC14..PC17 have more pull-up
 * strengths, set in the USB and USB PD blocks' own registers (AFIO_CTLR
 * for PC16 and PC17) - not here.
 *
 * THE BONDING IS THE PART'S, AND SO ARE THE SHORTS. A package brings out
 * some of each port's pads - `device::port_pins()` - and a Pin on a pad
 * the part does not bond does not compile. And on every package but the
 * QFN20 some PINS carry TWO pads shorted inside the chip (the datasheet's
 * table 2-1, notes 4 to 7: PC16 with PC11 and PC17 with PC10 on six parts,
 * PB1 with PB5 on the two 28-pin ones, PA12 with PC14 and PA13 with PC15
 * on the QSOP28, PA7 with PB0 on the CH32X033), of which the datasheet
 * says "both IOs are prohibited from being configured as output
 * functions". `device::port_twinned()` names them, and output() and
 * function() on one of them are REFUSED at compile time; input and analog
 * stay open.
 *
 * The port clock is opened by every configuring verb, the way the other
 * strata do it: a Pin that is configured works, with no separate step for
 * the caller to forget. The gate is written through RCC's own register
 * rather than through clock.hpp's `Rcc::enable`, because clock.hpp
 * INCLUDES this file (its clock-output pad is a pin).
 *
 * THE CONFIGURATION LOCK IS A ONE-WAY DOOR (RM 8.2.5, 8.3.1.7). A key
 * sequence in LCKR - LCKK at bit 24 - freezes the configuration of the
 * pins named in its low 24 bits "until the next reset". `Port::lock()` is
 * spelled at the PORT because the register is, and `Pin::lock()` is the
 * one-pin case.
 *
 * NOT HERE: the external interrupt lines and the remaps, which are AFIO's
 * (ch32x035/afio.hpp holds the remap field of every peripheral and the
 * USARTs' columns) - a pin is handed to "its" peripheral here, and WHICH
 * peripheral that is is the remap register's business.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"

namespace brio {

/// Input pull. The direction lives in OUTDR (see header).
enum class PinPull : uint8_t { none, up, down };

/// The nibbles this series has (8.3.1.1): the three inputs and the two
/// outputs, MODE 01 for an output as WCH's library writes it.
inline constexpr uint32_t pin_nibble_analog    = 0x0u;   ///< CNF 00, MODE 00
inline constexpr uint32_t pin_nibble_floating  = 0x4u;   ///< CNF 01, MODE 00
inline constexpr uint32_t pin_nibble_pulled    = 0x8u;   ///< CNF 10, MODE 00
inline constexpr uint32_t pin_nibble_output    = 0x1u;   ///< CNF 00, MODE 01
inline constexpr uint32_t pin_nibble_alternate = 0x9u;   ///< CNF 10, MODE 01

/// Does a nibble drive the pad (MODE non-zero)?
constexpr bool pin_nibble_drives(uint32_t nibble) { return (nibble & 0x3u) != 0u; }

/// The reset value of every configuration register: eight floating inputs.
inline constexpr uint32_t gpio_cfg_reset = 0x44444444UL;

/// A pad named at compile time - port letter and pin number - for the pin
/// tables a driver carries (which pads a peripheral's signals sit on).
/// `valid()` false is "no such pad": a signal a column does not have.
struct Pad {
    char port = 0;
    uint8_t pin = 0;

    constexpr bool valid() const { return port != 0; }
    constexpr bool operator==(const Pad&) const = default;
};

/// Does THIS package bring that pad out?
constexpr bool pad_bonded(Pad pad) {
    return pad.valid() && pad.pin < 24u &&
           (device::port_pins(pad.port) & (1UL << pad.pin)) != 0u;
}

/// Does that pad share its package pin with another (the datasheet's
/// notes 4 to 7)? Such a pad may be an input and never an output.
constexpr bool pad_twinned(Pad pad) {
    return pad.valid() && pad.pin < 24u &&
           (device::port_twinned(pad.port) & (1UL << pad.pin)) != 0u;
}

/// May a program drive that pad - bonded, and alone on its pin?
constexpr bool pad_can_drive(Pad pad) { return pad_bonded(pad) && !pad_twinned(pad); }

/// Has that pad a pull-down (a die fact, the same on every part)?
constexpr bool pad_has_pull_down(Pad pad) {
    return pad.valid() && pad.pin < 24u &&
           (gpio_pull_down_pads(pad.port) & (1UL << pad.pin)) != 0u;
}

/// A pin as a RUNTIME value: the port's registers and the mask. What a bus
/// request carries for its chip select, so the bus AO can drive a pin it
/// does not know the type of. The stores are BSHR/BSXR/BCR, atomic against
/// a handler on another pin of the same port. A null PinRef (the default)
/// drives nothing. Build one with Pin<...>::ref().
struct PinRef {
    GpioRegs* port = nullptr;
    uint32_t mask = 0;

    void set() const {
        if (port != nullptr) {
            if ((mask & 0xFFFFu) != 0u) {
                port->BSHR = mask & 0xFFFFu;
            } else {
                port->BSXR = (mask >> 16) & 0xFFu;
            }
        }
    }
    void clear() const {
        if (port != nullptr) {
            port->BCR = mask & 0x00FFFFFFu;
        }
    }
    void write(bool level) const {
        if (level) {
            set();
        } else {
            clear();
        }
    }
    /// The pad's own level, which is INDR's bit in every mode but analog
    /// (8.2.9: an analog input reads zero).
    bool read() const { return port != nullptr && (port->INDR & mask) != 0u; }
    /// One store, so a handler on another pin of the same port cannot be
    /// caught between the read and the write.
    void toggle() const {
        if (port != nullptr) {
            const uint32_t driven = port->OUTDR & mask;
            if ((mask & 0xFFFFu) != 0u) {
                port->BSHR = (driven << 16) | (mask & ~driven);
            } else {
                const uint32_t hi = mask >> 16;
                const uint32_t hi_driven = driven >> 16;
                port->BSXR = (hi_driven << 16) | (hi & ~hi_driven);
            }
        }
    }
    constexpr bool valid() const { return port != nullptr; }
};

/// A whole port: the register block, its clock, and the mask verbs the pin
/// type is built from.
template <char L>
struct Port {
    static_assert(gpio_base_for(L) != 0,
                  "brio Port: this series addresses ports A, B and C");
    static_assert(device::has_port(L),
                  "brio Port: this part bonds no pin of that port (parts/<part>.hpp)");

    Port() = delete;

    static constexpr char letter = L;

    static GpioRegs& regs() { return *reinterpret_cast<GpioRegs*>(gpio_base_for(L)); }

    /// Open the port's clock. Idempotent, and every configuring verb calls
    /// it, so a configured pin works.
    static void clock_on() { rcc()->APB2PCENR |= gpio_clock_for(L); }

    /// Which pins of this port the package brings out, and which of them
    /// share a package pin with another pad (parts/<part>.hpp).
    static constexpr uint32_t bonded = device::port_pins(L);
    static constexpr uint32_t twinned = device::port_twinned(L);

    static uint32_t in() { return regs().INDR & 0x00FFFFFFu; }
    static uint32_t out() { return regs().OUTDR & 0x00FFFFFFu; }

    /// Drive the whole port from one value: the store that says what all
    /// twenty-four are, which is what a parallel bus wants.
    static void out_write(uint32_t value) { regs().OUTDR = value & 0x00FFFFFFu; }

    /// Set pins by mask: BSHR for 0..15 and BSXR for 16..23, each store
    /// atomic against a handler that touches another pin of the same port
    /// - two stores when the mask spans both halves.
    static void out_set(uint32_t mask) {
        if ((mask & 0xFFFFu) != 0u) {
            regs().BSHR = mask & 0xFFFFu;
        }
        if ((mask & 0x00FF0000u) != 0u) {
            regs().BSXR = (mask >> 16) & 0xFFu;
        }
    }

    /// Clear pins by mask: BCR takes all twenty-four in one store.
    static void out_clear(uint32_t mask) { regs().BCR = mask & 0x00FFFFFFu; }

    /// No toggle register on this series: read what is driven and write
    /// set and reset halves in one store per register.
    static void out_toggle(uint32_t mask) {
        const uint32_t driven = regs().OUTDR & mask;
        const uint32_t lo = mask & 0xFFFFu;
        if (lo != 0u) {
            const uint32_t lo_driven = driven & 0xFFFFu;
            regs().BSHR = (lo_driven << 16) | (lo & ~lo_driven);
        }
        const uint32_t hi = (mask >> 16) & 0xFFu;
        if (hi != 0u) {
            const uint32_t hi_driven = (driven >> 16) & 0xFFu;
            regs().BSXR = (hi_driven << 16) | (hi & ~hi_driven);
        }
    }

    /**
     * Write the four configuration bits of one pin: CFGLR for pins 0..7,
     * CFGXR for 16..23 - each a read-modify-write of the register - and
     * for 8..15 the RAM copy of CFGHR, stored into the register WHOLE
     * (the file header). A pin the configuration lock holds is left as it
     * is, and so is the copy: the silicon would ignore the store, and a
     * copy that took it would no longer say what CFGHR holds. No other
     * check here: Pin does its checks at compile time and the whole-port
     * verb below does its own.
     */
    static void configure(uint8_t pin, uint32_t nibble) {
        clock_on();
        if (pin_locked(pin)) {
            return;
        }
        const uint32_t shift = static_cast<uint32_t>(pin & 7u) * 4u;
        const uint32_t field = 0xFUL << shift;
        const uint32_t value = (nibble & 0xFu) << shift;
        if (pin < 8u) {
            regs().CFGLR = (regs().CFGLR & ~field) | value;
        } else if (pin < 16u) {
            cfghr_ = (cfghr_ & ~field) | value;
            regs().CFGHR = cfghr_;
        } else {
            regs().CFGXR = (regs().CFGXR & ~field) | value;
        }
    }

    /// What one pin's four bits hold - CFGLR and CFGXR as read, pins 8..15
    /// as the copy says.
    static uint32_t nibble(uint8_t pin) {
        const uint32_t shift = static_cast<uint32_t>(pin & 7u) * 4u;
        const uint32_t reg = pin < 8u ? regs().CFGLR : pin < 16u ? cfghr_ : regs().CFGXR;
        return (reg >> shift) & 0xFu;
    }

    /// The RAM copy of CFGHR, and the register itself as a read answers -
    /// the pair a suite compares, since whether the read is the truth on
    /// this die is what the vendor's library does not trust.
    static uint32_t cfghr_copy() { return cfghr_; }
    static uint32_t cfghr_register() { return regs().CFGHR; }

    /**
     * The same nibble into every pin of `mask`, as ONE store per
     * configuration register: a port that changes mode together never
     * passes through the intermediate states a loop would walk. False, and
     * nothing written, when the mask names a pad this package does not
     * bond, or asks a driving nibble of a pad that shares its pin.
     */
    static bool configure_pins(uint32_t mask, uint32_t nibble_value) {
        if ((mask & bonded) != mask) {
            return false;
        }
        if (pin_nibble_drives(nibble_value) && (mask & twinned) != 0u) {
            return false;
        }
        clock_on();
        if (locked()) {
            mask &= ~locked_pins();   // what the lock holds stays, copy included
        }
        const uint32_t n = nibble_value & 0xFu;
        uint32_t field[3] = {0, 0, 0};
        uint32_t value[3] = {0, 0, 0};
        for (uint8_t pin = 0; pin < 24u; ++pin) {
            if ((mask & (1UL << pin)) == 0u) {
                continue;
            }
            const uint32_t shift = static_cast<uint32_t>(pin & 7u) * 4u;
            field[pin >> 3] |= 0xFUL << shift;
            value[pin >> 3] |= n << shift;
        }
        if (field[0] != 0u) {
            regs().CFGLR = (regs().CFGLR & ~field[0]) | value[0];
        }
        if (field[1] != 0u) {
            cfghr_ = (cfghr_ & ~field[1]) | value[1];
            regs().CFGHR = cfghr_;
        }
        if (field[2] != 0u) {
            regs().CFGXR = (regs().CFGXR & ~field[2]) | value[2];
        }
        return true;
    }

    // ---- the configuration lock (RM 8.2.5, 8.3.1.7) ---------------------
    //
    // ONE WAY. The key sequence freezes the configuration of the pins in
    // the mask; there is no unlock, only a reset. The sequence is write
    // LCKK|mask, write mask, write LCKK|mask, then read - the manual's
    // last two steps ("read 0, read 1") are a CHECK and not part of the
    // activation, and both reads are here because the check costs two
    // loads and tells a program whether the key took.

    /// Freeze the configuration of the pins in `mask` until the next reset.
    /// False when the mask names a pad this package does not bond (nothing
    /// is written), or when the key did not take.
    static bool lock(uint32_t mask) {
        if ((mask & bonded) != mask) {
            return false;
        }
        clock_on();
        const uint32_t key = 1UL << 24;
        const uint32_t pins = mask & 0x00FFFFFFu;
        regs().LCKR = key | pins;
        regs().LCKR = pins;
        regs().LCKR = key | pins;
        (void)regs().LCKR;
        (void)regs().LCKR;
        return locked();
    }

    /// The same, checked where the mask is a constant: a pad this part has
    /// not got is a compile error rather than a false at run time.
    template <uint32_t Mask>
    static bool lock() {
        static_assert((Mask & device::port_pins(L)) == Mask,
                      "brio Port::lock: the mask names a pad this part's package does not "
                      "bond, and locking a pin that is not there says nothing "
                      "(parts/<part>.hpp)");
        return lock(Mask);
    }

    /// LCKK: has a key sequence taken on this port? Once it has, it stands
    /// until the next reset.
    static bool locked() { return (regs().LCKR & (1UL << 24)) != 0u; }

    /// Which pins the standing lock covers (LCK[23:0]).
    static uint32_t locked_pins() { return regs().LCKR & 0x00FFFFFFu; }

private:
    /// Does a standing lock hold this pin? One read of LCKR.
    static bool pin_locked(uint8_t pin) {
        const uint32_t lckr = regs().LCKR;
        return (lckr & (1UL << 24)) != 0u && (lckr & (1UL << pin)) != 0u;
    }

    /// CFGHR as this file last stored it (the file header).
    static inline uint32_t cfghr_ = gpio_cfg_reset;
};

/**
 * One pin, as a type.
 *
 *   using Led = brio::Pin<'A', 0>;
 *   Led::output();
 *   Led::toggle();
 *
 * It also satisfies util/pwm_channel.hpp's PwmChannel with max = 1: the
 * role-level "one dimmable output", dimmable to exactly on and off, which
 * is what lets an RgbLamp be built over plain pins.
 */
template <char L, uint8_t N>
struct Pin {
    static_assert(N < 24u, "brio Pin: a port of this series has 24 pins, 0..23");
    static_assert((device::port_pins(L) & (1UL << N)) != 0u,
                  "brio Pin: this part's package does not bond that pad (parts/<part>.hpp)");

    Pin() = delete;

    using P = Port<L>;

    static constexpr char port_letter = L;
    static constexpr uint8_t pin_number = N;
    static constexpr uint32_t mask = 1UL << N;
    static constexpr Pad pad{L, N};

    /// Whether this pad may be driven: false where it shares its package
    /// pin with another pad (the file header).
    static constexpr bool can_drive = pad_can_drive(pad);
    /// Whether this pad has a pull-down.
    static constexpr bool has_pull_down = pad_has_pull_down(pad);

    /// The runtime descriptor (see PinRef).
    static PinRef ref() { return {&P::regs(), mask}; }

    // ---- PwmChannel (util/pwm_channel.hpp) --------------------------------
    static constexpr uint16_t max = 1;
    static void duty(uint16_t v) { if (v != 0u) { set(); } else { clear(); } }

    // ---- level ------------------------------------------------------------
    static void set() { P::out_set(mask); }
    static void clear() { P::out_clear(mask); }
    static void toggle() { P::out_toggle(mask); }
    static bool read() { return (P::in() & mask) != 0u; }
    static bool read_out() { return (P::out() & mask) != 0u; }

    // ---- configuration ----------------------------------------------------
    /// A push-pull output - the one output stage this series has.
    static void output() {
        static_assert(can_drive,
                      "brio Pin: this pad shares its package pin with another, and the "
                      "datasheet forbids both as outputs (table 2-1, notes 4 to 7; "
                      "parts/<part>.hpp's port_twinned)");
        P::configure(N, pin_nibble_output);
    }

    /// Drive the level BEFORE the pin becomes an output, so the pad never
    /// shows the other one for the width of two stores.
    static void output(bool level) {
        if (level) { set(); } else { clear(); }
        output();
    }

    /**
     * An input: floating, or pulled up or down. False - and nothing written
     * - for a pull-down on a pad that has none (only PA0..PA15, PC16 and
     * PC17 do). The pull direction IS the output data bit (file header):
     * it is written first, then the configuration takes effect.
     */
    static bool input(PinPull pull = PinPull::none) {
        if (pull == PinPull::none) {
            P::configure(N, pin_nibble_floating);
            return true;
        }
        if (pull == PinPull::down && !has_pull_down) {
            return false;
        }
        if (pull == PinPull::up) { set(); } else { clear(); }
        P::configure(N, pin_nibble_pulled);
        return true;
    }

    /// The same where the pull is a constant: a pull-down on a pad without
    /// one is a compile error.
    template <PinPull pull>
    static void input() {
        static_assert(pull != PinPull::down || has_pull_down,
                      "brio Pin: this pad has no pull-down - only PA0..PA15, PC16 and PC17 "
                      "do (RM ch. 8's opening)");
        (void)input(pull);
    }

    /// The analog input: output off, Schmitt trigger off, pulls off, INDR
    /// reading zero (8.2.9).
    static void analog() { P::configure(N, pin_nibble_analog); }

    /// Hand the pad to its peripheral as an alternate push-pull output (an
    /// I2C pad turns open-drain by itself, 8.3.1.1). Which peripheral is
    /// not a choice here: the remaps that move a signal to another pad are
    /// AFIO's, a verb of the resource that owns the signal. A peripheral
    /// INPUT (a USART's RX) takes input() instead (8.2.4).
    static void function() {
        static_assert(can_drive,
                      "brio Pin: this pad shares its package pin with another, and the "
                      "datasheet forbids both as outputs (table 2-1, notes 4 to 7; "
                      "parts/<part>.hpp's port_twinned)");
        P::configure(N, pin_nibble_alternate);
    }

    /// Back to the reset state: floating input, the pad driving nothing.
    static void release() { P::configure(N, pin_nibble_floating); }

    /// What the four bits hold now, for a read-back.
    static uint32_t nibble() { return P::nibble(N); }

    /// Freeze THIS pin's configuration until the next reset (see
    /// Port::lock - there is no unlock). The port's other pins are
    /// untouched: the mask carries this pin alone.
    static bool lock() { return P::lock(mask); }
    static bool locked() { return P::locked() && (P::locked_pins() & mask) != 0u; }
};

} // namespace brio
