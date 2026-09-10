/*
 * pin.hpp
 *
 * Compile-time GPIO for the CH32V00x: `Pin<'D', 5>` is a type, its port
 * and mask fold to constants, and every verb is a single register
 * access.
 *
 * THE ENCODING, AND THE TRAP IN IT (RM 7.3.1.1). Four bits per pin in
 * ONE configuration register - there is no CFGHR, because this family
 * bonds at most eight pins per port. The shape looks like the STM32F1's
 * CNF/MODE pair, and code carried over from one will appear to work,
 * but MODE HERE IS ONE BIT: 1 is output (at the port's only speed, 30
 * MHz) and 0 is input; the second bit of the field is reserved. So an
 * F1 nibble like 0xB (AF push-pull, "50 MHz") lands on this silicon as
 * AF push-pull with a reserved bit set - right by accident. This file
 * spells the two halves separately so the accident cannot happen.
 *
 * PULLS HAVE NO REGISTER. An input with CNF = 10 is pulled, and the
 * DIRECTION comes from that pin's bit in OUTDR: 1 pulls up, 0 pulls
 * down. Setting the pull therefore writes the output data register of a
 * pin that is an input, which reads oddly and is exactly what the
 * chapter prescribes.
 *
 * The port clock is opened by every configuring verb, the way the other
 * strata do it: a Pin that is configured works, with no separate step
 * for the caller to forget. Nothing here turns a clock back OFF - that
 * would be a decision about the whole port, and this file only speaks
 * about pins.
 *
 * NOT COVERED YET: the alternate-function REMAPS (AFIO_PCFR1), so a
 * peripheral is reachable only on its default pins - they arrive with
 * the first driver that needs a remapped pad; the pin-level bonding
 * table (which pins a package actually brings out), which needs the
 * second part to be worth writing; and the external interrupt lines,
 * which belong to an exti.hpp born with their first user.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"

namespace brio {

/// What a pin is: the MODE bit plus what CNF means under it.
enum class PinMode : uint8_t { input, output, alternate, analog };

/// Output stage, for `output` and `alternate`.
enum class PinDrive : uint8_t { push_pull, open_drain };

/// Input pull, for `input`. The direction lives in OUTDR (see header).
enum class PinPull : uint8_t { none, up, down };

/// A pad named at compile time - port letter and pin number - for the
/// pin tables a driver carries (which pads a peripheral's signals sit
/// on). `valid()` false is "no such pad": a signal the program does not
/// wire.
struct Pad {
    char port = 0;
    uint8_t pin = 0;

    constexpr bool valid() const { return port != 0; }
    constexpr bool operator==(const Pad&) const = default;
};

/// A pin as a RUNTIME value: the port's registers and the mask. What a
/// bus request carries for its chip select, so the bus AO can drive a
/// pin it does not know the type of. The stores are BSHR/BCR, atomic
/// against a handler on another pin of the same port. A null PinRef
/// (the default) drives nothing. Build one with Pin<...>::ref().
struct PinRef {
    GpioRegs* port = nullptr;
    uint32_t mask = 0;

    void set() const {
        if (port != nullptr) {
            port->BSHR = mask & 0xFFFFu;
        }
    }
    void clear() const {
        if (port != nullptr) {
            port->BCR = mask & 0xFFFFu;
        }
    }
    constexpr bool valid() const { return port != nullptr; }
};

/// The CNF/MODE nibble for a configuration (RM 7.3.1.1).
constexpr uint32_t pin_nibble(PinMode mode, PinDrive drive) {
    switch (mode) {
        case PinMode::analog:    return 0x0u;                                   // CNF 00, MODE 0
        case PinMode::input:     return 0x4u;                                   // CNF 01 floating
        case PinMode::output:    return (drive == PinDrive::push_pull) ? 0x1u   // CNF 00, MODE 1
                                                                       : 0x5u;  // CNF 01, MODE 1
        case PinMode::alternate: return (drive == PinDrive::push_pull) ? 0x9u   // CNF 10, MODE 1
                                                                       : 0xDu;  // CNF 11, MODE 1
    }
    return 0x4u;
}

/// A whole port: the register block, its clock, and the mask verbs the
/// pin type is built from.
template <char L>
struct Port {
    static_assert(gpio_base_for(L) != 0,
                  "brio Port: this family has ports A, B, C and D - and which of "
                  "their pins a package bonds is the datasheet's pin table");

    Port() = delete;

    static constexpr char letter = L;

    static GpioRegs& regs() { return *reinterpret_cast<GpioRegs*>(gpio_base_for(L)); }

    /// Open the port's clock. Idempotent, and every configuring verb
    /// calls it, so a configured pin works.
    static void clock_on() { rcc()->PB2PCENR |= gpio_clock_for(L); }

    static uint32_t in() { return regs().INDR; }
    static uint32_t out() { return regs().OUTDR; }

    /// BSHR sets from its low half and clears from its high half; BCR
    /// clears. Both are write-only and atomic against a handler that
    /// touches another pin of the same port - unlike a read-modify-write
    /// on OUTDR.
    static void out_set(uint32_t mask) { regs().BSHR = mask & 0xFFFFu; }
    static void out_clear(uint32_t mask) { regs().BCR = mask & 0xFFFFu; }

    /// No toggle register on this family: read what is driven and write
    /// the two halves of BSHR in one store, which keeps the operation
    /// atomic against another pin's handler.
    static void out_toggle(uint32_t mask) {
        const uint32_t driven = regs().OUTDR & mask;
        regs().BSHR = (driven << 16) | (mask & ~driven);
    }

    /// Write the four configuration bits of one pin.
    static void configure(uint8_t pin, uint32_t nibble) {
        clock_on();
        const uint32_t shift = static_cast<uint32_t>(pin) * 4u;
        uint32_t cfg = regs().CFGLR;
        cfg &= ~(0xFu << shift);
        cfg |= nibble << shift;
        regs().CFGLR = cfg;
    }
};

/**
 * One pin, as a type.
 *
 *   using Led = brio::Pin<'D', 0>;
 *   Led::output();
 *   Led::toggle();
 *
 * It also satisfies util/pwm_channel.hpp's PwmChannel with max = 1: the
 * role-level "one dimmable output", dimmable to exactly on and off,
 * which is what lets an RgbLamp be built over plain pins.
 */
template <char L, uint8_t N>
struct Pin {
    static_assert(N < 8u, "brio Pin: a port of this family has eight pins, 0..7");

    Pin() = delete;

    using P = Port<L>;

    static constexpr char port_letter = L;
    static constexpr uint8_t pin_number = N;
    static constexpr uint32_t mask = 1UL << N;
    static constexpr Pad pad{L, N};

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
    static void output(PinDrive drive = PinDrive::push_pull) {
        P::configure(N, pin_nibble(PinMode::output, drive));
    }

    /// Drive the level BEFORE the pin becomes an output, so the pad
    /// never shows the other one for the width of two stores.
    static void output(bool level, PinDrive drive = PinDrive::push_pull) {
        if (level) { set(); } else { clear(); }
        output(drive);
    }

    static void input(PinPull pull = PinPull::none) {
        if (pull == PinPull::none) {
            P::configure(N, pin_nibble(PinMode::input, PinDrive::push_pull));
            return;
        }
        // The pull direction IS the output data bit (see file header):
        // write it first, then let the configuration take effect.
        if (pull == PinPull::up) { set(); } else { clear(); }
        P::configure(N, 0x8u);   // CNF 10, MODE 0: pulled input
    }

    static void analog() { P::configure(N, pin_nibble(PinMode::analog, PinDrive::push_pull)); }

    /// Hand the pad to its peripheral. Which peripheral is not a choice
    /// on this family: a pin has ONE default alternate function (the
    /// datasheet's pin table), and the remaps that would move it live in
    /// AFIO, which this stratum does not touch yet.
    static void function(PinDrive drive = PinDrive::push_pull) {
        P::configure(N, pin_nibble(PinMode::alternate, drive));
    }

    /// Back to the reset state: floating input, the pad driving nothing.
    static void release() { P::configure(N, pin_nibble(PinMode::input, PinDrive::push_pull)); }
};

} // namespace brio
