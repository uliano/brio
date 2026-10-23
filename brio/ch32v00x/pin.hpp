/*
 * pin.hpp
 *
 * GPIO for the CH32V00x: one pad as a type. `Pin<'D', 5>` names port D,
 * pin 5 when the program is compiled; its port and mask fold to
 * constants, and the type has no object (a monostate: its functions are
 * all static and nothing is constructed). The smallest use, the pad of
 * docs/ch32v00x/pin.md's "How to use it" without its interrupt line, the
 * read() as the sentence after that example spells it:
 *
 *     using Button = brio::Pin<'D', 4>;
 *     Button::input(brio::PinPull::up);
 *     ... Button::read() ...
 *
 * The alias names the pad and configures nothing; input() configures
 * it (and opens the port's clock), and read() then answers the level.
 *
 * Who calls what. The level verbs - set(), clear(), toggle(), read(),
 * read_out() - may be called from any context: set() and clear() are
 * one store of BSHR or BCR, read() one load of INDR, and toggle() a
 * load of OUTDR and one BSHR store, so none disturbs a handler driving
 * another pin of the same port. The configuring verbs - output(),
 * input(), analog(), function(), release() - are read-modify-writes
 * with no guard: of CFGLR, which the port's eight pins share, and,
 * through Port::clock_on(), of RCC_PB2PCENR, the gate register every
 * GPIO port and the other PB2 blocks (AFIO, the ADC, TIM1, SPI1, the
 * USARTs) share. Nothing masks, so a configuring verb races an
 * interrupt handler that configures a pin of the same port or opens a
 * PB2 gate: configure from main or from an active object (a component
 * of a brio program with its own event queue, served one event at a
 * time), never where such a handler can run in between. Nothing here
 * waits.
 *
 * The encoding, and the trap in it (RM 7.3.1.1). Four bits per pin in
 * one configuration register - there is no CFGHR, because this family
 * bonds at most eight pins per port. The shape looks like the STM32F1's
 * CNF/MODE pair, and code carried over from one will appear to work,
 * but on the CH32V006 MODE is one bit (the CH32V003's is two, below):
 * 1 is output (at the port's only speed, 30 MHz) and 0 is input; the
 * second bit of the field is reserved. So an
 * F1 nibble like 0xB (AF push-pull, "50 MHz") lands on this silicon as
 * AF push-pull with a reserved bit set - right by accident. This file
 * spells the two halves separately so the accident cannot happen.
 *
 * Pulls have no register. An input with CNF = 10 is pulled, and the
 * direction comes from that pin's bit in OUTDR: 1 pulls up, 0 pulls
 * down. Setting the pull therefore writes the output data register of a
 * pin that is an input, which reads oddly and is exactly what the
 * chapter prescribes.
 *
 * The port clock is opened by every configuring verb: a Pin that is
 * configured works with no separate clock step to remember - with
 * one order to know: output(level) and input(pull) store the level or
 * the pull's direction, and open the port's clock BEFORE that store: a
 * store into a port whose clock is still gated is lost (measured on the
 * CH32V006K8U6: OUTDR's bit 0 and the pad low after a set() made before
 * the port's first configuration, both 1 after Port<L>::clock_on()), so
 * the gate comes first and the level asked holds from the first instant.
 * Nothing here turns a clock back off - that would be a decision about
 * the whole port, and this file only speaks about pins.
 *
 * Two parts, two MODE fields. The CH32V006's MODE is one bit (the
 * second reserved); the CH32V003's is the F1's two, with a speed, and
 * this file drives its outputs at that part's top speed - so a nibble
 * spelled here lands right on both (device::gpio_mode_output). Port B
 * is the CH32V006's alone: a Pin on it does not compile for the
 * CH32V003 (device::has_port_b through gpio_base_for).
 *
 * The bonding shows in the registers: port B of the CH32V006 has seven
 * pins (PB0..PB6, DS 2.1), and its eighth configuration nibble reads
 * zero after a reset where the seven read 0x4 (measured). The pin-level
 * table of which pins each package brings out is still the datasheet's.
 *
 * Not covered yet: the pin-level bonding table (which pins a package
 * actually brings out). The part tables state ports, not pins, so a Pin
 * on a pin of a bonded port that the package leaves out (PB7 on the
 * CH32V006, PA0 on the CH32V003) compiles; the datasheet's pinout is
 * the record, and the build names one package per part
 * (docs/ch32v00x/pin.md). The alternate-function
 * remaps (AFIO_PCFR1), which move a peripheral's pads to another column
 * of the chapter's tables, are ch32v00x/afio.hpp's; the external
 * interrupt lines are exti.hpp's.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"

namespace brio {

/// What a pin is: the MODE bit plus what CNF means under it.
enum class PinMode : uint8_t { input, output, alternate, analog };

/// Output stage, for `output` and `alternate`.
enum class PinDrive : uint8_t { push_pull, open_drain };

/// Input pull, for `input`. The direction lives in OUTDR (the file
/// header).
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

/// A pin as a runtime value: the port's registers and the mask. What a
/// bus request carries for its chip select, so the bus's active object
/// can drive a pin it does not know the type of. The stores are BSHR/BCR (RM
/// 7.3.1.4, 7.3.1.5), atomic against a handler on another pin of the
/// same port. A null PinRef (the default) drives nothing. Build one
/// with Pin<...>::ref().
///
/// The two edges are forced inline: left to -Os, gcc keeps the body
/// behind its call sites wherever it has two, and an edge costs a call
/// and a return on top of its five instructions - four times a
/// transaction on the SPI host, counted in the release listing
/// (docs/ch32v00x/spi.md). Inline, an edge is the port pointer's load,
/// the null test, the mask's load and one BSHR or BCR store (write-one
/// registers: no read, no window); the null test stays a predictable
/// branch, which is what makes an absent D/C pin cost one branch and
/// not a verb.
struct PinRef {
    GpioRegs* port = nullptr;
    uint32_t mask = 0;

    [[gnu::always_inline]] void set() const {
        if (port != nullptr) {
            port->BSHR = mask & 0xFFFFu;
        }
    }
    [[gnu::always_inline]] void clear() const {
        if (port != nullptr) {
            port->BCR = mask & 0xFFFFu;
        }
    }
    constexpr bool valid() const { return port != nullptr; }
};

/// The CNF/MODE nibble for a configuration (RM 7.3.1.1).
constexpr uint32_t pin_nibble(PinMode mode, PinDrive drive) {
    // The output MODE code is the part's (device::gpio_mode_output): a
    // single bit on the CH32V006, the F1's two bits with a speed on the
    // CH32V003, where the stratum drives at the top speed.
    const uint32_t out = device::gpio_mode_output;
    switch (mode) {
        case PinMode::analog:    return 0x0u;                                   // CNF 00, MODE 0
        case PinMode::input:     return 0x4u;                                   // CNF 01 floating
        case PinMode::output:    return (drive == PinDrive::push_pull) ? (0x0u | out)   // CNF 00, MODE out
                                                                       : (0x4u | out);  // CNF 01, MODE out
        case PinMode::alternate: return (drive == PinDrive::push_pull) ? (0x8u | out)   // CNF 10, MODE out
                                                                       : (0xCu | out);  // CNF 11, MODE out
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
    /// calls it, so a configured pin works. A read-modify-write of
    /// RCC_PB2PCENR with no guard (the file header).
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

    /// Write the four configuration bits of one pin: the gate opened,
    /// then a read-modify-write of CFGLR with no guard (the file header).
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
 * One pin, as a type (the file header gives the contexts).
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
    /// The level at the pad (INDR), one load: true is high. An analog
    /// pad's input buffer is off, so it reads false whatever the level.
    static bool read() { return (P::in() & mask) != 0u; }
    static bool read_out() { return (P::out() & mask) != 0u; }

    // ---- configuration ----------------------------------------------------
    /// The pin as an output, push-pull or open-drain, driving whatever
    /// OUTDR holds for it; the port's clock opened first.
    static void output(PinDrive drive = PinDrive::push_pull) {
        P::configure(N, pin_nibble(PinMode::output, drive));
    }

    /// Drive the level before the pin becomes an output, so the pad
    /// never shows the other one for the width of two stores. The level's
    /// store comes after the port's clock is opened and before output()
    /// configures the pin: a store into a gated block is lost (measured,
    /// the file header), so the gate comes first.
    static void output(bool level, PinDrive drive = PinDrive::push_pull) {
        P::clock_on();   // a store into a port whose gate is shut is lost (measured, the file header)
        if (level) { set(); } else { clear(); }
        output(drive);
    }

    /// The pin as an input, floating or pulled; for a pull, the
    /// direction's OUTDR bit is written first, then the nibble - the
    /// bit's store before configure() opens the port's clock, so on a
    /// port no earlier call has configured the same caveat as
    /// output(level) holds.
    static void input(PinPull pull = PinPull::none) {
        if (pull == PinPull::none) {
            P::configure(N, pin_nibble(PinMode::input, PinDrive::push_pull));
            return;
        }
        // The pull direction is the output data bit (the file header):
        // the gate open, the bit written, then the configuration takes
        // effect.
        P::clock_on();
        if (pull == PinPull::up) { set(); } else { clear(); }
        P::configure(N, 0x8u);   // CNF 10, MODE 0: pulled input
    }

    static void analog() { P::configure(N, pin_nibble(PinMode::analog, PinDrive::push_pull)); }

    /// Hand the pad to its peripheral. Which peripheral is not a choice
    /// here: a pin has one default alternate function (the datasheet's
    /// pin table), and the remaps that would move it are AFIO's
    /// (ch32v00x/afio.hpp), a verb of the resource that owns the pad.
    static void function(PinDrive drive = PinDrive::push_pull) {
        P::configure(N, pin_nibble(PinMode::alternate, drive));
    }

    /// Back to the reset state: floating input, the pad driving nothing.
    static void release() { P::configure(N, pin_nibble(PinMode::input, PinDrive::push_pull)); }
};

} // namespace brio
