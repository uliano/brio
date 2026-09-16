/*
 * pin.hpp
 *
 * Compile-time GPIO for the CH32V203: `Pin<'A', 9>` is a type, its port
 * and mask fold to constants, and every verb is a single register
 * access.
 *
 * THE ENCODING IS THE STM32F1's, WHOLE (RM 10.1.1) - which is worth
 * saying because the sister family's is not. Four bits per pin, CNF
 * above MODE, sixteen pins a port over TWO registers (CFGLR for 0..7,
 * CFGHR for 8..15), and MODE is two bits that carry a SPEED: 00 input,
 * 01 output at 10 MHz, 10 output at 2 MHz, 11 output at 50 MHz. On the
 * CH32V00x that second bit is reserved, so a nibble copied from there
 * would silently drop the speed here; the two strata spell it out
 * separately for that reason.
 *
 * WHICH SPEED, AND WHY IT IS A PARAMETER. 50 MHz is the default because
 * a pad that is too slow turns a bus into a puzzle, but the fastest
 * edge is not free - it is the loudest one too, and on other families a
 * pad's slew class has already been a correctness parameter of a bus
 * driver. So the choice is in the API, at the pin, with a default that
 * cannot surprise a logic level.
 *
 * PULLS HAVE NO REGISTER. An input with CNF = 10 is pulled, and the
 * DIRECTION comes from that pin's bit in OUTDR: 1 pulls up, 0 pulls
 * down. Setting the pull therefore writes the output data register of a
 * pin that is an input, which reads oddly and is exactly what the
 * chapter prescribes.
 *
 * THE BONDING IS THE PART'S. A package brings out some of the sixteen
 * pins of each port - thirty-seven of them on the LQFP48 - and
 * `device::port_pins()` is that table; a Pin on a pad this part does not
 * bond does not compile. The block itself answers either way, which is
 * exactly why the check has to be here.
 *
 * The port clock is opened by every configuring verb, the way the other
 * strata do it: a Pin that is configured works, with no separate step
 * for the caller to forget. Nothing here turns a clock back OFF - that
 * would be a decision about the whole port, and this file only speaks
 * about pins.
 *
 * NOT COVERED YET: the alternate-function REMAPS (AFIO_PCFR1 and
 * PCFR2), so a peripheral is reachable on its default pads only - they
 * arrive with the first driver that needs a remapped one; the
 * configuration LOCK (LCKR); the external interrupt lines, which are
 * exti.hpp's.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/device.hpp"

namespace brio {

/// What a pin is: the MODE field's kind plus what CNF means under it.
enum class PinMode : uint8_t { input, output, alternate, analog };

/// Output stage, for `output` and `alternate`.
enum class PinDrive : uint8_t { push_pull, open_drain };

/// Input pull, for `input`. The direction lives in OUTDR (see header).
enum class PinPull : uint8_t { none, up, down };

/// How hard an output drives (the MODE field, RM 10.1.1). The numbers
/// are megahertz, and they are the pad's, not the signal's.
enum class PinSpeed : uint8_t { slow = 2, medium = 10, fast = 50 };

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

/// The MODE code of an output at a given speed (RM 10.1.1).
constexpr uint32_t pin_mode_code(PinSpeed speed) {
    return speed == PinSpeed::fast ? 0x3u : speed == PinSpeed::medium ? 0x1u : 0x2u;
}

/// The CNF/MODE nibble for a configuration.
constexpr uint32_t pin_nibble(PinMode mode, PinDrive drive, PinSpeed speed) {
    const uint32_t out = pin_mode_code(speed);
    switch (mode) {
        case PinMode::analog:    return 0x0u;                                    // CNF 00, MODE 00
        case PinMode::input:     return 0x4u;                                    // CNF 01 floating
        case PinMode::output:    return (drive == PinDrive::push_pull) ? (0x0u | out)    // CNF 00
                                                                       : (0x4u | out);   // CNF 01
        case PinMode::alternate: return (drive == PinDrive::push_pull) ? (0x8u | out)    // CNF 10
                                                                       : (0xCu | out);   // CNF 11
    }
    return 0x4u;
}

/// A whole port: the register block, its clock, and the mask verbs the
/// pin type is built from.
template <char L>
struct Port {
    static_assert(gpio_base_for(L) != 0,
                  "brio Port: this family addresses ports A..E");
    static_assert(device::has_port(L),
                  "brio Port: this part bonds no pin of that port (parts/<part>.hpp)");

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

    /// Write the four configuration bits of one pin - in CFGLR for pins
    /// 0..7 and in CFGHR for 8..15, which is the only thing the two
    /// registers disagree about.
    static void configure(uint8_t pin, uint32_t nibble) {
        clock_on();
        volatile uint32_t& reg = (pin < 8u) ? regs().CFGLR : regs().CFGHR;
        const uint32_t shift = static_cast<uint32_t>(pin & 7u) * 4u;
        uint32_t cfg = reg;
        cfg &= ~(0xFu << shift);
        cfg |= nibble << shift;
        reg = cfg;
    }
};

/**
 * One pin, as a type.
 *
 *   using Led = brio::Pin<'B', 2>;
 *   Led::output();
 *   Led::toggle();
 *
 * It also satisfies util/pwm_channel.hpp's PwmChannel with max = 1: the
 * role-level "one dimmable output", dimmable to exactly on and off,
 * which is what lets an RgbLamp be built over plain pins.
 */
template <char L, uint8_t N>
struct Pin {
    static_assert(N < 16u, "brio Pin: a port of this family has sixteen pins, 0..15");
    static_assert((device::port_pins(L) & (1u << N)) != 0u,
                  "brio Pin: this part's package does not bond that pad (parts/<part>.hpp)");

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
    static void output(PinDrive drive = PinDrive::push_pull, PinSpeed speed = PinSpeed::fast) {
        P::configure(N, pin_nibble(PinMode::output, drive, speed));
    }

    /// Drive the level BEFORE the pin becomes an output, so the pad
    /// never shows the other one for the width of two stores.
    static void output(bool level, PinDrive drive = PinDrive::push_pull,
                       PinSpeed speed = PinSpeed::fast) {
        if (level) { set(); } else { clear(); }
        output(drive, speed);
    }

    static void input(PinPull pull = PinPull::none) {
        if (pull == PinPull::none) {
            P::configure(N, pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast));
            return;
        }
        // The pull direction IS the output data bit (see file header):
        // write it first, then let the configuration take effect.
        if (pull == PinPull::up) { set(); } else { clear(); }
        P::configure(N, 0x8u);   // CNF 10, MODE 00: pulled input
    }

    static void analog() {
        P::configure(N, pin_nibble(PinMode::analog, PinDrive::push_pull, PinSpeed::fast));
    }

    /// Hand the pad to its peripheral. Which peripheral is not a choice
    /// here: a pin has ONE alternate function at a time and the remaps
    /// that would move a signal to another pad are AFIO's, a verb of the
    /// resource that owns the signal.
    static void function(PinDrive drive = PinDrive::push_pull, PinSpeed speed = PinSpeed::fast) {
        P::configure(N, pin_nibble(PinMode::alternate, drive, speed));
    }

    /// Back to the reset state: floating input, the pad driving nothing.
    static void release() {
        P::configure(N, pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast));
    }
};

} // namespace brio
