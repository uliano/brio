/*
 * pin.hpp
 *
 * GPIO on the RP2040 (datasheet 2.19): the thirty pins of the user bank
 * as compile-time types, the two register blocks that govern them, and
 * the single-cycle path the processor drives them through.
 *
 * THREE BLOCKS OWN A PIN, and the chapter names them:
 *  - IO_BANK0 (2.19.2): a CTRL register per pin whose FUNCSEL chooses
 *    WHICH PERIPHERAL owns the pad's direction, level and input - a
 *    UART, an SPI, an I2C, a PWM slice, a PIO, a clock, USB control,
 *    or SIO for software - plus overrides nothing here uses. The
 *    peripheral's function number is the same on every pin it reaches
 *    (F2 is UART on every UART pin, F5 SIO everywhere), and WHICH
 *    instance and signal a pin carries under that number is table 279,
 *    which the driver of each peripheral keeps (uart.hpp's pin table);
 *  - PADS_BANK0 (2.19.4): a register per pin for the pad's electrical
 *    behaviour - drive strength, slew, hysteresis, the pull-up, the
 *    pull-down (both together = the bus keeper), the input buffer
 *    enable and an output DISABLE that overrides whoever owns the pad.
 *    The reset state is INPUT ENABLED, PULL-DOWN, 4 mA, hysteresis on
 *    (the register's reset value 0x56), so every pin comes up as an
 *    input reading low unless something drives it;
 *  - SIO (2.3.1.2): the processor's own path, one cycle per access,
 *    with the whole bank in one word: GPIO_OUT, GPIO_OE and their
 *    SET/CLR/XOR twins, GPIO_IN. A pin under FUNCSEL = SIO follows
 *    these; a pin under any other function does not, BUT GPIO_IN
 *    ALWAYS READS THE PAD, whatever owns it (table 280).
 *
 * The two banks are the reset controller's (2.14): every configuring
 * verb here releases IO_BANK0 and PADS_BANK0 first, once, the way the
 * other families open a port's clock.
 *
 * No port letters: one bank, pins 0..29, `Pin<25>`. The QSPI bank (the
 * six flash pins, 2.19.2's table 281) is the flash's and is not offered
 * here. GPIO26..29 double as the ADC's inputs; erratum RP2040-E6 (the
 * digital input left enabled on them, fixed in the B2 bootrom) is the
 * ADC chapter's to answer.
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "rp2040/resets.hpp"

namespace brio {

inline constexpr uint8_t gpio_count = 30;

/// FUNCSEL codes (2.19.2, table 279): what owns the pad.
enum class PinFunction : uint8_t {
    spi = 1,
    uart = 2,
    i2c = 3,
    pwm = 4,
    sio = 5,      ///< software control through SIO
    pio0 = 6,
    pio1 = 7,
    clock = 8,    ///< GPIN0/1, GPOUT0..3 on the pins that have them
    usb = 9,      ///< OVCUR DET / VBUS DET / VBUS EN
    none = IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_NULL,   ///< no owner (the reset state)
};

/// The pad's pulls (2.19.4): one of the two, or both = bus keeper mode
/// (2.19.4.1), which weakly holds whatever level the pad last had.
enum class PinPull : uint8_t { none, up, down, keeper };

/// Output drive strength (2.19.4).
enum class PinDrive : uint8_t { ma2 = 0, ma4 = 1, ma8 = 2, ma12 = 3 };

/// A pad's electrical setup. The defaults are the pad's own reset
/// values, pull aside.
struct PinConfig {
    PinPull pull = PinPull::none;
    PinDrive drive = PinDrive::ma4;
    bool slew_fast = false;
    bool schmitt = true;         ///< input hysteresis
    bool input_enable = true;    ///< the input buffer (off for an analogue pad)
};

/// A pin and the function it is handed to - what a peripheral's pin set
/// is written in (uart.hpp's UartPins).
struct PinSel {
    uint8_t pin;
    PinFunction function;
    constexpr bool valid() const { return pin < gpio_count; }
};

/// The pad register's value for a configuration (PADS_BANK0.GPIOn).
constexpr uint32_t pad_value(const PinConfig& cfg) {
    uint32_t v = static_cast<uint32_t>(cfg.drive) << PADS_BANK0_GPIO0_DRIVE_LSB;
    if (cfg.pull == PinPull::up || cfg.pull == PinPull::keeper) {
        v |= PADS_BANK0_GPIO0_PUE_BITS;
    }
    if (cfg.pull == PinPull::down || cfg.pull == PinPull::keeper) {
        v |= PADS_BANK0_GPIO0_PDE_BITS;
    }
    if (cfg.schmitt) {
        v |= PADS_BANK0_GPIO0_SCHMITT_BITS;
    }
    if (cfg.slew_fast) {
        v |= PADS_BANK0_GPIO0_SLEWFAST_BITS;
    }
    if (cfg.input_enable) {
        v |= PADS_BANK0_GPIO0_IE_BITS;
    }
    return v;   // OD clear: the owner drives when it wants to
}

/**
 * The bank: the SIO word-wide verbs, and the per-pin control and pad
 * registers by number. What a pin-check tool or a bus of parallel
 * outputs wants; single pins use Pin<n> below.
 */
struct Gpio {
    Gpio() = delete;

    /// Every configuring verb opens with this: both blocks out of reset.
    static bool ready() {
        constexpr uint32_t both = ResetBlock::io_bank0 | ResetBlock::pads_bank0;
        return Resets::released(both) || Resets::release(both);
    }

    // ---- SIO, the whole bank in one word --------------------------------

    static uint32_t in() { return SIO->GPIO_IN; }
    static uint32_t out() { return SIO->GPIO_OUT; }
    static void out_set(uint32_t mask) { SIO->GPIO_OUT_SET = mask; }
    static void out_clear(uint32_t mask) { SIO->GPIO_OUT_CLR = mask; }
    static void out_toggle(uint32_t mask) { SIO->GPIO_OUT_XOR = mask; }
    static uint32_t oe() { return SIO->GPIO_OE; }
    static void oe_set(uint32_t mask) { SIO->GPIO_OE_SET = mask; }
    static void oe_clear(uint32_t mask) { SIO->GPIO_OE_CLR = mask; }

    // ---- the two per-pin registers ---------------------------------------

    static volatile uint32_t& ctrl(uint8_t n) {
        return reg_at(IO_BANK0_BASE, IO_BANK0_GPIO0_CTRL_OFFSET + 8u * n);
    }
    static volatile uint32_t& status(uint8_t n) {
        return reg_at(IO_BANK0_BASE, IO_BANK0_GPIO0_STATUS_OFFSET + 8u * n);
    }
    static volatile uint32_t& pad(uint8_t n) {
        return reg_at(PADS_BANK0_BASE, PADS_BANK0_GPIO0_OFFSET + 4u * n);
    }

    /// Hand pin `n` to `fn` with the pad set up per `cfg`: the pad first
    /// (input enabled, output not disabled, the pulls and drive), then
    /// FUNCSEL, with every override at its neutral value.
    static void function(uint8_t n, PinFunction fn, const PinConfig& cfg = {}) {
        (void)ready();
        pad(n) = pad_value(cfg);
        ctrl(n) = static_cast<uint32_t>(fn) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    }

    /// Back to the reset state: no owner, the pad an input with its
    /// pull-down, SIO's output enable off.
    static void release(uint8_t n) {
        oe_clear(1u << n);
        ctrl(n) = static_cast<uint32_t>(PinFunction::none) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
        pad(n) = PADS_BANK0_GPIO0_RESET;
    }

    /// Every pin of `mask` a software-driven output at once (the pin
    /// check's wave): SIO on each, then the output enables in one word.
    static void outputs(uint32_t mask, const PinConfig& cfg = {}) {
        for (uint8_t n = 0; n < gpio_count; ++n) {
            if ((mask & (1u << n)) != 0u) {
                function(n, PinFunction::sio, cfg);
            }
        }
        oe_set(mask);
    }
};

/**
 * One pin as a type. Also a PwmChannel of one level (util/pwm_channel.hpp):
 * `max` 1, `duty(v)` = set or clear.
 */
/// A pin named at RUN TIME - what a bus request carries as its chip
/// select or D/C line (rp2040/spi.hpp's Request): the pin number, or
/// none. Every verb is a SIO word access on the pin's bit; a null
/// reference does nothing and reads false.
struct PinRef {
    uint8_t pin = 0xFFu;

    constexpr bool valid() const { return pin < gpio_count; }
    void set() const {
        if (valid()) { SIO->GPIO_OUT_SET = 1u << pin; }
    }
    void clear() const {
        if (valid()) { SIO->GPIO_OUT_CLR = 1u << pin; }
    }
    void toggle() const {
        if (valid()) { SIO->GPIO_OUT_XOR = 1u << pin; }
    }
    bool read() const { return valid() && (SIO->GPIO_IN & (1u << pin)) != 0u; }
    bool read_out() const { return valid() && (SIO->GPIO_OUT & (1u << pin)) != 0u; }
};

template <uint8_t n>
struct Pin {
    static_assert(n < gpio_count, "the RP2040's user bank has thirty pins, GPIO0..GPIO29");

    static constexpr uint8_t number = n;
    static constexpr uint32_t mask = 1u << n;

    static void output(const PinConfig& cfg = {}) {
        Gpio::function(n, PinFunction::sio, cfg);
        Gpio::oe_set(mask);
    }
    /// An output starting at `level`, the level written BEFORE the
    /// direction so the pin never shows the other one.
    static void output(bool level, const PinConfig& cfg = {}) {
        if (level) { set(); } else { clear(); }
        output(cfg);
    }
    static void input(PinPull pull = PinPull::none) {
        Gpio::oe_clear(mask);
        Gpio::function(n, PinFunction::sio, {.pull = pull});
    }
    /// Hand the pin to a peripheral.
    static void function(PinFunction fn, const PinConfig& cfg = {}) {
        Gpio::function(n, fn, cfg);
    }
    static void release() { Gpio::release(n); }

    /// This pin as a run-time reference (a bus request's select line).
    static constexpr PinRef ref() { return PinRef{n}; }

    static void set() { SIO->GPIO_OUT_SET = mask; }
    static void clear() { SIO->GPIO_OUT_CLR = mask; }
    static void toggle() { SIO->GPIO_OUT_XOR = mask; }
    /// The pad's level, whoever owns it.
    static bool read() { return (SIO->GPIO_IN & mask) != 0u; }
    static bool read_out() { return (SIO->GPIO_OUT & mask) != 0u; }
    static bool is_output() { return (SIO->GPIO_OE & mask) != 0u; }
    static PinFunction function() {
        return static_cast<PinFunction>(Gpio::ctrl(n) & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS);
    }

    /// The pad's pulls alone, the rest of the pad untouched.
    static void pull(PinPull p) {
        constexpr uint32_t both = PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS;
        uint32_t v = 0;
        if (p == PinPull::up || p == PinPull::keeper) { v |= PADS_BANK0_GPIO0_PUE_BITS; }
        if (p == PinPull::down || p == PinPull::keeper) { v |= PADS_BANK0_GPIO0_PDE_BITS; }
        hw_write_masked(Gpio::pad(n), v, both);
    }

    // PwmChannel of one level.
    static constexpr uint16_t max = 1;
    static void duty(uint16_t v) { if (v) { set(); } else { clear(); } }
};

} // namespace brio
