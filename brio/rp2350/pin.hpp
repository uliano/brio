/*
 * pin.hpp
 *
 * GPIO on the RP2350 (datasheet chapter 9): the user bank as
 * compile-time types, the two register blocks that govern it, and the
 * single-cycle path the processors drive it through.
 *
 * THREE BLOCKS OWN A PIN, and the chapter names them:
 *  - IO_BANK0 (9.4): a CTRL register per pin whose FUNCSEL chooses WHICH
 *    PERIPHERAL owns the pad's direction, level and input - an SPI, a
 *    UART, an I2C, a PWM slice, one of THREE PIOs, SIO for software, and
 *    per-pin functions above that - plus overrides nothing here uses.
 *    The low function numbers are the same on every pin they reach (F5
 *    is SIO everywhere), and WHICH instance and signal a pin carries is
 *    the per-peripheral table each driver keeps;
 *  - PADS_BANK0 (9.11): a register per pin for the pad's electrical
 *    behaviour - drive strength, slew, hysteresis, the pull-up, the
 *    pull-down (both together = the bus keeper), the input buffer
 *    enable, an output DISABLE that overrides whoever owns the pad - AND
 *    THE ISOLATION LATCH, which is this chip's own;
 *  - SIO (3.1.2): the processors' own path, one cycle per access, with
 *    the whole bank in TWO words: GPIO_OUT and GPIO_HI_OUT, their
 *    SET/CLR/XOR twins, GPIO_OE and GPIO_HI_OE, GPIO_IN and GPIO_HI_IN.
 *    A pin under FUNCSEL = SIO follows these; a pin under any other
 *    function does not, but GPIO_IN ALWAYS READS THE PAD.
 *
 * THE ISOLATION LATCH IS THE NEW THING, and it is the reason a pad
 * configured as on the RP2040 would do nothing at all here. PADS_BANK0's
 * reset value is 0x116 (9.11): ISO set, the input buffer DISABLED, a
 * pull-down on, 4 mA, hysteresis on. The latch exists so that pads keep
 * their state through a powered-down core domain and glitch nothing on
 * the way back up, and while it stands the pad is cut off from the
 * digital logic in both directions. Every configuring verb here writes
 * the whole pad register with ISO CLEAR, which is the documented order:
 * set the pad and the function up first, drop the isolation last.
 *
 * AND THE INPUT BUFFER IS OFF AT RESET, where the RP2040's was on. A pin
 * this stratum hands to SIO or a peripheral gets it enabled (PinConfig's
 * default); a pin nothing has configured reads nothing.
 *
 * ERRATUM RP2350-E9 IS LIVE ON THE BENCH SILICON (stepping A2). With the
 * input buffer enabled, a pad that nothing drives leaks enough current
 * through the input stage to sit HIGH against its own pull-down, once
 * the voltage has drifted into the undefined region; a pad driven low
 * and then released stays low. So an idle level on this chip is HISTORY
 * and not a measurement: a program that wants to read a floating pad
 * through its pulls must drive it first, or use the pull-UP and read a
 * low as the signal. A3 fixes the leakage path.
 *
 * NO PORT LETTERS: one bank, `Pin<25>`. THE BANK'S SIZE IS THE
 * PACKAGE'S: 48 pins on the QFN-80, 30 on the QFN-60 (device.hpp states
 * both and which one this build was told about). Since the die is one,
 * every pin's CTRL and PAD register exists on both packages - what a
 * QFN-60 has not got is the BOND WIRE - so a build that states its
 * package refuses GP30..GP47 at COMPILE time, and a build that states
 * none compiles them and refuses at RUN time, against SYSINFO's
 * PACKAGE_SEL. That is why the configuring verbs answer bool: a pad the
 * package has not got is not written, and the caller is told.
 *
 * The QSPI bank (the six flash pins) is the flash chapter's and is not
 * offered here. GP26..GP29 (QFN-60) or GP40..GP47 (QFN-80) double as the
 * ADC's inputs, which is the converter chapter's business.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "rp2350/resets.hpp"
#include "rp2350/sysinfo.hpp"

namespace brio {

/// FUNCSEL codes (9.4). The three low ones and SIO are the same on every
/// pin that has them; `gpck`, `usb` and `uart_alt` exist on SOME pins
/// only and mean different signals on each - the table belongs to the
/// driver that owns the function, as on the RP2040.
enum class PinFunction : uint8_t {
    spi = 1,
    uart = 2,
    i2c = 3,
    pwm = 4,
    sio = 5,      ///< software control through SIO
    pio0 = 6,
    pio1 = 7,
    pio2 = 8,     ///< the third PIO, which the RP2040 had not
    gpck = 9,     ///< the clock inputs and outputs, on the pins that carry them
    usb = 10,     ///< the USB muxing signals, on the pins that carry them
    uart_alt = 11, ///< the second UART route: every group's CTS/RTS pads
    none = IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_NULL,   ///< no owner (the reset state)
};

/// The pad's pulls (9.11): one of the two, or both = bus keeper mode,
/// which weakly holds whatever level the pad last had.
enum class PinPull : uint8_t { none, up, down, keeper };

/// Output drive strength (9.11).
enum class PinDrive : uint8_t { ma2 = 0, ma4 = 1, ma8 = 2, ma12 = 3 };

/// A pad's electrical setup. The defaults are the pad's own reset
/// values, except that the INPUT BUFFER IS ON (the reset state has it
/// off) and no pull is applied: a pad this stratum configures is a pad
/// meant to be used.
struct PinConfig {
    PinPull pull = PinPull::none;
    PinDrive drive = PinDrive::ma4;
    bool slew_fast = false;
    bool schmitt = true;         ///< input hysteresis
    bool input_enable = true;    ///< the input buffer (off for an analogue pad)
};

/// A pin and the function it is handed to - what a peripheral's pin set
/// is written in.
struct PinSel {
    uint8_t pin;
    PinFunction function;
    constexpr bool valid() const { return pin < gpio_count; }
};

/// The pad register's value for a configuration (PADS_BANK0.GPIOn).
/// ISO is left CLEAR: writing the pad IS taking it out of isolation.
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
 * The bank: the SIO word-wide verbs in both halves, and the per-pin
 * control and pad registers by number. What a pin-check tool or a bus of
 * parallel outputs wants; single pins use Pin<n> below.
 */
struct Gpio {
    Gpio() = delete;

    /// Every configuring verb opens with this: both blocks out of reset.
    static bool ready() {
        constexpr uint32_t both = ResetBlock::io_bank0 | ResetBlock::pads_bank0;
        return Resets::released(both) || Resets::release(both);
    }

    /// Whether the pad exists on THIS chip: the build's package when it
    /// stated one, the chip's own PACKAGE_SEL when it did not. The
    /// registers exist either way - the bond wire is what does not.
    static bool bonded(uint8_t n) {
        if (n >= gpio_count_max) {
            return false;
        }
        if constexpr (package_known) {
            return n < gpio_count;
        } else {
            return n < package_gpio_count(ChipId::package_sel());
        }
    }

    // ---- SIO, the bank in two words per direction -----------------------
    //
    // The low half is GPIO0..31, the high half GPIO32..47 - and the high
    // half's registers carry the QSPI pins and the USB pads above that,
    // which nothing here touches: every mask verb below is the caller's
    // own bits.

    static uint32_t in() { return SIO->GPIO_IN; }
    static uint32_t in_hi() { return SIO->GPIO_HI_IN; }
    static uint32_t out() { return SIO->GPIO_OUT; }
    static uint32_t out_hi() { return SIO->GPIO_HI_OUT; }
    static void out_set(uint32_t mask) { SIO->GPIO_OUT_SET = mask; }
    static void out_set_hi(uint32_t mask) { SIO->GPIO_HI_OUT_SET = mask; }
    static void out_clear(uint32_t mask) { SIO->GPIO_OUT_CLR = mask; }
    static void out_clear_hi(uint32_t mask) { SIO->GPIO_HI_OUT_CLR = mask; }
    static void out_toggle(uint32_t mask) { SIO->GPIO_OUT_XOR = mask; }
    static void out_toggle_hi(uint32_t mask) { SIO->GPIO_HI_OUT_XOR = mask; }
    static uint32_t oe() { return SIO->GPIO_OE; }
    static uint32_t oe_hi() { return SIO->GPIO_HI_OE; }
    static void oe_set(uint32_t mask) { SIO->GPIO_OE_SET = mask; }
    static void oe_set_hi(uint32_t mask) { SIO->GPIO_HI_OE_SET = mask; }
    static void oe_clear(uint32_t mask) { SIO->GPIO_OE_CLR = mask; }
    static void oe_clear_hi(uint32_t mask) { SIO->GPIO_HI_OE_CLR = mask; }

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
    /// (the input buffer, the pulls, the drive, and the isolation latch
    /// dropped), then FUNCSEL, with every override at its neutral value.
    /// False, and nothing written, for a pad this package has not got.
    static bool function(uint8_t n, PinFunction fn, const PinConfig& cfg = {}) {
        if (!bonded(n)) {
            return false;
        }
        (void)ready();
        pad(n) = pad_value(cfg);
        ctrl(n) = static_cast<uint32_t>(fn) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
        return true;
    }

    /// Pin `n` as an ANALOG input: the pad's digital input buffer off
    /// and its output disabled, no pull, no function, the isolation
    /// dropped - the converter reads the bare pad.
    static bool analog(uint8_t n, PinPull pull = PinPull::none) {
        if (!bonded(n)) {
            return false;
        }
        (void)ready();
        oe_clear_for(n);
        ctrl(n) = static_cast<uint32_t>(PinFunction::none) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
        uint32_t v = PADS_BANK0_GPIO0_OD_BITS;
        if (pull == PinPull::up || pull == PinPull::keeper) { v |= PADS_BANK0_GPIO0_PUE_BITS; }
        if (pull == PinPull::down || pull == PinPull::keeper) { v |= PADS_BANK0_GPIO0_PDE_BITS; }
        pad(n) = v;
        return true;
    }

    /// Back to the reset state: no owner, SIO's output enable off, and
    /// the pad ISOLATED again with its pull-down and its input buffer
    /// off - which is what a pad of this chip looks like out of reset.
    static bool release(uint8_t n) {
        if (!bonded(n)) {
            return false;
        }
        oe_clear_for(n);
        ctrl(n) = static_cast<uint32_t>(PinFunction::none) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
        pad(n) = PADS_BANK0_GPIO0_RESET;
        return true;
    }

    /// Every pin of `mask` (the low half of the bank) a software-driven
    /// output at once: SIO on each, then the output enables in one word.
    /// Pins the package has not got are skipped, here and in the enable.
    static void outputs(uint32_t mask, const PinConfig& cfg = {}) {
        uint32_t taken = 0;
        for (uint8_t n = 0; n < 32u; ++n) {
            if ((mask & (1UL << n)) != 0u && function(n, PinFunction::sio, cfg)) {
                taken |= 1UL << n;
            }
        }
        oe_set(taken);
    }

    /// The same for the high half: bit 0 of `mask` is GP32.
    static void outputs_hi(uint32_t mask, const PinConfig& cfg = {}) {
        uint32_t taken = 0;
        for (uint8_t n = 32u; n < gpio_count_max; ++n) {
            if ((mask & (1UL << (n - 32u))) != 0u && function(n, PinFunction::sio, cfg)) {
                taken |= 1UL << (n - 32u);
            }
        }
        oe_set_hi(taken);
    }

private:
    static void oe_clear_for(uint8_t n) {
        if (n < 32u) {
            oe_clear(1UL << n);
        } else {
            oe_clear_hi(1UL << (n - 32u));
        }
    }
};

/// A pin named at RUN TIME - what a bus request carries as its chip
/// select or D/C line: the pin number, or none. Every verb is a SIO word
/// access on the pin's bit, in whichever half it falls; a null reference
/// does nothing and reads false.
struct PinRef {
    uint8_t pin = 0xFFu;

    constexpr bool valid() const { return pin < gpio_count_max; }
    void set() const {
        if (!valid()) { return; }
        if (pin < 32u) { SIO->GPIO_OUT_SET = 1UL << pin; }
        else { SIO->GPIO_HI_OUT_SET = 1UL << (pin - 32u); }
    }
    void clear() const {
        if (!valid()) { return; }
        if (pin < 32u) { SIO->GPIO_OUT_CLR = 1UL << pin; }
        else { SIO->GPIO_HI_OUT_CLR = 1UL << (pin - 32u); }
    }
    void toggle() const {
        if (!valid()) { return; }
        if (pin < 32u) { SIO->GPIO_OUT_XOR = 1UL << pin; }
        else { SIO->GPIO_HI_OUT_XOR = 1UL << (pin - 32u); }
    }
    bool read() const {
        if (!valid()) { return false; }
        return pin < 32u ? (SIO->GPIO_IN & (1UL << pin)) != 0u
                         : (SIO->GPIO_HI_IN & (1UL << (pin - 32u))) != 0u;
    }
    bool read_out() const {
        if (!valid()) { return false; }
        return pin < 32u ? (SIO->GPIO_OUT & (1UL << pin)) != 0u
                         : (SIO->GPIO_HI_OUT & (1UL << (pin - 32u))) != 0u;
    }
};

/**
 * One pin as a type. Also a PwmChannel of one level
 * (util/pwm_channel.hpp): `max` 1, `duty(v)` = set or clear.
 *
 * The configuring verbs answer bool for the package's sake (the file
 * header): true when the pad was written, false when this chip has not
 * got it - which a build that states its package never sees, because
 * then the refusal is the static_assert below.
 */
template <uint8_t n>
struct Pin {
    static_assert(n < gpio_count_max,
                  "the RP2350's user bank has at most 48 pins, GPIO0..GPIO47");
    static_assert(!package_known || n < gpio_count,
                  "this pin is not bonded in the package this image is built for: the "
                  "QFN-60 brings out GPIO0..GPIO29, the QFN-80 GPIO0..GPIO47 "
                  "(rp2350/CMakeLists.txt states which)");

    static constexpr uint8_t number = n;
    /// The pin's bit in ITS OWN half of the SIO registers.
    static constexpr uint32_t mask = 1UL << (n < 32u ? n : n - 32u);
    /// Which half: false for GPIO0..31, true for GPIO32..47.
    static constexpr bool high_half = n >= 32u;

    static bool output(const PinConfig& cfg = {}) {
        if (!Gpio::function(n, PinFunction::sio, cfg)) {
            return false;
        }
        if constexpr (high_half) { Gpio::oe_set_hi(mask); } else { Gpio::oe_set(mask); }
        return true;
    }
    /// An output starting at `level`, the level written BEFORE the
    /// direction so the pin never shows the other one.
    static bool output(bool level, const PinConfig& cfg = {}) {
        if (level) { set(); } else { clear(); }
        return output(cfg);
    }
    static bool input(PinPull pull = PinPull::none) {
        if constexpr (high_half) { Gpio::oe_clear_hi(mask); } else { Gpio::oe_clear(mask); }
        return Gpio::function(n, PinFunction::sio, {.pull = pull});
    }
    /// Hand the pin to a peripheral.
    static bool function(PinFunction fn, const PinConfig& cfg = {}) {
        return Gpio::function(n, fn, cfg);
    }
    static bool release() { return Gpio::release(n); }
    /// The bare pad for the converter, a pull optional.
    static bool analog(PinPull pull = PinPull::none) { return Gpio::analog(n, pull); }

    /// Whether this chip brings the pad out (device.hpp's package fact,
    /// or SYSINFO's answer where the build stated none).
    static bool bonded() { return Gpio::bonded(n); }

    /// This pin as a run-time reference (a bus request's select line).
    static constexpr PinRef ref() { return PinRef{n}; }

    static void set() {
        if constexpr (high_half) { SIO->GPIO_HI_OUT_SET = mask; } else { SIO->GPIO_OUT_SET = mask; }
    }
    static void clear() {
        if constexpr (high_half) { SIO->GPIO_HI_OUT_CLR = mask; } else { SIO->GPIO_OUT_CLR = mask; }
    }
    static void toggle() {
        if constexpr (high_half) { SIO->GPIO_HI_OUT_XOR = mask; } else { SIO->GPIO_OUT_XOR = mask; }
    }
    /// The pad's level, whoever owns it.
    static bool read() {
        if constexpr (high_half) { return (SIO->GPIO_HI_IN & mask) != 0u; }
        else { return (SIO->GPIO_IN & mask) != 0u; }
    }
    static bool read_out() {
        if constexpr (high_half) { return (SIO->GPIO_HI_OUT & mask) != 0u; }
        else { return (SIO->GPIO_OUT & mask) != 0u; }
    }
    static bool is_output() {
        if constexpr (high_half) { return (SIO->GPIO_HI_OE & mask) != 0u; }
        else { return (SIO->GPIO_OE & mask) != 0u; }
    }
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

    /// Whether the pad is cut off from the digital logic (9.11's ISO
    /// latch): true out of reset, false once a configuring verb has
    /// written the pad.
    static bool isolated() { return (Gpio::pad(n) & PADS_BANK0_GPIO0_ISO_BITS) != 0u; }

    // PwmChannel of one level.
    static constexpr uint16_t max = 1;
    static void duty(uint16_t v) { if (v) { set(); } else { clear(); } }
};

} // namespace brio
