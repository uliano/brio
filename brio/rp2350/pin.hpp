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
 *    per-pin functions above that - plus the FOUR OVERRIDES that sit
 *    between the chosen function and the pad (the output level, the
 *    output enable, the input the function sees and the interrupt the
 *    pad raises, each of them passable, invertible or forced either way)
 *    and a STATUS register that reads back what actually reached the
 *    pad. The low function numbers are the same on every pin they reach
 *    (F5 is SIO everywhere), and WHICH instance and signal a pin carries
 *    is the per-peripheral table each driver keeps; the same block holds
 *    THE PIN INTERRUPTS (9.5), four events per pin over six registers
 *    per destination;
 *  - PADS_BANK0 (9.11): a register per pin for the pad's electrical
 *    behaviour - drive strength, slew, hysteresis, the pull-up, the
 *    pull-down (both together = the bus keeper), the input buffer
 *    enable, an output DISABLE that overrides whoever owns the pad - AND
 *    THE ISOLATION LATCH, which is this chip's own;
 *  - SIO (3.1.3): the processors' own path, one cycle per access, with
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
 * THE ERRATUM'S OWN WORKAROUND IS A VERB HERE. The leakage exists only
 * while the input buffer is enabled and stops the moment it is disabled,
 * so a pad that must be held down by its pull-down keeps the buffer OFF
 * and is read with the buffer enabled for the length of the read alone:
 * `input({.pull = PinPull::down, .input_enable = false})` and then
 * `read_pulsed()`. That is the pair the errata sheet prescribes, and on
 * this stepping it is the ONLY way a pull-down holds a pad that nothing
 * else drives.
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

/// What the IO mux does to a signal on its way past (9.4's OUTOVER,
/// INOVER and IRQOVER): let it through, invert it, or force it either
/// way whatever the function says. The forced values are what makes a
/// relaxation oscillator out of a clock output and an RC network (8.1.2.5),
/// and what lets a program test an interrupt path with no wire on it.
enum class PinOverride : uint8_t {
    pass = IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_NORMAL,
    invert = IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_INVERT,
    low = IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_LOW,
    high = IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_HIGH,
};

/// The same for the OUTPUT ENABLE (OEOVER), whose two forced values are
/// named for what they do rather than for a level.
enum class PinOeOverride : uint8_t {
    pass = IO_BANK0_GPIO0_CTRL_OEOVER_VALUE_NORMAL,
    invert = IO_BANK0_GPIO0_CTRL_OEOVER_VALUE_INVERT,
    disable = IO_BANK0_GPIO0_CTRL_OEOVER_VALUE_DISABLE,
    enable = IO_BANK0_GPIO0_CTRL_OEOVER_VALUE_ENABLE,
};

/// The input thresholds the whole bank is judged by (9.6): a FACT OF THE
/// BOARD, since both banks share one IOVDD supply. Read here and written
/// by nothing - the datasheet's warning is that driving pads at more
/// than 1.8 V with the 1.8 V thresholds selected may damage the chip, so
/// the value belongs to whoever wired the supply and not to a program.
enum class PadVoltage : uint8_t {
    v3v3 = PADS_BANK0_VOLTAGE_SELECT_VALUE_3V3,
    v1v8 = PADS_BANK0_VOLTAGE_SELECT_VALUE_1V8,
};

/// What actually reached the pad, and what came back (9.4's STATUS
/// register): the four signals AFTER the function select and the
/// overrides. It is the one place a program can see the difference
/// between what it asked for and what the pad has.
struct PinStatus {
    bool out_to_pad = false;    ///< the level the pad is being driven to
    bool oe_to_pad = false;     ///< whether it is being driven at all
    bool in_from_pad = false;   ///< the level read back off the pad
    bool irq_to_proc = false;   ///< this pin's interrupt, after IRQOVER
};

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
    static void oe_toggle(uint32_t mask) { SIO->GPIO_OE_XOR = mask; }
    static void oe_toggle_hi(uint32_t mask) { SIO->GPIO_HI_OE_XOR = mask; }

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

    // ---- the four overrides, between the function and the pad -----------

    /// The output level the chosen function drives, passed, inverted or
    /// forced. Forcing one leaves the function's own output where it is:
    /// nothing is lost, and `pass` puts it back.
    static void out_override(uint8_t n, PinOverride o) {
        hw_write_masked(ctrl(n), static_cast<uint32_t>(o) << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB,
                        IO_BANK0_GPIO0_CTRL_OUTOVER_BITS);
    }
    static PinOverride out_override(uint8_t n) {
        return static_cast<PinOverride>((ctrl(n) & IO_BANK0_GPIO0_CTRL_OUTOVER_BITS) >>
                                        IO_BANK0_GPIO0_CTRL_OUTOVER_LSB);
    }
    /// The output ENABLE, the one override whose forced values are
    /// `disable` (the pad floats whatever the function wants) and
    /// `enable` (the pad is driven whatever the function wants).
    static void oe_override(uint8_t n, PinOeOverride o) {
        hw_write_masked(ctrl(n), static_cast<uint32_t>(o) << IO_BANK0_GPIO0_CTRL_OEOVER_LSB,
                        IO_BANK0_GPIO0_CTRL_OEOVER_BITS);
    }
    static PinOeOverride oe_override(uint8_t n) {
        return static_cast<PinOeOverride>((ctrl(n) & IO_BANK0_GPIO0_CTRL_OEOVER_BITS) >>
                                          IO_BANK0_GPIO0_CTRL_OEOVER_LSB);
    }
    /// What the FUNCTION sees coming in - which is not what SIO's
    /// GPIO_IN reads: the input override sits between the pad and the
    /// peripheral, and GPIO_IN is the pad.
    static void in_override(uint8_t n, PinOverride o) {
        hw_write_masked(ctrl(n), static_cast<uint32_t>(o) << IO_BANK0_GPIO0_CTRL_INOVER_LSB,
                        IO_BANK0_GPIO0_CTRL_INOVER_BITS);
    }
    static PinOverride in_override(uint8_t n) {
        return static_cast<PinOverride>((ctrl(n) & IO_BANK0_GPIO0_CTRL_INOVER_BITS) >>
                                        IO_BANK0_GPIO0_CTRL_INOVER_LSB);
    }
    /// What the INTERRUPT logic of 9.5 sees: forcing this one high is a
    /// pin interrupt raised with nothing on the wire.
    static void irq_override(uint8_t n, PinOverride o) {
        hw_write_masked(ctrl(n), static_cast<uint32_t>(o) << IO_BANK0_GPIO0_CTRL_IRQOVER_LSB,
                        IO_BANK0_GPIO0_CTRL_IRQOVER_BITS);
    }
    static PinOverride irq_override(uint8_t n) {
        return static_cast<PinOverride>((ctrl(n) & IO_BANK0_GPIO0_CTRL_IRQOVER_BITS) >>
                                        IO_BANK0_GPIO0_CTRL_IRQOVER_LSB);
    }

    /// The four signals as they stand at the pad (9.4's STATUS).
    static PinStatus pin_status(uint8_t n) {
        const uint32_t s = status(n);
        return PinStatus{
            .out_to_pad = (s & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS) != 0u,
            .oe_to_pad = (s & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS) != 0u,
            .in_from_pad = (s & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS) != 0u,
            .irq_to_proc = (s & IO_BANK0_GPIO0_STATUS_IRQTOPROC_BITS) != 0u};
    }

    // ---- the pad's own controls (9.6, 9.7) ------------------------------

    /// The pad's output driver off, whoever owns the pad (PADS.OD): the
    /// one control that outranks the function's output enable.
    static void output_disable(uint8_t n, bool off) {
        hw_write_masked(pad(n), off ? PADS_BANK0_GPIO0_OD_BITS : 0u, PADS_BANK0_GPIO0_OD_BITS);
    }
    static bool output_disabled(uint8_t n) { return (pad(n) & PADS_BANK0_GPIO0_OD_BITS) != 0u; }

    /// The input buffer alone (PADS.IE), which is E9's own switch: with
    /// it clear the pad leaks nothing and its pull-down holds; with it
    /// set the pad can be read. `Pin::read_pulsed()` is the pair.
    static void input_enable(uint8_t n, bool on) {
        hw_write_masked(pad(n), on ? PADS_BANK0_GPIO0_IE_BITS : 0u, PADS_BANK0_GPIO0_IE_BITS);
    }
    static bool input_enabled(uint8_t n) { return (pad(n) & PADS_BANK0_GPIO0_IE_BITS) != 0u; }

    /// The isolation latch of 9.7: setting it freezes the output level,
    /// the output enable and the pulls AS THEY ARE, and the pad keeps
    /// them until it is cleared - through a reset of the IO and PAD
    /// register blocks, and through a power-down of the switched core.
    /// Clearing it makes the latch transparent again.
    static void isolate(uint8_t n, bool on) {
        hw_write_masked(pad(n), on ? PADS_BANK0_GPIO0_ISO_BITS : 0u, PADS_BANK0_GPIO0_ISO_BITS);
    }
    static bool isolated(uint8_t n) { return (pad(n) & PADS_BANK0_GPIO0_ISO_BITS) != 0u; }

    /// The input thresholds the whole bank is judged by - a board fact,
    /// read and never written here (the enum says why).
    static PadVoltage voltage() {
        return static_cast<PadVoltage>(PADS_BANK0->VOLTAGE_SELECT &
                                       PADS_BANK0_VOLTAGE_SELECT_BITS);
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
    /// An input whose pad is spelled out - the form E9's workaround
    /// wants, `{.pull = PinPull::down, .input_enable = false}`.
    static bool input(const PinConfig& cfg) {
        if constexpr (high_half) { Gpio::oe_clear_hi(mask); } else { Gpio::oe_clear(mask); }
        return Gpio::function(n, PinFunction::sio, cfg);
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
    /// Freeze the pad as it stands (9.7), or make the latch transparent
    /// again. What is frozen is the output level, the output enable and
    /// the pulls; the input keeps coming back.
    static void isolate(bool on) { Gpio::isolate(n, on); }

    /// THE READ THAT WORKS UNDER ERRATUM RP2350-E9: the input buffer is
    /// enabled for this read alone and disabled again, so a pad held by
    /// its pull-down is never left leaking. The pair with a pad
    /// configured `{.input_enable = false}`; on a pad whose buffer is on
    /// already it costs two pad writes and reads the same thing.
    ///
    /// NO SETTLE IS NEEDED BETWEEN THE ENABLE AND THE READ, and that is
    /// a measurement and not an assumption: the enable is a write to the
    /// pad register on the APB, which takes four cycles to land, while
    /// the read is a single-cycle SIO access - by the time it happens
    /// the buffer is awake and the synchronizer has the pad. The first
    /// of eight consecutive samples already reads the pad's level
    /// (docs/rp2350/pin.md).
    static bool read_pulsed() {
        const bool was_on = Gpio::input_enabled(n);
        Gpio::input_enable(n, true);
        const bool level = read();
        if (!was_on) {
            Gpio::input_enable(n, false);
        }
        return level;
    }
    /// The input buffer alone, for a caller doing its own pulsing.
    static void input_enable(bool on) { Gpio::input_enable(n, on); }
    static bool input_enabled() { return Gpio::input_enabled(n); }
    /// The pad's output driver off, over the function's head.
    static void output_disable(bool off) { Gpio::output_disable(n, off); }
    static bool output_disabled() { return Gpio::output_disabled(n); }

    /// The four overrides between the function and the pad.
    static void out_override(PinOverride o) { Gpio::out_override(n, o); }
    static PinOverride out_override() { return Gpio::out_override(n); }
    static void oe_override(PinOeOverride o) { Gpio::oe_override(n, o); }
    static PinOeOverride oe_override() { return Gpio::oe_override(n); }
    static void in_override(PinOverride o) { Gpio::in_override(n, o); }
    static PinOverride in_override() { return Gpio::in_override(n); }
    static void irq_override(PinOverride o) { Gpio::irq_override(n, o); }
    static PinOverride irq_override() { return Gpio::irq_override(n); }

    /// What reached the pad and what came back (9.4's STATUS).
    static PinStatus status() { return Gpio::pin_status(n); }

    // PwmChannel of one level.
    static constexpr uint16_t max = 1;
    static void duty(uint16_t v) { if (v) { set(); } else { clear(); } }
};

// ---- the pin interrupts (9.5) ----------------------------------------------

/**
 * The four things a pad can raise, as a SET: the two levels, which are
 * not latched and follow the pad, and the two edges, which are latched
 * in INTR and stay until they are written away.
 *
 * A set is a mask of these, and `|` composes it - which is what the
 * registers hold: four bits per pin, eight pins per word, six words for
 * the forty-eight of this package.
 */
enum class PinEvent : uint8_t {
    level_low = IO_BANK0_INTR0_GPIO0_LEVEL_LOW_BITS,
    level_high = IO_BANK0_INTR0_GPIO0_LEVEL_HIGH_BITS,
    edge_low = IO_BANK0_INTR0_GPIO0_EDGE_LOW_BITS,
    edge_high = IO_BANK0_INTR0_GPIO0_EDGE_HIGH_BITS,
};

/// A set of them, and the only arithmetic it needs.
struct PinEvents {
    uint8_t bits = 0;
    constexpr bool has(PinEvent e) const { return (bits & static_cast<uint8_t>(e)) != 0u; }
    constexpr bool none() const { return bits == 0u; }
    constexpr bool operator==(const PinEvents&) const = default;
};

constexpr PinEvents operator|(PinEvent a, PinEvent b) {
    return {static_cast<uint8_t>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b))};
}
constexpr PinEvents operator|(PinEvents a, PinEvent b) {
    return {static_cast<uint8_t>(a.bits | static_cast<uint8_t>(b))};
}
constexpr PinEvents pin_events(PinEvent e) { return {static_cast<uint8_t>(e)}; }
/// Both edges - the set an "it changed" handler wants.
inline constexpr PinEvents pin_edges{static_cast<uint8_t>(PinEvent::edge_low) |
                                     static_cast<uint8_t>(PinEvent::edge_high)};

/// Where a pin's events are sent. Each destination has its own enable,
/// force and status registers over the same INTR, and its own interrupt
/// line out of the block.
enum class PinIrqTarget : uint8_t {
    proc0 = 0,          ///< the core 0 line, IO_IRQ_BANK0 (`isr_io_bank0`)
    proc1 = 1,          ///< the core 1 line, the same number to the other NVIC
    dormant_wake = 2,   ///< the wake path out of DORMANT - the power chapter's
};

/**
 * The bank's interrupt controller (9.5).
 *
 * WHAT IS DIFFERENT FROM EVERY OTHER STRATUM'S PIN INTERRUPTS: nothing
 * is shared, so nothing is claimed. On the SAM's EIC and on the STM32's
 * EXTI a LINE is a resource several pads compete for, and a driver's
 * first job is to refuse the second claimant; here every one of the
 * forty-eight pins owns its own four event bits in its own destination's
 * own register, and the only thing they share is the interrupt LINE out
 * of the block - one per destination, which is why an ISR asks the
 * summary registers which pin rang.
 *
 * THE LEVELS ARE NOT LATCHED. A level event stands while the pad holds
 * that level and vanishes when it changes, so a handler that wants to
 * return must disable the event rather than clear it - clearing does
 * nothing at all. The edges ARE latched, in INTR, and clearing is a
 * write of one to the bit.
 *
 * BOTH SECURITY DOMAINS HAVE THEIR OWN LINE (twelve outputs in all,
 * 9.5), and everything in brio runs Secure: the verbs below drive the
 * Secure destinations, whose line is IO_IRQ_BANK0. Which pins a
 * Non-secure context may see at all is ACCESSCTRL's, a chapter this
 * stratum reads and does not write.
 */
struct PinIrq {
    PinIrq() = delete;

    /// Enable `events` on `pin` towards `target`. The pin's other
    /// events, and every other pin's, are untouched.
    static void enable(uint8_t pin, PinEvents events, PinIrqTarget target = PinIrqTarget::proc0) {
        hw_set(inte(pin, target), shifted(pin, events));
    }
    static void disable(uint8_t pin, PinEvents events, PinIrqTarget target = PinIrqTarget::proc0) {
        hw_clear(inte(pin, target), shifted(pin, events));
    }
    /// Which of the four are enabled towards `target`.
    static PinEvents enabled(uint8_t pin, PinIrqTarget target = PinIrqTarget::proc0) {
        return unshifted(pin, inte(pin, target));
    }

    /// What this pin is raising towards `target` right now (INTS: the
    /// raw events masked by the enables, with the forced ones in).
    static PinEvents status(uint8_t pin, PinIrqTarget target = PinIrqTarget::proc0) {
        return unshifted(pin, ints(pin, target));
    }
    /// The raw events (INTR), whether or not anybody asked for them: the
    /// two edges as they were latched, the two levels as they stand.
    static PinEvents raw(uint8_t pin) { return unshifted(pin, intr(pin)); }
    /// Clear the LATCHED events of `pin`. The levels are not latched and
    /// a write to them does nothing - the mask is taken as given and the
    /// hardware ignores what it must.
    static void clear(uint8_t pin, PinEvents events) { intr(pin) = shifted(pin, events); }

    /// Raise an event from software towards one destination (INTF),
    /// without a pad doing anything.
    ///
    /// IT STANDS UNTIL IT IS UNFORCED, AND IT BYPASSES THE ENABLE. The
    /// status register is "after masking AND FORCING" (9.11), so a
    /// forced event reaches the line whether or not that event is armed,
    /// and clearing does not touch it: a handler that means to return
    /// from a forced event must UNFORCE it - disarming the pad is not
    /// enough, which is what tells a force from a level. Measured on
    /// this silicon, where a forced edge with every enable cleared
    /// re-entered the handler for ever.
    static void force(uint8_t pin, PinEvents events, PinIrqTarget target = PinIrqTarget::proc0) {
        hw_set(intf(pin, target), shifted(pin, events));
    }
    static void unforce(uint8_t pin, PinEvents events, PinIrqTarget target = PinIrqTarget::proc0) {
        hw_clear(intf(pin, target), shifted(pin, events));
    }
    /// Which of this pin's events are being FORCED (INTF). A handler
    /// asks this, not INTS, to tell a standing event from a served one:
    /// the answer does not depend on a clear that may not have landed
    /// yet.
    static PinEvents forced(uint8_t pin, PinIrqTarget target = PinIrqTarget::proc0) {
        return unshifted(pin, intf(pin, target));
    }

    /// The summary registers of 9.5, this chip's addition: one bit per
    /// PIN, so a handler finds the pin that rang without reading six
    /// words. `half` 0 is GPIO0..31 and 1 is GPIO32..47.
    static uint32_t summary(PinIrqTarget target, uint8_t half) {
        return reg_at(IO_BANK0_BASE, summary_offset(target) + 4u * (half & 1u));
    }

    /// The line this destination's events come out on. proc0's and
    /// proc1's are the SAME NUMBER reaching two interrupt controllers -
    /// the rule of this chip - and an app binds it as `isr_io_bank0`.
    static constexpr IRQn_Type irq() { return IO_IRQ_BANK0_IRQn; }

    /// Disable every event of every pin towards `target`: what an ISR
    /// body's owner does before it hands the bank to somebody else, and
    /// what a suite does between letters.
    static void disable_all(PinIrqTarget target = PinIrqTarget::proc0) {
        for (uint8_t w = 0; w < words; ++w) {
            reg_at(IO_BANK0_BASE, inte_offset(target) + 4u * w) = 0u;
        }
    }

private:
    /// Six words of eight pins each cover the forty-eight of the larger
    /// package; the QFN-60's thirty use four of them and a half.
    static constexpr uint8_t words = 6;
    static constexpr uint32_t word_of(uint8_t pin) { return 4u * (pin / 8u); }
    static constexpr uint32_t shift_of(uint8_t pin) { return 4u * (pin % 8u); }
    static constexpr uint32_t shifted(uint8_t pin, PinEvents e) {
        return static_cast<uint32_t>(e.bits) << shift_of(pin);
    }
    static PinEvents unshifted(uint8_t pin, uint32_t reg) {
        return PinEvents{static_cast<uint8_t>((reg >> shift_of(pin)) & 0xFu)};
    }

    static constexpr uint32_t inte_offset(PinIrqTarget t) {
        switch (t) {
            case PinIrqTarget::proc1: return IO_BANK0_PROC1_INTE0_OFFSET;
            case PinIrqTarget::dormant_wake: return IO_BANK0_DORMANT_WAKE_INTE0_OFFSET;
            default: return IO_BANK0_PROC0_INTE0_OFFSET;
        }
    }
    static constexpr uint32_t intf_offset(PinIrqTarget t) {
        switch (t) {
            case PinIrqTarget::proc1: return IO_BANK0_PROC1_INTF0_OFFSET;
            case PinIrqTarget::dormant_wake: return IO_BANK0_DORMANT_WAKE_INTF0_OFFSET;
            default: return IO_BANK0_PROC0_INTF0_OFFSET;
        }
    }
    static constexpr uint32_t ints_offset(PinIrqTarget t) {
        switch (t) {
            case PinIrqTarget::proc1: return IO_BANK0_PROC1_INTS0_OFFSET;
            case PinIrqTarget::dormant_wake: return IO_BANK0_DORMANT_WAKE_INTS0_OFFSET;
            default: return IO_BANK0_PROC0_INTS0_OFFSET;
        }
    }
    static constexpr uint32_t summary_offset(PinIrqTarget t) {
        switch (t) {
            case PinIrqTarget::proc1: return IO_BANK0_IRQSUMMARY_PROC1_SECURE0_OFFSET;
            case PinIrqTarget::dormant_wake:
                return IO_BANK0_IRQSUMMARY_DORMANT_WAKE_SECURE0_OFFSET;
            default: return IO_BANK0_IRQSUMMARY_PROC0_SECURE0_OFFSET;
        }
    }

    static volatile uint32_t& inte(uint8_t pin, PinIrqTarget t) {
        return reg_at(IO_BANK0_BASE, inte_offset(t) + word_of(pin));
    }
    static volatile uint32_t& intf(uint8_t pin, PinIrqTarget t) {
        return reg_at(IO_BANK0_BASE, intf_offset(t) + word_of(pin));
    }
    static volatile uint32_t& ints(uint8_t pin, PinIrqTarget t) {
        return reg_at(IO_BANK0_BASE, ints_offset(t) + word_of(pin));
    }
    static volatile uint32_t& intr(uint8_t pin) {
        return reg_at(IO_BANK0_BASE, IO_BANK0_INTR0_OFFSET + word_of(pin));
    }
};

/**
 * One pad's interrupt, in the name the other strata use for the same
 * thing (`ExtInt<Pin>` on the SAM's EIC, the STM32s' EXTI and the
 * CH32s'): the pin interrupt as a TYPE, so an application names a pad
 * and not a line number.
 *
 * Here it is thinner than its namesakes, and the file header says why:
 * no line is shared, so there is nothing to claim and nothing to refuse.
 * What the type still buys is the PACKAGE CHECK the pad already carries
 * and one place for the ISR body - arm, the handler, disarm - written
 * once instead of at four call sites.
 *
 *   using Button = brio::ExtInt<brio::Pin<23>>;
 *   Button::arm(brio::pin_edges);              // the pad already an input
 *   brio::Irq::enable(Button::irq());
 *   extern "C" void isr_io_bank0() { if (Button::served()) { ... } }
 */
template <typename P, PinIrqTarget target = PinIrqTarget::proc0>
struct ExtInt {
    using Pad = P;
    static constexpr uint8_t pin = P::number;

    /// Arm `events` on this pad, with whatever is already latched
    /// cleared first so the first interrupt is news and not history.
    /// False when the package has not got the pad.
    static bool arm(PinEvents events) {
        if (!P::bonded()) {
            return false;
        }
        PinIrq::clear(pin, pin_edges);
        PinIrq::enable(pin, events, target);
        return true;
    }
    /// Disarm every event of this pad, latched or not.
    static void disarm() {
        PinIrq::disable(pin, all_events, target);
        PinIrq::clear(pin, pin_edges);
    }

    static PinEvents armed() { return PinIrq::enabled(pin, target); }
    static PinEvents pending() { return PinIrq::status(pin, target); }
    static PinEvents raw() { return PinIrq::raw(pin); }
    static void clear(PinEvents events = pin_edges) { PinIrq::clear(pin, events); }
    static void force(PinEvents events) { PinIrq::force(pin, events, target); }
    static void unforce(PinEvents events) { PinIrq::unforce(pin, events, target); }
    static PinEvents forced() { return PinIrq::forced(pin, target); }

    /// THE ISR BODY: which of this pad's events are standing, with the
    /// LATCHED ones cleared before it returns them - so a handler is
    /// told once per edge. An empty set means this pad is not why the
    /// line rang.
    ///
    /// TWO KINDS OF EVENT DO NOT GO AWAY WHEN THEY ARE CLEARED, and a
    /// handler that means to return must end each in its own way: a
    /// LEVEL, which is not latched and stands while the pad holds it,
    /// ends when the pad is DISARMED; an event FORCED through INTF
    /// stands whether or not it is armed, so only UNFORCING ends it.
    /// The served set names the first and `forced()` the second.
    [[gnu::always_inline]] static PinEvents served() {
        const PinEvents now = pending();
        if (!now.none()) {
            PinIrq::clear(pin, now);
        }
        return now;
    }

    static constexpr IRQn_Type irq() { return PinIrq::irq(); }

private:
    static constexpr PinEvents all_events{
        static_cast<uint8_t>(PinEvent::level_low) | static_cast<uint8_t>(PinEvent::level_high) |
        static_cast<uint8_t>(PinEvent::edge_low) | static_cast<uint8_t>(PinEvent::edge_high)};
};

} // namespace brio
