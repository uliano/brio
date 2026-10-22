/*
 * pin.hpp
 *
 * The STM32G0's I/O pins (GPIO, RM0444 ch. 7): what a program uses to
 * drive an LED, read a button, or hand a pad to a peripheral. A pin is a
 * type, `Pin<'A', 5>` for PA5, with the letter and the number fixed when
 * the program is compiled; its functions configure the pin and drive or
 * read it. The header has the two faces every target's pin header
 * offers:
 *
 *  Port<'A'>       the port as a resource - one GPIOx block: 16-bit mask
 *                  operations on ODR/IDR through the atomic BSRR/BRR
 *                  set/reset registers, the bus clock that gates the
 *                  block, and the per-pin mode/type/speed/pull/AF fields
 *                  written for a mask of pins at once;
 *  Pin<'A', 5>     the per-pin face: direction and value, the
 *                  configuring functions, the alternate-function
 *                  handoff, a PwmChannel (max 1) and a PinRef factory.
 *
 *   using Led = brio::Pin<'A', 5>;    // PA5, LD4 on the Nucleo-64
 *   Led::output();
 *   Led::toggle();
 *   Button::input(brio::PinPull::up);
 *   Tx::function(brio::PinFunction::af1);   // USART2_TX
 *
 * Which context may call what. The value functions (set, clear, toggle,
 * read) are single BSRR/BRR stores or IDR loads, atomic by the silicon,
 * and are safe from any context: an interrupt handler may toggle a pin
 * while the main loop drives another of the same port, which is what
 * PinRef's set()/clear() inside a bus active object's request depend
 * on. The configuring functions (output, input, analog, function,
 * release, pull, configure_mask) are read-modify-write sequences on
 * MODER/OTYPER/OSPEEDR/PUPDR/AFR, registers with no set/clear twins, so
 * two of them racing on one port from two contexts (a handler and the
 * loop) can lose a field: call them from setup and from an active
 * object's handler, never from an interrupt handler that may interrupt
 * another configuring call. None of them waits beyond the port clock's
 * two-cycle enable readback, and none masks interrupts.
 *
 * Three facts of this silicon shape everything below.
 *
 * 1. The port has a clock, and it is off at reset. RCC_IOPENR.GPIOxEN
 *    gates the whole block (5.2.17): with it clear every register of the
 *    port reads as zero and ignores writes, silently. So every
 *    configuring function here switches the port's clock on first (an
 *    idempotent read-modify-write of one RCC bit plus the readback that
 *    covers the two-cycle enable delay); the value functions (set/clear/
 *    toggle/read) do not, because a pin one has configured has a clocked
 *    port by construction. Nothing here ever turns a port clock off - a
 *    second pin of the same port would lose its block; releasing a port
 *    is a program-wide decision (Port<L>::clock(false) exists for it).
 * 2. The input buffer is always on in input, output and AF modes and off
 *    in analog mode (7.3.1). No enable bit to remember: read() reads
 *    the pad whenever the pin is not analog. Analog mode is the reset
 *    state of every pin except PA13/PA14 (SWD) and is also the
 *    low-power parking state.
 * 3. There is no pin interrupt in GPIO. Edge and level senses are the
 *    EXTI's (ch. 13), reached through its own multiplexer, and
 *    stm32g0/exti.hpp is what owns them. Alternate functions are a
 *    per-pin 4-bit number (AFRL/AFRH), and which peripheral signal AFn
 *    means on a given pad is a table of the datasheet (DS13560 tables
 *    13..24), not of the reference manual: a peripheral driver's pin
 *    claim names the AF and the app's static_asserts cannot check it
 *    against a header symbol, because the device header does not carry
 *    the pin table at all. The bench is the check.
 *
 * The package fact is a port-level one, read off the device header in
 * stm32g0/device_tables.hpp: ports A, B, C, D and F exist on every part,
 * E only on the G0B1/G0C1 class. Which pins of a present port a package
 * bonds is finer than that and stays open (a Pin on an unbonded pad
 * configures a register nobody wired).
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

/// Whether this device bonds out GPIO port `letter` at all (the device
/// header's GPIOx_BASE is the authority; stm32g0/device_tables.hpp).
constexpr bool port_exists(char letter) { return gpio_port_present(letter); }

/// Runtime pin descriptor: lets a pin chosen at compile time travel
/// inside a request event (the CS/DC of a SPI transaction, asserted by
/// the bus AO and not by its client). A null PinRef (the default) means
/// "no such pin": set/clear are no-ops, so an optional pin costs one
/// branch. Build one with Pin<...>::ref(). The stores are BSRR/BRR:
/// atomic on the silicon, legal from any context.
///
/// The two edges are forced inline: left to -Os, gcc keeps the body
/// behind its call sites, and an edge costs a BL and a return on top of
/// its six instructions - four times a transaction on the SPI host,
/// counted in the release listing (docs/stm32g0/spi.md). Inline, an
/// edge is the port pointer's load, the null test, the mask's load and
/// one BSRR or BRR store (7.4.7, 7.4.11: write-one registers, so no
/// read and no window); the null test stays a predictable branch,
/// which is what makes an absent D/C pin cost one branch and not a
/// verb.
struct PinRef {
    GPIO_TypeDef* port = nullptr;
    uint32_t mask = 0;

    [[gnu::always_inline]] void set() const {
        if (port != nullptr) {
            port->BSRR = mask;
        }
    }
    [[gnu::always_inline]] void clear() const {
        if (port != nullptr) {
            port->BRR = mask;
        }
    }
    constexpr bool valid() const { return port != nullptr; }
};

// ---- pin configuration vocabulary (7.4.1 .. 7.4.11) ---------------------------

/// GPIOx_MODER, two bits per pin.
enum class PinMode : uint8_t { input = 0, output = 1, alternate = 2, analog = 3 };

/// GPIOx_PUPDR. Reserved code 3 is not spelled.
enum class PinPull : uint8_t { none = 0, up = 1, down = 2 };

/// GPIOx_OSPEEDR: the output driver's slew class (the datasheet's
/// tables give the frequency each reaches per load and supply). `low`
/// is the reset value of every pin but PA13.
enum class PinSpeed : uint8_t { low = 0, medium = 1, high = 2, very_high = 3 };

/// GPIOx_AFRL/AFRH: the alternate function NUMBER. Which signal it is on
/// a given pad is the datasheet's table, not this enum's business.
enum class PinFunction : uint8_t {
    af0 = 0, af1, af2, af3, af4, af5, af6, af7,
    af8, af9, af10, af11, af12, af13, af14, af15,
};

/// What a configuring verb writes besides the mode. Open-drain applies to
/// output and alternate modes (7.4.2); the pull to any mode but analog,
/// where the chapter says it is left to the user to keep it off.
struct PinConfig {
    PinPull pull = PinPull::none;
    bool open_drain = false;
    PinSpeed speed = PinSpeed::low;
};

/// A pin claim a peripheral driver can carry in a constexpr config: the
/// port letter, the pin number and the AF the datasheet gives that
/// signal on that pad.
struct PinSel {
    char port = 0;
    uint8_t pin = 0;
    PinFunction function = PinFunction::af0;

    constexpr bool valid() const { return port_exists(port) && pin < 16u; }
};

// ---- the port resource ------------------------------------------------------

/// Port<'A'>: one GPIOx block - the mask operations a single pin cannot
/// express and the field engine every Pin<L, n> delegates to.
template <char L>
struct Port {
    static_assert(port_exists(L),
                  "brio Port: this device has no GPIO port of that letter (the device "
                  "header declares no GPIOx_BASE for it; ports A..D and F exist on every "
                  "STM32G0, E only on the G0B1/G0C1 class)");

    Port() = delete;

    static constexpr char letter = L;

    static GPIO_TypeDef& regs() { return *reinterpret_cast<GPIO_TypeDef*>(gpio_port_base(L)); }

    // ---- the bus clock (RCC_IOPENR, 5.2.17) ------------------------------------
    static void clock(bool on) { Rcc::io_clock(L, on); }
    static bool clock() { return Rcc::io_clock(L); }

    // ---- values ---------------------------------------------------------------
    static uint32_t in() { return regs().IDR; }
    static uint32_t out() { return regs().ODR; }
    static void out_set(uint32_t m) { regs().BSRR = m & 0xFFFFu; }
    static void out_clear(uint32_t m) { regs().BRR = m & 0xFFFFu; }
    /// One BSRR store: the set half carries the pins currently low, the
    /// reset half the pins currently high - atomic against a handler
    /// touching OTHER pins, unlike an ODR read-modify-write.
    static void out_toggle(uint32_t m) {
        const uint32_t odr = regs().ODR;
        regs().BSRR = ((odr & m) << 16) | (~odr & m & 0xFFFFu);
    }

    // ---- the field engine -------------------------------------------------------
    /// Write a 2-bit field (MODER/OSPEEDR/PUPDR layout) for every pin in
    /// `pins`. One read-modify-write per register.
    static void write_field2(volatile uint32_t& reg, uint32_t pins, uint8_t code) {
        uint32_t clear = 0, set = 0;
        for (uint8_t p = 0; p < 16; ++p) {
            if ((pins & (1u << p)) != 0u) {
                clear |= 0x3u << (2u * p);
                set |= static_cast<uint32_t>(code & 0x3u) << (2u * p);
            }
        }
        reg = (reg & ~clear) | set;
    }

    /// The whole configuration of a mask of pins in `mode`: AF nibble
    /// first (7.4.9's note: the AF is selected before the mode switches,
    /// so the pad never spends a cycle on the wrong function), then
    /// type, speed, pull, and the mode LAST - the store that hands the
    /// pad over. Turns the port clock on first.
    static void configure_mask(uint32_t pins, PinMode mode, const PinConfig& cfg,
                               PinFunction fn = PinFunction::af0) {
        clock(true);
        GPIO_TypeDef& g = regs();
        if (mode == PinMode::alternate) {
            uint32_t lo_clear = 0, lo_set = 0, hi_clear = 0, hi_set = 0;
            for (uint8_t p = 0; p < 16; ++p) {
                if ((pins & (1u << p)) == 0u) {
                    continue;
                }
                const uint32_t shift = 4u * (p & 7u);
                const uint32_t nibble = static_cast<uint32_t>(fn) << shift;
                if (p < 8) {
                    lo_clear |= 0xFu << shift;
                    lo_set |= nibble;
                } else {
                    hi_clear |= 0xFu << shift;
                    hi_set |= nibble;
                }
            }
            if (lo_clear != 0u) {
                g.AFR[0] = (g.AFR[0] & ~lo_clear) | lo_set;
            }
            if (hi_clear != 0u) {
                g.AFR[1] = (g.AFR[1] & ~hi_clear) | hi_set;
            }
        }
        const uint32_t p16 = pins & 0xFFFFu;
        g.OTYPER = cfg.open_drain ? (g.OTYPER | p16) : (g.OTYPER & ~p16);
        write_field2(g.OSPEEDR, p16, static_cast<uint8_t>(cfg.speed));
        write_field2(g.PUPDR, p16, static_cast<uint8_t>(mode == PinMode::analog ? PinPull::none : cfg.pull));
        write_field2(g.MODER, p16, static_cast<uint8_t>(mode));
    }
};

// ---- the per-pin face ---------------------------------------------------------

template <char PortLetter, uint8_t PinNum>
struct Pin {
    static_assert(PinNum < 16, "an STM32 GPIO port has 16 pins");
    using P = Port<PortLetter>;

    Pin() = delete;

    static constexpr char port_letter = PortLetter;
    static constexpr uint8_t pin_number = PinNum;
    static constexpr uint32_t mask = 1u << PinNum;

    static GPIO_TypeDef& port() { return P::regs(); }

    /// Runtime descriptor for events (see PinRef).
    static PinRef ref() { return {&port(), mask}; }

    /// PwmChannel role: a pin is a one-step dimmer.
    static constexpr uint16_t max = 1;
    static void duty(uint16_t v) { if (v) set(); else clear(); }

    // ---- values (BSRR/BRR: atomic, any context) ----------------------------------
    /// Drive the pad high / low / the other way, or read it: one store or
    /// one load, atomic, from any context. The port must have been
    /// configured first (its clock is then open); a store into an
    /// unclocked port is dropped in silence.
    static void set()    { port().BSRR = mask; }
    static void clear()  { port().BRR = mask; }
    static void toggle() { P::out_toggle(mask); }
    static bool read()   { return (port().IDR & mask) != 0u; }
    static bool read_out() { return (port().ODR & mask) != 0u; }
    static bool is_output() {
        return ((port().MODER >> (2u * PinNum)) & 0x3u) == static_cast<uint32_t>(PinMode::output);
    }

    // ---- configuration (setup / kernel time) ------------------------------------
    /// Push-pull output, low speed, no pull, driving whatever ODR holds.
    /// Opens the port's clock; a read-modify-write of the port's
    /// registers, so setup or an active object's handler, not an
    /// interrupt handler racing another configuring call (the header
    /// comment says why).
    static void output(const PinConfig& cfg = {}) { P::configure_mask(mask, PinMode::output, cfg); }
    /// Output that starts at a known level: the port's clock is opened
    /// first (a BSRR/BRR store into an unclocked port is dropped in
    /// silence, 5.2.17 - fact 1 above), then the value is written before
    /// the mode switches, so the pad never glitches through the old ODR.
    /// The second clock(true) inside output(cfg) is idempotent.
    static void output(bool level, const PinConfig& cfg = {}) {
        P::clock(true);
        if (level) set(); else clear();
        output(cfg);
    }
    static void input(PinPull p = PinPull::none) {
        P::configure_mask(mask, PinMode::input, {.pull = p});
    }
    /// Analog mode: input buffer off, the reset and lowest-power state.
    static void analog() { P::configure_mask(mask, PinMode::analog, {}); }
    /// Hand the pad to a peripheral: AF `fn`, with the driver options in
    /// `cfg` (an open-drain I2C line, a fast SPI clock). The number is
    /// the datasheet's for that signal on that pad and nothing here can
    /// check it; a peripheral's input reads a pad that is not in this
    /// mode as low, whatever its level (docs/stm32g0/port.md).
    static void function(PinFunction fn, const PinConfig& cfg = {}) {
        P::configure_mask(mask, PinMode::alternate, cfg, fn);
    }
    /// Give the pad back to its reset state (analog, no pull).
    static void release() { analog(); }
    static bool has_function() {
        return ((port().MODER >> (2u * PinNum)) & 0x3u) == static_cast<uint32_t>(PinMode::alternate);
    }
    static void pull(PinPull p) {
        P::write_field2(port().PUPDR, mask, static_cast<uint8_t>(p));
    }
};

/**
 * The dead-battery pull-downs, and why a GPIO header carries them.
 *
 * 7.3.16 and 5.x's SYSCFG_CFGR1: "Upon power on, internal pull-down
 * resistors on UCPD1 CC1 and CC2 pins are enabled (connected)", and the
 * only way to let go of them is the strobe bit. On this family those
 * pads are PA8 and PB15 for UCPD1 and PD0 and PD2 for UCPD2 - four
 * ordinary-looking GPIOs that come out of a power-on with a Type-C Rd on
 * them, some kilohms against the tens of kilohms of the port's own
 * pull-up. So a pad that will not follow its own pull is not always a
 * desk fault; on exactly these four it is the reset state, and the
 * chapter's own advice is "in applications that do not use the UCPD
 * peripheral, disable the internal pull-down resistor Rd at startup".
 *
 * It lives here because its SUBJECT is a pad and this stratum has no
 * UCPD driver to own it; the day one is built, this verb is its
 * strobe's first half and moves there. It opens SYSCFG's clock gate for
 * the same reason vref.hpp's init() does - the register is behind it.
 *
 * `instance` is the manual's own 1-based UCPD numbering. False for an
 * instance this part has not got.
 */
inline bool ucpd_dead_battery(uint8_t instance, bool on) {
    if (on || instance < 1u || instance > 2u) {
        return false;   // the strobe only ever releases; see 5.x's CFGR1
    }
    uint32_t bit = 0;
#if defined(SYSCFG_CFGR1_UCPD1_STROBE)
    if (instance == 1u) {
        bit = SYSCFG_CFGR1_UCPD1_STROBE;
    }
#endif
#if defined(SYSCFG_CFGR1_UCPD2_STROBE)
    if (instance == 2u) {
        bit = SYSCFG_CFGR1_UCPD2_STROBE;
    }
#endif
    if (bit == 0u) {
        return false;
    }
    Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN, true);
    SYSCFG->CFGR1 |= bit;
    return true;
}

/// A compile-time set of pins of possibly different ports - the mask
/// form is per port, so this is the per-port grouping helper the bus
/// drivers use for "these lines, all inputs now".
template <typename... Pins>
struct PinSet {
    static void configure(PinMode mode, const PinConfig& cfg = {}) {
        (Pins::P::configure_mask(Pins::mask, mode, cfg), ...);
    }
};

} // namespace brio
