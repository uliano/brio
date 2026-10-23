/*
 * pin.hpp
 *
 * The STM32F4's I/O pins (GPIO, RM0090 ch. 8, RM0390 ch. 7, RM0383
 * ch. 8): what a program uses to drive an LED, read a button, or hand a
 * pad to a peripheral. A pin is a type, `Pin<'A', 5>` for PA5, with the
 * letter and the number fixed when the program is compiled; its
 * functions configure the pin and drive or read it. The header has the
 * two faces every target's pin header offers:
 *
 *  Port<'A'>       the port as a resource - one GPIOx block: 16-bit mask
 *                  operations on ODR/IDR through the atomic BSRR
 *                  set/reset register, the bus clock that gates the
 *                  block, and the per-pin mode/type/speed/pull/AF fields
 *                  written for a mask of pins at once;
 *  Pin<'A', 5>     the per-pin face: direction and value, the
 *                  configuring functions, the alternate-function
 *                  handoff, a PwmChannel (util/pwm_channel.hpp's
 *                  one dimmable output, here of one step: max 1, so
 *                  duty(0) is low and duty(1) high) and a PinRef
 *                  factory.
 *
 *   using Led = brio::Pin<'A', 5>;    // PA5, LD2 on the Nucleo-64
 *   Led::output();
 *   Led::toggle();
 *   Button::input(brio::PinPull::up);
 *   Tx::function(brio::PinFunction::af7);   // USART2_TX
 *
 * Which context may call what. The value functions (set, clear, toggle,
 * read) are single BSRR stores or IDR loads (toggle reads ODR and makes
 * one BSRR store), atomic by the silicon against a handler touching
 * other pins of the port, and are safe from any context, which is what
 * PinRef's set()/clear() inside a bus active object's request depend on
 * (an active object is a component of a brio program with its own event
 * queue, served one event at a time by the kernel,
 * kernel/active_object.hpp). The configuring functions (output, input,
 * analog, function, release, pull, configure_mask) are read-modify-write
 * sequences on MODER/OTYPER/OSPEEDR/PUPDR/AFR, fields with no set/clear
 * twins, so configuring two pins of one port from two contexts (a
 * handler and the loop) can lose a field: they are meant for the main
 * context - setup in main() before the kernel runs, and an active
 * object's handlers and its state machine's entry and exit actions,
 * which run there one at a time - never for an interrupt handler that
 * may interrupt another configuring call. None of them
 * waits beyond the port clock's enable readback, and none masks
 * interrupts.
 *
 * Three facts of this family's GPIO shape the code below. The register
 * block is the STM32G0's register for register (MODER, OTYPER, OSPEEDR,
 * PUPDR, IDR, ODR, BSRR, LCKR, AFRL/AFRH), and the three facts, each the
 * STM32F4's, are where the two families differ:
 *
 * 1. There is no BRR. Bitwise reset goes through the upper half of BSRR
 *    (8.4.7: bits 31:16 reset, bits 15:0 set, set wins when both are
 *    written), so clear() stores `mask << 16` into BSRR. Still one
 *    store, still atomic against a handler touching other pins.
 * 2. The port's clock is RCC_AHB1ENR.GPIOxEN (7.3.10), off at reset:
 *    with it clear every register of the port reads as zero and ignores
 *    writes, silently. So every configuring verb here but pull() switches
 *    the port's clock on first (an idempotent read-modify-write of one
 *    RCC bit plus the readback that is the dummy access ES0206 2.2.7
 *    asks for); the value verbs (set/clear/toggle/read) do not, because
 *    a pin one has configured has a clocked port by construction, and
 *    pull() rewrites one field of a pin configured before it. Nothing
 *    here ever turns a port clock off - a second pin of the same port
 *    would lose its block; releasing a port is a program-wide decision
 *    (Port<L>::clock(false) exists for it).
 * 3. The reset state is input floating, not analog (8.4.1: MODER resets
 *    to 0, input, on every pin but PA13/PA14/PA15 and PB3/PB4, the debug
 *    port's, which come up in their alternate function with pulls). The
 *    input buffer is on in input, output and AF modes and off in analog
 *    mode (8.3.12), which is also the lowest-power parking state -
 *    release() therefore parks a pad in analog, as on the STM32G0,
 *    which is not this family's reset state and is said so here.
 *
 * Alternate functions are a per-pin 4-bit number (AFRL/AFRH), and which
 * peripheral signal AFn means on a given pad is a table of the datasheet
 * (DS10693 table 11, the F429's table 12, the F411's table 9), not of
 * the reference manual: a peripheral driver's pin claim names the AF and
 * no static_assert, the program's or a driver's, can check it against a
 * header symbol, because the device header does not carry the pin table
 * at all. The bench is the check. Two rows of those tables every serial console
 * needs: USART1..3 are AF7, UART4/5/7/8 and USART6 are AF8.
 *
 * There is no pin interrupt in GPIO. Edge and level senses are the
 * EXTI's (RM0090 ch. 12), reached through SYSCFG's multiplexer, and
 * stm32f4/exti.hpp is what owns them.
 *
 * The package fact is a port-level one, read off the device header in
 * stm32f4/device_tables.hpp: ports A, B, C and H exist on every part, D
 * and E from the 64- and 100-pin bondings up (and on every F401/F411), F
 * and G on the 144-pin classes, I, J and K on the big packages. Which
 * pins of a present port a package bonds is finer than that, is not
 * tabulated, and so is checked by nothing (a Pin on an unbonded pad
 * configures a register nobody wired).
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

/// Whether this device bonds out GPIO port `letter` at all (the device
/// header's GPIOx_BASE is the authority; stm32f4/device_tables.hpp).
constexpr bool port_exists(char letter) { return gpio_port_present(letter); }

/// Runtime pin descriptor: lets a pin chosen at compile time travel
/// inside a request event (the CS/DC of a SPI transaction, asserted by
/// the bus active object (AO) and not by its client). A null PinRef (the default) means
/// "no such pin": set/clear are no-ops, so an optional pin costs one
/// branch. Build one with Pin<...>::ref(). The stores are BSRR's two
/// halves: atomic on the silicon, legal from any context.
///
/// The two edges are forced inline: left to -Os, gcc keeps the body
/// behind its call sites, and an edge costs a BL and a return on top of
/// its six instructions - four times a transaction on the SPI host,
/// counted in the release listing (docs/stm32f4/spi.md). Inline, an
/// edge is the port pointer's load, the null test, the mask's load (a
/// shift into the upper half for the clear) and one BSRR store (8.4.7:
/// a write-one register, so no read and no window); the null test
/// stays a predictable branch, which is what makes an absent D/C pin
/// cost one branch and not a verb.
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
            port->BSRR = mask << 16;
        }
    }
    constexpr bool valid() const { return port != nullptr; }
};

// ---- pin configuration vocabulary (8.4.1 .. 8.4.10) ---------------------------

/// GPIOx_MODER, two bits per pin.
enum class PinMode : uint8_t { input = 0, output = 1, alternate = 2, analog = 3 };

/// GPIOx_PUPDR. Reserved code 3 is not spelled.
enum class PinPull : uint8_t { none = 0, up = 1, down = 2 };

/// GPIOx_OSPEEDR: the output driver's slew class (the datasheets' I/O
/// characteristics give the frequency each reaches per load and supply:
/// about 2, 25, 50 and 100 MHz). `low` is the reset value of every pin
/// but the debug port's.
enum class PinSpeed : uint8_t { low = 0, medium = 1, high = 2, very_high = 3 };

/// GPIOx_AFRL/AFRH: the alternate function NUMBER. Which signal it is on
/// a given pad is the datasheet's table, not this enum's business.
enum class PinFunction : uint8_t {
    af0 = 0, af1, af2, af3, af4, af5, af6, af7,
    af8, af9, af10, af11, af12, af13, af14, af15,
};

/// What a configuring verb writes besides the mode. Open-drain applies to
/// output and alternate modes (8.4.2); the pull to any mode but analog.
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
                  "header declares no GPIOx_BASE for it; ports A, B, C and H exist on every "
                  "STM32F4, D and E from the 64- and 100-pin bondings, F and G on the 144-pin "
                  "classes, I..K on the big packages)");

    Port() = delete;

    static constexpr char letter = L;

    static GPIO_TypeDef& regs() { return *reinterpret_cast<GPIO_TypeDef*>(gpio_port_base(L)); }

    // ---- the bus clock (RCC_AHB1ENR, 7.3.10) ------------------------------------
    static void clock(bool on) { Rcc::io_clock(L, on); }
    static bool clock() { return Rcc::io_clock(L); }

    // ---- values ---------------------------------------------------------------
    static uint32_t in() { return regs().IDR; }
    static uint32_t out() { return regs().ODR; }
    static void out_set(uint32_t m) { regs().BSRR = m & 0xFFFFu; }
    static void out_clear(uint32_t m) { regs().BSRR = (m & 0xFFFFu) << 16; }
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
    /// first (8.3.2's procedure: the AF is selected before the mode
    /// switches, so the pad never spends a cycle on the wrong function),
    /// then type, speed, pull, and the mode last - the store that hands
    /// the pad over. Turns the port clock on first.
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

    // ---- values (BSRR: atomic, any context) ---------------------------------------
    static void set()    { port().BSRR = mask; }
    static void clear()  { port().BSRR = mask << 16; }
    static void toggle() { P::out_toggle(mask); }
    static bool read()   { return (port().IDR & mask) != 0u; }
    static bool read_out() { return (port().ODR & mask) != 0u; }
    static bool is_output() {
        return ((port().MODER >> (2u * PinNum)) & 0x3u) == static_cast<uint32_t>(PinMode::output);
    }

    // ---- configuration (setup / kernel time) ------------------------------------
    /// Push-pull output, low speed, no pull, driving whatever ODR holds
    /// (zero on a pad nothing has written). Opens the port's clock first;
    /// a setup or kernel-time verb (the opening comment).
    static void output(const PinConfig& cfg = {}) { P::configure_mask(mask, PinMode::output, cfg); }
    /// Output that starts at a known level: the port's clock is opened
    /// first (a BSRR store into an unclocked port is dropped in silence
    /// - fact 2 above), then the value is written before the mode
    /// switches, so the pad never glitches through the old ODR. The
    /// second clock(true) inside output(cfg) is idempotent.
    static void output(bool level, const PinConfig& cfg = {}) {
        P::clock(true);
        if (level) set(); else clear();
        output(cfg);
    }
    /// Input - the reset state of every non-debug pad - with a pull.
    static void input(PinPull p = PinPull::none) {
        P::configure_mask(mask, PinMode::input, {.pull = p});
    }
    /// Analog mode: input buffer off, the lowest-power state (8.3.12).
    static void analog() { P::configure_mask(mask, PinMode::analog, {}); }
    /// Hand the pad to a peripheral: AF `fn`, with the driver options in
    /// `cfg` (an open-drain I2C line, a fast SPI clock).
    static void function(PinFunction fn, const PinConfig& cfg = {}) {
        P::configure_mask(mask, PinMode::alternate, cfg, fn);
    }
    /// Park the pad: analog, no pull - the low-power state, which on
    /// this family is not the reset state (fact 3 above).
    static void release() { analog(); }
    static bool has_function() {
        return ((port().MODER >> (2u * PinNum)) & 0x3u) == static_cast<uint32_t>(PinMode::alternate);
    }
    /// Change the pull alone: one read-modify-write of PUPDR. It opens no
    /// clock, so it follows a configuring call that has.
    static void pull(PinPull p) {
        P::write_field2(port().PUPDR, mask, static_cast<uint8_t>(p));
    }
};

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
