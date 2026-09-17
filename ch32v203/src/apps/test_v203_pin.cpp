// test_v203_pin - the reference bench suite for the CH32V203's PADS and
// its EXTERNAL INTERRUPT LINES: ch32v203/pin.hpp and ch32v203/afio.hpp
// over RM ch. 10, ch32v203/exti.hpp over 9.4.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE, AND NOTHING TO PRESS (but one letter). A pad reads
// its own level through the input register in every mode but analog
// (10.2.6..10.2.9), and that pad's own output feeds the EXTI line of its
// number - so edges, senses, flags and the event mode are all measurable
// with the board bare. The one thing a program cannot do to itself is
// press a button, which is why the KEY on PA0 is a letter by NAME (k)
// and not part of z.
//
// THE PADS. Free on this board and used here: PA1..PA4 (levels, pulls,
// the open drain, EXTI line 1), PB3..PB8 (the whole-port verbs, EXTI
// lines 5 and 6), PB12/PB13 (the high configuration register, EXTI lines
// 12 and 13), PC13 (the configuration LOCK). Never touched: PA9/PA10
// (the console), PA13/PA14 (the debug port - taking them loses the probe
// until a power cycle), PA11/PA12 (the USB pads), PC14/PC15 (the 32 kHz
// crystal), PD0/PD1 (the 8 MHz crystal, which this suite's tree does not
// run on), and PB2, which carries the LED and is toggled per command as
// every suite of this target does, never as a test pad.
//
// THE TREE RUNS ON THE HSI on purpose: letter f writes the remap that
// hands the oscillator's pads to GPIO, which would stop a crystal the
// clock was running on.
//
// What is exercised, letter by letter:
//   a  THE NIBBLE: a port's clock gate opened by a configuring verb, the
//      reset nibbles after a peripheral reset pulse, all fifteen
//      configurations of RM table 10.3.1.1 written and read back in both
//      configuration registers, and the atomic set/clear/toggle stores
//   b  THE PULLS, wirelessly: a floating pad reads what the pull says,
//      the switch between them read back a microsecond later, and the
//      direction proved to be the output register's bit
//   c  THE OPEN DRAIN: an open-drain output pulls its own pad low and
//      does NOT drive it high (measured against a push-pull output from
//      the same level), the released pad's decay printed with no verdict
//      on it, and the pulls shown to belong to input mode alone
//   d  THE WHOLE-PORT VERBS: one mask configured, driven, cleared,
//      toggled and read back in single stores, with the pin next to it
//      untouched
//   f  THE REMAPS: every field of AFIO_PCFR1 this part has written and
//      read back, the ones it has not refused, the debug port's own
//      field read and never written - and USART1's column, which exists
//      here and is never selected because its TX pad is this console's
//   g  THE EXTI LINES on the pads' own outputs: rising, falling and both
//      edges counted over a burst, the flag as a poll and as write-one,
//      the software trigger, and the two shared vectors dispatching by
//      pending bit
//   h  THE EVENT MODE: a line in EVENR ending the platform's idle() with
//      no handler and no flag, measured in cycles against an idle() that
//      only the tick ends
//   e  THE CONFIGURATION LOCK on PC13 - LAST in z, because the pad stays
//      locked until the next reset
//   k  THE KEY on PA0, by name: ten seconds of levels at 10 ms while a
//      person presses the button
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v203/afio.hpp"
#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"
#include "ch32v203/exti.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/ticker.hpp"
#include "ch32v203/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32v203Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

using SysClock = Clock<ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

TestBench<Serial> bench;

// The pads, once.
using PadA = Pin<'A', 1>;     ///< the low configuration register, EXTI line 1
using PadA2 = Pin<'A', 2>;
using PadA3 = Pin<'A', 3>;
using PadA4 = Pin<'A', 4>;
using PadB = Pin<'B', 12>;    ///< the high configuration register, EXTI line 12
using PadB13 = Pin<'B', 13>;
using PadB5 = Pin<'B', 5>;
using PadB6 = Pin<'B', 6>;
using LockPad = Pin<'C', 13>;
using KeyPad = Pin<'A', 0>;

/// The mask letter d drives together: PB3..PB8, which this package bonds
/// and nothing on the board uses. PB2 is the LED, one bit below it.
constexpr uint16_t port_mask = 0x01F8u;

using LineA = ExtInt<PadA>;      ///< line 1, a vector of its own
using Line5 = ExtInt<PadB5>;     ///< line 5, the 9..5 vector
using Line6 = ExtInt<PadB6>;     ///< line 6, the same one
using Line12 = ExtInt<PadB>;     ///< line 12, the 15..10 vector
using Line13 = ExtInt<PadB13>;   ///< line 13, the same one

static_assert(exti_lines_distinct<LineA, Line5, Line6, Line12, Line13>(),
              "two pads of this suite on one EXTI line");

/// What the three EXTI vectors counted, by line.
volatile uint32_t edges_line1 = 0;
volatile uint32_t edges_line5 = 0;
volatile uint32_t edges_line6 = 0;
volatile uint32_t edges_line12 = 0;
volatile uint32_t edges_line13 = 0;
volatile uint32_t shared_entries = 0;

uint32_t cycles_now() { return stk()->CNTL; }

/// Every line and pad this suite touches, back to reset. Called at the
/// top of each letter, so no letter inherits another's state.
void all_off() {
    for (const uint8_t line : {1u, 5u, 6u, 12u, 13u}) {
        (void)Exti::release(line);
    }
    Pfic::disable(Irq::exti1);
    Pfic::disable(Irq::exti9_5);
    Pfic::disable(Irq::exti15_10);
    Pfic::clear_pending(Irq::exti1);
    Pfic::clear_pending(Irq::exti9_5);
    Pfic::clear_pending(Irq::exti15_10);
    PadA::release();
    PadA2::release();
    PadA3::release();
    PadA4::release();
    PadB::release();
    PadB13::release();
    PadB5::release();
    PadB6::release();
    Port<'B'>::configure_pins(port_mask, pin_nibble(PinMode::input, PinDrive::push_pull,
                                                    PinSpeed::fast));
    edges_line1 = 0;
    edges_line5 = 0;
    edges_line6 = 0;
    edges_line12 = 0;
    edges_line13 = 0;
    shared_entries = 0;
}

// ===========================================================================
// a - the nibble: the gate, the reset state, all fifteen configurations
// ===========================================================================

struct NibbleCase {
    const char* name;
    PinMode mode;
    PinDrive drive;
    PinSpeed speed;
    uint32_t nibble;
};

/// RM 10.3.1.1's table, whole: three input configurations (the fourth,
/// CNF 11 under MODE 00, is reserved) and four output ones at each of the
/// three speeds.
constexpr NibbleCase nibble_cases[] = {
    {"analog", PinMode::analog, PinDrive::push_pull, PinSpeed::fast, 0x0},
    {"floating input", PinMode::input, PinDrive::push_pull, PinSpeed::fast, 0x4},
    {"push-pull 10 MHz", PinMode::output, PinDrive::push_pull, PinSpeed::medium, 0x1},
    {"push-pull 2 MHz", PinMode::output, PinDrive::push_pull, PinSpeed::slow, 0x2},
    {"push-pull 50 MHz", PinMode::output, PinDrive::push_pull, PinSpeed::fast, 0x3},
    {"open-drain 10 MHz", PinMode::output, PinDrive::open_drain, PinSpeed::medium, 0x5},
    {"open-drain 2 MHz", PinMode::output, PinDrive::open_drain, PinSpeed::slow, 0x6},
    {"open-drain 50 MHz", PinMode::output, PinDrive::open_drain, PinSpeed::fast, 0x7},
    {"alternate push-pull 10 MHz", PinMode::alternate, PinDrive::push_pull, PinSpeed::medium, 0x9},
    {"alternate push-pull 2 MHz", PinMode::alternate, PinDrive::push_pull, PinSpeed::slow, 0xA},
    {"alternate push-pull 50 MHz", PinMode::alternate, PinDrive::push_pull, PinSpeed::fast, 0xB},
    {"alternate open-drain 10 MHz", PinMode::alternate, PinDrive::open_drain, PinSpeed::medium, 0xD},
    {"alternate open-drain 2 MHz", PinMode::alternate, PinDrive::open_drain, PinSpeed::slow, 0xE},
    {"alternate open-drain 50 MHz", PinMode::alternate, PinDrive::open_drain, PinSpeed::fast, 0xF},
};
constexpr uint8_t nibble_case_count = sizeof(nibble_cases) / sizeof(nibble_cases[0]);

/// The pulled input (nibble 0x8) is the fifteenth configuration and has
/// no PinMode of its own: it is `input(PinPull::up)`, whose direction
/// letter b is about.
template <class Pad>
uint8_t nibbles_land_on() {
    uint8_t right = 0;
    for (const NibbleCase& c : nibble_cases) {
        Pad::P::configure(Pad::pin_number, pin_nibble(c.mode, c.drive, c.speed));
        if (Pad::nibble() == c.nibble && pin_nibble(c.mode, c.drive, c.speed) == c.nibble) {
            ++right;
        }
    }
    Pad::input(PinPull::up);
    const bool pulled = Pad::nibble() == 0x8u;
    Pad::release();
    return static_cast<uint8_t>(right + (pulled ? 1u : 0u));
}

void ta_nibble() {
    all_off();
    // The gate: port C's clock closed by hand, then a configuring verb
    // on one of its pads opens it.
    Rcc::disable(Bus::pb2, rcc_pb2_gpioc);
    const bool closed = !Rcc::enabled(Bus::pb2, rcc_pb2_gpioc);
    LockPad::input();
    const bool opened = Rcc::enabled(Bus::pb2, rcc_pb2_gpioc);
    bench.verdict("a configuring verb opens the port's clock gate (GPIOC closed, then PC13 "
                  "configured)",
                  closed && opened);

    // The reset state, on the one port this suite may reset: GPIOA
    // carries the console and the debug port, GPIOB the LED.
    Rcc::reset(Bus::pb2, rcc_pb2_gpioc);
    const uint32_t low = Port<'C'>::regs().CFGLR;
    const uint32_t high = Port<'C'>::regs().CFGHR;
    print(serial, "  GPIOC after its reset pulse: CFGLR=", hex(low), " CFGHR=", hex(high),
          " OUTDR=", hex(Port<'C'>::regs().OUTDR), " (this package bonds PC13..PC15 of it)", crlf);
    bench.verdict("a reset port holds 0x4 - a floating input - in the nibble of every pin it "
                  "bonds (10.3.1.1)",
                  Port<'C'>::nibble(13) == 0x4u && Port<'C'>::nibble(14) == 0x4u &&
                      Port<'C'>::nibble(15) == 0x4u);

    // All fifteen configurations, in both registers.
    const uint8_t on_low = nibbles_land_on<PadA>();
    const uint8_t on_high = nibbles_land_on<PadB>();
    print(serial, "  configurations that read back as spelled: PA1 (CFGLR) ", on_low, "/15, PB12 "
          "(CFGHR) ", on_high, "/15", crlf);
    bench.verdict("every configuration of table 10.3.1.1 lands as pin_nibble() spells it, in the "
                  "low register and in the high one",
                  on_low == 15u && on_high == 15u);

    // The atomic stores.
    PadA::output(false);
    Port<'A'>::out_set(PadA::mask);
    const bool set = PadA::read_out();
    Port<'A'>::regs().BSHR = PadA::mask << 16;
    const bool cleared_high_half = !PadA::read_out();
    PadA::set();
    Port<'A'>::out_clear(PadA::mask);
    const bool cleared_bcr = !PadA::read_out();
    PadA::toggle();
    const bool toggled = PadA::read_out();
    PadA::toggle();
    const bool back = !PadA::read_out();
    PadA::release();
    bench.verdict("BSHR sets from its low half and clears from its high half, BCR clears, and "
                  "toggle() flips in one store",
                  set && cleared_high_half && cleared_bcr && toggled && back);
    all_off();
}

// ===========================================================================
// b - the pulls, wirelessly
// ===========================================================================

template <class Pad>
bool pull_reads(PinPull pull, bool expected) {
    Pad::input(pull);
    (void)delay_us(clock, 1);
    return Pad::read() == expected;
}

void tb_pulls() {
    all_off();
    // Four free pads, each pulled both ways and read one microsecond
    // after the switch.
    uint8_t right = 0;
    print(serial, "  pulled up / down after 1 us:");
    PadA::input(PinPull::up);
    (void)delay_us(clock, 1);
    print(serial, " PA1=", PadA::read());
    PadA::input(PinPull::down);
    (void)delay_us(clock, 1);
    print(serial, "/", PadA::read());
    right = static_cast<uint8_t>(right + (pull_reads<PadA>(PinPull::up, true) &&
                                          pull_reads<PadA>(PinPull::down, false)));
    PadA2::input(PinPull::up);
    (void)delay_us(clock, 1);
    print(serial, " PA2=", PadA2::read());
    PadA2::input(PinPull::down);
    (void)delay_us(clock, 1);
    print(serial, "/", PadA2::read());
    right = static_cast<uint8_t>(right + (pull_reads<PadA2>(PinPull::up, true) &&
                                          pull_reads<PadA2>(PinPull::down, false)));
    PadA3::input(PinPull::up);
    (void)delay_us(clock, 1);
    print(serial, " PA3=", PadA3::read());
    PadA3::input(PinPull::down);
    (void)delay_us(clock, 1);
    print(serial, "/", PadA3::read());
    right = static_cast<uint8_t>(right + (pull_reads<PadA3>(PinPull::up, true) &&
                                          pull_reads<PadA3>(PinPull::down, false)));
    PadB::input(PinPull::up);
    (void)delay_us(clock, 1);
    print(serial, " PB12=", PadB::read());
    PadB::input(PinPull::down);
    (void)delay_us(clock, 1);
    print(serial, "/", PadB::read(), crlf);
    right = static_cast<uint8_t>(right + (pull_reads<PadB>(PinPull::up, true) &&
                                          pull_reads<PadB>(PinPull::down, false)));
    bench.verdict("four floating pads read high through their pull-up and low through their "
                  "pull-down, a microsecond after each switch",
                  right == 4u);

    // The direction IS the output register's bit (10.3.1.4), under the
    // one nibble that has no PinMode of its own.
    PadA::input(PinPull::up);
    const bool odr_up = PadA::read_out();
    const uint32_t nibble_up = PadA::nibble();
    PadA::input(PinPull::down);
    const bool odr_down = !PadA::read_out();
    const uint32_t nibble_down = PadA::nibble();
    PadA::release();
    bench.verdict("the pull's direction is OUTDR's bit under the pulled-input nibble 0x8, both "
                  "ways",
                  odr_up && odr_down && nibble_up == 0x8u && nibble_down == 0x8u);
    all_off();
}

// ===========================================================================
// c - the open drain, wirelessly
// ===========================================================================

void tc_open_drain() {
    all_off();
    // A push-pull output drives its own pad both ways: the input register
    // follows what is driven (10.2.7).
    PadA::output(true);
    (void)delay_us(clock, 1);
    const bool pp_high = PadA::read();
    PadA::clear();
    (void)delay_us(clock, 1);
    const bool pp_low = !PadA::read();
    bench.verdict("a push-pull output drives its own pad high and low, read back on INDR",
                  pp_high && pp_low);

    // From that low level, the SAME pad as an open-drain output with a
    // one in its data register: the N-MOS is off and no P-MOS takes its
    // place, so the pad is left where it was instead of going high.
    PadA::output(false, PinDrive::open_drain);
    PadA::set();
    const bool od_not_driven_high = !PadA::read();
    const uint32_t nibble = PadA::nibble();
    (void)delay_us(clock, 1);
    const bool after_1us = PadA::read();
    (void)delay_us(clock, 100);
    const bool after_100us = PadA::read();
    print(serial, "  the released open-drain pad from a low level: at once ", !od_not_driven_high,
          ", after 1 us ", after_1us, ", after 100 us ", after_100us,
          " (floating: no verdict on the last two)", crlf);
    bench.verdict("an open-drain output with a one in OUTDR does not drive its pad high "
                  "(nibble 0x7, the level unchanged at the store)",
                  od_not_driven_high && nibble == 0x7u);

    // And it does pull low.
    PadA::clear();
    (void)delay_us(clock, 1);
    const bool od_low = !PadA::read();
    bench.verdict("an open-drain output pulls its own pad low", od_low);

    // The pulls are the INPUT driver's (10.2.7 says the output modes
    // disable them), so the released pad is taken high by a pull only
    // once the pin is an input again.
    PadA::input(PinPull::up);
    (void)delay_us(clock, 10);
    const bool pulled_high = PadA::read();
    PadA::release();
    bench.verdict("the pulls belong to input mode alone: the same pad reads high once it is a "
                  "pulled-up input",
                  pulled_high);

    // An analog pad's input driver is off and its input register reads
    // zero whatever the pad is at (10.2.9) - measured from a pad the
    // program has just driven high.
    PadA2::output(true);
    (void)delay_us(clock, 1);
    const bool driven_high = PadA2::read();
    PadA2::analog();
    (void)delay_us(clock, 1);
    const bool analog_reads_zero = !PadA2::read();
    PadA2::release();
    bench.verdict("an analog pad's input driver is off: INDR reads zero on a pad that read high "
                  "an instant before",
                  driven_high && analog_reads_zero);
    all_off();
}

// ===========================================================================
// d - the whole-port verbs
// ===========================================================================

void td_port() {
    all_off();
    // The LED's pin is one below the mask and must not move: its output
    // bit is read before and after.
    const bool led_before = Led::read_out();

    Port<'B'>::configure_pins(port_mask, pin_nibble(PinMode::output, PinDrive::push_pull,
                                                    PinSpeed::fast));
    uint8_t configured = 0;
    for (uint8_t pin = 3; pin <= 8u; ++pin) {
        if (Port<'B'>::nibble(pin) == 0x3u) {
            ++configured;
        }
    }
    bench.verdict("configure_pins() writes one nibble into every pin of a mask, in one store per "
                  "configuration register",
                  configured == 6u);

    // The three MASKED stores, with the pin next to the mask driven high
    // so that a store touching it would show.
    Led::set();
    Port<'B'>::out_set(port_mask);
    const uint32_t after_set = Port<'B'>::in() & port_mask;
    Port<'B'>::out_clear(port_mask);
    const uint32_t after_clear = Port<'B'>::in() & port_mask;
    Port<'B'>::out_toggle(port_mask);
    const uint32_t after_toggle = Port<'B'>::in() & port_mask;
    const bool led_after_masked = Led::read_out();

    // And the WHOLE-PORT store, which is the other kind of verb: it says
    // what all sixteen pins are, the ones outside the mask included.
    Port<'B'>::out_write(port_mask);
    const uint32_t after_write = Port<'B'>::in() & port_mask;
    const bool led_after_write = Led::read_out();
    if (led_before) {
        Led::set();
    } else {
        Led::clear();
    }
    print(serial, "  PB3..PB8 read back: set=", hex(after_set), " clear=", hex(after_clear),
          " toggle=", hex(after_toggle), " write(mask)=", hex(after_write),
          "; PB2 beside them: after the masked stores ", led_after_masked, ", after the whole-port "
          "store ", led_after_write, crlf);
    bench.verdict("a mask set, cleared, toggled and written whole reads back on INDR",
                  after_set == port_mask && after_clear == 0u && after_toggle == port_mask &&
                      after_write == port_mask);
    bench.verdict("the three masked stores leave the pin next to the mask alone (BSHR and BCR "
                  "are write-one), and the whole-port store takes it with them",
                  led_after_masked && !led_after_write);
    all_off();
}

// ===========================================================================
// f - the remaps
// ===========================================================================

struct RemapCase {
    const char* name;
    Remap remap;
    uint8_t code;
};

/// Every field of the two remap registers this driver reaches, at a code
/// that is not the reset one - with USART1's left out, since its other
/// column carries this console's TX away to PB6.
constexpr RemapCase remap_cases[] = {
    {"SPI1", Remap::spi1, 1},         {"I2C1", Remap::i2c1, 1},
    {"USART2", Remap::usart2, 1},     {"USART3", Remap::usart3, 1},
    {"UART4", Remap::uart4, 1},       {"TIM1", Remap::tim1, 1},
    {"TIM2", Remap::tim2, 3},         {"TIM3", Remap::tim3, 2},
    {"TIM3 full", Remap::tim3, 3},    {"TIM4", Remap::tim4, 1},
    {"CAN1", Remap::can1, 2},         {"CAN1 on the crystal pads", Remap::can1, 3},
    {"PD0/PD1 as GPIO", Remap::pd0_pd1, 1},
    {"TIM5_CH4", Remap::tim5_ch4, 1}, {"TIM2_ITR1", Remap::tim2_itr1, 1},
    {"PTP_PPS", Remap::ptp_pps, 1},
};

void tf_remaps() {
    all_off();
    // The tree must not be running off the crystal: one of the columns
    // below hands its pads to GPIO.
    const bool on_hsi = Rcc::sysclk_status() != SysclkSource::hse;
    print(serial, "  SYSCLK's root is ", on_hsi ? "the HSI's PLL" : "THE CRYSTAL", crlf);

    uint8_t taken = 0;
    uint8_t refused = 0;
    uint8_t wrong = 0;
    for (const RemapCase& c : remap_cases) {
        const bool allowed = afio_remap_has_code(c.remap, c.code);
        if (!allowed) {
            print(serial, "  ", c.name, " code ", c.code, ": refused (not this part's column)",
                  crlf);
            if (Afio::remap(c.remap, c.code)) {
                ++wrong;
            }
            ++refused;
            continue;
        }
        if (!on_hsi && c.remap == Remap::pd0_pd1) {
            continue;
        }
        const uint8_t before = Afio::remap_code(c.remap);
        const bool written = Afio::remap(c.remap, c.code);
        const uint8_t read = Afio::remap_code(c.remap);
        (void)Afio::remap(c.remap, before);
        const uint8_t restored = Afio::remap_code(c.remap);
        if (written && read == c.code && restored == before) {
            ++taken;
        } else {
            ++wrong;
            print(serial, "  ", c.name, " code ", c.code, ": wrote ", written, " read ", read,
                  " restored ", restored, crlf);
        }
    }
    print(serial, "  columns taken and restored: ", taken, ", refused as absent: ", refused,
          "; PCFR1=", hex(Afio::regs().PCFR1), " PCFR2=", hex(Afio::regs().PCFR2), crlf);
    bench.verdict("every remap column this part has reads back as written and restores, and every "
                  "column it has not is refused with nothing written",
                  wrong == 0u && taken > 0u);

    // THE TWO FIELDS THIS CLASS DOES NOT IMPLEMENT, written raw. The
    // driver refuses both (afio.hpp), and this is why: the manual gives
    // USART3 four columns and the internal-trigger bit to the whole
    // series, and on this part the silicon holds both at zero.
    Afio::clock_on();
    const uint32_t kept = Afio::regs().PCFR1 & ~((0x3UL << 4) | (1UL << 29));
    Afio::regs().PCFR1 = kept | (0x1UL << 4) | (1UL << 29);
    const uint32_t usart3_field = (Afio::regs().PCFR1 >> 4) & 0x3u;
    const uint32_t itr1_field = (Afio::regs().PCFR1 >> 29) & 0x1u;
    Afio::regs().PCFR1 = kept;
    print(serial, "  written raw: USART3_RM = 01 reads ", usart3_field,
          ", TIM2_ITR1_RM = 1 reads ", itr1_field, crlf);
    bench.verdict("the two fields the manual gives this series and the silicon does not "
                  "implement are read-only at zero (USART3's columns, TIM2's internal trigger)",
                  usart3_field == 0u && itr1_field == 0u);

    // USART1's second column exists on this package - and is never
    // selected here, because PB6 is not where the probe listens.
    bench.verdict("USART1's remapped column exists on this part (and is left alone: its TX is "
                  "this console's)",
                  afio_remap_has_code(Remap::usart1, 1) == pad_bonded(Pad{'B', 6}));

    // The debug port's field: read, reported, never written.
    print(serial, "  SW_CFG reads ", Afio::debug_config(), ": the two-wire debug port is ",
          Afio::debug_port_enabled() ? "alive" : "GONE", crlf);
    bench.verdict("the debug port's own field is untouched by every verb above",
                  Afio::debug_port_enabled());

    // The event output register, whose signal this core may or may not
    // have: the register takes a port and a pin and reads them back.
    const bool armed = Afio::event_output('A', 1);
    const bool reads_back = Afio::event_output_enabled() && Afio::event_output_port() == 'A' &&
                            Afio::event_output_pin() == 1u;
    Afio::event_output_off();
    const bool off = !Afio::event_output_enabled();
    const bool refused_e = !Afio::event_output('E', 0);
    bench.verdict("AFIO_ECR takes a port and a pin, reads them back and turns off again - ports "
                  "A..D only (what drives EVENTOUT on this core is not stated anywhere)",
                  armed && reads_back && off && refused_e);
    all_off();
}

// ===========================================================================
// g - the EXTI lines, on the pads' own outputs
// ===========================================================================

/// Toggle a pad `toggles` times with an interval no bus can miss, and
/// return what the line's vector counted. The pad drives itself: its
/// output is what the line's edge detector sees (10.2.7).
template <class Line, class Pad>
uint32_t edges_of(ExtiSense sense, uint8_t toggles, volatile uint32_t& counter, Irq vector) {
    (void)Exti::release(Line::line);
    Pad::output(false);
    (void)Line::claim();
    Pad::output(false);
    (void)Line::configure(sense);
    (void)Line::clear();
    counter = 0;
    (void)Line::arm(true);
    Pfic::clear_pending(vector);
    Pfic::enable(vector);
    for (uint8_t i = 0; i < toggles; ++i) {
        Pad::toggle();
        (void)delay_us(clock, 2);
    }
    (void)delay_us(clock, 10);
    Pfic::disable(vector);
    (void)Line::arm(false);
    return counter;
}

void tg_edges() {
    all_off();
    // Line 1 on PA1, a vector of its own: ten toggles from a low level
    // are five rising edges, five falling and ten of both.
    const uint32_t rising = edges_of<LineA, PadA>(ExtiSense::rising, 10, edges_line1, Irq::exti1);
    const uint32_t falling = edges_of<LineA, PadA>(ExtiSense::falling, 10, edges_line1, Irq::exti1);
    const uint32_t both = edges_of<LineA, PadA>(ExtiSense::both, 10, edges_line1, Irq::exti1);
    print(serial, "  ten toggles of PA1 on its own line 1: rising ", rising, ", falling ", falling,
          ", both ", both, " interrupts", crlf);
    bench.verdict("a pad's own output raises its line: five rising, five falling and ten of both "
                  "edges out of ten toggles, one interrupt each",
                  rising == 5u && falling == 5u && both == 10u);

    // The sense read back, and a line with neither edge enabled.
    (void)Exti::release(LineA::line);
    (void)LineA::configure(ExtiSense::both);
    const bool both_read = LineA::sense() == ExtiSense::both;
    (void)LineA::configure(ExtiSense::none);
    const bool none_read = LineA::sense() == ExtiSense::none;
    PadA::output(false);
    (void)LineA::arm(true);
    (void)LineA::clear();
    PadA::toggle();
    (void)delay_us(clock, 5);
    const bool silent = !LineA::pending();
    (void)LineA::arm(false);
    bench.verdict("the sense pair reads back, and a line with neither edge enabled is silent "
                  "under a toggling pad",
                  both_read && none_read && silent);

    // The flag as a poll: the line armed in INTENR with its vector shut
    // at the controller. Does an edge raise the flag of a line enabled
    // NOWHERE? The sister family says no; this is where this one answers.
    (void)Exti::release(LineA::line);
    PadA::output(false);
    (void)LineA::configure(ExtiSense::rising);
    (void)LineA::clear();
    Pfic::disable(Irq::exti1);
    PadA::set();
    (void)delay_us(clock, 5);
    const bool flag_unarmed = LineA::pending();
    (void)LineA::clear();
    PadA::clear();
    (void)LineA::arm(true);
    PadA::set();
    (void)delay_us(clock, 5);
    const bool flag_armed = LineA::pending();
    (void)LineA::clear();
    const bool flag_cleared = !LineA::pending();
    print(serial, "  an edge with the line enabled nowhere: flag ", flag_unarmed ? "UP" : "down",
          "; enabled in INTENR with its vector shut: flag ", flag_armed ? "up" : "DOWN",
          ", after writing one: ", flag_cleared ? "down" : "UP", crlf);
    bench.verdict("the flag of an armed line with its vector shut stands for a poller and is "
                  "cleared by writing one (rc_w1)",
                  flag_armed && flag_cleared);

    // The software trigger, with no pad at all.
    (void)Exti::release(LineA::line);
    (void)LineA::clear();
    (void)LineA::trigger();
    const bool soft_unarmed = LineA::pending();
    const bool bit_unarmed = LineA::triggered();
    (void)LineA::clear();
    (void)LineA::arm(true);
    (void)LineA::trigger();
    const bool soft_armed = LineA::pending();
    const bool bit_stands = LineA::triggered();
    (void)LineA::clear();
    const bool bit_after_clear = LineA::triggered();
    edges_line1 = 0;
    Pfic::clear_pending(Irq::exti1);
    Pfic::enable(Irq::exti1);
    (void)LineA::trigger();
    (void)delay_us(clock, 20);
    const uint32_t soft_interrupts = edges_line1;
    Pfic::disable(Irq::exti1);
    (void)LineA::arm(false);
    print(serial, "  SWIEVR on line 1: flag with the line enabled nowhere ", soft_unarmed,
          " (the bit ", bit_unarmed, "), enabled in INTENR ", soft_armed, " (the bit ", bit_stands,
          ", after clearing the flag ", bit_after_clear, "); ", soft_interrupts,
          " interrupt from one trigger", crlf);
    bench.verdict("the software trigger raises the flag of a line armed in INTENR and, with its "
                  "vector open, one interrupt per trigger",
                  soft_armed && soft_interrupts == 1u);

    // The two shared vectors, each with two lines: the body dispatches by
    // pending bit, so a burst on one line leaves the other's tally alone.
    edges_line5 = 0;
    edges_line6 = 0;
    shared_entries = 0;
    const uint32_t on5 = edges_of<Line5, PadB5>(ExtiSense::rising, 6, edges_line5, Irq::exti9_5);
    const uint32_t stray6 = edges_line6;
    edges_line5 = 0;
    edges_line6 = 0;
    const uint32_t on6 = edges_of<Line6, PadB6>(ExtiSense::rising, 4, edges_line6, Irq::exti9_5);
    const uint32_t stray5 = edges_line5;
    const uint32_t entries_9_5 = shared_entries;
    print(serial, "  the 9..5 vector: three rising edges of PB5 counted ", on5, " (line 6 saw ",
          stray6, "), two of PB6 counted ", on6, " (line 5 saw ", stray5, "), ", entries_9_5,
          " entries", crlf);
    bench.verdict("a shared vector's body attributes each edge to the line whose flag stands "
                  "(lines 5 and 6, one vector)",
                  on5 == 3u && on6 == 2u && stray6 == 0u && stray5 == 0u && entries_9_5 == 5u);

    edges_line12 = 0;
    edges_line13 = 0;
    shared_entries = 0;
    const uint32_t on12 = edges_of<Line12, PadB>(ExtiSense::both, 6, edges_line12, Irq::exti15_10);
    const uint32_t stray13 = edges_line13;
    edges_line12 = 0;
    edges_line13 = 0;
    const uint32_t on13 = edges_of<Line13, PadB13>(ExtiSense::both, 4, edges_line13,
                                                   Irq::exti15_10);
    const uint32_t stray12 = edges_line12;
    print(serial, "  the 15..10 vector: six edges of PB12 counted ", on12, " (line 13 saw ",
          stray13, "), four of PB13 counted ", on13, " (line 12 saw ", stray12, "), ",
          shared_entries, " entries", crlf);
    bench.verdict("the other shared vector counts both edges of each of its two lines, one entry "
                  "per edge (12 and 13)",
                  on12 == 6u && on13 == 4u && stray13 == 0u && stray12 == 0u &&
                      shared_entries == 10u);

    // One line is one PORT: line 1 pointed at port B hears nothing of
    // PA1, and select() refuses to take a line another port is using.
    (void)Exti::release(1);
    PadA::output(false);
    (void)LineA::claim();
    (void)LineA::configure(ExtiSense::both);
    const bool refused = !Exti::select(1, 'B');
    const bool still_a = LineA::selected();
    (void)LineA::arm(true);
    Pfic::clear_pending(Irq::exti1);
    edges_line1 = 0;
    Pfic::enable(Irq::exti1);
    const bool stolen = Exti::steal(1, 'B');
    for (uint8_t i = 0; i < 4u; ++i) {
        PadA::toggle();
        (void)delay_us(clock, 2);
    }
    const uint32_t after_steal = edges_line1;
    Pfic::disable(Irq::exti1);
    (void)Exti::steal(1, 'A');
    print(serial, "  line 1 claimed by PA1: a second port refused ", refused, ", stolen ", stolen,
          ", then PA1's four toggles counted ", after_steal, crlf);
    bench.verdict("a line is ONE pin of ONE port: select() refuses a line another port holds, "
                  "steal() takes it, and the old pad's edges stop arriving",
                  refused && still_a && stolen && after_steal == 0u);
    all_off();
}

// ===========================================================================
// h - the event mode
// ===========================================================================

void th_event() {
    all_off();
    // The console has to be quiet first: its own interrupt would end the
    // wait, and so would an event left latched by anything earlier - a
    // WFE consumes the latch, so the first idle() below is the drain and
    // not a measurement.
    for (uint32_t i = 0; i < 1'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 200);
    Pfic::clear_pending(Irq::usart1);
    P::idle();

    // The control: with nothing armed, the kernel tick is what ends the
    // wait, so exactly one tick passes in it.
    const uint32_t tick0 = Ticker::ticks();
    P::idle();
    const uint32_t ticks_waited = Ticker::ticks() - tick0;

    // Now the line in EVENR - not INTENR - with its edge already raised:
    // 9.4.2's second way, which needs no handler and leaves no flag.
    PadA::output(false);
    (void)LineA::claim();
    PadA::output(false);
    (void)LineA::configure(ExtiSense::rising);
    (void)LineA::clear();
    (void)LineA::event(true);
    PadA::set();
    const uint32_t tick1 = Ticker::ticks();
    const uint32_t c0 = cycles_now();
    P::idle();
    const uint32_t event_cycles = cycles_now() - c0;
    const uint32_t ticks_in_event = Ticker::ticks() - tick1;
    const bool flag_down = !LineA::pending();
    const bool pfic_quiet = !Pfic::pending(Irq::exti1);
    (void)LineA::event(false);
    print(serial, "  idle() with nothing armed waited ", ticks_waited,
          " kernel tick; with a latched line event it returned after ", event_cycles,
          " core cycles and ", ticks_in_event, " ticks (flag ", flag_down ? "down" : "UP",
          ", the vector's pending bit ", pfic_quiet ? "down" : "UP", ")", crlf);
    bench.verdict("a line in EVENT mode ends the platform's idle() inside the tick that would "
                  "otherwise have ended it, with no flag raised and no interrupt pending (9.4.2)",
                  ticks_waited == 1u && ticks_in_event == 0u && event_cycles < 2000u &&
                      flag_down && pfic_quiet);
    all_off();
}

// ===========================================================================
// e - the configuration lock (last in z: the pad stays locked)
// ===========================================================================

void te_lock() {
    all_off();
    const bool was_locked = Port<'C'>::locked();
    LockPad::output(false, PinDrive::push_pull, PinSpeed::slow);
    const uint32_t before = LockPad::nibble();
    const bool took = LockPad::lock();
    const bool lckk = Port<'C'>::locked();
    const uint16_t pins = Port<'C'>::locked_pins();
    // A configuration write after the lock: the verb runs, the silicon
    // ignores it.
    LockPad::input(PinPull::up);
    const uint32_t after = LockPad::nibble();
    print(serial, "  PC13: LCKK before this letter ", was_locked, ", the nibble locked ",
          hex(before), ", LCKR now ", hex(Port<'C'>::regs().LCKR), ", the nibble after a write ",
          hex(after), crlf);
    bench.verdict("the key sequence takes: LCKK reads one and LCK[15:0] names the pin locked",
                  took && lckk && (pins & LockPad::mask) != 0u);
    bench.verdict("a configuration write after the lock is ignored - the nibble stands (10.2.5: "
                  "only a reset changes it back)",
                  after == before);
    // A mask with a pad this package does not bond is refused before
    // anything is written.
    bench.verdict("a lock mask naming a pad this package does not bond is refused",
                  !Port<'C'>::lock(0x0001u));
}

// ===========================================================================
// k - the key on PA0, by name
// ===========================================================================

void tk_key() {
    all_off();
    KeyPad::input();
    print(serial, "  press and release the KEY twice in the next ten seconds; the pad is read "
          "every 10 ms with no pull of its own", crlf, "  ");
    uint32_t high = 0;
    uint32_t low = 0;
    uint32_t changes = 0;
    bool last = KeyPad::read();
    uint8_t column = 0;
    for (uint32_t i = 0; i < 1000u; ++i) {
        const uint32_t start = Ticker::ticks();
        while (Ticker::ticks() - start < 10u) {
        }
        const bool level = KeyPad::read();
        if (level) {
            ++high;
        } else {
            ++low;
        }
        if (level != last) {
            ++changes;
            last = level;
            print(serial, level ? "^" : "v");
            column = static_cast<uint8_t>(column + 1u);
            if (column >= 60u) {
                print(serial, crlf, "  ");
                column = 0;
            }
        }
    }
    print(serial, crlf, "  ten seconds of PA0: ", high, " samples high, ", low, " low, ", changes,
          " changes", crlf);
    // With no external resistor and no pull of its own the level is the
    // board's to explain: this letter states what it saw and judges only
    // that it ran.
    bench.verdict("the KEY's pad was read a thousand times (what the levels mean is the board's "
                  "to say)",
                  high + low == 1000u);
    KeyPad::release();
}

void banner() {
    print(serial, crlf, "test_v203_pin on ", device::part_name,
          " - the pads (RM ch. 10) and the EXTI lines (9.4)", crlf,
          "  nothing to wire: a pad reads its own level and feeds its own line", crlf,
          "  e runs LAST in z - it locks PC13 until the next reset; k wants a finger", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

// The three EXTI vectors this program owns. Each body reads AND CLEARS
// the flags of its own vector first (Exti::isr), then counts what fired:
// the two shared ones are where that matters, since one entry may carry
// two lines.
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void exti1_handler() {
    const uint32_t fired = brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::exti1));
    if (brio::Exti::served(fired, 1)) {
        edges_line1 = edges_line1 + 1u;
    }
}

extern "C" BRIO_CH32_INTERRUPT void exti9_5_handler() {
    const uint32_t fired = brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::exti9_5));
    shared_entries = shared_entries + 1u;
    if (brio::Exti::served(fired, 5)) {
        edges_line5 = edges_line5 + 1u;
    }
    if (brio::Exti::served(fired, 6)) {
        edges_line6 = edges_line6 + 1u;
    }
}

extern "C" BRIO_CH32_INTERRUPT void exti15_10_handler() {
    const uint32_t fired = brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::exti15_10));
    shared_entries = shared_entries + 1u;
    if (brio::Exti::served(fired, 12)) {
        edges_line12 = edges_line12 + 1u;
    }
    if (brio::Exti::served(fired, 13)) {
        edges_line13 = edges_line13 + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the nibble: the gate, the reset state, all fifteen configurations",
                 ta_nibble);
    bench.letter('b', "the pulls, wirelessly: four pads up and down", tb_pulls);
    bench.letter('c', "the open drain and the analog pad, wirelessly", tc_open_drain);
    bench.letter('d', "the whole-port verbs over one mask", td_port);
    bench.letter('f', "the remaps: every column this part has, USART1's left alone", tf_remaps);
    bench.letter('g', "the EXTI lines on the pads' own outputs", tg_edges);
    bench.letter('h', "the event mode ending idle()", th_event);
    bench.letter('k', "THE KEY on PA0: ten seconds of levels while you press it", tk_key, false);
    bench.letter('e', "the configuration LOCK on PC13 (it stays locked until a reset)", te_lock);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
