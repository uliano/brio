// test_rp2350_pio - the reference bench suite for the RP2350's PIO
// (rp2350/pio.hpp over datasheet chapter 11): the assembler against the
// vendor's assembled words, the three blocks and their machines
// wireless, the chapter's own programs on two of the standing wires, and
// EVERY FEATURE THIS CHIP ADDED TO THE RP2040'S PIO - the version field,
// the GPIO window, the neighbour masks, all eight flags on a line, the
// masked input count, the receive FIFO as four registers, and the
// instruction forms that go with them - measured, on both architectures
// from one source.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// TWO WIRES, both part of the standing self-links of this board:
//
//   GP13  ->  GP15   (the serial port: PIO0 machine 0 sends, machine 1
//                     receives; the square wave and the sampler; the net
//                     carries a pull-up to 3V3, which a push-pull output
//                     drives through)
//   GP17  ->  GP9    (the PIO PWM into a level-counting machine)
//
// With a wire missing its input reads HIGH and not low - erratum
// RP2350-E9 on this stepping - so the wire check DRIVES both levels, and
// a letter that finds no wire declines with the reason.
//
// THE RULERS ARE PIO'S OWN MACHINES. This chapter needs a frequency
// counter and a duty counter at the far end of each wire, and on this
// target those are two more state machines: a three-instruction program
// that counts periods into scratch Y, and a ten-instruction one that
// samples its branch pin for a window it counts down itself, so the
// window is exact in machine cycles and no other chapter's driver is
// wanted. The microsecond ruler for spans is the platform timer
// (rp2350/mtime.hpp).
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the encodings against the vendor's assembled
//      words - the nine instructions and the forms this chip added -,
//      DBG_CFGINFO on all THREE blocks with its VERSION field, the reset
//      state, a program loaded and run with its words popped, an
//      instruction executed on the side, the FIFO flags and the join
//   b  THE GPIO WINDOW: GPIOBASE refusing every value but 0 and 16, the
//      pin index each window gives, and one free pad driven from both
//      windows by the same machine
//   c  THE SQUARE WAVE on GP13 counted on GP15 by a second machine at
//      1 MHz, 100 kHz and 10 kHz
//   d  THE SERIAL PORT: 64 bytes at 115200, 1 Mbaud and 3 Mbaud
//      byte-exact, the frame error flagged on a held line and the
//      receiver back after it
//   e  THE PIO PWM as a PwmChannel on GP17, its duty counted on GP9 by a
//      machine that owns its own window
//   f  THE INTERRUPTS: ALL EIGHT flags on a line (the RP2040 offered
//      four), a program raising flag 6, IRQ WAIT stalling a machine
//      until the system clears the flag, and the receive FIFO's source
//   g  THE SAMPLER: one instruction with autopush filling the joined
//      receive FIFO with 256 samples of the 37.5 MHz wave at clk_sys
//   h  THE THREE BLOCKS: an IRQ aimed at the NEXT and the PREVIOUS block
//      (and the ring's wrap from the last to the first), and CTRL's
//      neighbour masks starting and stopping a machine of another block
//   j  THE RECEIVE FIFO AS FOUR REGISTERS: PUT written by the machine
//      and read by the system, GET written by the system and read by the
//      machine
//   k  THE MASKED INPUT AND THE NEW PIN FORMS: SHIFTCTRL's IN_COUNT
//      masking MOV X, PINS, MOV PINDIRS turning an OUT-mapped pin around
//      in one instruction, and a WAIT on the machine's own branch pin
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/pio.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial, 16> bench;

using P0 = Pio<0>;
using P1 = Pio<1>;
using P2 = Pio<2>;

using Tx = PioUartTx<0, 0, 13>;
using Rx = PioUartRx<0, 1, 15>;
using Wave = PioSquareWave<0, 2, 13>;
using Periods = PioSm<0, 3>;        ///< the frequency counter on GP15
using Sampler = PioSm<0, 3>;        ///< and the logic analyser, at another time
using Probe = PioSm<0, 0>;          ///< the wireless machine of letters a, j, k
using Pwm17 = PioPwm<1, 0, 17, 999>;
using Levels = PioSm<1, 1>;         ///< the duty counter on GP9
using Window = PioSm<2, 0>;         ///< the machine letter b drives GP22 with

/// The free pad letter b drives: GP22 is in BOTH windows (index 22 with
/// GPIOBASE 0, index 6 with GPIOBASE 16), which is what makes it the one
/// pad that can show the relocation with no wire.
constexpr uint8_t window_pin = 22;

volatile uint32_t line0_entries = 0;
volatile uint32_t line1_entries = 0;
volatile uint32_t line0_last = 0;
volatile uint32_t line1_last = 0;
volatile bool rx_to_buffer = false;
uint8_t isr_rx[64];
volatile uint8_t isr_rx_count = 0;

uint8_t tx_buf[256];
uint8_t rx_buf[256];
uint32_t words[8];

uint32_t us_now() { return Mtime::micros(); }
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}
bool within(uint32_t got, uint32_t want, uint32_t per_mille) {
    const uint32_t tol = static_cast<uint32_t>(static_cast<uint64_t>(want) * per_mille / 1000u);
    return got + tol >= want && got <= want + tol;
}
void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + 3u * i);
    }
}
bool same(const uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

/// A wire proven by DRIVING both levels: on this stepping an undriven
/// input pad idles high (RP2350-E9), so the low half of this check is
/// the half that carries the information.
template <uint8_t out_pin, uint8_t in_pin>
bool wire_present() {
    (void)Pin<in_pin>::input(PinPull::none);
    (void)Pin<out_pin>::output(false);
    spin_us(20);
    const bool low = !Pin<in_pin>::read();
    Pin<out_pin>::set();
    spin_us(20);
    const bool high = Pin<in_pin>::read();
    (void)Pin<out_pin>::release();
    (void)Pin<in_pin>::release();
    return low && high;
}
bool wire_13_15() {
    if (wire_present<13, 15>()) {
        return true;
    }
    bench.verdict("DECLINED: GP13 -> GP15 is not wired", false);
    return false;
}
bool wire_17_9() {
    if (wire_present<17, 9>()) {
        return true;
    }
    bench.verdict("DECLINED: GP17 -> GP9 is not wired", false);
    return false;
}

/// All three blocks back to their reset state, the lines this suite
/// binds masked, the counters zeroed. THE WINDOW GOES BACK TO 0 with the
/// block, which is why every letter but b can write a pad below GP32 as
/// its own block index - the two counting helpers convert anyway, so
/// that the rule is in the code and not only in this comment.
void pio_fresh() {
    Irq::disable(P0::irq(0));
    Irq::disable(P0::irq(1));
    P0::interrupts_only(0, 0);
    P0::interrupts_only(1, 0);
    (void)P0::reset();
    (void)P1::reset();
    (void)P2::reset();
    rx_to_buffer = false;
    line0_entries = 0;
    line1_entries = 0;
}

// =============================================================================
// The programs this suite writes for itself
// =============================================================================

/// Letter a's probe: a SET, a MOV with each of its two operations, and a
/// countdown that runs out - four words pushed and the machine parked.
constexpr PioProgram<13> probe_program = [] {
    PioProgram<13> p{};
    p.code = {
        pio_set(PioSetTo::x, 5),                                   // 0
        pio_mov(PioMovTo::isr, PioMovFrom::x),                     // 1
        pio_push(),                                                // 2: 5
        pio_set(PioSetTo::y, 3),                                   // 3
        pio_mov(PioMovTo::isr, PioMovFrom::y, PioMovOp::invert),   // 4
        pio_push(),                                                // 5: ~3
        pio_mov(PioMovTo::isr, PioMovFrom::y, PioMovOp::reverse),  // 6
        pio_push(),                                                // 7: 3 reversed
        pio_set(PioSetTo::x, 4),                                   // 8
        pio_jmp(PioJmp::x_dec, 9),                                 // 9: the countdown, on itself
        pio_mov(PioMovTo::isr, PioMovFrom::x),                     // 10
        pio_push(),                                                // 11: what X ended at
        pio_jmp(PioJmp::always, 12),                               // 12: parked
    };
    return p;
}();

/// THE FREQUENCY COUNTER: one period of the IN pin per iteration, Y
/// counting DOWN from zero, so the count is its two's complement. The
/// program wraps, so the JMP not taken is the loop too.
constexpr PioProgram<3> period_count_program = [] {
    PioProgram<3> p{};
    p.code = {
        pio_wait(true, PioWaitOn::pin, 0),     // wait 1 pin 0
        pio_wait(false, PioWaitOn::pin, 0),    // wait 0 pin 0
        pio_jmp(PioJmp::y_dec, 0),             // jmp y-- again
    };
    return p;
}();

/// THE DUTY COUNTER: the window arrives in the FIFO, X counts it down,
/// and BOTH BRANCHES ARE THREE INSTRUCTIONS LONG - which is what makes
/// the sampling uniform and the window exact at three cycles a sample.
/// Y counts the samples the branch pin was high for, downwards.
constexpr PioProgram<10> level_count_program = [] {
    PioProgram<10> p{};
    p.code = {
        pio_pull(false, true),                 // 0: pull block - the window
        pio_set(PioSetTo::y, 0),               // 1
        pio_out(PioOut::x, 32),                // 2: x = samples - 1
        pio_jmp(PioJmp::pin, 5),               // 3: sample: jmp pin high
        pio_jmp(PioJmp::always, 6),            // 4: low
        pio_jmp(PioJmp::y_dec, 6),             // 5: high: count it
        pio_jmp(PioJmp::x_dec, 3),             // 6: next sample
        pio_mov(PioMovTo::isr, PioMovFrom::y), // 7
        pio_push(),                            // 8
        pio_jmp(PioJmp::always, 9),            // 9: parked
    };
    return p;
}();

/// THE LOGIC ANALYSER: one pin into the ISR every machine cycle,
/// autopush at 32, the FIFO joined into eight - 256 samples and then a
/// stall.
constexpr PioProgram<1> sample_program = [] {
    PioProgram<1> p{};
    p.code = {pio_in(PioIn::pins, 1)};
    return p;
}();

/// Letter f: IRQ 0 WAIT, which raises the flag and holds the machine
/// until the system lowers it.
constexpr PioProgram<4> flag_wait_program = [] {
    PioProgram<4> p{};
    p.code = {
        pio_irq(0, false, true),
        pio_set(PioSetTo::x, 1),
        pio_mov(PioMovTo::isr, PioMovFrom::x),
        pio_push(),
    };
    return p;
}();

/// Letter f: flag SIX raised and left - one of the four the RP2040 could
/// not route to an interrupt line at all.
constexpr PioProgram<2> flag6_program = [] {
    PioProgram<2> p{};
    p.code = {pio_irq(6), pio_jmp(PioJmp::always, 1)};
    return p;
}();

/// Letter h: a flag of the NEXT block in the ring, and one of the
/// PREVIOUS.
constexpr PioProgram<2> irq_next_program = [] {
    PioProgram<2> p{};
    p.code = {pio_irq(3, false, false, PioIrqScope::next), pio_jmp(PioJmp::always, 1)};
    return p;
}();
constexpr PioProgram<2> irq_prev_program = [] {
    PioProgram<2> p{};
    p.code = {pio_irq(5, false, false, PioIrqScope::prev), pio_jmp(PioJmp::always, 1)};
    return p;
}();

/// Letter h's parked machine: something for a neighbour's CTRL write to
/// start and stop.
constexpr PioProgram<1> park_program = [] {
    PioProgram<1> p{};
    p.code = {pio_jmp(PioJmp::always, 0)};
    return p;
}();

/// Letter k: a WAIT on the machine's own branch pin, then a word.
constexpr PioProgram<5> jmppin_wait_program = [] {
    PioProgram<5> p{};
    p.code = {
        pio_wait_jmppin(true),
        pio_set(PioSetTo::x, 7),
        pio_mov(PioMovTo::isr, PioMovFrom::x),
        pio_push(),
        pio_jmp(PioJmp::always, 4),
    };
    return p;
}();

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("the nine instructions against the vendor's assembled words: the transmitter's four (0x9FA0 0xF727 "
                  "0x6001 0x0642), the receiver's nine, `set pins, 1 [1]` 0xE101, a relocated JMP",
                  pio_uart_tx_program.code[0] == 0x9FA0u && pio_uart_tx_program.code[1] == 0xF727u &&
                      pio_uart_tx_program.code[3] == 0x0642u && pio_uart_rx_program.code[0] == 0x2020u &&
                      pio_uart_rx_program.code[1] == 0xEA27u && pio_uart_rx_program.code[6] == 0x20A0u &&
                      pio_uart_rx_program.code[8] == 0x8020u && pio_delay(pio_set(PioSetTo::pins, 1), 1) == 0xE101u &&
                      pio_relocate(pio_jmp(PioJmp::x_dec, 2), 10) == 0x004Cu);
    bench.verdict("and the forms this chip added (11.4): `irq next 3` 0xC01B, `irq prev clear 3` 0xC04B, `wait 1 irq "
                  "prev 3` 0x20CB, `wait 1 jmppin` 0x20E0, `mov pindirs, ~null` 0xA06B, `mov rxfifo[2], isr` 0x801A, "
                  "`mov osr, rxfifo[1]` 0x8099",
                  pio_irq(3, false, false, PioIrqScope::next) == 0xC01Bu &&
                      pio_irq(3, true, false, PioIrqScope::prev) == 0xC04Bu &&
                      pio_wait_irq(true, 3, PioIrqScope::prev) == 0x20CBu && pio_wait_jmppin(true) == 0x20E0u &&
                      pio_mov(PioMovTo::pindirs, PioMovFrom::null, PioMovOp::invert) == 0xA06Bu && pio_put(2) == 0x801Au &&
                      pio_get(1) == 0x8099u);
    pio_fresh();
    print(serial, "  PIO0/1/2 as built: version ", P0::version(), "/", P1::version(), "/", P2::version(), ", ",
          P0::memory_size(), " instructions, ", P0::machine_count(), " machines, FIFOs ", P0::fifo_depth(),
          " deep; PIO0 CTRL=", hex(P0::reg(PIO_CTRL_OFFSET)), " FSTAT=", hex(P0::fifo_status()),
          " GPIOBASE=", P0::gpio_base(), crlf);
    bench.verdict("DBG_CFGINFO says VERSION 1 on all three blocks (the RP2040's PIO reads 0), 32 instructions, 4 "
                  "machines, FIFOs 4 deep; the reset state: no machine enabled, every FIFO empty, the window at 0",
                  P0::version() == pio_version && P1::version() == pio_version && P2::version() == pio_version &&
                      P0::memory_size() == 32u && P0::machine_count() == 4u && P0::fifo_depth() == 4u &&
                      P0::enabled() == 0u && P0::fifo_status() == 0x0F000F00u && P0::gpio_base() == 0u);
    // A program run: SET, MOV with its two operations, the countdown.
    const auto at = P0::add(probe_program);
    const bool up = at && Probe::init(probe_program, *at, {.clock = {1, 0}});
    Probe::enable(true);
    spin_us(50);
    Probe::enable(false);
    const uint8_t level = Probe::rx_level();
    const uint32_t w0 = level >= 4u ? Probe::pop() : 0xDEADu;
    const uint32_t w1 = level >= 4u ? Probe::pop() : 0xDEADu;
    const uint32_t w2 = level >= 4u ? Probe::pop() : 0xDEADu;
    const uint32_t w3 = level >= 4u ? Probe::pop() : 0xDEADu;
    print(serial, "  the probe program at offset ", at ? *at : 99u, ": ", level, " words, ", hex(w0), " ", hex(w1), " ",
          hex(w2), " ", hex(w3), ", parked at ", Probe::address(), crlf);
    bench.verdict("a program loaded and run: SET X 5 pushed, Y 3 inverted (0xFFFFFFFC) and bit-reversed (0xC0000000), "
                  "the machine parked on a JMP to itself",
                  up && level == 4u && w0 == 5u && w1 == 0xFFFFFFFCu && w2 == 0xC0000000u && at &&
                      Probe::address() == *at + 12u);
    bench.verdict("JMP X-- run to its end leaves X at all ones: the decrement is unconditional", w3 == 0xFFFFFFFFu);
    (void)Probe::exec_wait(pio_set(PioSetTo::x, 9));
    (void)Probe::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)Probe::exec_wait(pio_push());
    const auto nine = Probe::pop_wait(1000);
    bench.verdict("SET X 9 executed on the side of a DISABLED machine and pushed: 9", nine && *nine == 9u);
    // The FIFO flags.
    Probe::drain_rx();
    Probe::clear_fifo_debug();
    for (uint8_t i = 0; i < 4u; ++i) {
        Probe::push(i);
    }
    const bool full = Probe::tx_full() && Probe::tx_level() == 4u;
    Probe::push(4);   // the fifth
    const bool over = Probe::tx_overflowed();
    (void)Probe::pop();
    const bool under = Probe::rx_underflowed();
    Probe::drain_tx();
    Probe::clear_fifo_debug();
    bench.verdict("the transmit FIFO is full at four and a fifth push sets TXOVER; a pop of the empty receive FIFO sets "
                  "RXUNDER",
                  full && over && under && Probe::tx_empty());
    // The join: eight words.
    (void)Probe::configure({.fifo_join = PioFifoJoin::tx});
    for (uint8_t i = 0; i < 8u; ++i) {
        Probe::push(i);
    }
    const bool eight = Probe::tx_full() && Probe::tx_level() == 8u;
    Probe::drain_tx();
    (void)Probe::configure({});
    bench.verdict("joined, the transmit FIFO takes eight", eight);
    if (at) {
        P0::unload(*at, probe_program.length);
    }
}

// =============================================================================
// b - the GPIO window
// =============================================================================
void tb_window() {
    pio_fresh();
    const bool base_zero = P2::gpio_base() == 0u;
    const bool refused = !P2::gpio_base(8u) && !P2::gpio_base(32u) && P2::gpio_base() == 0u;
    const auto at_zero = P2::pin_index(window_pin);
    const auto past_zero = P2::pin_index(40u);
    const bool moved = P2::gpio_base(16u) && P2::gpio_base() == 16u;
    const auto at_sixteen = P2::pin_index(window_pin);
    const auto below = P2::pin_index(13u);
    const auto top = P2::pin_index(47u);
    print(serial, "  GPIOBASE 0: GP", window_pin, " is index ", at_zero ? *at_zero : 99u, ", GP40 ",
          past_zero ? "in" : "outside", " the window; GPIOBASE 16: GP", window_pin, " is index ",
          at_sixteen ? *at_sixteen : 99u, ", GP13 ", below ? "in" : "outside", ", GP47 index ", top ? *top : 99u, crlf);
    bench.verdict("GPIOBASE takes 0 and 16 and refuses every other value; the window is 32 pads wide, so with the "
                  "window at 0 a pad above GP31 has no index and with it at 16 a pad below GP16 has none",
                  base_zero && refused && moved && at_zero && *at_zero == window_pin && !past_zero && at_sixteen &&
                      *at_sixteen == static_cast<uint8_t>(window_pin - 16u) && !below && top && *top == 31u);

    // One pad driven from the far window: the machine writes index 6 and
    // the level lands on GP22, which only the relocation can explain.
    (void)Pin<window_pin>::function(PinFunction::pio2);
    Window::pin_directions(static_cast<uint8_t>(window_pin - 16u), 1, true);
    Window::pin_levels(static_cast<uint8_t>(window_pin - 16u), 1, false);
    spin_us(5);
    const bool low_far = !Pin<window_pin>::read();
    Window::pin_levels(static_cast<uint8_t>(window_pin - 16u), 1, true);
    spin_us(5);
    const bool high_far = Pin<window_pin>::read();
    const uint32_t padoe_far = P2::pad_oe();
    bench.verdict("with the window at 16, a SET on the block's index 6 drives GP22 low and high - the pad the same "
                  "index would miss by sixteen with the window at 0",
                  low_far && high_far && (padoe_far & (1UL << (window_pin - 16u))) != 0u);

    // And from the near window, at the pad's own number.
    (void)P2::reset();
    (void)Pin<window_pin>::function(PinFunction::pio2);
    Window::pin_directions(window_pin, 1, true);
    Window::pin_levels(window_pin, 1, false);
    spin_us(5);
    const bool low_near = !Pin<window_pin>::read();
    Window::pin_levels(window_pin, 1, true);
    spin_us(5);
    const bool high_near = Pin<window_pin>::read();
    const uint32_t padoe_near = P2::pad_oe();
    print(serial, "  DBG_PADOE reads ", hex(padoe_far), " with the window at 16 and ", hex(padoe_near), " at 0", crlf);
    bench.verdict("with the window back at 0, index 22 is the same pad: DBG_PADOE moves with the window and the pad "
                  "follows either way",
                  low_near && high_near && (padoe_near & (1UL << window_pin)) != 0u);
    (void)Pin<window_pin>::release();
    (void)P2::reset();
}

// =============================================================================
// The two counting machines
// =============================================================================

/// Count the periods on the system pad `gpio` (a block-0 machine) over a
/// window the CPU times. Nothing when the pad is outside the block's
/// window or the program would not load.
std::optional<uint32_t> count_periods(uint8_t gpio, uint32_t window_us) {
    const auto in_pin = P0::pin_index(gpio);
    const auto at = in_pin ? P0::add(period_count_program) : std::optional<uint8_t>{};
    if (!at) {
        return {};
    }
    PioSmConfig c{};
    c.clock = PioClockDiv{1, 0};
    c.in_base = *in_pin;
    if (!Periods::init(period_count_program, *at, c)) {
        P0::unload(*at, period_count_program.length);
        return {};
    }
    (void)Periods::exec_wait(pio_set(PioSetTo::y, 0));
    Periods::enable(true);
    spin_us(window_us);
    Periods::enable(false);
    // The machine is stopped inside a WAIT; SM_RESTART drops that and
    // leaves X and Y, which are the measurement.
    Periods::restart();
    (void)Periods::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::y));
    (void)Periods::exec_wait(pio_push());
    const auto y = Periods::pop_wait(10'000);
    P0::unload(*at, period_count_program.length);
    if (!y) {
        return {};
    }
    return static_cast<uint32_t>(0u - *y);
}

/// The fraction of `samples` samples (three machine cycles each) for
/// which the system pad `gpio` was high, in per mille - a block-1
/// machine, its branch pin the block's index for that pad.
std::optional<uint32_t> count_high(uint8_t gpio, uint32_t samples) {
    const auto pin = P1::pin_index(gpio);
    const auto at = pin ? P1::add(level_count_program) : std::optional<uint8_t>{};
    if (!at || samples == 0u) {
        return {};
    }
    PioSmConfig c{};
    c.clock = PioClockDiv{1, 0};
    c.jmp_pin = *pin;
    c.in_base = *pin;
    if (!Levels::init(level_count_program, *at, c)) {
        P1::unload(*at, level_count_program.length);
        return {};
    }
    Levels::enable(true);
    Levels::push(samples - 1u);
    const auto y = Levels::pop_wait(2'000'000);
    Levels::enable(false);
    P1::unload(*at, level_count_program.length);
    if (!y) {
        return {};
    }
    const uint32_t highs = static_cast<uint32_t>(0u - *y);
    return static_cast<uint32_t>((static_cast<uint64_t>(highs) * 1000u) / samples);
}

// =============================================================================
// c - the square wave, counted by a second machine
// =============================================================================
void tc_square_wave() {
    if (!wire_13_15()) {
        return;
    }
    const uint32_t rates[] = {1'000'000, 100'000, 10'000};
    uint8_t ok = 0;
    for (uint32_t hz : rates) {
        pio_fresh();
        (void)Pin<15>::function(PinFunction::pio0);
        const bool up = Wave::init(clock, hz);
        const uint32_t window = hz >= 1'000'000u ? 20'000u : (hz >= 100'000u ? 50'000u : 200'000u);
        const auto periods = count_periods(15u, window);
        const uint32_t got = periods ? static_cast<uint32_t>((static_cast<uint64_t>(*periods) * 1'000'000u) / window) : 0u;
        Wave::release();
        (void)Pin<15>::release();
        const bool good = up && periods && within(got, hz, 10);
        const auto div = pio_clock_div_for(SysClock::hz, hz * 4u);
        print(serial, "  ", hz, " Hz asked (divider ", div ? div->integer : 0u, " + ", div ? div->frac : 0u, "/256): ",
              periods ? *periods : 0u, " periods in ", window, " us = ", got, " Hz on GP15", good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
    }
    bench.verdict("the four-cycle square wave at 1 MHz, 100 kHz and 10 kHz, each within one per cent of what was asked, "
                  "counted period by period by a three-instruction machine at the far end of the wire",
                  ok == 3u);
}

// =============================================================================
// d - the serial port
// =============================================================================
void td_serial() {
    if (!wire_13_15()) {
        return;
    }
    const uint32_t bauds[] = {115'200, 1'000'000, 3'000'000};
    uint8_t ok = 0;
    for (uint32_t baud : bauds) {
        pio_fresh();
        const bool up = Tx::init(clock, baud) && Rx::init(clock, baud);
        fill_pattern(tx_buf, 64, static_cast<uint8_t>(baud >> 12));
        for (uint16_t i = 0; i < 64; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint32_t t0 = us_now();
        uint16_t got = 0;
        uint16_t sent = 0;
        while (got < 64u && us_now() - t0 < 100'000u) {
            if (sent < 64u && Tx::writable()) {
                Tx::write(tx_buf[sent++]);
            }
            if (Rx::readable()) {
                rx_buf[got++] = Rx::read();
            }
        }
        const uint32_t took = us_now() - t0;
        const bool exact = got == 64u && same(tx_buf, rx_buf, 64);
        const bool frame = Rx::frame_error();
        const uint32_t want_us = static_cast<uint32_t>(64ULL * 10u * 1'000'000u / baud);
        print(serial, "  ", baud, " baud: 64 bytes ", exact ? "exact" : "WRONG", " in ", took, " us (", want_us,
              " on the wire), frame error=", frame, crlf);
        if (up && exact && !frame && took >= want_us && took < want_us + 600u) {
            ++ok;
        }
        Tx::release();
        Rx::release();
    }
    bench.verdict("64 bytes at 115200, 1 Mbaud and 3 Mbaud byte-exact from machine 0 to machine 1 on one wire, in the "
                  "wire's own time, no frame error",
                  ok == 3u);
    // The frame error: the line held low past a frame, then released.
    pio_fresh();
    (void)Tx::init(clock, 115'200);
    (void)Rx::init(clock, 115'200);
    (void)Rx::frame_error();
    Tx::Sm::enable(false);
    Tx::Sm::pin_levels(13, 1, false);   // the line held low: a break
    spin_us(200);
    Tx::Sm::pin_levels(13, 1, true);
    spin_us(200);
    const bool flagged = Rx::frame_error();
    const bool nothing = !Rx::readable();
    Tx::Sm::enable(true);
    Tx::write(0x5A);
    const auto after = Rx::read_wait();
    print(serial, "  a 200 us break: frame error=", flagged, ", nothing received=", nothing, ", the next byte ",
          after ? hex(*after) : hex(0xFFu), crlf);
    bench.verdict("a line held low past a frame raises the receiver's frame flag (flag 4 + the machine's number) and "
                  "delivers nothing; the receiver takes the next byte once the line is idle again",
                  flagged && nothing && after && *after == 0x5Au);
    Tx::release();
    Rx::release();
}

// =============================================================================
// e - the PIO PWM, counted by a second machine
// =============================================================================
void te_pwm() {
    if (!wire_17_9()) {
        return;
    }
    pio_fresh();
    const bool up = Pwm17::init({1, 0});
    (void)Pin<9>::function(PinFunction::pio1);
    const uint16_t levels[] = {250, 500, 750};
    constexpr uint32_t samples = 150'000;   // 450 000 machine cycles = 3 ms at 150 MHz
    uint8_t ok = 0;
    for (uint16_t lv : levels) {
        Pwm17::duty(lv);
        spin_us(500);
        const auto pm = count_high(9u, samples);
        const bool good = pm && within(*pm, lv, 20);
        print(serial, "  level ", lv, " of 999: ", pm ? *pm : 0u, " per mille high on GP9", good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
    }
    bench.verdict("the PIO PWM (11.6.8) as a PwmChannel: levels 250, 500 and 750 of 999 read 25, 50 and 75 % by a "
                  "machine sampling its branch pin every three cycles for a window of its own counting, within two "
                  "per cent",
                  up && ok == 3u);
    (void)Pin<9>::release();
    Pwm17::release();
}

// =============================================================================
// f - the interrupts and the flags
// =============================================================================
void tf_interrupts() {
    pio_fresh();
    // ALL EIGHT flags on a line: flag 7 alone, then the eight together.
    P0::clear_flags(0xFF);
    P0::interrupts_only(0, PioInterrupt::all_flags);
    Irq::enable(P0::irq(0));
    P0::raise_flags(0x80);
    spin_us(50);
    const uint32_t after_seven = line0_last;
    const uint32_t entries_seven = line0_entries;
    line0_entries = 0;
    P0::raise_flags(0xFF);
    spin_us(50);
    const uint32_t after_all = line0_last;
    const bool cleared = P0::flags() == 0u;
    Irq::disable(P0::irq(0));
    P0::interrupts_only(0, 0);
    print(serial, "  flag 7 alone: INTS=", hex(after_seven), " in ", entries_seven, " entries; all eight: INTS=",
          hex(after_all), ", flags after the ISR=", hex(P0::flags()), crlf);
    bench.verdict("ALL EIGHT flags reach an interrupt line here, where the RP2040 offered the lower four: flag 7 "
                  "raised through IRQ_FORCE lands in bit 15 of INTS, the eight together in bits 15..8, and the ISR "
                  "body clears every one it took",
                  entries_seven == 1u && (after_seven & PioInterrupt::flag(7)) != 0u &&
                      (after_all & PioInterrupt::all_flags) == PioInterrupt::all_flags && cleared);

    // A program's own flag, above the RP2040's four, on line 1.
    const auto at6 = P0::add(flag6_program);
    const bool up6 = at6 && PioSm<0, 2>::init(flag6_program, *at6, {});
    P0::clear_flags(0xFF);
    line1_entries = 0;
    P0::interrupts_only(1, PioInterrupt::flag(6));
    Irq::enable(P0::irq(1));
    PioSm<0, 2>::enable(true);
    spin_us(50);
    PioSm<0, 2>::enable(false);
    Irq::disable(P0::irq(1));
    P0::interrupts_only(1, 0);
    bench.verdict("a program's IRQ 6 reaches line 1 through IRQ1_INTE, the ISR body clears the flag, one entry",
                  up6 && line1_entries == 1u && (line1_last & PioInterrupt::flag(6)) != 0u && !P0::flag(6));
    if (at6) {
        P0::unload(*at6, flag6_program.length);
    }

    // IRQ WAIT: the machine stalls until the system clears the flag.
    const auto at0 = P0::add(flag_wait_program);
    const bool upw = at0 && PioSm<0, 2>::init(flag_wait_program, *at0, {});
    P0::clear_flags(0xFF);
    PioSm<0, 2>::enable(true);
    spin_us(100);
    const bool flag_up = P0::flag(0);
    const bool nothing = PioSm<0, 2>::rx_empty();
    const uint8_t held_at = PioSm<0, 2>::address();
    P0::clear_flags(0x01);
    spin_us(100);
    const bool went = !PioSm<0, 2>::rx_empty() && PioSm<0, 2>::pop() == 1u;
    PioSm<0, 2>::enable(false);
    print(serial, "  IRQ 0 WAIT at ", at0 ? *at0 : 99u, ": flag up=", flag_up, ", nothing pushed=", nothing,
          ", the machine at ", held_at, "; cleared by the system: the word arrived=", went, crlf);
    bench.verdict("IRQ WAIT raises the flag and holds the machine until the system clears the flag, then the program "
                  "goes on and pushes",
                  upw && flag_up && nothing && went);
    if (at0) {
        P0::unload(*at0, flag_wait_program.length);
    }

    // The receive FIFO's source, with the serial port on the wire.
    if (!wire_13_15()) {
        return;
    }
    pio_fresh();
    (void)Tx::init(clock, 1'000'000);
    (void)Rx::init(clock, 1'000'000);
    isr_rx_count = 0;
    rx_to_buffer = true;
    P0::interrupts_only(0, Rx::Sm::interrupt_rx_not_empty);
    Irq::enable(P0::irq(0));
    fill_pattern(tx_buf, 32, 0x70);
    for (uint8_t i = 0; i < 32u; ++i) {
        (void)Tx::write_wait(tx_buf[i]);
    }
    spin_us(2000);
    Irq::disable(P0::irq(0));
    P0::interrupts_only(0, 0);
    rx_to_buffer = false;
    print(serial, "  the receive FIFO on line 0: ", isr_rx_count, " bytes taken by the ISR in ", line0_entries,
          " entries", crlf);
    bench.verdict("the receive FIFO's not-empty source on line 0: 32 bytes taken by the ISR, byte-exact",
                  isr_rx_count == 32u && same(tx_buf, isr_rx, 32) && line0_entries >= 1u);
    Tx::release();
    Rx::release();
}

// =============================================================================
// g - the sampler
// =============================================================================
void tg_sampler() {
    if (!wire_13_15()) {
        return;
    }
    pio_fresh();
    const bool wave_up = Wave::init(clock, 37'500'000);   // clk_sys / 4, the machine at full speed
    (void)Pin<15>::function(PinFunction::pio0);
    const auto at = P0::add(sample_program);
    PioSmConfig c{};
    c.clock = PioClockDiv{1, 0};
    c.in_base = 15;
    c.in_shift_right = false;      // the first sample ends up in bit 31
    c.autopush = true;
    c.push_threshold = 32;
    c.fifo_join = PioFifoJoin::rx;
    const bool up = at && Sampler::init(sample_program, *at, c);
    for (uint8_t i = 0; i < 8u; ++i) {
        words[i] = 0;
    }
    Sampler::enable(true);
    spin_us(50);                   // 256 samples take 1.7 us; the machine then stalls
    Sampler::enable(false);
    uint8_t got = 0;
    while (got < 8u && !Sampler::rx_empty()) {
        words[got++] = Sampler::pop();
    }
    uint32_t ones = 0;
    uint32_t transitions = 0;
    uint8_t last = 2;
    for (uint8_t w = 0; w < got; ++w) {
        for (int8_t b = 31; b >= 0; --b) {
            const uint8_t bit = static_cast<uint8_t>((words[w] >> b) & 1u);
            ones += bit;
            if (last != 2u && bit != last) {
                ++transitions;
            }
            last = bit;
        }
    }
    const uint32_t hz = transitions == 0u ? 0u
                                          : static_cast<uint32_t>((static_cast<uint64_t>(transitions) * SysClock::hz) /
                                                                  (2u * (32u * got - 1u)));
    Sampler::enable(false);
    Wave::release();
    if (at) {
        P0::unload(*at, sample_program.length);
    }
    (void)Pin<15>::release();
    print(serial, "  ", got, " words of 32 samples at clk_sys: ", ones, " ones, ", transitions, " transitions, first ",
          hex(words[0]), " ", hex(words[1]), "; the wave reads ", hz, " Hz", crlf);
    bench.verdict("a one-instruction sampler with autopush fills the joined receive FIFO with eight words - 256 "
                  "samples at clk_sys - of a 37.5 MHz wave: half of them ones, a transition every other sample, and "
                  "the rate read back within five per cent",
                  wave_up && up && got == 8u && within(ones, 128u, 50) && within(transitions, 127u, 50) &&
                      within(hz, 37'500'000u, 50));
}

// =============================================================================
// h - the three blocks: the cross-block flags and CTRL's neighbour masks
// =============================================================================
void th_blocks() {
    pio_fresh();
    // PIO0's machine raises a flag of the NEXT block.
    const auto at_next = P0::add(irq_next_program);
    const bool up_next = at_next && PioSm<0, 0>::init(irq_next_program, *at_next, {});
    P0::clear_flags(0xFF);
    P1::clear_flags(0xFF);
    PioSm<0, 0>::enable(true);
    spin_us(50);
    PioSm<0, 0>::enable(false);
    const bool landed_next = P1::flag(3) && !P0::flag(3);
    if (at_next) {
        P0::unload(*at_next, irq_next_program.length);
    }

    // PIO2's NEXT wraps round the ring to PIO0.
    const auto at_wrap = P2::add(irq_next_program);
    const bool up_wrap = at_wrap && PioSm<2, 0>::init(irq_next_program, *at_wrap, {});
    P0::clear_flags(0xFF);
    P2::clear_flags(0xFF);
    PioSm<2, 0>::enable(true);
    spin_us(50);
    PioSm<2, 0>::enable(false);
    const bool landed_wrap = P0::flag(3) && !P2::flag(3);
    if (at_wrap) {
        P2::unload(*at_wrap, irq_next_program.length);
    }

    // PIO1's machine raises a flag of the PREVIOUS block, PIO0.
    const auto at_prev = P1::add(irq_prev_program);
    const bool up_prev = at_prev && PioSm<1, 0>::init(irq_prev_program, *at_prev, {});
    P0::clear_flags(0xFF);
    P1::clear_flags(0xFF);
    PioSm<1, 0>::enable(true);
    spin_us(50);
    PioSm<1, 0>::enable(false);
    const bool landed_prev = P0::flag(5) && !P1::flag(5);
    if (at_prev) {
        P1::unload(*at_prev, irq_prev_program.length);
    }
    P0::clear_flags(0xFF);
    P1::clear_flags(0xFF);
    P2::clear_flags(0xFF);
    print(serial, "  the ring: PIO0's `irq next 3` landed on PIO1=", landed_next, ", PIO2's on PIO0=", landed_wrap,
          ", PIO1's `irq prev 5` on PIO0=", landed_prev, crlf);
    bench.verdict("an IRQ instruction names a flag of a NEIGHBOURING block: PIO0's `irq next 3` raises PIO1's flag 3 "
                  "and not its own, PIO1's `irq prev 5` raises PIO0's, and the three blocks are a RING - PIO2's next "
                  "is PIO0",
                  up_next && up_prev && up_wrap && landed_next && landed_wrap && landed_prev);

    // CTRL's neighbour masks: PIO0's CTRL starts and stops a machine of
    // PIO1, and one of its own in the same write.
    pio_fresh();
    const auto at_park0 = P0::add(park_program);
    const auto at_park1 = P1::add(park_program);
    const bool parked = at_park0 && at_park1 && PioSm<0, 0>::init(park_program, *at_park0, {}) &&
                        PioSm<1, 3>::init(park_program, *at_park1, {});
    const bool both_off = P0::enabled() == 0u && P1::enabled() == 0u;
    P0::enable_across(0x1u, {.next = 0x8});
    const uint8_t on0 = P0::enabled();
    const uint8_t on1 = P1::enabled();
    P0::disable_across(0x1u, {.next = 0x8});
    const uint8_t off0 = P0::enabled();
    const uint8_t off1 = P1::enabled();
    P0::restart_clocks_across(0x1u, {.next = 0x8});
    print(serial, "  one write to PIO0's CTRL: PIO0 SM_ENABLE=", hex(on0), " PIO1 SM_ENABLE=", hex(on1),
          "; the undo: ", hex(off0), " ", hex(off1), crlf);
    bench.verdict("CTRL's NEXT_PIO_MASK reaches the neighbouring block: ONE write to PIO0's CTRL enables PIO0's "
                  "machine 0 and PIO1's machine 3 together, and one undoes both - the only way on this chip to start "
                  "machines of two blocks on the same cycle",
                  parked && both_off && on0 == 0x1u && on1 == 0x8u && off0 == 0u && off1 == 0u);
    if (at_park0) {
        P0::unload(*at_park0, park_program.length);
    }
    if (at_park1) {
        P1::unload(*at_park1, park_program.length);
    }
    pio_fresh();
}

// =============================================================================
// j - the receive FIFO as four registers
// =============================================================================
void tj_putget() {
    pio_fresh();
    // PUT: the machine writes a cell, the system reads it.
    const bool put_up = Probe::configure({.fifo_join = PioFifoJoin::rx_put});
    (void)Probe::exec_wait(pio_set(PioSetTo::x, 21));
    (void)Probe::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)Probe::exec_wait(pio_put(2));
    const uint32_t cell2 = Probe::putget(2);
    (void)Probe::exec_wait(pio_set(PioSetTo::x, 6));
    (void)Probe::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)Probe::exec_wait(pio_put(0));
    const uint32_t cell0 = Probe::putget(0);
    print(serial, "  PUT: cell 2 reads ", cell2, ", cell 0 reads ", cell0, crlf);
    bench.verdict("FJOIN_RX_PUT turns the receive FIFO into four cells the machine writes IN ANY ORDER and the system "
                  "reads at RXFn_PUTGETm: 21 written into cell 2 and 6 into cell 0 read back as written",
                  put_up && cell2 == 21u && cell0 == 6u);

    // GET: the system writes a cell, the machine reads it.
    const bool get_up = Probe::configure({.fifo_join = PioFifoJoin::rx_get});
    Probe::putget(1) = 0x12345678u;
    Probe::putget(3) = 0x0BADC0DEu;
    (void)Probe::exec_wait(pio_get(1));
    (void)Probe::exec_wait(pio_mov(PioMovTo::x, PioMovFrom::osr));
    (void)Probe::exec_wait(pio_get(3));
    (void)Probe::exec_wait(pio_mov(PioMovTo::y, PioMovFrom::osr));
    // The cells go back to being a queue, and the two words come out of
    // the scratch registers that survived the change.
    const bool back = Probe::configure({});
    (void)Probe::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)Probe::exec_wait(pio_push());
    (void)Probe::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::y));
    (void)Probe::exec_wait(pio_push());
    const auto first = Probe::pop_wait(10'000);
    const auto second = Probe::pop_wait(10'000);
    print(serial, "  GET: cell 1 came back as ", hex(first ? *first : 0u), ", cell 3 as ", hex(second ? *second : 0u),
          crlf);
    bench.verdict("FJOIN_RX_GET is the mirror - the system writes the four cells and the machine reads them with GET: "
                  "0x12345678 from cell 1 and 0x0BADC0DE from cell 3, taken into X and Y and pushed once the pair is "
                  "a queue again",
                  get_up && back && first && *first == 0x12345678u && second && *second == 0x0BADC0DEu);
    (void)Probe::configure({});
}

// =============================================================================
// k - the masked input, MOV PINDIRS and the WAIT on the branch pin
// =============================================================================
void tk_pins() {
    if (!wire_13_15()) {
        return;
    }
    pio_fresh();
    // IN_COUNT: MOV X, PINS masked to one pin, and unmasked.
    (void)Pin<13>::output(true);
    (void)Pin<15>::input(PinPull::none);
    spin_us(20);
    PioSmConfig c{};
    c.in_base = 13;
    c.in_count = 1;
    (void)Probe::configure(c);
    (void)Probe::exec_wait(pio_mov(PioMovTo::x, PioMovFrom::pins));
    (void)Probe::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)Probe::exec_wait(pio_push());
    const auto masked_high = Probe::pop_wait(10'000);
    c.in_count = 3;
    (void)Probe::configure(c);
    (void)Probe::exec_wait(pio_mov(PioMovTo::x, PioMovFrom::pins));
    (void)Probe::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)Probe::exec_wait(pio_push());
    const auto three_high = Probe::pop_wait(10'000);
    Pin<13>::clear();
    spin_us(20);
    c.in_count = 1;
    (void)Probe::configure(c);
    (void)Probe::exec_wait(pio_mov(PioMovTo::x, PioMovFrom::pins));
    (void)Probe::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)Probe::exec_wait(pio_push());
    const auto masked_low = Probe::pop_wait(10'000);
    print(serial, "  MOV X, PINS at IN_BASE 13: IN_COUNT 1 reads ", hex(masked_high ? *masked_high : 0xFFu),
          " with the pad high and ", hex(masked_low ? *masked_low : 0xFFu), " with it low; IN_COUNT 3 reads ",
          hex(three_high ? *three_high : 0xFFu), crlf);
    bench.verdict("SHIFTCTRL's IN_COUNT masks the IN-mapped pins above the count to zero, which is what makes MOV X, "
                  "PINS usable: at IN_BASE 13 with IN_COUNT 1 the whole word is GP13's level and nothing else, and "
                  "with IN_COUNT 3 it is three pads and no more - GP13 in bit 0, GP15 in bit 2 through the wire, "
                  "every bit above 2 zero",
                  masked_high && *masked_high == 1u && masked_low && *masked_low == 0u && three_high &&
                      (*three_high & 0x5u) == 0x5u && *three_high <= 7u);

    // MOV PINDIRS: one instruction turns every OUT-mapped pin around.
    pio_fresh();
    (void)Pin<13>::function(PinFunction::pio0);
    PioSmConfig d{};
    d.out_base = 13;
    d.out_count = 1;
    (void)Probe::configure(d);
    const uint32_t oe_before = P0::pad_oe();
    (void)Probe::exec_wait(pio_mov(PioMovTo::pindirs, PioMovFrom::null, PioMovOp::invert));
    const uint32_t oe_out = P0::pad_oe();
    (void)Probe::exec_wait(pio_mov(PioMovTo::pindirs, PioMovFrom::null));
    const uint32_t oe_in = P0::pad_oe();
    print(serial, "  DBG_PADOE before ", hex(oe_before), ", after `mov pindirs, ~null` ", hex(oe_out),
          ", after `mov pindirs, null` ", hex(oe_in), crlf);
    bench.verdict("MOV PINDIRS is this chip's own destination (reserved on the RP2040): `mov pindirs, ~null` makes "
                  "every OUT-mapped pin an output and `mov pindirs, null` an input again, in one instruction and "
                  "without a SET's five-pin limit",
                  (oe_before & (1UL << 13)) == 0u && (oe_out & (1UL << 13)) != 0u && (oe_in & (1UL << 13)) == 0u);

    // WAIT on the branch pin: the machine holds until GP15 goes high.
    pio_fresh();
    (void)Pin<13>::output(false);
    (void)Pin<15>::function(PinFunction::pio0);
    spin_us(20);
    const auto at = P0::add(jmppin_wait_program);
    PioSmConfig w{};
    w.clock = PioClockDiv{1, 0};
    w.jmp_pin = 15;
    w.in_base = 15;
    const bool up = at && Probe::init(jmppin_wait_program, *at, w);
    Probe::enable(true);
    spin_us(100);
    const bool waiting = Probe::rx_empty();
    Pin<13>::set();
    spin_us(100);
    const auto seven = Probe::pop_wait(10'000);
    Probe::enable(false);
    if (at) {
        P0::unload(*at, jmppin_wait_program.length);
    }
    (void)Pin<13>::release();
    (void)Pin<15>::release();
    print(serial, "  WAIT 1 JMPPIN: held while GP15 was low=", waiting, ", then pushed ", seven ? *seven : 0u, crlf);
    bench.verdict("WAIT can take the machine's OWN branch pin as its source (11.4.3's fourth source, this chip's): "
                  "the machine holds while GP15 is low and goes on the moment the wire pulls it high",
                  up && waiting && seven && *seven == 7u);
}

void banner() {
    print(serial, crlf,
          "test_rp2350_pio - the RP2350 PIO (datasheet chapter 11), three blocks: GP13 -> GP15 and GP17 -> GP9; "
          "clk_sys=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_pio0_0() {
    line0_entries = line0_entries + 1u;
    const uint32_t up = P0::isr(0);
    line0_last = up;
    if (rx_to_buffer) {
        while (Rx::readable()) {
            const uint8_t b = Rx::read();
            if (isr_rx_count < 64u) {
                isr_rx[isr_rx_count] = b;
            }
            isr_rx_count = isr_rx_count + 1u;
        }
    }
}
extern "C" void isr_pio0_1() {
    line1_entries = line1_entries + 1u;
    line1_last = P0::isr(1);
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool mtime_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output(false);
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless", ta_block);
    bench.letter('b', "the GPIO window", tb_window);
    bench.letter('c', "the square wave, counted on the wire", tc_square_wave);
    bench.letter('d', "the serial port at three rates, the frame error", td_serial);
    bench.letter('e', "the PIO PWM as a PwmChannel", te_pwm);
    bench.letter('f', "the interrupts and all eight flags", tf_interrupts);
    bench.letter('g', "the sampler at clk_sys", tg_sampler);
    bench.letter('h', "the three blocks: the ring and the neighbour masks", th_blocks);
    bench.letter('j', "the receive FIFO as four registers", tj_putget);
    bench.letter('k', "the masked input and the new pin forms", tk_pins);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED", " mtime=",
                    mtime_ok ? "1us" : "FAILED", " tick=", tick_ok ? "on" : "FAILED", brio::crlf);
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
        bench.prompt();
    }
}
