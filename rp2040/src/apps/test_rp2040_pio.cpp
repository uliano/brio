// test_rp2040_pio - the reference bench suite for the RP2040's PIO
// (rp2040/pio.hpp over datasheet chapter 3): the assembler against the
// vendor's assembled words, a block and a machine wireless, and the
// chapter's own programs on the standing wires - a square wave and a
// PWM measured by the PWM block's counters, a serial port sent and
// received by two machines with and without the DMA, the interrupt
// lines and the flags, a sampler as a logic analyser.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// TWO WIRES BETWEEN THE CHIP'S OWN PADS, which this desk does NOT
// carry: the board is wired to its desk partner (docs/boards/
// weact-rp2040.md) and every self-link is retired, so letters b..g run
// BY NAME ONLY, outside z, and decline with that reason when the wire is
// missing. Fitted, the wires are
//
//   GP13  ->  GP15   (the serial port: PIO0 machine 0 sends, machine 1
//                     receives; the square wave and the sampled PWM)
//   GP17  ->  GP9    (the PIO PWM into the PWM block's level counter)
//
// - no pad the pair uses.
//
// THE RULERS: the system timer, the PWM block's edge and level
// counters (rp2040/pwm.hpp) on the far end of each wire.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the encodings against the vendor's words,
//      DBG_CFGINFO, the reset state, a program loaded and run with its
//      words popped (SET, MOV with invert and reverse, a JMP countdown
//      that leaves X at all ones), an instruction executed on the
//      side, the FIFO flags and the join, a flag raised and seen on a
//      line
//   b  THE SQUARE WAVE on GP13 counted by edges on GP15 at 31.25 MHz,
//      1 MHz, 100 kHz and 10 kHz
//   c  THE SERIAL PORT: 64 bytes at 115200, 1 Mbaud and 3 Mbaud
//      byte-exact, the frame error flagged on a held line and the
//      receiver back after it
//   d  THE SERIAL PORT ON THE DMA: 256 bytes poured into the transmit
//      FIFO and collected from the top byte of the receive FIFO
//   e  THE PIO PWM as a PwmChannel, its duty counted by level on GP9
//   f  THE INTERRUPTS: the receive FIFO on line 0 taking 32 bytes, a
//      flag raised by a program on line 1, IRQ WAIT stalling a machine
//      until the system clears the flag
//   g  THE SAMPLER: a machine shifting one pin into words at 1 MHz
//      while the PWM block drives it at 25 %, 8192 samples counted
//   h  WIRELESS, the corners: IRQ RELATIVE raised by machines 1..3
//      (flag 0 + n, and 1 + 3 wrapping to 0 - the modulo-4 sum) and
//      WAIT IRQ RELATIVE released by its own flag alone; MOV X, STATUS
//      branched on (a machine that pulls only while its TX level is
//      at or past two, the words it takes counted); THE 65536
//      DIVIDER (PioClockDiv{0, 0}) on machine 3, its 476.8 Hz wave
//      counted on its own pad's input; and the serial port on PIO1's
//      machines 2 and 3 sharing ONE pad - the receiver reads the
//      transmitter's output through the pad's input, no wire
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/pio.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/pwm.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

TestBench<Serial, 16> bench;

using P0 = Pio<0>;
using P1 = Pio<1>;
using Tx = PioUartTx<0, 0, 13>;
using Rx = PioUartRx<0, 1, 15>;
using Wave = PioSquareWave<0, 2, 13>;
using Pwm17 = PioPwm<1, 0, 17, 999>;
using Sampler = PioSm<0, 2>;
using Edges15 = PwmEdgeCounter<15>;
using Level9 = PwmLevelCounter<9>;
using TxEngine = DmaTxEngine<9, uint8_t>;
using RxEngine = DmaRxEngine<10, uint8_t>;
using WordEngine = DmaRxEngine<10, uint32_t>;

volatile uint32_t line0_entries = 0;
volatile uint32_t line1_entries = 0;
volatile uint32_t line0_last = 0;
volatile uint32_t line1_last = 0;
volatile uint32_t flags_seen = 0;
volatile bool rx_to_buffer = false;
uint8_t isr_rx[64];
volatile uint8_t isr_rx_count = 0;
volatile bool dma_tx_done = false;
volatile bool dma_rx_done = false;

uint8_t tx_buf[256];
uint8_t rx_buf[256];
uint32_t words[256];

uint32_t us_now() { return Timer::now_low(); }
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
template <uint8_t out_pin, uint8_t in_pin>
bool wire_present() {
    Pin<in_pin>::input(PinPull::none);
    Pin<out_pin>::output(false);
    spin_us(20);
    const bool low = !Pin<in_pin>::read();
    Pin<out_pin>::set();
    spin_us(20);
    const bool high = Pin<in_pin>::read();
    Pin<out_pin>::release();
    Pin<in_pin>::release();
    return low && high;
}
bool wire_13_15() {
    if (wire_present<13, 15>()) {
        return true;
    }
    bench.verdict("DECLINED: GP13 -> GP15 is not wired - the pair desk carries no self-link (fit the jumper to run this letter)", false);
    return false;
}
bool wire_17_9() {
    if (wire_present<17, 9>()) {
        return true;
    }
    bench.verdict("DECLINED: GP17 -> GP9 is not wired - the pair desk carries no self-link (fit the jumper to run this letter)", false);
    return false;
}

void pio_fresh() {
    Nvic::disable(P0::irq(0));
    Nvic::disable(P0::irq(1));
    P0::interrupts_only(0, 0);
    P0::interrupts_only(1, 0);
    (void)P0::reset();
    (void)P1::reset();
    rx_to_buffer = false;
    line0_entries = 0;
    line1_entries = 0;
    flags_seen = 0;
}

// =============================================================================
// a - the block, wireless
// =============================================================================
constexpr PioProgram<13> probe_program = [] {
    PioProgram<13> p{};
    p.code = {
        pio_set(PioSetTo::x, 5),                            // 0
        pio_mov(PioMovTo::isr, PioMovFrom::x),              // 1
        pio_push(),                                         // 2: 5
        pio_set(PioSetTo::y, 3),                            // 3
        pio_mov(PioMovTo::isr, PioMovFrom::y, PioMovOp::invert),   // 4
        pio_push(),                                         // 5: ~3
        pio_mov(PioMovTo::isr, PioMovFrom::y, PioMovOp::reverse),  // 6
        pio_push(),                                         // 7: 3 reversed
        pio_set(PioSetTo::x, 4),                            // 8
        pio_jmp(PioJmp::x_dec, 9),                          // 9: the countdown, on itself
        pio_mov(PioMovTo::isr, PioMovFrom::x),              // 10
        pio_push(),                                         // 11: what X ended at
        pio_jmp(PioJmp::always, 12),                        // 12: parked
    };
    return p;
}();
constexpr PioProgram<2> push_x_program = [] {
    PioProgram<2> p{};
    p.code = {pio_mov(PioMovTo::isr, PioMovFrom::x), pio_push()};
    return p;
}();

void ta_block() {
    bench.verdict("the encodings against the vendor's assembled words: the transmitter's four (0x9FA0 0xF727 0x6001 "
                  "0x0642), the receiver's nine, `set pins, 1 [1]` 0xE101, a relocated JMP",
                  pio_uart_tx_program.code[0] == 0x9FA0u && pio_uart_tx_program.code[1] == 0xF727u &&
                      pio_uart_tx_program.code[3] == 0x0642u && pio_uart_rx_program.code[0] == 0x2020u &&
                      pio_uart_rx_program.code[1] == 0xEA27u && pio_uart_rx_program.code[6] == 0x20A0u &&
                      pio_uart_rx_program.code[8] == 0x8020u && pio_delay(pio_set(PioSetTo::pins, 1), 1) == 0xE101u &&
                      pio_relocate(pio_jmp(PioJmp::x_dec, 2), 10) == 0x004Cu);
    pio_fresh();
    print(serial, "  PIO0 as built: ", P0::memory_size(), " instructions, ", P0::machine_count(), " machines, FIFOs ", P0::fifo_depth(),
          " deep; CTRL=", hex(P0::reg(PIO_CTRL_OFFSET)), " FSTAT=", hex(P0::fifo_status()), crlf);
    bench.verdict("DBG_CFGINFO says 32 instructions, 4 machines, FIFOs 4 deep; the reset state: no machine enabled, "
                  "every FIFO empty",
                  P0::memory_size() == 32u && P0::machine_count() == 4u && P0::fifo_depth() == 4u && P0::enabled() == 0u &&
                      P0::fifo_status() == 0x0F000F00u);
    // A program run: SET, MOV with its two operations, the countdown.
    using S = PioSm<0, 0>;
    const auto at = P0::add(probe_program);
    bool ok = at && S::init(probe_program, *at, {.clock = {1, 0}});
    S::enable(true);
    spin_us(50);
    S::enable(false);
    const uint8_t level = S::rx_level();
    const uint32_t w0 = level >= 4u ? S::pop() : 0xDEADu;
    const uint32_t w1 = level >= 4u ? S::pop() : 0xDEADu;
    const uint32_t w2 = level >= 4u ? S::pop() : 0xDEADu;
    const uint32_t w3 = level >= 4u ? S::pop() : 0xDEADu;
    print(serial, "  the probe program at offset ", at ? *at : 99u, ": ", level, " words, ", hex(w0), " ", hex(w1), " ", hex(w2), " ",
          hex(w3), ", parked at ", S::address(), crlf);
    bench.verdict("a program loaded and run: SET X 5 pushed, Y 3 inverted (0xFFFFFFFC) and bit-reversed (0xC0000000), "
                  "the machine parked on a JMP to itself",
                  ok && level == 4u && w0 == 5u && w1 == 0xFFFFFFFCu && w2 == 0xC0000000u && S::address() == *at + 12u);
    bench.verdict("JMP X-- run to its end leaves X at all ones: the decrement is unconditional", w3 == 0xFFFFFFFFu);
    (void)S::exec_wait(pio_set(PioSetTo::x, 9));
    (void)S::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)S::exec_wait(pio_push());
    const auto nine = S::pop_wait(1000);
    bench.verdict("SET X 9 executed on the side and pushed: 9", nine && *nine == 9u);
    // The FIFO flags.
    S::drain_rx();
    S::clear_fifo_debug();
    for (uint8_t i = 0; i < 4u; ++i) {
        S::push(i);
    }
    const bool full = S::tx_full() && S::tx_level() == 4u;
    S::push(4);   // the fifth
    const bool over = S::tx_overflowed();
    (void)S::pop();
    const bool under = S::rx_underflowed();
    S::drain_tx();
    S::clear_fifo_debug();
    bench.verdict("the transmit FIFO is full at four and a fifth push sets TXOVER; a pop of the empty receive FIFO sets RXUNDER",
                  full && over && under && S::tx_empty());
    // The join: eight words.
    (void)S::configure({.fifo_join = PioFifoJoin::tx});
    for (uint8_t i = 0; i < 8u; ++i) {
        S::push(i);
    }
    const bool eight = S::tx_full() && S::tx_level() == 8u;
    S::drain_tx();
    (void)S::configure({});
    bench.verdict("joined, the transmit FIFO takes eight", eight);
    // A flag raised by the system, seen on line 0 and cleared by the ISR.
    P0::clear_flags(0xFF);
    P0::interrupts_only(0, PioInterrupt::flag(0));
    Nvic::enable(P0::irq(0));
    P0::raise_flags(0x01);
    spin_us(20);
    Nvic::disable(P0::irq(0));
    P0::interrupts_only(0, 0);
    bench.verdict("flag 0 raised through IRQ_FORCE reaches line 0, the ISR body clears it, one entry",
                  line0_entries == 1u && (line0_last & PioInterrupt::flag(0)) != 0u && !P0::flag(0));
    P0::unload(*at, probe_program.length);
}

// =============================================================================
// h - the corners, wireless
// =============================================================================
constexpr PioProgram<2> irq_rel0_program = [] {
    PioProgram<2> p{};
    p.code = {pio_irq(0, false, false, true), pio_jmp(PioJmp::always, 1)};   // irq 0 rel; parked
    return p;
}();
constexpr PioProgram<2> irq_rel1_program = [] {
    PioProgram<2> p{};
    p.code = {pio_irq(1, false, false, true), pio_jmp(PioJmp::always, 1)};   // irq 1 rel; parked
    return p;
}();
constexpr PioProgram<5> wait_rel_program = [] {
    PioProgram<5> p{};
    p.code = {
        pio_wait_irq(true, 0, true),             // wait 1 irq 0 rel: this machine's own flag, cleared on the way out
        pio_set(PioSetTo::x, 21),
        pio_mov(PioMovTo::isr, PioMovFrom::x),
        pio_push(),
        pio_jmp(PioJmp::always, 4),               // parked
    };
    return p;
}();
/// Pulls only while STATUS says the TX level is at or past N (STATUS is
/// all ones below N, zero at or above): `mov x, status; jmp !x take;
/// jmp top; take: pull; mov isr, osr; push; jmp top`.
constexpr PioProgram<7> status_program = [] {
    PioProgram<7> p{};
    p.code = {
        pio_mov(PioMovTo::x, PioMovFrom::status),   // 0
        pio_jmp(PioJmp::x_zero, 3),                 // 1: level >= N
        pio_jmp(PioJmp::always, 0),                 // 2
        pio_pull(false, true),                      // 3
        pio_mov(PioMovTo::isr, PioMovFrom::osr),    // 4
        pio_push(),                                 // 5
        pio_jmp(PioJmp::always, 0),                 // 6
    };
    return p;
}();

/// Rising edges of pin `pin`'s pad input over `us`, the loop polling it.
template <uint8_t pin>
uint32_t pad_rises(uint32_t us) {
    uint32_t rises = 0;
    bool last = Pin<pin>::read();
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
        const bool now = Pin<pin>::read();
        if (now && !last) {
            ++rises;
        }
        last = now;
    }
    return rises;
}

void th_corners() {
    pio_fresh();
    // IRQ relative: machine n raises flag (index + n) mod 4 (3.4.10's
    // "modulo-4 addition on the two LSBs").
    const auto at0 = P0::add(irq_rel0_program);
    const auto at1 = P0::add(irq_rel1_program);
    uint8_t raised[4] = {};
    for (uint8_t m = 1; m < 4u; ++m) {
        P0::clear_flags(0xFF);
        bool ok = false;
        if (m == 1u) { ok = at0 && PioSm<0, 1>::init(irq_rel0_program, *at0, {}); PioSm<0, 1>::enable(true); }
        if (m == 2u) { ok = at0 && PioSm<0, 2>::init(irq_rel0_program, *at0, {}); PioSm<0, 2>::enable(true); }
        if (m == 3u) { ok = at0 && PioSm<0, 3>::init(irq_rel0_program, *at0, {}); PioSm<0, 3>::enable(true); }
        spin_us(10);
        P0::disable(0x0F);
        raised[m] = ok ? P0::flags() : 0xEEu;
    }
    P0::clear_flags(0xFF);
    const bool ok3 = at1 && PioSm<0, 3>::init(irq_rel1_program, *at1, {});
    PioSm<0, 3>::enable(true);
    spin_us(10);
    P0::disable(0x0F);
    const uint8_t wrapped = ok3 ? P0::flags() : 0xEEu;
    P0::clear_flags(0xFF);
    print(serial, "  IRQ 0 REL raised by machines 1, 2, 3: flags ", hex(raised[1]), " ", hex(raised[2]), " ", hex(raised[3]),
          "; IRQ 1 REL by machine 3: ", hex(wrapped), crlf);
    bench.verdict("IRQ relative from machines 1..3: machine n raises flag n alone, and index 1 from machine 3 wraps to flag 0",
                  raised[1] == 0x02u && raised[2] == 0x04u && raised[3] == 0x08u && wrapped == 0x01u);
    // WAIT IRQ relative on machine 2: flags 0, 1 and 3 do not release
    // it; flag 2 does, and the wait clears it.
    using W = PioSm<0, 2>;
    P0::unload(*at0, irq_rel0_program.length);
    P0::unload(*at1, irq_rel1_program.length);
    const auto atw = P0::add(wait_rel_program);
    const bool okw = atw && W::init(wait_rel_program, *atw, {});
    W::enable(true);
    spin_us(10);
    P0::raise_flags(0x0B);   // 0, 1 and 3
    spin_us(10);
    const uint8_t held = W::rx_level();
    P0::clear_flags(0x0B);
    P0::raise_flags(0x04);   // 2: this machine's own
    spin_us(10);
    const uint8_t freed = W::rx_level();
    const uint32_t word = freed != 0u ? W::pop() : 0u;
    const bool cleared = !P0::flag(2);
    W::enable(false);
    P0::unload(*atw, wait_rel_program.length);
    print(serial, "  WAIT 1 IRQ 0 REL on machine 2: after flags 0, 1, 3 ", held, " words; after flag 2 ", freed, " (", word,
          "), flag 2 cleared ", cleared, crlf);
    bench.verdict("WAIT IRQ relative on machine 2 waits for flag 2 alone, and clears it on the way out",
                  okw && held == 0u && freed == 1u && word == 21u && cleared);

    // MOV X, STATUS branched on: TX level >= 2 to pull. Four words (the
    // FIFO's depth) pushed with the machine stopped; it takes them while
    // the level is 4, 3 and 2, then stops at 1 - three out, one left.
    using St = PioSm<0, 1>;
    const auto ats = P0::add(status_program);
    const bool oks = ats && St::init(status_program, *ats, {.status_select = PioStatus::tx_level, .status_n = 2});
    for (uint32_t i = 1; i <= 4u; ++i) {
        St::push(i);
    }
    St::enable(true);
    spin_us(20);
    St::enable(false);
    const uint8_t taken = St::rx_level();
    const uint8_t left = St::tx_level();
    uint32_t w[3] = {};
    for (uint8_t i = 0; i < 3u && !St::rx_empty(); ++i) {
        w[i] = St::pop();
    }
    St::drain_tx();
    // The other source: STATUS on the RX level, executed on the side.
    (void)St::configure({.status_select = PioStatus::rx_level, .status_n = 2});
    St::drain_rx();
    uint32_t st[3] = {};
    for (uint8_t i = 0; i < 3u; ++i) {
        (void)St::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::status));
        (void)St::exec_wait(pio_push());
    }
    for (uint8_t i = 0; i < 3u; ++i) {
        st[i] = St::pop();
    }
    P0::unload(*ats, status_program.length);
    print(serial, "  MOV X, STATUS on TX level >= 2: ", taken, " words taken (", w[0], " ", w[1], " ", w[2], "), ", left,
          " left; on RX level < 2 at levels 0, 1, 2: ", hex(st[0]), " ", hex(st[1]), " ", hex(st[2]), crlf);
    bench.verdict("STATUS as a branch: a machine pulling while its TX level is at or past two takes 1, 2, 3 of four and "
                  "leaves one; on the RX level it reads all ones below N and zero at N",
                  oks && taken == 3u && left == 1u && w[0] == 1u && w[1] == 2u && w[2] == 3u && st[0] == 0xFFFFFFFFu &&
                      st[1] == 0xFFFFFFFFu && st[2] == 0u);

    // The 65536 divider: the square wave's four cycles a period at
    // 125 MHz / 65536 - 476.84 Hz, so 476 or 477 rising edges in one
    // second (the machine and the timer share the crystal; the window's
    // phase decides between the two).
    using Sq = PioSm<0, 3>;
    const auto atq = P0::add(pio_square_wave_program);
    Sq::pin_directions(12, 1, true);
    Pin<12>::function(P0::pin_function);
    const bool okq = atq && Sq::init(pio_square_wave_program, *atq, {.clock = {0, 0}, .set_base = 12, .set_count = 1});
    Sq::enable(true);
    const uint32_t rises = pad_rises<12>(1'000'000);
    const uint32_t div_reg = Sq::clkdiv();
    Sq::enable(false);
    P0::unload(*atq, pio_square_wave_program.length);
    Pin<12>::release();
    print(serial, "  CLKDIV ", hex(div_reg), " (PioClockDiv{0, 0}) on machine 3: ", rises, " rising edges in 1 s on GP12's own input",
          crlf);
    bench.verdict("the 65536 divider: CLKDIV's integer 0 divides by 65536, the four-cycle wave at 476.8 Hz (476..477 edges in 1 s)",
                  okq && div_reg == 0u && rises >= 476u && rises <= 477u);

    // The serial port on PIO1, machines 2 and 3, on ONE pad: the
    // receiver set up first (the pin an input), the transmitter after
    // (the pin an output, its level high) - the receiver reads the pad.
    using LTx = PioUartTx<1, 2, 14>;
    using LRx = PioUartRx<1, 3, 14>;
    uint16_t good[2] = {};
    constexpr uint32_t rates[2] = {115200, 1'000'000};
    bool inits = true;
    for (uint8_t r = 0; r < 2u; ++r) {
        inits = LRx::init(clock, rates[r]) && inits;
        inits = LTx::init(clock, rates[r]) && inits;
        spin_us(100);
        fill_pattern(tx_buf, 64, static_cast<uint8_t>(0x41u + r));
        for (uint16_t i = 0; i < 64u; ++i) {
            (void)LTx::write_wait(tx_buf[i]);
            const auto b = LRx::read_wait(200'000u);
            if (b && *b == tx_buf[i]) {
                ++good[r];
            }
        }
        const bool fe = LRx::frame_error();
        if (fe) {
            good[r] = 0;
        }
        LTx::release();
        LRx::release();
    }
    print(serial, "  PIO1 machine 2 sending and machine 3 receiving on GP14 alone: ", good[0], "/64 at 115200, ", good[1],
          "/64 at 1 Mbaud", crlf);
    bench.verdict("the serial tasks on PIO1's machines 2 and 3, one pad: 64 bytes exact at 115200 and at 1 Mbaud, no frame error",
                  inits && good[0] == 64u && good[1] == 64u);
    pio_fresh();
}

// =============================================================================
// b - the square wave
// =============================================================================
void tb_square_wave() {
    if (!wire_13_15()) {
        return;
    }
    pio_fresh();
    const uint32_t rates[] = {31'250'000, 1'000'000, 100'000, 10'000};
    uint8_t ok = 0;
    for (uint32_t hz : rates) {
        const bool up = Wave::init(clock, hz);
        (void)Edges15::setup();
        const uint32_t window = hz >= 1'000'000u ? 2000u : (hz >= 100'000u ? 20'000u : 200'000u);
        Edges15::restart();
        Edges15::run(true);
        spin_us(window);
        Edges15::run(false);
        const uint32_t got = static_cast<uint32_t>(static_cast<uint64_t>(Edges15::count()) * 1'000'000u / window);
        Edges15::release();
        Wave::release();
        const bool good = up && within(got, hz, 10);
        print(serial, "  ", hz, " Hz asked (divider ", pio_clock_div_for(SysClock::hz, hz * 4u)->integer, " + ",
              pio_clock_div_for(SysClock::hz, hz * 4u)->frac, "/256): ", got, " Hz counted on GP15", good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
    }
    bench.verdict("the four-cycle square wave at 31.25 MHz (the machine at clk_sys), 1 MHz, 100 kHz and 10 kHz, each "
                  "within one per cent by edges on the wire",
                  ok == 4u);
}

// =============================================================================
// c - the serial port
// =============================================================================
void tc_serial() {
    if (!wire_13_15()) {
        return;
    }
    pio_fresh();
    const uint32_t bauds[] = {115'200, 1'000'000, 3'000'000};
    uint8_t ok = 0;
    for (uint32_t baud : bauds) {
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
        print(serial, "  ", baud, " baud: 64 bytes ", exact ? "exact" : "WRONG", " in ", took, " us (", want_us, " on the wire), frame error=",
              frame, crlf);
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
    print(serial, "  a 200 us break: frame error=", flagged, ", nothing received=", nothing, ", the next byte ", after ? hex(*after) : hex(0xFFu),
          crlf);
    bench.verdict("a line held low past a frame raises the receiver's frame flag and delivers nothing; the receiver "
                  "takes the next byte once the line is idle again",
                  flagged && nothing && after && *after == 0x5Au);
    Tx::release();
    Rx::release();
}

// =============================================================================
// d - the serial port on the DMA
// =============================================================================
void td_dma() {
    if (!wire_13_15()) {
        return;
    }
    pio_fresh();
    (void)Tx::init(clock, 1'000'000);
    (void)Rx::init(clock, 1'000'000);
    fill_pattern(tx_buf, 256, 0x40);
    for (uint16_t i = 0; i < 256; ++i) {
        rx_buf[i] = 0xEE;
    }
    dma_tx_done = false;
    dma_rx_done = false;
    RxEngine::arm(Rx::rx_top_byte_address(), Rx::Sm::dreq_rx);
    TxEngine::arm(Tx::tx_address(), Tx::Sm::dreq_tx);
    (void)RxEngine::start(rx_buf, 256);
    const uint32_t t0 = us_now();
    (void)TxEngine::start(tx_buf, 256);
    while (!dma_rx_done && us_now() - t0 < 100'000u) {
    }
    const uint32_t took = us_now() - t0;
    print(serial, "  256 bytes at 1 Mbaud through two engines: tx ", dma_tx_done ? "done" : "NOT done", ", rx ",
          dma_rx_done ? "done" : "NOT done", " in ", took, " us (2560 on the wire), ", same(tx_buf, rx_buf, 256) ? "byte-exact" : "MISMATCH",
          crlf);
    bench.verdict("256 bytes poured into the transmit FIFO by a byte engine and collected from the receive FIFO's top "
                  "byte by another, byte-exact, in the wire's time",
                  dma_tx_done && dma_rx_done && same(tx_buf, rx_buf, 256) && within(took, 2560, 100));
    TxEngine::stop();
    RxEngine::stop();
    Tx::release();
    Rx::release();
}

// =============================================================================
// e - the PIO PWM
// =============================================================================
void te_pwm() {
    if (!wire_17_9()) {
        return;
    }
    pio_fresh();
    const bool up = Pwm17::init({1, 0});
    (void)Level9::setup({0, 0});   // 256: 48828 counts full over 100 ms
    const uint16_t levels[] = {250, 500, 750};
    uint8_t ok = 0;
    for (uint16_t lv : levels) {
        Pwm17::duty(lv);
        spin_us(500);
        Level9::restart();
        Level9::run(true);
        spin_us(100'000);
        Level9::run(false);
        const uint32_t pm = static_cast<uint32_t>(static_cast<uint64_t>(Level9::count()) * 1000u / 48828u);
        const bool good = within(pm, lv, 20);
        print(serial, "  level ", lv, " of 999: ", pm, " per mille high on GP9", good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
    }
    bench.verdict("the PIO PWM (3.6.8) as a PwmChannel: levels 250, 500 and 750 of 999 read 25, 50 and 75 % by level on "
                  "the wire, within two per cent (a pulse of 3000 machine cycles)",
                  up && ok == 3u);
    Level9::release();
    Pwm17::release();
}

// =============================================================================
// f - the interrupts and the flags
// =============================================================================
constexpr PioProgram<4> flag_program = [] {
    PioProgram<4> p{};
    p.code = {
        pio_irq(0, false, true),                     // irq 0 wait: stalls until the system clears it
        pio_set(PioSetTo::x, 1),
        pio_mov(PioMovTo::isr, PioMovFrom::x),
        pio_push(),
    };
    p.wrap_top = 3;
    return p;
}();
constexpr PioProgram<2> flag1_program = [] {
    PioProgram<2> p{};
    p.code = {pio_irq(1, false, false), pio_jmp(PioJmp::always, 1)};   // irq 1, then spin
    return p;
}();

void tf_interrupts() {
    if (!wire_13_15()) {
        return;
    }
    pio_fresh();
    // The receive FIFO on line 0: 32 bytes taken by the ISR.
    (void)Tx::init(clock, 1'000'000);
    (void)Rx::init(clock, 1'000'000);
    isr_rx_count = 0;
    rx_to_buffer = true;
    P0::interrupts_only(0, Rx::Sm::interrupt_rx_not_empty);
    Nvic::enable(P0::irq(0));
    fill_pattern(tx_buf, 32, 0x70);
    for (uint8_t i = 0; i < 32u; ++i) {
        (void)Tx::write_wait(tx_buf[i]);
    }
    spin_us(2000);
    Nvic::disable(P0::irq(0));
    P0::interrupts_only(0, 0);
    rx_to_buffer = false;
    print(serial, "  the receive FIFO on line 0: ", isr_rx_count, " bytes taken by the ISR in ", line0_entries, " entries", crlf);
    bench.verdict("the receive FIFO's not-empty source on line 0: 32 bytes taken by the ISR, byte-exact",
                  isr_rx_count == 32u && same(tx_buf, isr_rx, 32) && line0_entries >= 1u);
    Tx::release();
    Rx::release();
    // A flag raised by a program on line 1.
    using S3 = PioSm<0, 3>;
    const auto at1 = P0::add(flag1_program);
    (void)S3::init(flag1_program, *at1, {});
    P0::clear_flags(0xFF);
    line1_entries = 0;
    P0::interrupts_only(1, PioInterrupt::flag(1));
    Nvic::enable(P0::irq(1));
    S3::enable(true);
    spin_us(50);
    S3::enable(false);
    Nvic::disable(P0::irq(1));
    P0::interrupts_only(1, 0);
    bench.verdict("a program's IRQ 1 reaches line 1 (IRQ1_INTE), the ISR body clears the flag, one entry",
                  line1_entries == 1u && (line1_last & PioInterrupt::flag(1)) != 0u && !P0::flag(1));
    P0::unload(*at1, flag1_program.length);
    // IRQ WAIT: the machine stalls until the system clears the flag.
    using S2 = PioSm<0, 2>;
    const auto at0 = P0::add(flag_program);
    (void)S2::init(flag_program, *at0, {});
    P0::clear_flags(0xFF);
    S2::enable(true);
    spin_us(100);
    const bool flag_up = P0::flag(0);
    const bool nothing = S2::rx_empty();
    const uint8_t held_at = S2::address();
    // The IRQ executes, then the wait holds the machine: ADDR already
    // reads the following instruction (measured).
    const bool stalled = flag_up && nothing && held_at == *at0 + 1u;
    P0::clear_flags(0x01);
    spin_us(100);
    const bool went = !S2::rx_empty() && S2::pop() == 1u;
    S2::enable(false);
    print(serial, "  IRQ 0 WAIT at ", *at0, ": flag up=", flag_up, ", nothing pushed=", nothing, ", the machine at ", held_at,
          "; cleared by the system: the word arrived=", went, crlf);
    bench.verdict("IRQ WAIT raises the flag and holds the machine - ADDR reading the instruction after - until the "
                  "system clears the flag, then the program goes on and pushes",
                  stalled && went);
    P0::unload(*at0, flag_program.length);
}

// =============================================================================
// g - the sampler
// =============================================================================
constexpr PioProgram<1> sample_program = [] {
    PioProgram<1> p{};
    p.code = {pio_in(PioIn::pins, 1)};
    return p;
}();

void tg_sampler() {
    if (!wire_13_15()) {
        return;
    }
    pio_fresh();
    (void)PwmOutput<13, 999>::setup_hz(clock, 10'000);
    PwmOutput<13, 999>::duty(250);
    Pin<15>::function(PinFunction::pio0);
    const auto at = P0::add(sample_program);
    PioSmConfig c{};
    c.clock = *pio_clock_div_for(SysClock::hz, 1'000'000);
    c.in_base = 15;
    c.in_shift_right = false;
    c.autopush = true;
    c.push_threshold = 32;
    c.fifo_join = PioFifoJoin::rx;
    const bool up = at && Sampler::init(sample_program, *at, c);
    for (uint16_t i = 0; i < 256u; ++i) {
        words[i] = 0;
    }
    dma_rx_done = false;
    WordEngine::arm(Sampler::rx_address(), Sampler::dreq_rx);
    (void)WordEngine::start(words, 256);
    const uint32_t t0 = us_now();
    Sampler::enable(true);
    while (!dma_rx_done && us_now() - t0 < 100'000u) {
    }
    const uint32_t took = us_now() - t0;
    Sampler::enable(false);
    uint32_t ones = 0;
    for (uint16_t i = 0; i < 256u; ++i) {
        ones += static_cast<uint32_t>(__builtin_popcount(words[i]));
    }
    const uint32_t pm = ones * 1000u / 8192u;
    print(serial, "  256 words of 32 samples at 1 MHz: ", dma_rx_done ? "collected" : "NOT collected", " in ", took, " us, ", ones,
          " ones of 8192 = ", pm, " per mille (the PWM at 25 %); the first words ", hex(words[0]), " ", hex(words[1]), " ", hex(words[2]),
          crlf);
    bench.verdict("a one-instruction sampler with autopush: 8192 samples of a 25 % wave at 1 MHz collected by the DMA "
                  "in 8 ms, the ones within one per cent of a quarter",
                  up && dma_rx_done && within(took, 8192, 50) && within(pm, 250, 10));
    WordEngine::stop();
    P0::unload(*at, sample_program.length);
    Pin<15>::release();
    PwmOutput<13, 999>::release();
    Pwm::stop(0xFF);
}

void banner() {
    print(serial, crlf, "test_rp2040_pio - the RP2040 PIO (datasheet chapter 3): GP13 -> GP15 and GP17 -> GP9; clk_sys=",
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
extern "C" void isr_dma_0() {
    const uint8_t t = TxEngine::service();
    if ((t & TxEngine::flag_complete) != 0u) {
        (void)TxEngine::complete();
        dma_tx_done = true;
    }
    const uint8_t r = RxEngine::service();   // channel 10, the byte or the word engine
    if ((r & RxEngine::flag_complete) != 0u) {
        dma_rx_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool dma_ok = brio::Dma::init();
    const bool pwm_ok = brio::Pwm::reset();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless", ta_block);
    bench.letter('b', "the square wave by edges", tb_square_wave, false);
    bench.letter('c', "the serial port at three rates, the frame error", tc_serial, false);
    bench.letter('d', "the serial port on the DMA", td_dma, false);
    bench.letter('e', "the PIO PWM as a PwmChannel", te_pwm, false);
    bench.letter('f', "the interrupts and the flags", tf_interrupts, false);
    bench.letter('g', "the sampler", tg_sampler, false);
    bench.letter('h', "the corners: relative IRQ, STATUS, the 65536 divider, PIO1's machines 2 and 3 (wireless)", th_corners);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " dma=", dma_ok ? "released" : "FAILED", " pwm=", pwm_ok ? "released" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
