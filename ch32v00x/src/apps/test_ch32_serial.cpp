// test_ch32_serial - the reference bench suite for the CH32V00x's USART
// (RM ch. 14) beyond the console personality: the resource
// ch32v00x/usart.hpp's `Usart<n>` and the transport `Uart`'s options,
// on the second instance while the console keeps the first.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS USART2 ON ITS COLUMN 3 (AFIO table 7-11): TX PD2,
// RX PD3, CTS PA0, RTS PA1 - pads the module leaves free - while the
// console stays on USART1 (PD5/PD6, the WCH-Link's own serial). Two
// facts and one jumper make the suite need almost no wire:
//
// 1. THE RX PAD IS A BIT-BANGED TRANSMITTER. A pad handed to the USART
//    as RX is an input, and an input's level follows its own pull -
//    which OUTDR moves in a few cycles (pin.hpp: the pull direction IS
//    the output data bit). At 9600 baud a bit is 5000 cycles, so
//    software paced on the STK puts an ARBITRARY frame on the receive
//    line: every format, a parity that does not add up, a stop bit
//    that is low, a glitch inside a bit, a break of ten or eleven bits,
//    an address mark, a frame at a rate a few per cent off, an RZI
//    frame for the infrared decoder.
// 2. THE JUMPER PD2 TO PD4, the same one the timer and pad suites use,
//    lends TIM2's channel 1 on PD4 as the ruler for what TX puts on
//    PD2: a start bit's width is the baud generator to the cycle, the
//    low run of a 0x00 frame is start + data + parity, the gap between
//    two frames is the stop length, a break is ten or thirteen bits,
//    an IrDA pulse is 3/16 of a bit. The letters that need it decline
//    with the reason when the jumper is missing.
// 3. THERE IS NO LOOP-BACK. Half duplex (14.4) makes the receiver
//    listen on the TX pad, but not to the instance's own frames
//    (measured six ways - the driver header says so): so the single
//    wire is proven as a BUS, a frame driven onto it from PD4's pull
//    arriving through the transport, and the DMA receive engine is fed
//    from the bit-banged line.
//
// What is exercised, letter by letter:
//   a  the instance and the facts: a reset pulse's register values, the
//      RESERVED bits of the register description refusing to stick (no
//      synchronous mode, no smartcard, no guard time on this silicon),
//      the vocabulary's refusals, the modes excluding each other
//   b  every frame format of 14.8.4/14.8.5, both ways: the transmitter's
//      frames measured on the jumper (start + data + parity as one low
//      run, the stop length as the gap between two), the receiver fed
//      each format from the banged line - 7, 8 and 9 data bits, both
//      parities, the four stop lengths; and the single wire as a bus
//   c  the baud generator: a start bit measured to the cycle at eight
//      rates from 2400 to 3 Mbaud (the bit IS the divisor), the
//      fractional divisor, the receiver at 9600, the refusal below 16
//   d  the bit-banged line: a clean frame, a parity error, a framing
//      error, noise, and the receiver's tolerance to a rate off by a few
//      per cent
//   e  LIN: the break SBK sends timed on the jumper (thirteen bits in
//      LIN mode, ten outside), the break detected at ten and eleven bits
//      with its flag and its interrupt
//   f  mute mode: the receiver asleep through frames until the line
//      idles, or until a nine-bit frame carries its own address
//   g  IrDA: the encoder's pulse measured on the jumper in normal and
//      low-power mode, and the decoder fed a bit-banged RZI frame
//   h  hardware flow control: CTS held high stalls the transmitter, RTS
//      rises while a received byte waits and drops when it is read
//   i  the DMA engines on USART2's own channels 6 and 7: the receive
//      engine fed from the banged line, the transmit engine timed
//   j  the flags and the vector: TC after TXE, IDLE once per line idle,
//      one interrupt per enable
//   y  (outside z, host-assisted: brio stress) the CONSOLE's own error
//      counters, provoked from the host side
//   k  THE SYNCHRONOUS MODE, on the console's own USART1 (both parts):
//      on the CH32V003 the CK pad PD4 carries a clock pulse per data bit
//      while the console prints, counted by TIM2's channel 1 off the
//      same pad with NO WIRE - seven pulses a byte, eight with LBCL,
//      the idle level CPOL's; on the CH32V006 the verb refuses (the
//      bits are reserved there)
//
// THE CH32V003 BUILD carries letter k alone (one group image): its
// USART is the console's, there being no second instance to be the
// instrument, so every letter that names USART2 is compiled out there.
//
// build: boards = v006k8,v003f4
// build: groups = k
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

TestBench<Serial, 16> bench;

#if BRIO_CH32_HAS_USART2
// The instrument: USART2 on column 3.
constexpr uint8_t column = 3;
using U2 = Usart<2>;
using TxPad = Pin<'D', 2>;
using RxPad = Pin<'D', 3>;
using CtsPad = Pin<'A', 0>;
using RtsPad = Pin<'A', 1>;
using RulerPad = Pin<'D', 4>;   // TIM2_CH1, on the jumper from PD2
using T2 = Tim<2>;

constexpr UartOptions wire_opts{.half_duplex = true};
using WireUart = Uart<2, P, 64, 64, NoDmaEngine, NoDmaEngine, column, wire_opts>;
using DmaUart = Uart<2, P, 256, 256, DmaTxEngine<6>, DmaRxEngine<7>, column>;
constexpr UartOptions flow_opts{.rts = true, .cts = true};
using FlowUart = Uart<2, P, 64, 64, NoDmaEngine, NoDmaEngine, column, flow_opts>;

// Which transport owns the USART2 vector at the moment: the letters
// hand it over before init() and take it back in their all_off().
using IsrFn = bool (*)();
volatile IsrFn u2_isr = nullptr;
volatile uint32_t u2_interrupts = 0;
volatile uint32_t u2_txe = 0;
volatile uint32_t u2_tc = 0;
volatile uint32_t u2_idle = 0;
volatile uint32_t u2_pe = 0;
volatile uint32_t u2_lbd = 0;
volatile uint32_t u2_cts = 0;
volatile uint16_t u2_last_word = 0;
volatile uint32_t u2_words = 0;
bool jumper_present = false;

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;

// ---- time -----------------------------------------------------------------

/// Cycles since boot from the tick count and the STK's own counter,
/// with the wrap the tick handler has not counted yet folded in
/// (CNTIF read on both sides of CNT; test_ch32_tim's finding).
uint32_t cycles_now() {
    const uint32_t period = stk()->CMP + 1u;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const bool wrapped0 = (stk()->SR & stk_cntif) != 0u;
        const uint32_t cnt = stk()->CNT;
        const bool wrapped1 = (stk()->SR & stk_cntif) != 0u;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1 && wrapped0 == wrapped1) {
            return (t0 + (wrapped1 ? 1u : 0u)) * period + cnt;
        }
    }
}

void spin_until(uint32_t deadline) {
    while (static_cast<int32_t>(cycles_now() - deadline) < 0) {
    }
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// Wait for a flag on the instrument, bounded.
bool wait_flag(uint16_t mask, uint32_t us = 200'000) {
    const uint32_t t0 = cycles_now();
    while (!U2::flag(mask)) {
        if (cycles_now() - t0 > us * cycles_per_us) {
            return false;
        }
    }
    return true;
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

// ---- the instrument's life cycle -------------------------------------------

/// Everything back: the vector released, USART2 reset and gated off,
/// its pads floating, the ruler stopped.
void all_off() {
    Pfic::disable(Irq::usart2);
    Pfic::disable(dma_channel_irq(6));
    Pfic::disable(dma_channel_irq(7));
    Pfic::disable(T2::cc_irq());
    Pfic::disable(T2::update_irq());
    u2_isr = nullptr;
    U2::reset();
    U2::bus_clock(false);
    TxPad::release();
    RxPad::release();
    CtsPad::release();
    RtsPad::release();
    RulerPad::release();
    T2::init();
    u2_interrupts = 0;
    u2_txe = 0;
    u2_tc = 0;
    u2_idle = 0;
    u2_pe = 0;
    u2_lbd = 0;
    u2_cts = 0;
    u2_words = 0;
}

/// The resource brought up bare on its column: gate, remap, pads,
/// the frame and the rate, TE and RE, no interrupt. The RX pad takes a
/// pull-up so an idle line reads idle and the bit-banger has a level
/// to move.
bool resource_up(const UartFormat& f, uint32_t baud) {
    U2::bus_clock(true);
    U2::reset();
    U2::remap(column);
    TxPad::function();
    RxPad::input(PinPull::up);
    if (!U2::configure(f, static_cast<uint16_t>(usart_divisor(SysClock::pclk_hz, baud)))) {
        return false;
    }
    U2::enable(true);
    U2::transmitter(true);
    U2::receiver(true);
    settle_ms(3);   // TE's idle frame (14.2) out before anything is timed
    return true;
}

/// Wait for the transmitter to be idle: TXE and TC both up.
bool tx_idle(uint32_t timeout_us = 200'000) {
    return wait_flag(usart_txe, timeout_us) && wait_flag(usart_tc, timeout_us);
}

// ---- the bit-banged line ----------------------------------------------------

/// The receive line's level, through the pull of an input pad.
void line(bool high) {
    if (high) { RxPad::set(); } else { RxPad::clear(); }
}

/// One frame on the receive line at `baud`, every edge at an ABSOLUTE
/// offset from the start so a late store cannot accumulate: a start
/// bit, `bits` data bits LSB first, an optional parity bit (given, so
/// a wrong one can be sent), then `stop_bits` of stop - which is a
/// high line for that long, or a LOW one when `bad_stop` asks for a
/// framing error. `glitch_bit`, when not 0xFF, names the data bit in
/// whose middle the line is flipped for a quarter of a bit time.
struct Frame {
    uint16_t data = 0;
    uint8_t bits = 8;
    bool parity = false;      ///< send a parity bit at all
    bool parity_value = false;
    uint8_t stop_bits = 1;
    bool bad_stop = false;
    uint8_t glitch_bit = 0xFF;
    uint32_t baud = 2400;
};

void bang_frame(const Frame& fr) {
    const uint32_t bit = SysClock::hz / fr.baud;
    const uint32_t t0 = cycles_now();
    uint32_t n = 0;
    line(false);   // start
    ++n;
    for (uint8_t i = 0; i < fr.bits; ++i) {
        spin_until(t0 + n * bit);
        const bool level = ((fr.data >> i) & 1u) != 0u;
        line(level);
        if (fr.glitch_bit == i) {
            // Over TWO of the three centre samples (measured at about 13,
            // 15 and 17 thirty-seconds of the bit): the majority flips the
            // bit AND the disagreement raises NE. Over one sample NE rises
            // and the bit holds; over all three it flips in silence.
            spin_until(t0 + n * bit + (bit * 14u) / 32u);
            line(!level);
            spin_until(t0 + n * bit + (bit * 18u) / 32u);
            line(level);
        }
        ++n;
    }
    if (fr.parity) {
        spin_until(t0 + n * bit);
        line(fr.parity_value);
        ++n;
    }
    spin_until(t0 + n * bit);
    line(!fr.bad_stop);
    spin_until(t0 + (n + fr.stop_bits) * bit);
    line(true);
}

/// A frame of `f`'s shape carrying `v`, the parity computed: what a
/// peer speaking that format would put on the line.
void bang_word(uint16_t v, const UartFormat& f, uint32_t baud = 9600) {
    const uint8_t bits = static_cast<uint8_t>(f.bits);
    Frame fr{.data = v, .bits = bits, .baud = baud};
    if (f.parity != UartParity::none) {
        fr.parity = true;
        bool p = false;
        for (uint8_t i = 0; i < bits; ++i) {
            p ^= ((v >> i) & 1u) != 0u;
        }
        fr.parity_value = f.parity == UartParity::even ? p : !p;
    }
    fr.stop_bits = f.stop == UartStop::two ? 2 : 1;   // a half or a one-and-a-half stop is at least one high bit
    bang_frame(fr);
}

/// Even parity of the low `bits` of v.
bool even_parity(uint16_t v, uint8_t bits) {
    bool p = false;
    for (uint8_t i = 0; i < bits; ++i) {
        p ^= ((v >> i) & 1u) != 0u;
    }
    return p;
}

/// A break: the line low for `bit_times` bits, then high.
void bang_break(uint32_t baud, uint32_t bit_times) {
    const uint32_t bit = SysClock::hz / baud;
    const uint32_t t0 = cycles_now();
    line(false);
    spin_until(t0 + bit_times * bit);
    line(true);
}

void bang_idle(uint32_t baud, uint32_t bit_times) {
    const uint32_t bit = SysClock::hz / baud;
    const uint32_t t0 = cycles_now();
    line(true);
    spin_until(t0 + bit_times * bit);
}

// ---- the ruler on the jumper --------------------------------------------------

bool probe_jumper() {
    RulerPad::input();
    TxPad::output(true);
    (void)delay_us(clock, 5);
    const bool high = RulerPad::read();
    TxPad::clear();
    (void)delay_us(clock, 5);
    const bool low = !RulerPad::read();
    TxPad::release();
    RulerPad::release();
    return high && low;
}

bool need_jumper() {
    if (jumper_present) {
        return true;
    }
    jumper_present = probe_jumper();
    if (!jumper_present) {
        print(serial, "  SKIPPED, no verdict claimed: no jumper between PD2 (USART2_TX) and PD4 (TIM2_CH1)", crlf);
    }
    return jumper_present;
}

/// TIM2 at HCLK capturing one pulse on PD4: channel 1 on the edge that
/// opens it, channel 2 (indirect, the same input) on the edge that
/// closes it. Returns the width in cycles, or nothing when either edge
/// did not come.
void ruler_arm(bool opens_low) {
    RulerPad::input();
    (void)T2::configure({.prescaler = 0, .period = 0xFFFF});
    (void)T2::capture_channel(0, {.select = TimChannelSelect::direct,
                                  .polarity = opens_low ? TimCapturePolarity::falling : TimCapturePolarity::rising});
    (void)T2::capture_channel(1, {.select = TimChannelSelect::indirect,
                                  .polarity = opens_low ? TimCapturePolarity::rising : TimCapturePolarity::falling});
    T2::clear_flags(T2::compare_flag(0) | T2::compare_flag(1));
    T2::enable(true);
}

std::optional<uint32_t> ruler_read(uint32_t timeout_us) {
    const uint32_t t0 = cycles_now();
    while (!(T2::flag(T2::compare_flag(0)) && T2::flag(T2::compare_flag(1)))) {
        if (cycles_now() - t0 > timeout_us * cycles_per_us) {
            return std::nullopt;
        }
    }
    const uint16_t open = T2::compare(0);
    const uint16_t close = T2::compare(1);
    return static_cast<uint16_t>(close - open);
}

// ===========================================================================
// a - the instance and the facts
// ===========================================================================

void ta_facts() {
    all_off();
    U2::bus_clock(true);
    U2::reset();
    const bool reset_values = U2::status() == 0x00C0u && U2::regs().BRR == 0u && U2::regs().CTLR1 == 0u &&
                              U2::regs().CTLR2 == 0u && U2::regs().CTLR3 == 0u && U2::regs().GPR == 0u;
    print(serial, "  after the reset pulse: STATR ", hex(U2::status()), " BRR ", hex(U2::regs().BRR), " CTLR1..3 ",
          hex(U2::regs().CTLR1), " ", hex(U2::regs().CTLR2), " ", hex(U2::regs().CTLR3), " GPR ", hex(U2::regs().GPR),
          crlf);
    bench.verdict("the reset values are table 14-3's (STATR 0xC0, the rest zero)", reset_values);

    // The reserved bits of the register description: written one, read
    // back zero. CTLR2 bits 11:7 (the F1's synchronous mode), CTLR3
    // bits 5:4 (the F1's smartcard), GPR bits 15:8 (the F1's guard time).
    U2::regs().CTLR2 = 0x0F80u;
    const uint16_t ctlr2_back = U2::regs().CTLR2;
    U2::regs().CTLR2 = 0;
    U2::regs().CTLR3 = 0x0030u;
    const uint16_t ctlr3_back = U2::regs().CTLR3;
    U2::regs().CTLR3 = 0;
    U2::regs().GPR = 0xFF00u;
    const uint16_t gpr_back = U2::regs().GPR;
    U2::regs().GPR = 0;
    print(serial, "  reserved bits written one read back: CTLR2[11:7] ", hex(ctlr2_back), " CTLR3[5:4] ",
          hex(ctlr3_back), " GPR[15:8] ", hex(gpr_back), crlf);
    bench.verdict("CTLR2 bits 11:7 do not stick - no synchronous mode on this silicon (the header's has_synchronous)",
                  ctlr2_back == 0u && !U2::has_synchronous);
    bench.verdict("CTLR3 bits 5:4 and GPR bits 15:8 do not stick - no smartcard, no guard time (has_smartcard)",
                  ctlr3_back == 0u && gpr_back == 0u && !U2::has_smartcard);

    // The verbs that live in the named bits DO stick, each read back.
    bool sticks = true;
    sticks = sticks && U2::mute_mode({.wake = MuteWake::address_mark, .address = 9}) &&
             (U2::regs().CTLR1 & usart_wake) != 0u && (U2::regs().CTLR2 & usart_add_mask) == 9u;
    sticks = sticks && U2::lin({.break_11bit = true, .break_interrupt = true}) &&
             (U2::regs().CTLR2 & (usart_linen | usart_lbdl | usart_lbdie)) == (usart_linen | usart_lbdl | usart_lbdie);
    U2::lin_off();
    sticks = sticks && U2::half_duplex(true) && U2::half_duplex();
    (void)U2::half_duplex(false);
    sticks = sticks && U2::irda({.low_power = true, .prescaler = 17}) && U2::irda_enabled() && U2::prescaler() == 17u &&
             (U2::regs().CTLR3 & usart_irlp) != 0u;
    U2::irda_off();
    U2::flow_control(true, true);
    sticks = sticks && U2::rts_enabled() && U2::cts_enabled();
    U2::flow_control(false, false);
    bench.verdict("every named bit sticks: WAKE and ADD, LINEN/LBDL/LBDIE, HDSEL, IREN/IRLP and PSC, RTSE/CTSE", sticks);

    // The vocabulary's refusals, and the modes excluding each other.
    const bool vocabulary = !uart_format_valid({.bits = UartBits::seven}) &&
                            !uart_format_valid({.bits = UartBits::nine, .parity = UartParity::even}) &&
                            !U2::configure({}, 15) && !U2::set_brr(15) && !U2::irda({.prescaler = 0}) &&
                            !U2::mute_mode({.address = 16});
    bench.verdict("the vocabulary refuses: seven bits without parity, nine with, a divisor of 15, a prescaler of 0, an "
                  "address of 16",
                  vocabulary);
    bool exclusions = true;
    exclusions = exclusions && U2::half_duplex(true) && !U2::lin({}) && !U2::irda({});
    (void)U2::half_duplex(false);
    exclusions = exclusions && U2::lin({}) && !U2::half_duplex(true) && !U2::irda({});
    U2::lin_off();
    exclusions = exclusions && U2::stop_bits(UartStop::two) && !U2::irda({}) && U2::stop_bits(UartStop::one);
    exclusions = exclusions && U2::irda({}) && !U2::lin({}) && !U2::half_duplex(true) && !U2::stop_bits(UartStop::two);
    U2::irda_off();
    bench.verdict("the modes exclude each other as 14.4 and 14.5 say: half duplex, LIN and IrDA, and IrDA's one stop bit",
                  exclusions);
    print(serial, "  USART2 on column ", column, ": TX PD2, RX PD3, CTS PA0, RTS PA1", crlf);
    all_off();
}

/// One banged frame's outcome on the receiver: what STATR held with
/// RXNE, and the word.
struct Received {
    bool came;
    uint16_t status;
    uint16_t word;
};

Received receive_one(uint32_t timeout_us = 50'000) {
    if (!wait_flag(usart_rxne, timeout_us)) {
        return {false, U2::status(), 0};
    }
    const uint16_t st = U2::status();
    const uint16_t w = U2::read_word();
    return {true, st, w};
}

// ===========================================================================
// b - every frame format, both ways; the single wire as a bus
// ===========================================================================

uint32_t xorshift_step(uint32_t& x) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

struct Fmt {
    const char* name;
    UartFormat f;
    uint8_t low_bits;      ///< start + data + an even parity bit of a 0x00 frame: the low run
    uint8_t stop_halves;   ///< the high gap to the next frame in half bits: an odd parity bit plus the stops
};

const Fmt formats[] = {
    {"8N1", {}, 9, 2},
    {"8E1", {.parity = UartParity::even}, 10, 2},
    {"8O1", {.parity = UartParity::odd}, 9, 4},   // odd parity of 0x00 is a ONE: the low run stops before it and the gap carries it
    {"8N2", {.stop = UartStop::two}, 9, 4},
    {"8N0.5", {.stop = UartStop::half}, 9, 1},
    {"8N1.5", {.stop = UartStop::one_and_half}, 9, 3},
    {"7E1", {.bits = UartBits::seven, .parity = UartParity::even}, 9, 2},
    {"7O2", {.bits = UartBits::seven, .parity = UartParity::odd, .stop = UartStop::two}, 8, 6},
    {"9N1", {.bits = UartBits::nine}, 10, 2},
    {"9N2", {.bits = UartBits::nine, .stop = UartStop::two}, 10, 4},
};

void tb_formats() {
    all_off();
    // The transmitter, on the jumper: the low run of a 0x00 frame at
    // 9600 (5000 cycles a bit) is the start bit plus the data bits plus
    // an even parity bit; the high gap between two 0x00 frames sent
    // back to back is the stop length.
    if (need_jumper()) {
        uint8_t exact = 0;
        for (const Fmt& fmt : formats) {
            all_off();
            (void)resource_up(fmt.f, 9600);
            ruler_arm(true);
            U2::write_word(0);
            const auto low = ruler_read(50'000);
            (void)tx_idle();
            settle_ms(2);
            // The gap: the first frame's low run ends (a rising edge on
            // channel 1), the second's start bit begins (a falling edge on
            // channel 2). Channel 2 also catches the FIRST start bit, so
            // its flag is cleared once channel 1 has fired.
            ruler_arm(false);
            U2::write_word(0);
            (void)wait_flag(usart_txe, 20'000);
            U2::write_word(0);   // queued behind the first: back to back
            std::optional<uint32_t> gap;
            {
                const uint32_t t0 = cycles_now();
                while (!T2::flag(T2::compare_flag(0)) && cycles_now() - t0 < 50'000u * cycles_per_us) {
                }
                T2::clear_flags(T2::compare_flag(1));
                while (!T2::flag(T2::compare_flag(1)) && cycles_now() - t0 < 50'000u * cycles_per_us) {
                }
                if (T2::flag(T2::compare_flag(0)) && T2::flag(T2::compare_flag(1))) {
                    gap = static_cast<uint16_t>(T2::compare(1) - T2::compare(0));
                }
            }
            (void)tx_idle();
            const uint32_t low_bits_x2 = low ? (*low * 2u + 2500u) / 5000u : 0u;
            const uint32_t gap_halves = gap ? (*gap + 1250u) / 2500u : 0u;
            const bool ok = low && gap && low_bits_x2 == fmt.low_bits * 2u && gap_halves == fmt.stop_halves;
            if (ok) {
                ++exact;
            }
            print(serial, "  ", fmt.name, " sent: low run ", low ? *low : 0u, " cycles (", fmt.low_bits,
                  " bits), stop gap ", gap ? *gap : 0u, " (", fmt.stop_halves, " half bits)", ok ? "" : "  OFF", crlf);
        }
        bench.verdict("the transmitter's frames are the format's, measured on the jumper: start + data + parity as one "
                      "low run, the stop length as the gap between two",
                      exact == 10u);
    }
    // The receiver, from the banged line at 9600: eight words per
    // format, the parity computed the format's way.
    uint8_t exact = 0;
    for (const Fmt& fmt : formats) {
        all_off();
        (void)resource_up(fmt.f, 9600);
        const uint16_t mask = uart_data_mask(fmt.f);
        const uint16_t values[] = {0x000, 0x0FF, 0x055, 0x0AA, 0x1FF, 0x101, 0x07F, 0x080};
        uint8_t good = 0;
        bang_idle(9600, 4);
        for (const uint16_t v : values) {
            const uint16_t want = static_cast<uint16_t>(v & mask);
            bang_word(want, fmt.f);
            const Received r = receive_one(20'000);
            if (r.came && (r.word & mask) == want && (r.status & (usart_pe | usart_fe | usart_ne)) == 0u) {
                ++good;
            }
        }
        const bool ok = good == 8u;
        if (ok) {
            ++exact;
        }
        print(serial, "  ", fmt.name, " received: ", good, " of 8 words exact and clean", ok ? "" : "  OFF", crlf);
    }
    bench.verdict("the receiver takes every format from the banged line: 7, 8 and 9 data bits, both parities, the four "
                  "stop lengths",
                  exact == 10u);
    all_off();

    // The single wire as a bus: the transport with the half-duplex
    // option listens on its TX pad (AF open drain, PD4's pull-up through
    // the jumper as the bus's), and a frame driven onto the wire from
    // PD4's pull arrives.
    if (need_jumper()) {
        u2_isr = &WireUart::isr;
        const bool up = WireUart::init(clock, 9600);
        RulerPad::input(PinPull::up);
        settle_ms(3);
        {
            const uint32_t bit = SysClock::hz / 9600u;
            const uint32_t t0 = cycles_now();
            uint32_t n = 0;
            RulerPad::clear();
            ++n;
            for (uint8_t i = 0; i < 8u; ++i) {
                spin_until(t0 + n * bit);
                if (((0xC3u >> i) & 1u) != 0u) { RulerPad::set(); } else { RulerPad::clear(); }
                ++n;
            }
            spin_until(t0 + n * bit);
            RulerPad::set();
            spin_until(t0 + (n + 2u) * bit);
        }
        settle_ms(1);
        uint8_t b = 0;
        const bool heard = WireUart::read_byte(b);
        // And its own frame is NOT heard back: the wire is a bus, not a loop.
        (void)WireUart::write_byte(0x3C);
        settle_ms(3);
        uint8_t echo = 0;
        const bool echoed = WireUart::read_byte(echo);
        print(serial, "  the single-wire transport: a frame driven on the wire ", heard ? "heard" : "NOT heard", " (",
              hex(b), "); its own frame ", echoed ? "ECHOED" : "not echoed", crlf);
        bench.verdict("the half-duplex option hears a frame driven onto its wire and not its own - a bus, not a loop",
                      up && heard && b == 0xC3u && !echoed);
    }
    all_off();
}

// ===========================================================================
// c - the baud generator
// ===========================================================================

void tc_baud() {
    all_off();
    // A 0xFF frame is one low bit, the start bit: its width on the
    // jumper IS the divisor, in cycles (BRR counts sixteenths of a bit
    // in peripheral clocks, and the bit is sixteen of them).
    const uint32_t rates[] = {2400, 9600, 76800, 115200, 460800, 921600, 1'500'000, 3'000'000};
    if (need_jumper()) {
        uint8_t exact = 0;
        for (const uint32_t baud : rates) {
            all_off();
            const bool ok_up = resource_up({}, baud);
            const uint32_t brr = usart_divisor(SysClock::pclk_hz, baud);
            const uint32_t actual = U2::actual_baud(SysClock::pclk_hz);
            ruler_arm(true);
            U2::write_data(0xFF);
            const auto bit = ruler_read(50'000);
            (void)tx_idle();
            const bool ok = ok_up && bit && *bit == brr;
            if (ok) {
                ++exact;
            }
            print(serial, "  ", baud, " baud: BRR ", brr, " (", brr / 16u, " and ", brr % 16u, "/16), actual ", actual,
                  ", the start bit ", bit ? *bit : 0u, " cycles", ok ? "" : "  OFF", crlf);
        }
        bench.verdict("the start bit measures exactly BRR cycles at eight rates from 2400 to 3 Mbaud - the fractional "
                      "divisor included (76800 = 39 and 1/16)",
                      exact == 8u);
    }
    // The receiver at 9600 from the banged line: 64 words exact.
    all_off();
    (void)resource_up({}, 9600);
    uint8_t good = 0;
    bang_idle(9600, 4);
    for (uint16_t v = 0; v < 64u; ++v) {
        const uint16_t w = static_cast<uint16_t>((v * 37u + 11u) & 0xFFu);
        bang_frame({.data = w, .baud = 9600});
        const Received r = receive_one(20'000);
        if (r.came && r.word == w && (r.status & (usart_fe | usart_ne | usart_pe)) == 0u) {
            ++good;
        }
    }
    print(serial, "  64 banged words at 9600: ", good, " exact", crlf);
    bench.verdict("the receiver takes 64 banged words at 9600 exact", good == 64u);
    all_off();
    U2::bus_clock(true);
    U2::reset();
    const bool refused = !U2::configure({}, static_cast<uint16_t>(usart_divisor(SysClock::pclk_hz, 4'000'000)));
    bench.verdict("4 Mbaud asks for a divisor of 12 and is refused (the generator's floor is 16)", refused);
    all_off();
}

// ===========================================================================
// d - the bit-banged line
// ===========================================================================

void td_banged() {
    all_off();
    // 8N1 at 2400: a clean frame.
    (void)resource_up({}, 2400);
    bang_idle(2400, 4);
    bang_frame({.data = 0xA5});
    const Received clean = receive_one();
    print(serial, "  a clean 8N1 frame at 2400: came ", clean.came, " word ", hex(clean.word), " STATR ",
          hex(clean.status), crlf);
    bench.verdict("a bit-banged 8N1 frame is received exact with no error flag", clean.came && clean.word == 0xA5u &&
                                                                                    (clean.status & (usart_pe | usart_fe | usart_ne | usart_ore)) == 0u);

    // 8E1: the right parity, then the wrong one.
    all_off();
    (void)resource_up({.parity = UartParity::even}, 2400);
    bang_idle(2400, 4);
    bang_frame({.data = 0x3C, .parity = true, .parity_value = even_parity(0x3C, 8)});
    const Received right = receive_one();
    bang_frame({.data = 0x3C, .parity = true, .parity_value = !even_parity(0x3C, 8)});
    const Received wrong = receive_one();
    print(serial, "  8E1 right parity: word ", hex(right.word), " PE ", (right.status & usart_pe) != 0u,
          "; wrong parity: word ", hex(wrong.word), " PE ", (wrong.status & usart_pe) != 0u, crlf);
    bench.verdict("the parity bit is checked: right passes, wrong raises PE with the byte still delivered",
                  right.came && (right.status & usart_pe) == 0u && right.word == 0x3Cu && wrong.came &&
                      (wrong.status & usart_pe) != 0u);

    // A low stop bit: framing error.
    all_off();
    (void)resource_up({}, 2400);
    bang_idle(2400, 4);
    bang_frame({.data = 0x5A, .bad_stop = true});
    const Received framed = receive_one();
    bang_idle(2400, 4);
    print(serial, "  a low stop bit: came ", framed.came, " word ", hex(framed.word), " FE ",
          (framed.status & usart_fe) != 0u, crlf);
    bench.verdict("a low stop bit raises FE", framed.came && (framed.status & usart_fe) != 0u);

    // A glitch in the middle of a data bit: noise.
    all_off();
    (void)resource_up({}, 2400);
    bang_idle(2400, 4);
    bang_frame({.data = 0x0F, .glitch_bit = 2});
    const Received noisy = receive_one();
    print(serial, "  a glitch over two centre samples of bit 2: came ", noisy.came, " word ", hex(noisy.word), " NE ",
          (noisy.status & usart_ne) != 0u, " FE ", (noisy.status & usart_fe) != 0u, crlf);
    bench.verdict("a glitch over two of the three centre samples flips the bit and raises NE",
                  noisy.came && (noisy.status & usart_ne) != 0u && noisy.word == 0x0Bu);

    // The tolerance: frames at rates off by +3%, +6% and -6% (14.3: not
    // less than 3% for the module's own share).
    all_off();
    (void)resource_up({}, 2400);
    bang_idle(2400, 4);
    const int8_t offs[] = {3, -3, 6, -6};
    uint8_t taken = 0;
    for (const int8_t off : offs) {
        const uint32_t baud = static_cast<uint32_t>(2400 + (2400 * off) / 100);
        bang_frame({.data = 0x69, .baud = baud});
        const Received r = receive_one();
        bang_idle(2400, 4);
        const bool ok = r.came && r.word == 0x69u && (r.status & (usart_fe | usart_ne)) == 0u;
        if (ok) {
            ++taken;
        }
        print(serial, "  a frame at ", static_cast<int32_t>(off), "%: ", ok ? "taken clean" : "not clean", " (word ",
              hex(r.word), " STATR ", hex(r.status), ")", crlf);
    }
    bench.verdict("frames 3% off the rate are taken clean (the chapter's floor)", taken >= 2u);
    all_off();
}

// ===========================================================================
// e - LIN
// ===========================================================================

void te_lin() {
    all_off();
    // The break sent, timed on the jumper: ten bits outside LIN mode,
    // thirteen in it, at 9600 (a bit is 5000 cycles).
    bool timed = false;
    uint32_t plain = 0;
    uint32_t lin = 0;
    if (need_jumper()) {
        (void)resource_up({}, 9600);
        ruler_arm(true);
        U2::send_break();
        if (const auto w = ruler_read(50'000)) {
            plain = *w;
        }
        all_off();
        (void)resource_up({}, 9600);
        (void)U2::lin({});
        ruler_arm(true);
        U2::send_break();
        if (const auto w = ruler_read(50'000)) {
            lin = *w;
        }
        print(serial, "  the break on the pad at 9600: ", plain, " cycles plain (10 bits = 50000), ", lin,
              " in LIN mode (13 bits = 65000)", crlf);
        timed = plain >= 49'500u && plain <= 50'500u && lin >= 64'500u && lin <= 65'500u;
        bench.verdict("SBK puts ten bits of low on the line, thirteen in LIN mode - measured on the jumper", timed);
    }
    all_off();

    // The break detected at ten and eleven bits, from the banged line.
    struct Case {
        bool eleven;
        uint32_t low_bits;
        bool expect;
    };
    const Case cases[] = {{false, 10, true}, {false, 9, false}, {true, 11, true}, {true, 10, false}};
    uint8_t right = 0;
    for (const Case& c : cases) {
        all_off();
        (void)resource_up({}, 9600);
        (void)U2::lin({.break_11bit = c.eleven, .break_interrupt = true});
        u2_isr = nullptr;
        Pfic::enable(Irq::usart2);
        bang_idle(9600, 4);
        U2::clear_flags(usart_lbd);
        u2_lbd = 0;
        bang_break(9600, c.low_bits);
        bang_idle(9600, 4);
        const bool detected = U2::flag(usart_lbd) || u2_lbd != 0u;
        const uint32_t interrupts = u2_lbd;
        Pfic::disable(Irq::usart2);
        U2::clear_flags(usart_lbd);
        const bool ok = detected == c.expect && (!c.expect || interrupts == 1u);
        if (ok) {
            ++right;
        }
        print(serial, "  LBDL ", c.eleven ? 11 : 10, " bits, a break of ", c.low_bits, ": LBD ", detected,
              ", interrupts ", interrupts, ok ? "" : "  OFF", crlf);
    }
    bench.verdict("the break detector sees ten low bits under LBDL 0 and eleven under LBDL 1, one short of each is a frame, "
                  "one interrupt per break",
                  right == 4u);
    all_off();
}

// ===========================================================================
// f - mute mode
// ===========================================================================

void tf_mute() {
    all_off();
    // Idle-line wake: a byte first (note 1 of 14.8.4), then mute, then
    // frames back to back that must not arrive, then an idle line, then
    // a frame that must.
    (void)resource_up({}, 2400);
    bang_idle(2400, 4);
    bang_frame({.data = 0x11});
    const Received first = receive_one();
    const bool muted = U2::mute();
    bang_frame({.data = 0x22});
    bang_frame({.data = 0x33});
    const bool silent = !U2::flag(usart_rxne);
    bang_idle(2400, 12);
    const bool woke = !U2::muted();
    bang_frame({.data = 0x44});
    const Received after = receive_one();
    print(serial, "  idle-line wake: first byte ", hex(first.word), ", muted ", muted, ", two frames back to back: RXNE ",
          !silent, ", after an idle line RWU ", U2::muted() ? "still set" : "cleared", ", next byte ", hex(after.word),
          crlf);
    bench.verdict("in mute mode frames back to back are not received; an idle line clears RWU and the next frame is",
                  first.word == 0x11u && muted && silent && woke && after.came && after.word == 0x44u);
    all_off();

    // Address-mark wake: nine-bit frames, MSB set = an address; the
    // receiver on address 5 sleeps through address 3's frames and
    // wakes on its own.
    (void)resource_up({.bits = UartBits::nine}, 2400);
    (void)U2::mute_mode({.wake = MuteWake::address_mark, .address = 5});
    bang_idle(2400, 4);
    const bool muted2 = U2::mute();
    bang_frame({.data = 0x103, .bits = 9});   // address 3: not ours
    bang_frame({.data = 0x0AB, .bits = 9});   // its data
    const bool ignored = !U2::flag(usart_rxne);
    bang_frame({.data = 0x105, .bits = 9});   // address 5: ours
    const Received addr = receive_one();
    bang_frame({.data = 0x0CD, .bits = 9});   // our data
    const Received data = receive_one();
    print(serial, "  address-mark wake on 5: muted ", muted2, ", address 3 and its byte ignored ", ignored,
          ", address 5 received ", hex(addr.word), ", then ", hex(data.word), ", RWU ", U2::muted() ? "set" : "clear",
          crlf);
    bench.verdict("under an address mark the receiver sleeps through another node's frames and wakes on its own address, "
                  "receiving it and what follows",
                  muted2 && ignored && addr.came && addr.word == 0x105u && data.came && data.word == 0x0CDu &&
                      !U2::muted());
    all_off();
}

// ===========================================================================
// g - IrDA
// ===========================================================================

void tg_irda() {
    all_off();
    if (!need_jumper()) {
        return;
    }
    // The encoder: a 0x00 byte at 9600 is a start bit plus eight zeros,
    // each a pulse of 3/16 of a bit (104.2 us x 3/16 = 19.5 us = 938
    // cycles) - the first pulse measured on the jumper.
    (void)resource_up({}, 9600);
    (void)U2::irda({});
    settle_ms(3);   // TE's idle frame out, the encoder's line at its idle level
    print(serial, "  the IrDA line idles ", TxPad::read() ? "high" : "low", " on the pad", crlf);
    ruler_arm(false);
    U2::write_data(0xFF);   // a start bit alone: one pulse
    const auto normal = ruler_read(50'000);
    print(serial, "  normal mode edges: opened at ", T2::compare(0), " closed at ", T2::compare(1),
          " (rising then falling)", crlf);
    all_off();
    // Low-power mode: the pulse is three periods of the prescaled clock
    // (14.8.7): with PSC 16 that is 3 x 16 = 48 cycles.
    (void)resource_up({}, 9600);
    (void)U2::irda({.low_power = true, .prescaler = 16});
    settle_ms(3);
    ruler_arm(false);
    U2::write_data(0xFF);
    const auto low = ruler_read(50'000);
    print(serial, "  the IrDA pulse of a start bit at 9600: normal ", normal ? *normal : 0u,
          " cycles (3/16 bit = 938), low power PSC 16: ", low ? *low : 0u, " cycles (3 x 16 = 48)", crlf);
    bench.verdict("the SIR encoder's pulse is 3/16 of a bit in normal mode", normal && *normal >= 900u && *normal <= 980u);
    bench.verdict("and three prescaled clock periods in low-power mode", low && *low >= 40u && *low <= 60u);
    all_off();

    // The decoder, fed a bit-banged RZI frame on RX: the line idles
    // LOW in SIR receive logic and a zero bit is a HIGH pulse of 3/16
    // bit in the middle of its cell (14.5). Tried both ways and the
    // outcome printed, the verdict on the one the chapter names.
    uint8_t decoded = 0;
    bool decoded_idle_high = false;
    uint8_t decoded_width16 = 0;
    const uint8_t widths16[] = {3, 8};
    for (uint8_t pol = 0; pol < 2u; ++pol) {
        for (const uint8_t w16 : widths16) {
            all_off();
            (void)resource_up({}, 2400);
            (void)U2::irda({});
            const bool idle_high = pol == 1u;
            line(idle_high);
            settle_ms(6);   // longer than a frame: whatever the decoder made of the level change is over
            U2::clear_by_read();
            U2::clear_flags(usart_lbd);
            const uint16_t before = U2::status();
            const uint32_t bit = SysClock::hz / 2400u;
            const uint32_t pulse = bit * w16 / 16u;
            const uint16_t data = 0x5A;
            const uint32_t t0 = cycles_now();
            auto cell = [&](uint32_t n, bool zero) {
                spin_until(t0 + n * bit + bit / 2u - pulse / 2u);
                if (zero) {
                    line(!idle_high);
                }
                spin_until(t0 + n * bit + bit / 2u + pulse / 2u);
                line(idle_high);
            };
            cell(0, true);   // the start bit is a zero
            for (uint8_t i = 0; i < 8u; ++i) {
                cell(1u + i, ((data >> i) & 1u) == 0u);
            }
            cell(9, false);   // the stop bit is a one: no pulse
            spin_until(t0 + 11u * bit);
            const Received r = receive_one(20'000);
            print(serial, "  RZI frame, line idling ", idle_high ? "high" : "low", ", pulses of ", w16, "/16 bit: STATR before ",
                  hex(before), ", came ", r.came, " word ", hex(r.word), " STATR ", hex(r.status), crlf);
            if (r.came && r.word == data && decoded == 0u) {
                decoded = 1;
                decoded_idle_high = idle_high;
                decoded_width16 = w16;
            }
        }
    }
    print(serial, "  decoded: ", decoded ? "yes" : "no", decoded ? (decoded_idle_high ? ", line idling high" : ", line idling low") : "",
          ", pulses of ", decoded_width16, "/16", crlf);
    bench.verdict("the SIR decoder takes a bit-banged RZI frame: a zero is a pulse against the idle level",
                  decoded == 1u);
    all_off();
}

// ===========================================================================
// h - hardware flow control
// ===========================================================================

void th_flow() {
    all_off();
    // CTS: the pad pulled high (not clear to send) - a byte written
    // stays in the shift register's queue: TC does not come; the pull
    // moved low, it leaves.
    (void)resource_up({}, 115200);
    CtsPad::input(PinPull::up);
    U2::flow_control(false, true);
    U2::clear_flags(usart_tc);
    U2::write_data(0x5A);
    settle_ms(2);
    const bool held = !U2::tx_complete();
    CtsPad::clear();   // the pull down: clear to send
    const bool left = wait_flag(usart_tc, 5000);
    print(serial, "  CTS high: TC after 2 ms ", !held, "; CTS low: TC ", left, crlf);
    bench.verdict("CTS high holds the frame back, CTS low lets it out", held && left);
    U2::flow_control(false, false);
    all_off();

    // RTS: driven by the receiver - low while it can take a frame, high
    // while a received byte waits unread.
    (void)resource_up({}, 2400);
    RtsPad::function();
    U2::flow_control(true, false);
    settle_ms(1);
    const bool rts_idle_low = !RtsPad::read();
    bang_idle(2400, 4);
    bang_frame({.data = 0x77});
    settle_ms(1);
    const bool rts_high_pending = RtsPad::read();
    const Received r = receive_one();
    settle_ms(1);
    const bool rts_low_again = !RtsPad::read();
    print(serial, "  RTS: idle ", rts_idle_low ? "low" : "HIGH", ", a byte pending ", rts_high_pending ? "high" : "LOW",
          ", read (", hex(r.word), ") ", rts_low_again ? "low" : "HIGH", crlf);
    bench.verdict("RTS is low while the receiver can take a frame, high while a byte waits, low again once it is read",
                  rts_idle_low && rts_high_pending && r.came && r.word == 0x77u && rts_low_again);
    all_off();
}

// ===========================================================================
// i - the DMA engines on channels 6 and 7
// ===========================================================================

void ti_dma() {
    all_off();
    Dma::open();
    u2_isr = &DmaUart::isr;
    const bool up = DmaUart::init(clock, 9600);
    RxPad::input(PinPull::up);   // the banged line, over the transport's own floating input
    Pfic::enable(dma_channel_irq(6));
    Pfic::enable(dma_channel_irq(7));
    settle_ms(3);
    // The receive engine fed sixteen banged frames, then harvested.
    bang_idle(9600, 4);
    uint32_t seed = 0x12345678u;
    for (uint8_t i = 0; i < 16u; ++i) {
        bang_frame({.data = static_cast<uint16_t>(xorshift_step(seed) & 0xFFu), .baud = 9600});
    }
    settle_ms(2);
    (void)DmaUart::harvest();
    uint32_t seed2 = 0x12345678u;
    uint8_t good = 0;
    uint8_t got = 0;
    uint8_t b;
    while (DmaUart::read_byte(b)) {
        ++got;
        if (b == static_cast<uint8_t>(xorshift_step(seed2) & 0xFFu)) {
            ++good;
        }
    }
    print(serial, "  the receive engine on channel 7: ", got, " bytes harvested, ", good, " exact, faults ",
          DmaUart::dma_faults(), crlf);
    bench.verdict("USART2's receive engine on channel 7 delivers sixteen banged frames byte-exact through harvest()",
                  up && got == 16u && good == 16u && DmaUart::dma_faults() == 0u);
    // The transmit engine: 256 bytes queued at once, the run timed -
    // 2560 bits at 9600 is 266.7 ms - and the ring empty at the end.
    const uint32_t t0 = Ticker::millis();
    uint32_t queued = 0;
    for (uint16_t i = 0; i < 256u; ++i) {
        if (DmaUart::write_byte(static_cast<uint8_t>(i))) {
            ++queued;
        }
    }
    while (!DmaUart::tx_idle() && Ticker::millis() - t0 < 1000u) {
    }
    (void)wait_flag(usart_tc, 20'000);
    const uint32_t took = Ticker::millis() - t0;
    print(serial, "  the transmit engine on channel 6: ", queued, " bytes queued, the run took ", took,
          " ms (267 for 2560 bits), faults ", DmaUart::dma_faults(), crlf);
    bench.verdict("USART2's transmit engine on channel 6 moves a 256-byte ring in one run at the wire's pace",
                  queued == 256u && took >= 260u && took <= 285u && DmaUart::dma_faults() == 0u);
    Pfic::disable(dma_channel_irq(6));
    Pfic::disable(dma_channel_irq(7));
    all_off();
}

// ===========================================================================
// j - the flags and the vector
// ===========================================================================

void tj_flags() {
    all_off();
    (void)resource_up({}, 9600);
    // TXE comes when the byte moves to the shift register, TC when its
    // stop bit is out: at 9600 a frame is 1042 us.
    U2::clear_flags(usart_tc);
    const uint32_t t0 = cycles_now();
    U2::write_data(0x55);
    uint32_t txe_at = 0;
    while (!U2::tx_empty()) {
    }
    txe_at = cycles_now() - t0;
    while (!U2::tx_complete()) {
    }
    const uint32_t tc_at = cycles_now() - t0;
    print(serial, "  TXE after ", txe_at / cycles_per_us, " us, TC after ", tc_at / cycles_per_us, " us (a frame is 1042)",
          crlf);
    bench.verdict("TXE returns within a bit time and TC at the end of the frame", txe_at < 110u * cycles_per_us &&
                                                                                       tc_at >= 1000u * cycles_per_us &&
                                                                                       tc_at <= 1200u * cycles_per_us);
    U2::clear_by_read();
    all_off();

    // The interrupts, one each, through the vector: TXE, TC, RXNE (the
    // suite's own counter of words), IDLE, PE.
    (void)resource_up({.parity = UartParity::even}, 2400);
    u2_isr = nullptr;
    Pfic::enable(Irq::usart2);
    U2::interrupts(usart_tcie | usart_idleie | usart_peie | usart_rxneie, true);
    bang_idle(2400, 4);
    bang_frame({.data = 0x3C, .parity = true, .parity_value = even_parity(0x3C, 8)});
    bang_idle(2400, 14);   // IDLE wants a whole frame of high line after a reception
    settle_ms(2);
    const uint32_t words_clean = u2_words;
    const uint32_t idle_clean = u2_idle;
    const uint32_t pe_clean = u2_pe;
    bang_frame({.data = 0x3C, .parity = true, .parity_value = !even_parity(0x3C, 8)});
    bang_idle(2400, 4);
    settle_ms(2);
    const uint32_t pe_wrong = u2_pe;
    U2::interrupts(usart_txeie, true);
    settle_ms(1);
    const uint32_t txe_n = u2_txe;
    U2::interrupts(usart_txeie, false);
    Pfic::disable(Irq::usart2);
    print(serial, "  one clean frame: RXNE ", words_clean, " IDLE ", idle_clean, " PE ", pe_clean,
          "; one bad-parity frame: PE ", pe_wrong, "; TXEIE armed on an idle transmitter: ", txe_n,
          " interrupt (self-disarmed)", crlf);
    bench.verdict("one interrupt per event: RXNE and IDLE once for a clean frame, PE once for a bad parity, TXE once "
                  "when armed",
                  words_clean == 1u && idle_clean == 1u && pe_clean == 0u && pe_wrong == 1u && txe_n == 1u);
    all_off();
}

// ===========================================================================
// y - host-assisted (outside z): the console's own error counters
// ===========================================================================

void ty_host() {
    // The console's transport counts framing, noise, parity and hardware
    // overruns; brio stress can send frames the console was not told
    // about. HOST poke: the host waits out half the window, sends
    // `count` bytes at the announced rate and frame, and waits out the
    // rest; the board, which stays at 115200 8N1, counts what it makes
    // of them.
    Serial::clear_errors();
    print(serial, "HOST poke 0 115200 8E1 1500 64", crlf);
    console_drain();
    settle_ms(1600);
    uint8_t b;
    uint32_t bytes = 0;
    while (Serial::read_byte(b)) {
        ++bytes;
    }
    print(serial, "  64 bytes sent 8E1 into an 8N1 console: ", bytes, " bytes taken, frame errors ",
          Serial::frame_errors(), ", noise ", Serial::noise_errors(), ", parity ", Serial::parity_errors(),
          ", hw overruns ", Serial::hw_overruns(), crlf);
    bench.verdict("frames with an extra parity bit into an 8N1 console are counted as framing errors or taken, "
                  "never lost silently",
                  bytes + Serial::frame_errors() == 64u);
    Serial::clear_errors();
}


#endif   // BRIO_CH32_HAS_USART2

// ===========================================================================
// k - the synchronous mode, on the console's own USART1 (both parts)
// ===========================================================================
//
// The CH32V003's USART has the F1's synchronous mode (its 12.4): with
// CLKEN the column's CK pad - PD4 on USART1's default column - carries
// one clock pulse per data bit while the transmitter shifts, and only
// then. The measurement needs NO WIRE: PD4 is also TIM2's channel 1,
// and a pad handed to one peripheral as an output is still an input
// to the other, so the timer counts the clock's transitions off the
// very pad the USART drives while the console prints - the console's
// own bytes are the clocked frames. On the CH32V006 the bits are
// reserved and the verb refuses.

using U1 = Usart<1>;
using CkPad = Pin<'D', 4>;
using CkTimer = Tim<2>;
using CkCounter = TimEventCounter<CkTimer>;

void console_quiet() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 200);
}

/// The console's transmitter and receiver paused around the verb (12.4:
/// CPOL, CPHA and LBCL are set with TE and RE clear), then both back.
bool console_sync(bool on, const UsartSyncConfig& c = {}) {
    console_quiet();
    U1::transmitter(false);
    U1::receiver(false);
    const bool ok = on ? U1::synchronous(c) : U1::synchronous_off();
    U1::receiver(true);
    U1::transmitter(true);
    return ok;
}

/// Sixteen bytes through the console, the clock's transitions on PD4
/// counted from the first to the last. THE RECEIVER IS OFF MEANWHILE:
/// in synchronous mode it samples RX on the output clock (12.4), so
/// with nothing answering on RX it collects one frame of the idle
/// line per clocked byte (measured: a run of unknown letters at the
/// console while RE stayed on through the clocked run) - a program in
/// this mode either reads those frames as the full-duplex answer or
/// keeps RE clear while it transmits alone.
constexpr uint8_t clocked_count = 16;
uint16_t clocked_bytes() {
    console_quiet();
    U1::receiver(false);
    CkCounter::restart();
    print(serial, "  0123456789abcd");   // sixteen bytes exactly
    console_quiet();
    U1::receiver(true);
    return CkCounter::count();
}

void tk_synchronous() {
    if constexpr (!U1::has_synchronous) {
        // The bits are reserved on this part: the verb refuses, nothing
        // written, the console untouched.
        const uint16_t before = U1::regs().CTLR2;
        const bool refused = !console_sync(true, {.last_bit_clock = true});
        print(serial, "  no synchronous mode on this part: synchronous() answers ", !refused, ", CTLR2 ",
              hex(before), " -> ", hex(U1::regs().CTLR2), crlf);
        bench.verdict("the synchronous mode is refused on this part (has_synchronous false), CTLR2 untouched",
                      refused && U1::regs().CTLR2 == before && !U1::synchronous_enabled());
        return;
    } else {
        // TIM2 counting every transition of TI1 (PD4), the pad handed to
        // the USART as the CK output.
        (void)CkTimer::init();
        (void)CkTimer::capture_channel(0, {.select = TimChannelSelect::direct, .filter = 0});
        const bool counter = CkCounter::setup(TimTrigger::ti1_edge);
        CkPad::function();

        // With TE and RE on, the verb refuses (12.4's rule); paused, it takes.
        const bool refused_live = !U1::synchronous({});
        const bool took = console_sync(true, {.last_bit_clock = false});
        const uint16_t edges7 = clocked_bytes();
        print(serial, crlf, "  CLKEN, LBCL clear: ", edges7, " transitions on PD4 for ", clocked_count,
              " bytes (", clocked_count * 14u, " expected: seven pulses a byte)", crlf);
        bench.verdict("the synchronous mode refuses with the transmitter live and takes with it paused",
                      refused_live && took && U1::synchronous_enabled());
        bench.verdict("without LBCL the CK pad carries SEVEN clock pulses per eight-bit frame, counted by "
                      "TIM2 off the same pad with no wire",
                      counter && edges7 == clocked_count * 14u);

        const bool took8 = console_sync(true, {.last_bit_clock = true});
        const uint16_t edges8 = clocked_bytes();
        print(serial, crlf, "  CLKEN, LBCL set: ", edges8, " transitions (", clocked_count * 16u,
              " expected: eight a byte)", crlf);
        bench.verdict("with LBCL the last data bit gets its pulse too: eight per frame",
                      took8 && edges8 == clocked_count * 16u);

        // CPOL: the clock's idle level, read on the pad between frames.
        console_quiet();
        const bool idle_low = !CkPad::read();
        (void)console_sync(true, {.clock_idle_high = true});
        console_quiet();
        const bool idle_high = CkPad::read();
        const uint16_t edges_pol = clocked_bytes();
        print(serial, crlf, "  CPOL clear: CK idles ", idle_low ? "low" : "HIGH", "; CPOL set: idles ",
              idle_high ? "high" : "LOW", ", ", edges_pol, " transitions for ", clocked_count, " bytes", crlf);
        bench.verdict("CPOL is the clock's idle level on the pad, and the pulse count does not depend on it",
                      idle_low && idle_high && edges_pol == clocked_count * 14u);

        // CPHA: a companion bit for the receiver's sampling edge - it
        // lands and the clock still counts (the sampling itself wants a
        // synchronous peer, the gap list's).
        const bool took_cpha = console_sync(true, {.capture_second_edge = true});
        const bool cpha_landed = (U1::regs().CTLR2 & usart_cpha) != 0u;
        const uint16_t edges_cpha = clocked_bytes();
        print(serial, crlf, "  CPHA set: lands ", cpha_landed, ", ", edges_cpha, " transitions", crlf);
        bench.verdict("CPHA lands and the transmitter's clock is unchanged by it",
                      took_cpha && cpha_landed && edges_cpha == clocked_count * 14u);

        // Off: the pad quiet again, the console still the console.
        const bool off = console_sync(false);
        CkPad::release();
        const uint16_t edges_off = clocked_bytes();
        CkTimer::init();
        print(serial, crlf, "  synchronous mode off, PD4 released: ", edges_off, " transitions", crlf);
        bench.verdict("synchronous_off() drops the clock: no transition on PD4 for sixteen more bytes, and "
                      "the console printed every one of them",
                      off && !U1::synchronous_enabled() && edges_off == 0u);
        // Whatever the receiver took between the runs is not a command.
        uint8_t junk = 0;
        while (Serial::read_byte(junk)) {
        }
    }
}

void banner() {
    print(serial, crlf, "test_ch32_serial - ", device::part_name, " USART (RM ch. 14)", crlf);
#if BRIO_CH32_HAS_USART2
    print(serial, "  USART2 on column 3 as the instrument; the jumper for b, c, e and g: PD2 (USART2_TX) <-> PD4 "
                  "(TIM2_CH1): ", jumper_present ? "PRESENT" : "ABSENT", crlf);
#else
    print(serial, "  one USART, the console's: the synchronous mode measured on its CK pad PD4 by TIM2, no wire", crlf);
#endif
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
#if BRIO_CH32_HAS_USART2
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() {
    u2_interrupts = u2_interrupts + 1u;
    if (const IsrFn fn = u2_isr) {
        (void)fn();
        return;
    }
    // The resource's own accounting: which enabled flags fired.
    const uint16_t st = U2::status();
    const uint16_t c1 = U2::regs().CTLR1;
    if ((st & usart_rxne) != 0u && (c1 & usart_rxneie) != 0u) {
        u2_last_word = U2::read_word();
        u2_words = u2_words + 1u;
    }
    if ((st & usart_pe) != 0u && (c1 & usart_peie) != 0u) {
        u2_pe = u2_pe + 1u;
        if ((st & usart_rxne) == 0u) {
            U2::clear_by_read();
        }
    }
    if ((st & usart_idle) != 0u && (c1 & usart_idleie) != 0u) {
        u2_idle = u2_idle + 1u;
        U2::clear_by_read();
    }
    if ((st & usart_tc) != 0u && (c1 & usart_tcie) != 0u) {
        u2_tc = u2_tc + 1u;
        U2::clear_flags(usart_tc);
    }
    if ((st & usart_txe) != 0u && (c1 & usart_txeie) != 0u) {
        u2_txe = u2_txe + 1u;
        U2::txe_interrupt(false);
    }
    if ((st & usart_lbd) != 0u && (U2::regs().CTLR2 & usart_lbdie) != 0u) {
        u2_lbd = u2_lbd + 1u;
        U2::clear_flags(usart_lbd);
    }
    if ((st & usart_cts) != 0u && (U2::regs().CTLR3 & usart_ctsie) != 0u) {
        u2_cts = u2_cts + 1u;
        U2::clear_flags(usart_cts);
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)DmaUart::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)DmaUart::dma_isr(); }
#endif   // BRIO_CH32_HAS_USART2

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();
#if BRIO_CH32_HAS_USART2
    jumper_present = probe_jumper();

    bench.letter('a', "the instance and the facts: reset values, the reserved bits, the refusals, the exclusions", ta_facts);
    bench.letter('b', "every frame format both ways, and the single wire as a bus", tb_formats);
    bench.letter('c', "the baud generator: a start bit to the cycle at eight rates, the floor", tc_baud);
    bench.letter('d', "the bit-banged line: clean, parity, framing, noise, the tolerance", td_banged);
    bench.letter('e', "LIN: the break timed on the jumper, detected at ten and eleven bits", te_lin);
    bench.letter('f', "mute mode: the idle-line wake and the address mark", tf_mute);
    bench.letter('g', "IrDA: the pulse on the jumper, normal and low power; the decoder", tg_irda);
    bench.letter('h', "hardware flow control: CTS holds, RTS follows the receiver", th_flow);
    bench.letter('i', "the DMA engines on USART2's channels 6 and 7: fed, then timed", ti_dma);
    bench.letter('j', "the flags and the vector: TXE, TC, RXNE, IDLE, PE, one interrupt each", tj_flags);
    bench.letter('y', "host-assisted (brio stress): the console's own error counters", ty_host, false);
#endif
    bench.letter('k', "THE SYNCHRONOUS MODE on the console's USART1: CK counted by TIM2 off PD4, no wire", tk_synchronous);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED", " tick=", tick_ok ? "STK" : "FAILED",
                    brio::crlf);
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
