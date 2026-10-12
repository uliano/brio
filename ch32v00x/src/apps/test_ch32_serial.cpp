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
// 4. NO LETTER IS A SELF-LOOP, but a timer capture of a start bit is the
//    DIVISOR, not the rate: a divisor computed wrong would measure itself
//    exactly. So letters b, c, i and t time a run of frames against its
//    wire time at the rate ASKED, in thousandths, judged -1 % to +3 %.
//    The receiving letters are fed by the banger, whose edges the cycle
//    count places at the rate asked, and the receiver divides by the BRR
//    letter c times.
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
//      parities, the four stop lengths - and each format's run of 96
//      frames timed against its bits at 9600; and the single wire as a
//      bus, its run timed the same way
//   c  the baud generator: a start bit measured to the cycle at eight
//      rates from 2400 to 3 Mbaud (the bit IS the divisor), the
//      fractional divisor, and at each rate A RUN OF FRAMES timed against
//      its wire at the rate asked - the start bit alone is the divisor,
//      which a wrong one would match; the receiver at 9600, the refusal
//      below 16
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
//      engine fed from the banged line, the transmit engine's runs timed
//      against the wire at 9600, 115200 and 3 Mbaud
//   j  the flags and the vector: TC after TXE, IDLE once per line idle,
//      one interrupt per enable
//   q  ERRORS UNDER THE RECEIVE ENGINE: breaks banged into a continuous
//      stream into USART2's ring - every data byte delivered intact and
//      in order, no byte taken by a clear, the frame errors counted one a
//      break; back-to-back breaks counting every other one
//      (ch32v00x/usart.hpp's header); the interrupt receiver's breaks,
//      dropped, counted and skipped - a reader that looks between them
//      handed every data byte, one that does not handed nothing the ring
//      held with a break behind it
//   r  tx_idle() IS THE WIRE'S: the moment it turns true against the last
//      stop bit's start on the jumper (TIM2's channel 1 capturing PD4),
//      for the interrupt transmitter and the transmit engine
//   s  THE BURST EDGE FROM THE VECTOR, nothing polled, on banged bursts:
//      one frame told, sixteen told within two frames of the last stop
//      bit with two interrupts a burst and none a byte, four laps of the
//      ring with no silence read whole on the lap's marks
//   t  the transport's four rate verbs: can_baud() and min_hz_for() at
//      the divisor's two ends, set_baud() moving a live port (the start
//      bit measured on the jumper before and after, a run through the
//      interrupt transmitter timed against the wire at each rate) and
//      refusing an unreachable rate, release() giving the instance and
//      its pads back
//   u  AN OVERRUN AT A BURST'S TAIL under the receive engine: a fill on
//      channel 1 at the ring's own level starving the ring under a banged
//      burst's last three frames, the vector held off until the channel
//      took the frame DATAR held - ORE up with RXNE down, no frame after
//      it: the vector entered a handful of times, no channel given up, the
//      overrun and the gap counted once each; the next burst whole
//   y  (outside z, host-assisted: brio stress) the CONSOLE's own error
//      counters, provoked from the host side
//   k  THE SYNCHRONOUS MODE, on the console's own USART1 (both parts):
//      on the CH32V003 the CK pad PD4 carries a clock pulse per data bit
//      while the console prints, counted by TIM2's channel 1 off the
//      same pad with NO WIRE - seven pulses a byte, eight with LBCL,
//      the idle level CPOL's; on the CH32V006 the verb refuses (the
//      bits are reserved there)
//
// THE TWO-BOARD KEY `w`, letters 1 to 8, outside z: USART2 on its column
// 4 (TX PB0, RX PB1) as the DUT, the desk's link to a CH32X035 running
// usart_peer - driven in band over the link itself (usart_link.hpp), every
// leg judged from what the far chip reports or from what this end
// received, FAILING loudly when nobody answers; a leg the peer REFUSES is
// SKIPPED with the reason. The far chip's only clock is an RC, so letter 1
// calibrates it against this module's 24 MHz crystal, and every letter
// calibrates it again first (its HSI wanders by a few hundred ppm in a
// second):
//   1  the rendezvous - ping, IDENT (kind 0x07), a bad checksum, a desync,
//      the refusals - and THE CALIBRATION: gapless runs at 1 Mbaud until
//      the peer moves its trim to the step nearest 48 MHz and adopts the
//      rate it measures there; its HSI as found, and run to run after
//   2  the echo ladder 9600 to 3 Mbaud, and at each rate a gapless run
//      timed by the far receiver against a 1 Mbaud run back to back:
//      HCLK over BRR; the peer's blast stating its own rate
//   3  the frame matrix echoed, the half and the one-and-a-half stop as
//      lengths, the peer's refusals of what neither chip has
//   4  PE, the rate tolerance from the calibrated peer's edge engine,
//      glitches on the idle line and NE, ORE from a run read late, LIN's
//      break detection at both lengths
//   5  the peer's engine shapes, one stretched cell against this end's own
//      pull-banger on the same wire, a LIN break ahead of a frame
//   6  the receive engine on channel 7 under the peer's blast, about a
//      second at 1, 2 and 3 Mbaud, every byte in sequence
//   7  half duplex on PB0 hearing the peer's drive_rxd, and the wired AND
//      of an open-drain wire under its collide
//   8  mute by address mark and by idle line, IrDA between two WCH codecs
//
// THE CH32V006K8's 40 KB HOLD THE SUITE AS TWO IMAGES (the build's group
// axis): -1 the single-board letters, -2 the two-board key. -2 runs on the
// module's crystal, its reference; -1 on the HSI's PLL, because the
// crystal's pads are PA1 and PA2 and letter h's RTS is PA1 (the clock's
// comment below).
//
// THE CH32V003 BUILD carries letter k alone (one group image): its
// USART is the console's, there being no second instance to be the
// instrument, so every letter that names USART2 is compiled out there.
//
// build: boards = v006k8,v003f4
// build: groups = k
// build: groups.v006k8 = abcdefghijkqrstuy,12345678
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

#include "../../../avrdx/src/apps/usart_link.hpp"

using P = brio::Ch32v00xPlatform<>;
/// THE CLOCK BY THE LETTERS THE IMAGE CARRIES. The two-board key's
/// reference is the module's 24 MHz crystal, doubled; the single-board
/// letters run on the HSI's PLL at the same 48 MHz, because the crystal's
/// pads are PA1 and PA2 and letter h's RTS is PA1 (column 3): driving it
/// stops the oscillator and HCLK with it (measured: the core froze mid-
/// print). So an image that carries a digit letter starts the crystal and
/// one that does not never touches it; letter h refuses to run on the
/// crystal, and every divisor and wait is the same either way.
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000, 24'000'000>;
using HsiClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
static_assert(SysClock::hz == HsiClock::hz && SysClock::pclk_hz == HsiClock::pclk_hz);
constexpr SysClock clock;
constexpr bool two_board_image =
    BRIO_CH32_HAS_USART2 && (brio::test_letters == nullptr || brio::digit_mask_of(brio::test_letters) != 0u);

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;

TwoKeyTestBench<Serial, 'w', 26> bench;
bool crystal_ok = false;

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

/// A cycle count that spans ticks: the kernel ticker's own cycles(), the
/// tick count and the counter's position composed right across the reload
/// (util/cycle_count.hpp); it wraps at 2^32, so only differences are used.
uint32_t cycles_now() {
    return Ticker::cycles();
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

// ---- the rate on the wire ----------------------------------------------------
//
// A START BIT IS BRR CYCLES TO THE CYCLE (letter c), and that is the
// divisor, not the rate: a divisor computed wrong would measure itself
// exactly and pass. So every letter that sets a rate also times A RUN of
// frames - from the first store into DATAR (or into the transport) to TC,
// on the ticker's cycle count - against the run's wire time AT THE RATE
// ASKED, HCLK over the baud the letter named, and reads it in thousandths.
// The window is -1 % to +3 %: BRR is HCLK over the baud rounded to the
// cycle, which costs at most half a cycle a sixteenth of a bit - 1.6
// thousandths at 460800 and 921600 (104 and 52 for 104.2 and 52.1), 0.8
// at 115200 (417 for 416.7), nothing at the other rates of letter c -
// and the run's fixed cost, a store and a poll, stays under a thousandth.

/// Frames enough for a hundredth of a second of wire, sixteen at least
/// and 1024 at most (3.4 ms at 3 Mbaud, 160 cycles a frame).
uint32_t run_frames(uint32_t baud) {
    const uint32_t n = baud / 100u;
    return n < 16u ? 16u : (n > 1024u ? 1024u : n);
}

/// 1000 x num / den, exact in 32 bits for a den under 429 M: three
/// decimal digits of long division. The suite does no 64-bit division -
/// libgcc's costs a kilobyte this image has not got to spare.
uint32_t permille_ratio(uint32_t num, uint32_t den) {
    if (den == 0u) {
        return 0u;
    }
    uint32_t q = num / den;
    uint32_t r = num % den;
    for (uint8_t k = 0; k < 3u; ++k) {
        r *= 10u;
        q = q * 10u + r / den;
        r %= den;
    }
    return q;
}

/// The wire time of `n` frames of `halves` half bits at `baud`, in HCLK
/// cycles: the half bit taken in 32nds of a cycle (HCLK x 16 / baud,
/// 768 M over the baud) so the product stays in 32 bits for a run of up
/// to two seconds; the truncation is under a thousandth of a cycle a
/// half bit.
uint32_t wire_cycles(uint32_t n, uint32_t halves, uint32_t baud) {
    const uint32_t half_x32 = SysClock::hz * 16u / baud;
    return (n * halves * half_x32) >> 5;
}

/// `took` cycles against `n` frames of `halves` half bits at `baud`.
uint32_t permille_of(uint32_t took, uint32_t n, uint32_t halves, uint32_t baud) {
    return permille_ratio(took, wire_cycles(n, halves, baud));
}

/// What the divisor in force gives against the rate asked, in thousandths:
/// the reading a correct generator must show (BRR x baud / HCLK; the rates
/// asked here are whole hundreds).
uint32_t divisor_permille(uint32_t baud) {
    return permille_ratio(U2::brr() * (baud / 100u), SysClock::pclk_hz / 100u);
}

bool on_the_wire(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

/// The bare resource: `n` words polled into DATAR on TXE - the data
/// register one frame ahead of the shifter, so the line never idles - and
/// TC waited; the frame `halves` half bits long.
uint32_t resource_permille(uint32_t baud, uint32_t halves, uint32_t n) {
    U2::clear_flags(usart_tc);
    const uint32_t budget = 4u * wire_cycles(n, halves, baud);
    const uint32_t t0 = cycles_now();
    for (uint32_t i = 0; i < n; ++i) {
        while (!U2::tx_empty() && cycles_now() - t0 < budget) {
        }
        U2::write_word(static_cast<uint16_t>(0x55u + i));
    }
    while (!U2::tx_complete() && cycles_now() - t0 < budget) {
    }
    return permille_of(cycles_now() - t0, n, halves, baud);
}

/// The same through a transport: `n` 8N1 frames handed to write_bulk()
/// as fast as it takes them, until tx_idle() - the wire's (letter r).
template <typename T>
uint32_t transport_permille(uint32_t baud, uint32_t n) {
    static uint8_t frames[64];
    for (uint32_t i = 0; i < sizeof(frames); ++i) {
        frames[i] = static_cast<uint8_t>(0x30u + i);
    }
    const uint32_t budget = 4u * wire_cycles(n, 20u, baud);
    const uint32_t t0 = cycles_now();
    uint32_t q = 0;
    while (q < n && cycles_now() - t0 < budget) {
        const uint32_t at = q % sizeof(frames);
        const uint32_t len = sizeof(frames) - at < n - q ? sizeof(frames) - at : n - q;
        q += T::write_bulk(std::span<const uint8_t>(frames + at, len));
    }
    while (!T::tx_idle() && cycles_now() - t0 < budget) {
    }
    return permille_of(cycles_now() - t0, n, 20u, baud);
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
/// framing error. `glitch_bit`, when not 0xFF, names the data bit whose
/// line is flipped over a part of it: twelve 128ths from the 53rd, or -
/// with `glitch_all` or'ed in - the 40th to the 84th (letter d says why
/// those two).
constexpr uint8_t glitch_all = 0x80;
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
        if ((fr.glitch_bit & ~glitch_all) == i) {
            const bool all = (fr.glitch_bit & glitch_all) != 0u;
            spin_until(t0 + n * bit + (bit * (all ? 40u : 53u)) / 128u);
            line(!level);
            spin_until(t0 + n * bit + (bit * (all ? 84u : 65u)) / 128u);
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
    // format, the parity computed the format's way. Before it, the
    // format's run on the wire: 96 frames at 9600 against their bits -
    // the start, the word (the parity bit inside it, 14.8.4) and the
    // stop length - at the rate asked.
    uint8_t exact = 0;
    uint8_t timed = 0;
    for (const Fmt& fmt : formats) {
        all_off();
        (void)resource_up(fmt.f, 9600);
        const uint32_t word = static_cast<uint32_t>(fmt.f.bits == UartBits::seven ? 7u
                                                    : fmt.f.bits == UartBits::eight ? 8u
                                                                                    : 9u) +
                              (fmt.f.parity != UartParity::none ? 1u : 0u);
        const uint32_t stop_halves = fmt.f.stop == UartStop::half           ? 1u
                                     : fmt.f.stop == UartStop::one          ? 2u
                                     : fmt.f.stop == UartStop::one_and_half ? 3u
                                                                            : 4u;
        const uint32_t halves = 2u * (1u + word) + stop_halves;
        const uint32_t pm = resource_permille(9600, halves, run_frames(9600));
        print(serial, "  ", fmt.name, " on the wire: 96 frames of ", halves / 2u, halves % 2u != 0u ? ".5" : "",
              " bits in ", pm, " thousandths of their time at 9600", on_the_wire(pm) ? "" : "  OFF", crlf);
        if (on_the_wire(pm)) {
            ++timed;
        }
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
    bench.verdict("and every format's run takes its own bits at the rate asked: 96 frames at 9600 within -1 % to +3 % "
                  "of their wire time, the half and the one-and-a-half stop included",
                  timed == 10u);
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
        // Its rate on the wire: the single wire's transport set to 9600,
        // 96 frames through it.
        const uint32_t pm = transport_permille<WireUart>(9600, run_frames(9600));
        print(serial, "  the single-wire transport: a frame driven on the wire ", heard ? "heard" : "NOT heard", " (",
              hex(b), "); its own frame ", echoed ? "ECHOED" : "not echoed", "; 96 frames out in ", pm,
              " thousandths of their wire time at 9600", crlf);
        bench.verdict("the half-duplex option hears a frame driven onto its wire and not its own - a bus, not a loop",
                      up && heard && b == 0xC3u && !echoed);
        bench.verdict("and drives its wire at the rate asked: 96 frames at 9600 within -1 % to +3 % of their wire time",
                      on_the_wire(pm));
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
    // And a run of frames at each rate against its wire time at the rate
    // ASKED - the start bit is the divisor, which a wrong one would match.
    if (need_jumper()) {
        uint8_t exact = 0;
        uint8_t timed = 0;
        for (const uint32_t baud : rates) {
            all_off();
            const bool ok_up = resource_up({}, baud);
            const uint32_t brr = usart_divisor(SysClock::pclk_hz, baud);
            const uint32_t actual = U2::actual_baud(SysClock::pclk_hz);
            ruler_arm(true);
            // TC stands from TE's idle frame: a DATAR write alone does not
            // clear it (14.8.1: a STATR read then a DATAR write, or a zero
            // written), so it is cleared here and tx_idle() waits for this
            // frame's end - measured: left standing, the run behind it
            // queued behind the rest of this frame, 37 thousandths over at
            // 2400.
            U2::clear_flags(usart_tc);
            U2::write_data(0xFF);
            const auto bit = ruler_read(50'000);
            (void)tx_idle();
            const bool ok = ok_up && bit && *bit == brr;
            if (ok) {
                ++exact;
            }
            const uint32_t n = run_frames(baud);
            const uint32_t pm = resource_permille(baud, 20u, n);
            const uint32_t own = divisor_permille(baud);
            if (ok_up && on_the_wire(pm)) {
                ++timed;
            }
            print(serial, "  ", baud, " baud: BRR ", brr, " (", brr / 16u, " and ", brr % 16u, "/16), actual ", actual,
                  ", the start bit ", bit ? *bit : 0u, " cycles", ok ? "" : "  OFF", "; ", n, " frames in ", pm,
                  " thousandths of their wire time (the divisor's own ", own, ")", on_the_wire(pm) ? "" : "  OFF",
                  crlf);
        }
        bench.verdict("the start bit measures exactly BRR cycles at eight rates from 2400 to 3 Mbaud - the fractional "
                      "divisor included (76800 = 39 and 1/16)",
                      exact == 8u);
        bench.verdict("and the line runs at the rate ASKED: at each of the eight rates a run of frames takes its wire "
                      "time within -1 % to +3 % (BRR's rounding 1.6 thousandths at most)",
                      timed == 8u);
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

    // A glitch inside a data bit: noise. The receiver takes three samples
    // a sixteenth of a bit apart, and the three move together by up to a
    // sixteenth from frame to frame - the start edge is found on a tick of
    // its 16x clock, which runs free of the line - and the banger's edges
    // land a spin of the cycle count late, some tens of cycles (measured:
    // the first sample between 46 and 56 128ths of the bit). So how many
    // samples a glitch covers is a matter of chance unless it is none or
    // all: one wider than a sixteenth and narrower than two (twelve
    // 128ths, from the 53rd) covers one or two whatever the phase - they
    // disagree, NE, and the bit is the majority's - and one over the three's
    // whole span (40th to 84th) flips the bit with no NE. The console is
    // drained first: its vector entered under a banged edge moves it.
    while (!Serial::tx_idle()) {
    }
    Received noisy[2];
    for (uint8_t k = 0; k < 2u; ++k) {
        all_off();
        (void)resource_up({}, 2400);
        bang_idle(2400, 4);
        bang_frame({.data = 0x0F, .glitch_bit = static_cast<uint8_t>(k == 0u ? 2u : 2u | glitch_all)});
        noisy[k] = receive_one();
    }
    print(serial, "  noise on bit 2 of 0x0F, one or two samples: ", hex(noisy[0].word), " STATR ", hex(noisy[0].status),
          "; all three: ", hex(noisy[1].word), " STATR ", hex(noisy[1].status), crlf);
    bench.verdict("a glitch over one or two of the three samples raises NE, the bit the majority's",
                  noisy[0].came && (noisy[0].status & (usart_ne | usart_fe)) == usart_ne &&
                      (noisy[0].word == 0x0Fu || noisy[0].word == 0x0Bu));
    bench.verdict("a glitch over all three flips the bit with no NE",
                  noisy[1].came && (noisy[1].status & (usart_ne | usart_fe)) == 0u && noisy[1].word == 0x0Bu);

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
    if (crystal_ok) {
        print(serial, "  SKIPPED, no verdict claimed: RTS is PA1, the 24 MHz crystal's pad, and the crystal runs "
                      "HCLK in this image", crlf);
        return;
    }
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
    // The receive engine fed sixteen banged frames, then read: the bytes
    // are in the channel's ring as they land, nothing published.
    bang_idle(9600, 4);
    uint32_t seed = 0x12345678u;
    for (uint8_t i = 0; i < 16u; ++i) {
        bang_frame({.data = static_cast<uint16_t>(xorshift_step(seed) & 0xFFu), .baud = 9600});
    }
    settle_ms(2);
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
    print(serial, "  the receive engine on channel 7: ", got, " bytes read, ", good, " exact, faults ",
          DmaUart::dma_faults(), crlf);
    bench.verdict("USART2's receive engine on channel 7 delivers sixteen banged frames byte-exact into its ring",
                  up && got == 16u && good == 16u && DmaUart::dma_faults() == 0u);
    // The transmit engine: 256 bytes queued at once, the run timed on the
    // cycle count - 2560 bits at 9600 is 266.7 ms - against the wire at the
    // rate asked, and the ring empty at the end.
    const uint32_t t0 = cycles_now();
    uint32_t queued = 0;
    for (uint16_t i = 0; i < 256u; ++i) {
        if (DmaUart::write_byte(static_cast<uint8_t>(i))) {
            ++queued;
        }
    }
    while (!DmaUart::tx_idle() && cycles_now() - t0 < SysClock::hz) {
    }
    const uint32_t pm = permille_of(cycles_now() - t0, 256u, 20u, 9600u);
    print(serial, "  the transmit engine on channel 6: ", queued, " bytes queued, the run in ", pm,
          " thousandths of its wire time at 9600, faults ", DmaUart::dma_faults(), crlf);
    bench.verdict("USART2's transmit engine on channel 6 moves a 256-byte ring in one run at the wire's pace: "
                  "2560 bits at 9600 within -1 % to +3 %",
                  queued == 256u && on_the_wire(pm) && DmaUart::dma_faults() == 0u);
    // And at the rates the engine is for: the port moved live, 1024 frames
    // through the 256-byte ring - four laps, a block restarted from the
    // channel's vector at each - at 115200 and at 3 Mbaud.
    const bool moved = DmaUart::set_baud(SysClock::pclk_hz, 115200);
    const uint32_t pm_115 = transport_permille<DmaUart>(115200, 1024);
    const bool moved_3m = DmaUart::set_baud(SysClock::pclk_hz, 3'000'000);
    const uint32_t pm_3m = transport_permille<DmaUart>(3'000'000, 1024);
    print(serial, "  1024 frames through the engine: ", pm_115, " thousandths of their wire time at 115200, ", pm_3m,
          " at 3 Mbaud; faults ", DmaUart::dma_faults(), crlf);
    bench.verdict("and at 115200 and 3 Mbaud, 1024 frames through the ring within -1 % to +3 % of their wire time",
                  moved && moved_3m && on_the_wire(pm_115) && on_the_wire(pm_3m) && DmaUart::dma_faults() == 0u);
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
// q, r, s - the transport's promises: errors under the engine, tx_idle()
// on the jumper, the burst edge
// ===========================================================================

using PlainUart = Uart<2, P, 256, 64, NoDmaEngine, NoDmaEngine, column>;
volatile uint32_t u2_edges = 0;
volatile uint32_t u2_edge_at = 0;

bool dma_edge_isr() {
    const bool e = DmaUart::isr();
    if (e) {
        u2_edge_at = cycles_now();
        u2_edges = u2_edges + 1u;
    }
    return e;
}
bool plain_edge_isr() {
    const bool e = PlainUart::isr();
    if (e) {
        u2_edges = u2_edges + 1u;
    }
    return e;
}

constexpr uint8_t stream_byte(uint32_t i) { return static_cast<uint8_t>(i * 151u + 7u); }

template <typename T>
bool transport_on(IsrFn fn, uint32_t baud) {
    all_off();
    Dma::open();
    u2_isr = fn;
    const bool up = T::init(clock, baud);
    RxPad::input(PinPull::up);   // the banged line, over the transport's floating input
    Pfic::enable(dma_channel_irq(6));
    Pfic::enable(dma_channel_irq(7));
    settle_ms(3);
    uint8_t b = 0;
    while (T::read_byte(b)) {
    }
    T::clear_errors();
    u2_edges = 0;
    u2_interrupts = 0;
    return up;
}

void tq_errors() {
    if (!transport_on<DmaUart>(&dma_edge_isr, 9600)) {
        bench.verdict("USART2 with both engines comes up", false);
        all_off();
        return;
    }
    // A continuous stream: frames back to back, a banged break after every
    // eighth, each break followed by a clean frame.
    constexpr uint32_t n = 64;
    uint32_t breaks = 0;
    bang_idle(9600, 2);
    for (uint32_t i = 0; i < n; ++i) {
        bang_frame({.data = stream_byte(i), .baud = 9600});
        if (i % 8u == 7u && i + 1u < n) {
            bang_break(9600, 12);
            bang_idle(9600, 1);   // the break's own stop bit: a start bit wants a high line before it
            ++breaks;
        }
    }
    settle_ms(3);
    uint32_t data = 0;
    uint32_t zeros = 0;
    bool in_order = true;
    for (;;) {
        const auto run = DmaUart::read_span();
        if (run.empty()) {
            break;
        }
        for (const uint8_t b : run) {
            if (b == stream_byte(data)) {
                ++data;
            } else if (b == 0u) {
                ++zeros;
            } else {
                in_order = false;
            }
        }
        (void)DmaUart::consume(static_cast<uint32_t>(run.size()));
    }
    const uint16_t fe = DmaUart::frame_errors();
    print(serial, "  ", n, " data bytes and ", breaks, " breaks through the receive engine: ", data,
          " in order, ", zeros, " break frames; FE ", fe, ", ORE ", DmaUart::hw_overruns(), crlf);
    bench.verdict("every data byte delivered, intact and in order: no byte taken by a clear",
                  data == n && in_order);
    bench.verdict("each break stored as the frame it is (0x00)", zeros == breaks);
    bench.verdict("the frame errors counted one a break", fe == breaks);

    uint16_t counts[2] = {0, 0};
    for (uint8_t k = 0; k < 2u; ++k) {
        (void)transport_on<DmaUart>(&dma_edge_isr, 9600);
        bang_idle(9600, 2);
        bang_frame({.data = 0x11, .baud = 9600});
        bang_frame({.data = 0x12, .baud = 9600});
        for (uint8_t i = 0; i < k + 2u; ++i) {
            bang_break(9600, 12);
            bang_idle(9600, 1);   // the break's own stop bit: a start bit wants a high line before it
        }
        bang_frame({.data = 0x22, .baud = 9600});
        bang_frame({.data = 0x23, .baud = 9600});
        settle_ms(3);
        counts[k] = DmaUart::frame_errors();
    }
    print(serial, "  back to back: two breaks count ", counts[0], ", three count ", counts[1], crlf);
    bench.verdict("back-to-back breaks count every other one (two count 1, three count 2)",
                  counts[0] == 1u && counts[1] == 2u);

    // The interrupt receiver drops each break's frame, counts it and tells
    // its ring, whose next look skips what it holds: read between the
    // breaks, every data byte is handed out ...
    (void)transport_on<PlainUart>(&plain_edge_isr, 9600);
    breaks = 0;
    bang_idle(9600, 2);
    uint32_t back = 0;
    bool ok = true;
    uint8_t b = 0;
    auto read_all = [&] {
        settle_ms(1);
        while (PlainUart::read_byte(b)) {
            if (b != stream_byte(back)) {
                ok = false;
            }
            ++back;
        }
    };
    const uint32_t skips0 = PlainUart::rx_skips();
    for (uint32_t i = 0; i < 24u; ++i) {
        bang_frame({.data = stream_byte(i), .baud = 9600});
        if (i % 4u == 3u) {
            read_all();   // the four before the break
            bang_break(9600, 12);
            bang_idle(9600, 1);   // the break's own stop bit: a start bit wants a high line before it
            ++breaks;
            read_all();   // the skip, with nothing queued
        }
    }
    const uint32_t skips = (PlainUart::rx_skips() - skips0) & 0xFFu;
    print(serial, "  the interrupt receiver, read between: 24 bytes and ", breaks, " breaks, ", back,
          " delivered, FE ", PlainUart::frame_errors(), ", skips ", skips, crlf);
    bench.verdict("the interrupt receiver drops each break's frame, counts it and skips it: a reader that "
                  "looks between the breaks is handed every data byte",
                  back == 24u && ok && PlainUart::frame_errors() == breaks && skips == breaks);
    // ... and read after, nothing the ring held with a break behind it:
    // no run joins the two sides.
    const uint32_t skips1 = PlainUart::rx_skips();
    for (uint32_t i = 0; i < 8u; ++i) {
        bang_frame({.data = stream_byte(24u + i), .baud = 9600});
        if (i == 3u) {
            bang_break(9600, 12);
            bang_idle(9600, 1);
        }
    }
    settle_ms(3);
    uint32_t joined = 0;
    while (PlainUart::read_byte(b)) {
        ++joined;
    }
    const uint32_t skips_after = (PlainUart::rx_skips() - skips1) & 0xFFu;
    print(serial, "  read after: 8 bytes around a break, ", joined, " delivered, FE ",
          PlainUart::frame_errors(), ", skips ", skips_after, crlf);
    bench.verdict("bytes queued with a break among them are skipped whole, in one skip",
                  joined == 0u && skips_after == 1u && PlainUart::frame_errors() == breaks + 1u);
    all_off();
}

/// Four 0x55 frames through `T` (bit 7 low: the stop bit starts with a
/// rising edge), TIM2's channel 1 latching PD4's last rising edge off the
/// jumper; the timer counts from it to tx_idle()'s first true, the count
/// read after the answer (never early, a poll's few cycles late).
template <typename T>
int32_t idle_after_stop(IsrFn fn, uint32_t baud) {
    static const uint8_t frames[4] = {0x55, 0x55, 0x55, 0x55};
    (void)transport_on<T>(fn, baud);
    RulerPad::input();
    T2::init();
    (void)T2::configure({.prescaler = 0, .period = 0xFFFF});
    (void)T2::capture_channel(0, {.select = TimChannelSelect::direct, .polarity = TimCapturePolarity::rising});
    T2::clear_flags(T2::compare_flag(0));
    T2::enable(true);
    (void)T::write_bulk(std::span<const uint8_t>(frames, sizeof(frames)));
    uint32_t t_idle = 0;
    const uint32_t t0 = cycles_now();
    for (;;) {
        if (T::tx_idle()) {
            t_idle = T2::count();
            break;
        }
        if (cycles_now() - t0 > 20'000u * cycles_per_us) {
            break;
        }
    }
    const uint32_t edge = T2::compare(0);
    all_off();
    return static_cast<int32_t>((t_idle - edge) & 0xFFFFu);
}

void tr_tx_idle() {
    if (!need_jumper()) {
        print(serial, "  the jumper PD2-PD4 is ABSENT: tx_idle() on the pad measures nothing", crlf);
        bench.verdict("the letter declines without its jumper, and says so", true);
        return;
    }
    // 250000 and not a megabaud: at 48 MHz a bit of 1 Mbaud is 48 cycles,
    // less than one turn of the poll with a tick landing in it, so the
    // instrument could not say "within a bit" there.
    static constexpr uint32_t rates[] = {115200, 250'000};
    for (const uint32_t baud : rates) {
        const uint32_t bit = SysClock::hz / baud;
        const int32_t d_plain = idle_after_stop<PlainUart>(&plain_edge_isr, baud);
        const int32_t d_dma = idle_after_stop<DmaUart>(&dma_edge_isr, baud);
        print(serial, "  ", baud, " baud, a bit ", bit, " timer counts: tx_idle() true ", d_plain,
              " counts after the stop bit's rising edge (interrupt transmitter), ", d_dma,
              " (transmit engine)", crlf);
        const int32_t lo = static_cast<int32_t>(bit);
        const int32_t hi = static_cast<int32_t>(2u * bit);
        bench.verdict("tx_idle() never before the last stop bit is out, within a bit after it: ",
                      baud == 115200 ? "115200" : "250000",
                      d_plain >= lo && d_plain <= hi && d_dma >= lo && d_dma <= hi);
    }
}

/// Everything the engined transport holds, against the stream from `from`.
uint32_t dma_read(uint32_t from, bool& in_order) {
    uint32_t got = 0;
    for (;;) {
        const auto run = DmaUart::read_span();
        if (run.empty()) {
            return got;
        }
        for (uint32_t i = 0; i < run.size(); ++i) {
            if (run[i] != stream_byte(from + got + i)) {
                in_order = false;
            }
        }
        got += static_cast<uint32_t>(run.size());
        (void)DmaUart::consume(static_cast<uint32_t>(run.size()));
    }
}

void ts_edge() {
    const bool up = transport_on<DmaUart>(&dma_edge_isr, 9600);
    const uint32_t frame = 10u * SysClock::hz / 9600u;
    bool all_told = true;
    bang_idle(9600, 12);
    for (uint8_t k = 0; k < 3u; ++k) {
        bool ok = true;
        (void)dma_read(0, ok);
        const uint32_t e0 = u2_edges;
        bang_frame({.data = 0x42, .baud = 9600});
        bang_idle(9600, 24);
        const uint32_t e1 = u2_edges;
        uint8_t b = 0;
        uint32_t got = 0;
        while (DmaUart::read_byte(b)) {
            ++got;
        }
        print(serial, "  a burst of one frame: ", e1 - e0, " edge(s), ", got, " byte read", crlf);
        if (e1 == e0 || got != 1u) {
            all_told = false;
        }
    }
    bench.verdict("a burst of one frame is told, every time", up && all_told);

    // Sixteen frames: the first frame's edge drained at once, as a
    // consumer would; the last edge timed from the end of the last stop
    // bit, which the banging thread knows to the cycle.
    bool ok = true;
    u2_interrupts = 0;
    const uint32_t e0 = u2_edges;
    uint32_t got = 0;
    for (uint32_t i = 0; i < 16u; ++i) {
        bang_frame({.data = stream_byte(i), .baud = 9600});
        if (u2_edges != e0) {
            got += dma_read(got, ok);
        }
    }
    const uint32_t t_end = cycles_now();
    const uint32_t before = u2_edges;
    while (u2_edges == before && cycles_now() - t_end < 5u * frame) {
    }
    const uint32_t late = u2_edge_at - t_end;
    settle_ms(2);
    got += dma_read(got, ok);
    print(serial, "  16 frames: ", u2_edges - e0, " edge(s), the last ", late,
          " cycles after the last stop bit (", late * 10u / frame, " tenths of a frame); ", u2_interrupts,
          " USART interrupt(s); ", got, " read", crlf);
    bench.verdict("the burst read whole, nothing polled", got == 16u && ok);
    bench.verdict("its edge within two frame times of the last stop bit",
                  u2_edges != before && late <= 2u * frame);
    bench.verdict("at most two USART interrupts a burst, none a byte",
                  u2_interrupts >= 1u && u2_interrupts <= 2u);

    // Four laps of the 256-byte ring with no silence: the marks tell the
    // consumer twice a lap, and it drains between two banged frames.
    (void)dma_read(0, ok);
    DmaUart::clear_errors();
    uint32_t read = 0;
    ok = true;
    uint32_t seen = u2_edges;
    for (uint32_t i = 0; i < 1024u; ++i) {
        bang_frame({.data = stream_byte(i), .baud = 9600});
        if (u2_edges != seen) {
            seen = u2_edges;
            read += dma_read(read, ok);
        }
    }
    settle_ms(3);
    read += dma_read(read, ok);
    print(serial, "  four laps without silence: ", read, " of 1024 read on the edges, laps missed ",
          DmaUart::rx_overruns(), crlf);
    bench.verdict("a stream with no silence is read whole on the lap's marks",
                  read == 1024u && ok && DmaUart::rx_overruns() == 0u);
    all_off();
}

// ===========================================================================
// t - set_baud(), can_baud(), min_hz_for(), release()
// ===========================================================================

void tt_rate_verbs() {
    // 14.3: BRR is the peripheral clock over the baud, sixteen at least
    // and 0xFFFF at most.
    constexpr uint32_t hz = SysClock::pclk_hz;
    const bool ends = PlainUart::can_baud(hz, 3'000'000) && !PlainUart::can_baud(hz, 3'200'000) &&
                      PlainUart::can_baud(hz, 733) && !PlainUart::can_baud(hz, 732) &&
                      PlainUart::min_hz_for(115200) == 16u * 115200u;
    print(serial, "  can_baud at 48 MHz: 3 Mbaud ", PlainUart::can_baud(hz, 3'000'000) ? "yes" : "no",
          ", 733 ", PlainUart::can_baud(hz, 733) ? "yes" : "no", ", 732 ", PlainUart::can_baud(hz, 732) ? "yes" : "no",
          "; min_hz_for(115200) ", PlainUart::min_hz_for(115200), crlf);
    bench.verdict("can_baud() and min_hz_for() answer the divisor's two ends (16 and 0xFFFF)", ends);
    if (!need_jumper()) {
        return;
    }
    (void)transport_on<PlainUart>(&plain_edge_isr, 115200);
    static const uint8_t ff[1] = {0xFF};
    ruler_arm(true);
    (void)PlainUart::write_bulk(std::span<const uint8_t>(ff, 1));
    const auto before = ruler_read(50'000);
    while (!PlainUart::tx_idle()) {
    }
    // The run at the rate init() set, through the interrupt transmitter.
    const uint32_t pm_before = transport_permille<PlainUart>(115200, run_frames(115200));
    const bool moved = PlainUart::set_baud(hz, 9600);
    const bool refused = !PlainUart::set_baud(hz, 4'000'000);
    ruler_arm(true);
    (void)PlainUart::write_bulk(std::span<const uint8_t>(ff, 1));
    const auto after = ruler_read(50'000);
    while (!PlainUart::tx_idle()) {
    }
    const uint32_t pm_after = transport_permille<PlainUart>(9600, run_frames(9600));
    print(serial, "  the start bit at 115200: ", before ? *before : 0u, " cycles; after set_baud(9600): ",
          after ? *after : 0u, " cycles (BRR ", usart_divisor(hz, 9600), ")", crlf);
    print(serial, "  the runs through the interrupt transmitter: 1024 frames in ", pm_before,
          " thousandths of their wire time at 115200, then 96 in ", pm_after, " at 9600", crlf);
    bench.verdict("set_baud() moves a live port: the start bit is the new divisor",
                  moved && before && *before == usart_divisor(hz, 115200) && after &&
                      *after == usart_divisor(hz, 9600));
    bench.verdict("and the line runs at each rate asked: a run at 115200 before the move and at 9600 after it, "
                  "each within -1 % to +3 % of its wire time",
                  on_the_wire(pm_before) && on_the_wire(pm_after));
    bench.verdict("and refuses an unreachable rate, the divisor left as it was",
                  refused && U2::brr() == usart_divisor(hz, 9600));
    PlainUart::release();
    U2::bus_clock(true);   // the gate reopened to read what release() left
    const bool gone = !U2::enabled() && U2::regs().CTLR1 == 0u;
    print(serial, "  release(): CTLR1 ", hex(U2::regs().CTLR1), crlf);
    bench.verdict("release() stops the port: UE and every enable clear", gone);
    all_off();
}

// ===========================================================================
// u - an overrun at a burst's tail under the receive engine: no storm
// ===========================================================================

/// THE STAGING. USART2's ring runs on channel 7 at very_high (the Uart's
/// default); a fill on channel 1 at the same level wins every tie (8.2.1)
/// and starves it. Under the mask: four frames banged at 115200 baud
/// and taken, then the fill started and three more frames - the first
/// lands in DATAR with nothing to take it, the next overrun it - and the
/// fill's end lets the channel take the frame DATAR held; the line then
/// idles. ORE stands with RXNE down and no frame after it, and the vector
/// first looks only now: the state ch32v00x/usart.hpp's rx_overrun_probe
/// answers. A storm would show as the vector entered hundreds of times
/// and a channel given up.
using Starver = DmaCopyEngine<1, uint8_t>;
constexpr uint32_t storm_baud = 115200;

/// A masked stretch begins right after a tick: the bit-banger's clock,
/// cycles_now(), composes the tick count with STK's position, and with the
/// tick's handler held off it stays right for two periods after the last
/// tick counted and no longer - every masked stretch here is under 1.2 ms.
void after_a_tick() {
    const uint32_t m = Ticker::millis();
    while (Ticker::millis() == m) {
    }
}

void tu_overrun_tail() {
    static uint8_t fill_dst[2048];
    static const uint8_t fill_cell = 0x5Au;
    if (!transport_on<DmaUart>(&dma_edge_isr, storm_baud)) {
        bench.verdict("USART2 with both engines comes up", false);
        all_off();
        return;
    }
    Starver::arm(DmaPriority::very_high, false);
    const uint16_t hw0 = DmaUart::hw_overruns();
    const uint16_t f0 = DmaUart::dma_faults();
    const uint32_t k0 = DmaUart::rx_skips();
    bool filled = false;
    after_a_tick();
    {
        P::CriticalSection cs;
        bang_idle(storm_baud, 2);
        for (uint32_t i = 0; i < 4u; ++i) {
            bang_frame({.data = stream_byte(i), .baud = storm_baud});
        }
        filled = Starver::fill(fill_dst, &fill_cell, sizeof fill_dst);
        for (uint32_t i = 4; i < 7u; ++i) {
            bang_frame({.data = stream_byte(i), .baud = storm_baud});
        }
        while (Starver::busy()) {
        }
        bang_idle(storm_baud, 20);   // the line idle, IDLE up
        u2_interrupts = 0;
    }
    settle_ms(2);
    const uint32_t entries = u2_interrupts;
    const uint16_t hw = static_cast<uint16_t>(DmaUart::hw_overruns() - hw0);
    const uint16_t faults = static_cast<uint16_t>(DmaUart::dma_faults() - f0);
    const uint32_t skips = DmaUart::rx_skips() - k0;
    uint32_t got = 0;
    bool in_order = true;
    uint8_t b = 0;
    while (DmaUart::read_byte(b)) {
        if (got < 4u && b != stream_byte(got)) {
            in_order = false;
        }
        ++got;
    }
    const uint16_t c1 = U2::regs().CTLR1;
    const bool waiting_for_the_end = (c1 & usart_idleie) != 0u && (c1 & usart_rxneie) == 0u;
    print(serial, "  seven frames, the fill ", filled ? "run" : "REFUSED", " under the last three: ",
          got, " delivered, USART2's vector ", entries, " entries, ", hw, " overruns counted, ",
          skips, " gaps, ", faults, " channels given up; the vector ",
          waiting_for_the_end ? "waiting for the end" : "NOT waiting for the end", crlf);
    bench.verdict("AN OVERRUN AT A BURST'S TAIL, ORE up with nothing in DATAR: no storm - a handful "
                  "of entries, no channel given up - the overrun counted once and the gap once, the "
                  "frames before it delivered in order",
                  filled && entries <= 8u && faults == 0u && hw == 1u && skips == 1u &&
                      got >= 4u && got < 7u && in_order && waiting_for_the_end);

    // The next burst, whole - banged under the mask too: a bit at this
    // rate is 417 cycles, and the tick's handler would stretch one.
    after_a_tick();
    {
        P::CriticalSection cs;
        bang_idle(storm_baud, 2);
        for (uint32_t i = 0; i < 8u; ++i) {
            bang_frame({.data = stream_byte(40u + i), .baud = storm_baud});
        }
        bang_idle(storm_baud, 2);
    }
    settle_ms(2);
    uint32_t next = 0;
    bool next_in_order = true;
    uint8_t seen[8] = {};
    while (DmaUart::read_byte(b)) {
        if (next < 8u) {
            seen[next] = b;
            if (b != stream_byte(40u + next)) {
                next_in_order = false;
            }
        }
        ++next;
    }
    if (!next_in_order) {
        print(serial, "  received:");
        for (uint32_t i = 0; i < 8u; ++i) {
            print(serial, " ", hex(seen[i]), "/", hex(stream_byte(40u + i)));
        }
        print(serial, crlf);
    }
    print(serial, "  the next burst: ", next, " of 8", next_in_order ? " in order" : " NOT in order",
          crlf);
    bench.verdict("... and the next burst arrives whole, nothing else counted",
                  next == 8u && next_in_order && DmaUart::hw_overruns() - hw0 == 1u &&
                      DmaUart::dma_faults() == f0);
    all_off();
}

// ===========================================================================
// THE TWO-BOARD KEY: the desk's link to the CH32X035's usart_peer
// ===========================================================================
//
// USART2 on its column 4 (AFIO table 7-11): TX PB0, RX PB1, wired crossed to
// the CH32X035's USART4 on PB1/PB0, HCLK on this module's 24 MHz crystal.
// The peer is driven in band over the link itself (usart_link.hpp) and
// every leg is judged from what it reports or from what this end received;
// a leg the peer REFUSES (a capability its silicon has not got) is
// SKIPPED with the reason and judges nothing. The peer's only clock is an
// RC: letter 1 calibrates it against this crystal - it moves its trim to
// the step nearest 48 MHz and ADOPTS the rate it measures - and every
// letter that judges a figure finer than the HSI's wander (a few hundred
// ppm in a second) calibrates again first or takes a reference back to back.

namespace two {

constexpr uint8_t wire_column = 4;
using WTx = Pin<'B', 0>;
using WRx = Pin<'B', 1>;
static_assert(afio_usart2_pads(wire_column).tx == WTx::pad && afio_usart2_pads(wire_column).rx == WRx::pad,
              "table 7-11: USART2's column 4 is TX PB0, RX PB1");

link::Decoder dec;
const uint8_t no_payload[1] = {0};
bool leg_refused = false;
bool link_quiet = false;

/// A wait of whole milliseconds on the tick count: delay_us serves less
/// than one tick period and refuses the rest (delay.hpp).
void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms + 1u) {
    }
}

/// Exactly `us` microseconds on the cycle count: the rendezvous's window.
void wait_us(uint32_t us) {
    const uint32_t c0 = cycles_now();
    const uint32_t n = us * cycles_per_us;
    while (cycles_now() - c0 < n) {
    }
}

void put_w(uint8_t b) {
    while (!U2::tx_empty()) {
    }
    U2::write_data(b);
}

void wait_tc() {
    const uint32_t t0 = Ticker::ticks();
    while (!U2::tx_complete() && Ticker::ticks() - t0 < 50u) {
    }
}

void w_flush() {
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)U2::status();
        (void)U2::read_word();
    }
}

/// USART2 on column 4 with the frame and rate, its reset line pulsed
/// first. The TX pad is parked as a GPIO output high across the reset: a
/// USART in reset drives its alternate output low, an edge the far
/// receiver takes for a start bit (measured on this block, the usart_peer's
/// finding).
bool w_open(const UartFormat& f, uint32_t rate) {
    WTx::output(true);
    U2::bus_clock(true);
    U2::reset();
    U2::remap(wire_column);
    if (!U2::configure(f, static_cast<uint16_t>(usart_divisor(SysClock::pclk_hz, rate)))) {
        return false;
    }
    U2::enable(true);
    U2::transmitter(true);
    U2::receiver(true);
    return true;
}

/// Everything of column 4 let go: the instance reset and gated off, PB0
/// and PB1 floating inputs (the far board's discipline is the same).
void w_release() {
    U2::reset();
    U2::bus_clock(false);
    WTx::release();
    WRx::release();
}

UartFormat format_of(const link::Cfg& c) {
    UartFormat f{};
    switch (c.bits) {
        case 7: f.bits = UartBits::seven; break;
        case link::bits9_low:
        case link::bits9_high: f.bits = UartBits::nine; break;
        default: f.bits = UartBits::eight; break;
    }
    f.parity = c.parity == 1u ? UartParity::even : c.parity == 2u ? UartParity::odd : UartParity::none;
    f.stop = c.stop == 2u ? UartStop::two
           : c.stop == link::stop_half ? UartStop::half
           : c.stop == link::stop_one_half ? UartStop::one_and_half : UartStop::one;
    return f;
}

/// This end in a commanded configuration: the TX pad the function's and
/// the RX pad pulled up (a peer between answers drives nothing until its
/// wiring is proven, and a floating RX invents bytes). The pad is handed
/// over two bit times and 20 us after TE: a TX pad handed over as TE
/// rises shows the far receiver a low pulse (the usart_peer's finding).
bool dut_link(const link::Cfg& c) {
    if (!w_open(format_of(c), c.rate)) {
        return false;
    }
    if (c.mode == link::Mode::ircom && !U2::irda(IrdaConfig{})) {
        return false;
    }
    (void)delay_us(clock, 2u * (usart_divisor(SysClock::pclk_hz, c.rate) / 48u) + 20u);
    WTx::function();
    WRx::input(PinPull::up);
    w_flush();
    return true;
}

bool link_command_mode() {
    dec.reset();
    return dut_link(link::Cfg{});
}

bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::ticks();
    dec.reset();
    while (Ticker::ticks() - t0 < ms) {
        const uint16_t st = U2::status();
        if ((st & usart_rxne) == 0u) {
            continue;
        }
        const uint8_t b = U2::read_data();
        if ((st & (usart_fe | usart_pe | usart_ne)) != 0u) {
            dec.reset();
            continue;
        }
        if (dec.feed(b) == link::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    w_flush();
    link::write_frame(put_w, op, p, len);
    wait_tc();
    link::Frame f;
    if (!recv_frame(f, ms)) {
        return false;
    }
    return f.op == link::Op::ack && f.len == 2u && f.data[0] == link::byte_of(op);
}

/// The recovery guarantee in action: three attempts, each after a quiet
/// interval longer than the peer's own partial-frame reset.
bool command(link::Op op, const uint8_t* p = no_payload, uint8_t len = 0, uint16_t ms = 80) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len, ms)) {
            return true;
        }
        (void)link_command_mode();
        wait_ms(70);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(link::byte_of(op)),
              ": no ack - is the far board running usart_peer?", crlf);
    }
    return false;
}

bool query(link::Op op, link::Frame& data, uint16_t ms = 80) {
    return command(op, no_payload, 0, ms) && recv_frame(data, ms);
}

/// Find the peer: the crossed pair is the desk's only wiring, so this
/// pings until the peer's dark search hears a frame that checks out.
bool ensure_link() {
    link_quiet = true;
    bool found = false;
    for (uint8_t k = 0; k < 3u && !found; ++k) {
        found = link_command_mode() && command(link::Op::ping);
    }
    link_quiet = false;
    if (!found) {
        print(serial, "  the peer did not answer: the CH32X035 runs usart_peer on USART4 (PB0/PB1)?", crlf);
    }
    return found;
}

bool peer_ident(link::Ident& d) {
    link::Frame f;
    if (!query(link::Op::ident, f) || f.op != link::Op::ident_data || f.len != link::ident_size) {
        return false;
    }
    d = link::get_ident(f.data);
    return true;
}

/// The report of the last action, after its bound has had time to run.
/// A refusal raises leg_refused and says so once.
bool peer_report(link::Report& r, uint16_t after_ms) {
    (void)link_command_mode();
    if (after_ms != 0u) {
        wait_ms(after_ms);
    }
    for (uint8_t k = 0; k < 4u; ++k) {
        link::Frame f;
        const bool answered = query(link::Op::report, f) && f.op == link::Op::report_data;
        if (answered && f.len != link::report_size) {
            print(serial, "  the peer's REPORT is ", f.len, " bytes, this DUT reads ", link::report_size,
                  " - flash both boards from one tree", crlf);
            return false;
        }
        if (answered) {
            r = link::get_report(f.data);
            if ((r.flags & link::report_cfg_failed) != 0u && !leg_refused) {
                leg_refused = true;
                print(serial, "  the peer REFUSED op ", hex(r.op),
                      " (report_cfg_failed): a capability its silicon has not got", crlf);
            }
            return true;
        }
        wait_ms(40);
    }
    return false;
}

/// Command an action; the peer starts its own traffic link::settle_ms +
/// 2 ms after its ack IN ITS TICKS, as early as 4 ms, so this end takes up
/// its end settle_ms after the ack exactly, on the cycle count.
bool peer_act(link::Op op, const link::Params& a, const uint8_t* extra = nullptr, uint8_t n = 0) {
    leg_refused = false;
    uint8_t p[link::max_payload] = {};
    link::put_params(p, a);
    for (uint8_t i = 0; i < n; ++i) {
        p[link::params_size + i] = extra[i];
    }
    if (!command(op, p, static_cast<uint8_t>(link::params_size + n))) {
        return false;
    }
    wait_us(static_cast<uint32_t>(link::settle_ms) * 1000u);
    return true;
}

/// A verdict, unless the peer refused the leg: then SKIP, counting nothing.
void judge(const char* name, bool ok) {
    if (leg_refused) {
        print(serial, "  SKIP  ", name, " (the peer refused the leg)", crlf);
        return;
    }
    bench.verdict(name, ok);
}
void judge(const char* name, const char* suffix, bool ok) {
    if (leg_refused) {
        print(serial, "  SKIP  ", name, suffix, " (the peer refused the leg)", crlf);
        return;
    }
    bench.verdict(name, suffix, ok);
}

/// Milliseconds a frame count needs at a rate, four times over plus a floor.
uint16_t bound_ms(uint32_t rate, uint32_t frames) {
    const uint32_t ms = (12u * frames * 1000u * 4u) / (rate != 0u ? rate : 1u) + 80u;
    return ms > 6000u ? 6000u : static_cast<uint16_t>(ms);
}

/// The peer opens its window link::settle_ms + 2 ms after its ack and
/// flushes what this end's reconfiguration left on the line.
void wait_for_peer_window() { wait_ms(6); }

/// `n` frames back to back: DATAR written on TXE, a frame ahead of the
/// shifter, so the line never idles.
void dut_blast(uint16_t n, uint16_t mask) {
    for (uint16_t i = 0; i < n; ++i) {
        while (!U2::tx_empty()) {
        }
        U2::write_word(static_cast<uint16_t>((0x55u + i) & mask));
    }
    wait_tc();
}

/// What this end's receiver made of a run, polled.
struct Drain {
    uint16_t frames = 0;
    uint8_t fe = 0, pe = 0, ne = 0, ore = 0;
    uint16_t last = 0;
    uint16_t first = 0;
    bool lbd = false;
};

Drain collect(uint16_t ms) {
    Drain d{};
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        const uint16_t st = U2::status();
        if ((st & usart_rxne) == 0u) {
            continue;
        }
        const uint16_t w = U2::read_word();
        if (d.frames == 0u) d.first = w;
        d.last = w;
        ++d.frames;
        if ((st & usart_fe) != 0u) ++d.fe;
        if ((st & usart_pe) != 0u) ++d.pe;
        if ((st & usart_ne) != 0u) ++d.ne;
        if ((st & usart_ore) != 0u) ++d.ore;
    }
    return d;
}

uint16_t mask_of(const link::Cfg& c) {
    return c.bits >= link::bits9_low ? 0x1FFu : c.bits == 7u ? 0x7Fu : 0xFFu;
}

/// One timed run: this end sends `n` gapless frames of `c`; the peer
/// counts them (`op` calibrate - which an RC peer ADOPTS this end's
/// crystal from - or sink, which only measures) and reports the span on
/// its clock; the answer is this end's rate against the peer's clock in
/// ppm (calibrate_ppm over the frame's own half bits).
struct Timed {
    bool ok = false;
    int32_t ppm = 0;
    uint32_t span = 0;
};

Timed timed_run(link::Op op, const link::Cfg& c, uint16_t n) {
    Timed k{};
    link::Params a{};
    a.cfg = c;
    a.count = n;
    a.ms = bound_ms(c.rate, n);
    if (!peer_act(op, a)) {
        return k;
    }
    if (!dut_link(c)) {
        return k;
    }
    wait_for_peer_window();
    dut_blast(n, mask_of(c));
    link::Report r{};
    if (!peer_report(r, 5) || leg_refused) {
        return k;
    }
    k.ok = r.count == n && r.span != 0u && r.ferr == 0u && r.perr == 0u && r.ovf == 0u;
    if (!k.ok) {
        print(serial, "    the peer counted ", r.count, " of ", n, " at ", c.rate, ", ferr ", r.ferr, " perr ", r.perr,
              " ovf ", r.ovf, " span ", r.span, " flags ", hex(r.flags), crlf);
    }
    k.span = r.span;
    k.ppm = link::calibrate_ppm(link::nominal_span(c.rate, link::frame_half_bits(c.bits, c.parity, c.stop), n),
                                r.span);
    return k;
}

constexpr uint32_t calib_rate = 1'000'000;   ///< a divisor of 48 at 48 MHz: this end exact
constexpr uint16_t calib_frames = 4000;      ///< 40 ms of span

/// Calibrate the peer at 1 Mbaud, where this end's divisor is exact.
Timed calibrate_peer() {
    link::Cfg c{};
    c.rate = calib_rate;
    return timed_run(link::Op::calibrate, c, calib_frames);
}

/// ppm printed with its sign and as per cent.
void print_ppm(int32_t ppm) {
    const uint32_t m = static_cast<uint32_t>(ppm < 0 ? -ppm : ppm);
    print(serial, ppm < 0 ? "-" : "+", m, " ppm (", ppm < 0 ? "-" : "+", m / 10000u, ".", (m / 1000u) % 10u,
          (m / 100u) % 10u, " %)");
}

/// What this end's divisor gives against the rate asked, in ppm: HCLK over
/// BRR against the baud.
int32_t divisor_ppm(uint32_t rate) {
    const uint32_t brr = usart_divisor(SysClock::pclk_hz, rate);
    return link::rate_error_ppm(SysClock::pclk_hz, brr, rate);
}

bool near(int32_t a, int32_t b, int32_t tol) { return a - b <= tol && b - a <= tol; }

/// Decimal for a verdict label (one call per expression).
char num_buf[12];
const char* num(uint32_t v) {
    uint8_t i = 11;
    num_buf[i] = 0;
    if (v == 0u) num_buf[--i] = '0';
    while (v != 0u && i != 0u) {
        num_buf[--i] = static_cast<char>('0' + (v % 10u));
        v /= 10u;
    }
    return &num_buf[i];
}

/// The start of every letter: the peer found, and calibrated again against
/// this crystal (the HSI wanders; usart_peer.cpp's TIME).
bool ready() {
    if (!crystal_ok) {
        print(serial, "  this module's crystal did not start: the two-board key needs it as the reference", crlf);
        bench.verdict("HCLK on the 24 MHz crystal", false);
        return false;
    }
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        return false;
    }
    // A calibrate that finds another trim step nearer 48 MHz moves the
    // trim and answers xtal 0 (usart_peer.cpp's TIME): run until adopted.
    for (uint8_t k = 0; k < 4u; ++k) {
        const Timed t = calibrate_peer();
        link::Ident d{};
        if (t.ok && peer_ident(d) && d.xtal == 2u) {
            return true;
        }
    }
    bench.verdict("the peer calibrated against this crystal (xtal 2)", false);
    return false;
}

// ---------------------------------------------------------------------------
// 1 - the rendezvous, and the calibration
// ---------------------------------------------------------------------------
void t1_link() {
    bench.verdict("HCLK runs on the module's 24 MHz crystal (the reference of this key)", crystal_ok);
    const bool found = ensure_link();
    bench.verdict("command mode on USART2's column 4 (8N1, 115200) and the peer answers a ping", found);
    if (!found) {
        w_release();
        return;
    }
    link::Ident d{};
    const bool got = peer_ident(d);
    char label[9] = {};
    for (uint8_t i = 0; i < 8u; ++i) label[i] = d.label[i];
    if (got) {
        print(serial, "  peer '", label, "' kind ", hex(link::peer_kind(d.version)), " fw ", hex(d.version),
              " xtal=", d.xtal, " sanity=", hex(d.sanity), crlf);
    }
    bench.verdict("IDENT: the CH32X035's usart_peer (kind 0x07), an RC peer or one already calibrated",
                  got && d.sanity == link::ident_sanity && link::peer_kind(d.version) == link::peer_ch32x035 &&
                      label[0] == 'X' && (d.xtal == 0u || d.xtal == 2u));

    w_flush();
    put_w(link::magic);
    put_w(link::byte_of(link::Op::ping));
    put_w(0);
    put_w(0x00);   // not the checksum
    wait_tc();
    link::Frame bad;
    bench.verdict("a bad checksum comes back as a NAK", recv_frame(bad, 120) && bad.op == link::Op::nak);
    for (uint8_t i = 0; i < 12u; ++i) put_w(static_cast<uint8_t>(0x11u * i));
    wait_tc();
    wait_ms(120);
    (void)link_command_mode();
    bench.verdict("the link recovers after a desync", command(link::Op::ping));

    // THE REFUSAL PATH: onewire (no USART TX of the CH32X035F8U6 reaches its
    // own RX pad), a synchronous client (USART4's CK pad is not bonded),
    // a drawn train (stage): acked, reported refused, ping answered after.
    bool refused_all = true;
    for (const link::Op op : {link::Op::onewire, link::Op::set_link, link::Op::stage}) {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.cfg.mode = op == link::Op::set_link ? link::Mode::sync_client : link::Mode::async;
        a.ms = 50;
        link::Report r{};
        refused_all = refused_all && peer_act(op, a) && peer_report(r, 0) && leg_refused &&
                      (r.flags & link::report_ran) == 0u;
    }
    leg_refused = false;
    bench.verdict("the peer refuses onewire, a synchronous client and a drawn train, and answers at once after",
                  refused_all && command(link::Op::ping));

    // THE CALIBRATION: gapless runs at 1 Mbaud (this end's divisor exact)
    // until the peer reports it adopted this crystal - each run's figure is
    // this crystal against the peer's clock as it stood before the run.
    int32_t first = 0;
    bool first_ok = false;
    uint8_t rounds = 0;
    bool adopted = false;
    for (uint8_t k = 0; k < 4u && !adopted; ++k) {
        const Timed t = calibrate_peer();
        link::Ident after{};
        const bool asked = peer_ident(after);
        ++rounds;
        print(serial, "  calibrate ", rounds, ": this crystal reads ");
        print_ppm(t.ppm);
        print(serial, " against the peer's clock; the peer answers xtal ", after.xtal, crlf);
        if (k == 0u) {
            first = t.ppm;
            first_ok = t.ok;
        }
        adopted = t.ok && asked && after.xtal == 2u;
    }
    print(serial, "  the CH32X035's HSI at its trim before this letter: ");
    print_ppm(-first);
    print(serial, " (the peer's console 'c' names the trim)", crlf);
    bench.verdict("the peer timed a gapless run of 4000 frames at 1 Mbaud", first_ok);
    bench.verdict("its HSI, as found, lies inside table 3-9's -1.7 % to +1.6 % (0 to 70 C)",
                  first_ok && -first >= -17'000 && -first <= 16'000);
    bench.verdict("the peer ADOPTS this crystal - trim moved to its nearest step, the rate kept - within four "
                  "runs, and says so (xtal 2)", adopted);
    // After the adoption: three more runs read this crystal against the
    // adopted clock - its residual and its wander, run to run.
    int32_t after_ppm[3] = {};
    bool all = adopted;
    for (int32_t& p : after_ppm) {
        const Timed t = calibrate_peer();
        all = all && t.ok;
        p = t.ppm;
    }
    print(serial, "  after the adoption, run to run: ", after_ppm[0], ", ", after_ppm[1], ", ", after_ppm[2],
          " ppm (the HSI's wander)", crlf);
    bench.verdict("each run after the adoption reads this crystal within 1000 ppm of the adopted clock", all &&
                  near(after_ppm[0], 0, 1000) && near(after_ppm[1], 0, 1000) && near(after_ppm[2], 0, 1000));
    (void)link_command_mode();
    w_release();
}

// ---------------------------------------------------------------------------
// 2 - the echo ladder, and the rate on the wire timed by the far chip
// ---------------------------------------------------------------------------
uint16_t echo_mismatches(uint16_t n, uint16_t mask) {
    uint16_t bad = 0;
    wait_for_peer_window();
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = static_cast<uint16_t>((0x5Au + i * 37u) & mask);
        while (!U2::tx_empty()) {
        }
        U2::write_word(v);
        bool ok = false;
        const uint32_t t0 = Ticker::ticks();
        while (Ticker::ticks() - t0 < 20u) {
            const uint16_t st = U2::status();
            if ((st & usart_rxne) != 0u) {
                ok = (U2::read_word() & mask) == v && (st & (usart_fe | usart_pe | usart_ne)) == 0u;
                break;
            }
        }
        if (!ok) ++bad;
    }
    return bad;
}

/// Frames for a timed run at `rate`: a fiftieth of a second of wire, 192
/// at least and 2000 at most.
uint16_t span_frames(uint32_t rate) {
    const uint32_t n = rate / 500u;
    return static_cast<uint16_t>(n < 192u ? 192u : (n > 2000u ? 2000u : n));
}

void t2_rates() {
    if (!ready()) {
        w_release();
        return;
    }
    static constexpr uint32_t rates[] = {9600, 115'200, 921'600, 1'000'000, 1'500'000, 2'000'000, 2'400'000,
                                         3'000'000};
    for (const uint32_t rate : rates) {
        link::Cfg c{};
        c.rate = rate;
        link::Params a{};
        a.cfg = c;
        a.count = 16;
        a.ms = bound_ms(rate, 32);
        const bool acted = peer_act(link::Op::echo, a);
        const bool up = acted && dut_link(c);
        const uint16_t bad = up ? echo_mismatches(16, 0xFFu) : 16u;
        link::Report r{};
        const bool have = peer_report(r, 5);
        judge(num(rate), " baud: 16 bytes echoed by the far chip, every one equal, 16 clean frames counted there",
              up && bad == 0u && have && r.count == 16u && r.ferr == 0u && r.perr == 0u && r.ovf == 0u);
        // THE FOREIGN SPAN: a gapless run timed by the far receiver on its
        // adopted clock, against a 1 Mbaud run back to back (the HSI wanders
        // between the two by far less than the bound). Half the smallest
        // divisor step on this ladder (1/417, 2400 ppm) is far above it: a
        // wrong divisor cannot hide.
        link::Cfg ref{};
        ref.rate = calib_rate;
        const Timed r0 = timed_run(link::Op::sink, ref, 2000);
        const Timed k = timed_run(link::Op::sink, c, span_frames(rate));
        const int32_t expect = divisor_ppm(rate);
        print(serial, "  ", rate, ": BRR ", usart_divisor(SysClock::pclk_hz, rate), ", the far chip times ");
        print_ppm(k.ppm - r0.ppm);
        print(serial, " against the divisor's ", expect, " ppm", crlf);
        judge(num(rate), " baud: the rate on the wire is HCLK / BRR within 500 ppm, timed by the far chip",
              r0.ok && k.ok && near(k.ppm - r0.ppm, expect, 500));
    }
    // The peer's own rate, stated in its blast's report once it adopted a
    // crystal: its HCLK over its divisor against the rate asked.
    {
        link::Cfg c{};
        c.rate = 921'600;
        link::Params a{};
        a.cfg = c;
        a.count = 64;
        a.ms = bound_ms(c.rate, 128);
        const bool acted = peer_act(link::Op::blast, a);
        const bool up = acted && dut_link(c);
        const Drain d = up ? collect(10) : Drain{};
        link::Report r{};
        const bool have = peer_report(r, 5);
        const int32_t stated = static_cast<int16_t>(r.aux16);
        // The peer's divisor at 921600 is 52 on 48 MHz, +1600 ppm; its
        // adopted HCLK moves that by its residual, under a trim step.
        print(serial, "  the peer's blast at 921600: ", d.frames, " frames here, its rate stated ",
              (r.flags & link::report_rate_stated) != 0u ? "" : "NOT ", stated, " ppm", crlf);
        judge("the peer STATES its blast's rate once calibrated (report_rate_stated), within a trim step of "
              "48 MHz / 52 against 921600",
              have && d.frames == 64u && (r.flags & link::report_rate_stated) != 0u && near(stated, 1600, 2500));
    }
    (void)link_command_mode();
    w_release();
}

// ---------------------------------------------------------------------------
// 3 - the frame matrix across the wire
// ---------------------------------------------------------------------------
struct FmtCase {
    uint8_t bits;
    uint8_t parity;
    uint8_t stop;
    const char* name;
};

void t3_frames() {
    if (!ready()) {
        w_release();
        return;
    }
    static constexpr FmtCase cases[] = {
        {8, 0, 1, "8N1"}, {8, 1, 1, "8E1"}, {8, 2, 1, "8O1"}, {7, 1, 1, "7E1"}, {7, 2, 1, "7O1"},
        {link::bits9_low, 0, 1, "9N1"}, {8, 0, 2, "8N2"}, {8, 0, link::stop_half, "8N0.5"},
        {8, 0, link::stop_one_half, "8N1.5"},
    };
    for (const FmtCase& fc : cases) {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = fc.bits;
        c.parity = fc.parity;
        c.stop = fc.stop;
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = bound_ms(115'200, 24);
        const bool acted = peer_act(link::Op::echo, a);
        const bool up = acted && dut_link(c);
        const uint16_t bad = up ? echo_mismatches(8, mask_of(c)) : 8u;
        link::Report r{};
        const bool have = peer_report(r, 5);
        if (bad != 0u) {
            print(serial, "    ", fc.name, ": wrong=", bad, " peer count=", r.count, " ferr=", r.ferr, " perr=", r.perr,
                  crlf);
        }
        judge(fc.name, " across the wire, echoed",
              up && bad == 0u && have && r.count == 8u && r.ferr == 0u && r.perr == 0u);
    }
    // The two odd stops as LENGTHS: a gapless run timed by the far chip is
    // a frame of 19 and of 21 half bits, against an 8N1 run back to back.
    for (const uint8_t stop : {link::stop_half, link::stop_one_half}) {
        link::Cfg ref{};
        ref.rate = calib_rate;
        const Timed r0 = timed_run(link::Op::sink, ref, 2000);
        link::Cfg c{};
        c.rate = calib_rate;
        c.stop = stop;
        const Timed k = timed_run(link::Op::sink, c, 2000);
        print(serial, "  ", stop == link::stop_half ? "8N0.5" : "8N1.5", " at 1 Mbaud, against frames of ",
              link::frame_half_bits(8, 0, stop), " half bits: ");
        print_ppm(k.ppm - r0.ppm);
        print(serial, crlf);
        judge(stop == link::stop_half ? "half a stop bit" : "one and a half stop bits",
              ": the frame on the wire is that long, within 500 ppm of an 8N1 run's",
              r0.ok && k.ok && near(k.ppm - r0.ppm, 0, 500));
    }
    // What neither chip has: 5- and 6-bit words, nine data bits with a
    // parity bit - the peer refuses each, and this end's vocabulary too.
    bool refused = true;
    for (const uint8_t bits : {uint8_t{5}, uint8_t{6}, link::bits9_low}) {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.cfg.bits = bits;
        a.cfg.parity = bits == link::bits9_low ? 1u : 0u;
        a.count = 4;
        a.ms = 50;
        link::Report r{};
        refused = refused && peer_act(link::Op::echo, a) && peer_report(r, 0) && leg_refused;
    }
    leg_refused = false;
    bench.verdict("the peer refuses 5- and 6-bit words and nine bits with parity, as this end's format does",
                  refused && !uart_format_valid(UartFormat{UartBits::nine, UartParity::even, UartStop::one}));
    (void)link_command_mode();
    w_release();
}

// ---------------------------------------------------------------------------
// 4 - the receiver's errors, from the calibrated peer
// ---------------------------------------------------------------------------
void t4_errors() {
    if (!ready()) {
        w_release();
        return;
    }
    // 1. Odd parity into an even receiver.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.parity = 2;
        link::Cfg dc = pc;
        dc.parity = 1;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 12);
        a.value = 0x5A;
        const bool acted = peer_act(link::Op::send, a);
        const bool up = acted && dut_link(dc);
        const Drain d = collect(bound_ms(115'200, 12));
        print(serial, "  odd into even: frames=", d.frames, " pe=", d.pe, " fe=", d.fe, crlf);
        judge("every frame of the wrong parity raises PE", up && d.frames == 6u && d.pe == 6u && d.fe == 0u);
        (void)link_command_mode();
    }
    // 2. A uniform rate error from the peer's engine - its adopted clock
    //    makes the per cent asked the per cent on the wire: 0x00 frames at
    //    9600 x (1 + e), into this receiver at 9600 on a crystal.
    {
        static constexpr int8_t errs[] = {0, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 8, -8};
        int8_t onset_pos = 0, onset_neg = 0;
        bool inside_clean = true;
        link::Cfg dc{};
        dc.rate = 9600;
        for (const int8_t e : errs) {
            link::Params a{};
            a.cfg.rate = static_cast<uint32_t>(9600 + (9600 * e) / 100);
            a.count = 6;
            a.ms = 400;
            a.value = 0x00;
            const bool acted = peer_act(link::Op::bitbang, a);
            const bool up = acted && dut_link(dc);
            const Drain d = collect(70);
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, "  ", e, " %: frames=", d.frames, " fe=", d.fe, " ne=", d.ne, " last ", hex(d.last),
                  "; the peer banged ", r.count, crlf);
            // A fast sender's frame ends inside the idle that follows it, so
            // its stop bit is never sampled low: it breaks as WRONG DATA (the
            // last data bit sampled in the stop), a slow one as FE.
            if (up && (d.fe != 0u || d.frames != 6u || d.first != 0u || d.last != 0u)) {
                if (e > 0 && onset_pos == 0) onset_pos = e;
                if (e < 0 && onset_neg == 0) onset_neg = e;
            }
            if (e >= -3 && e <= 3) {
                inside_clean = inside_clean && up && d.frames == 6u && d.fe == 0u;
            }
            (void)link_command_mode();
        }
        print(serial, "  the frames break from +", onset_pos, " % (wrong data) and from ", onset_neg, " % (FE)", crlf);
        judge("every rate within 3 % of this receiver's is clean (14.3's tolerance)", inside_clean);
        judge("and a rate far enough off breaks the frames both ways - fast as wrong data, slow as FE",
              onset_pos != 0 && onset_neg != 0);
    }
    // 3. Low pulses on the idle line: where a start bit starts being
    //    believed, and the noise flag.
    {
        static constexpr uint16_t widths[] = {6, 60, 160, 300, 625, 1250, 1875, 2500};
        uint16_t first_taken = 0;
        uint16_t noisy = 0;
        link::Cfg dc{};
        dc.rate = 9600;
        bool tiny_ignored = false, whole_taken = false;
        for (const uint16_t w : widths) {
            link::Params a{};
            a.cfg.rate = 9600;
            a.count = 4;
            a.ms = 300;
            a.aux16 = w;
            a.aux8 = 200;
            const bool acted = peer_act(link::Op::glitch, a);
            const bool up = acted && dut_link(dc);
            const Drain d = collect(60);
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, "  a low pulse of ", w, " ticks (a bit is 2500): frames=", d.frames, " fe=", d.fe,
                  " ne=", d.ne, crlf);
            if (up && d.frames != 0u && first_taken == 0u) first_taken = w;
            if (up && d.ne != 0u && noisy == 0u) noisy = w;
            if (w == 6u) tiny_ignored = up && d.frames == 0u;
            if (w == 2500u) whole_taken = up && d.frames != 0u;
            (void)link_command_mode();
        }
        print(serial, "  the narrowest pulse taken as a start bit: ", first_taken, " ticks; NE first at ", noisy,
              crlf);
        judge("a pulse of 6 ticks is not a start bit", tiny_ignored);
        judge("a whole bit low is one", whole_taken);
    }
    // 4. A run into a receiver that reads late: ORE.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = bound_ms(115'200, 16);
        a.value = 0xB0;
        const bool acted = peer_act(link::Op::blast, a);
        const bool up = acted && dut_link(pc);
        wait_ms(6 + 2);   // the window, and the run, unread
        const uint16_t st = U2::status();
        const uint16_t kept = U2::read_word();
        const Drain d = collect(5);
        print(serial, "  a run of 8 read late: STATR ", hex(st), ", DATAR ", hex(kept), ", then ", d.frames, " more",
              crlf);
        judge("a run read late raises ORE", up && (st & usart_ore) != 0u && (st & usart_rxne) != 0u);
        judge("and DATAR keeps the first frame of the run, the rest lost", up && kept == 0xB0u && d.frames == 0u);
        (void)link_command_mode();
    }
    // 5. LIN's break detection at both lengths, from the peer's engine.
    {
        struct BreakCase {
            uint16_t bits;
            bool eleven;
            bool detect;
        };
        static constexpr BreakCase cases[] = {{9, false, false}, {13, false, true}, {10, true, false}, {13, true, true}};
        bool all = true;
        for (const BreakCase& bc : cases) {
            link::Cfg c{};
            c.rate = 9600;
            link::Params a{};
            a.cfg = c;
            a.count = bc.bits;
            a.ms = 200;
            const bool acted = peer_act(link::Op::brk, a);
            bool up = acted && dut_link(c);
            U2::enable(false);
            up = up && U2::lin(LinConfig{bc.eleven, false});
            U2::enable(true);
            U2::clear_flags(usart_lbd);
            const Drain d = collect(30);
            const bool lbd = U2::flag(usart_lbd);
            U2::lin_off();
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, "  a ", bc.bits, "-bit break, ", bc.eleven ? "11" : "10", "-bit detection: LBD ", lbd,
                  ", frames ", d.frames, " fe ", d.fe, crlf);
            all = all && up && lbd == bc.detect;
            (void)link_command_mode();
        }
        judge("LIN detects a 13-bit break at both lengths, a 9-bit low at neither, 10 bits not at 11", all);
    }
    (void)link_command_mode();
    w_release();
}

// ---------------------------------------------------------------------------
// 5 - the peer's engine shapes, and this end's own pull-banger on the wire
// ---------------------------------------------------------------------------
Drain bitbang_run(uint8_t value, uint16_t frames, int8_t cell, int8_t pct, uint16_t break_bits, bool lin) {
    link::Params a{};
    a.cfg.rate = 9600;
    a.count = frames;
    a.ms = 600;
    a.value = value;
    a.cell = cell;
    a.pct = pct;
    a.aux16 = break_bits;
    link::Cfg dc{};
    dc.rate = 9600;
    if (!peer_act(link::Op::bitbang, a) || !dut_link(dc)) return Drain{};
    if (lin) {
        U2::enable(false);
        (void)U2::lin(LinConfig{});
        U2::enable(true);
        U2::clear_flags(usart_lbd);
    }
    Drain d = collect(static_cast<uint16_t>(30u + 15u * frames + break_bits / 4u));
    d.lbd = U2::flag(usart_lbd);
    U2::lin_off();
    link::Report r{};
    (void)peer_report(r, 0);
    return d;
}

/// THIS END'S PULL-BANGER on PB1, the receive pad of column 4: the pad's
/// pull moved by OUTDR at absolute offsets on the cycle count, the far pad
/// floating while the peer stands off. One 8N1 frame of `value` at 9600,
/// cell `cell` stretched by `pct` per cent.
void pull_bang(uint8_t value, int8_t cell, int8_t pct) {
    const uint32_t bit = SysClock::hz / 9600u;
    uint32_t at = 0;
    const uint32_t t0 = cycles_now();
    for (int8_t k = 0; k < 10; ++k) {
        const bool high = k == 0 ? false : (k == 9 ? true : ((value >> (k - 1)) & 1u) != 0u);
        if (high) { WRx::set(); } else { WRx::clear(); }
        uint32_t w = bit;
        if (k == cell) {
            w = static_cast<uint32_t>(static_cast<int32_t>(bit) + (static_cast<int32_t>(bit) * pct) / 100);
        }
        at += w;
        spin_until(t0 + at);
    }
    WRx::set();
    spin_until(t0 + at + 4u * bit);
}

void t5_shapes() {
    if (!ready()) {
        w_release();
        return;
    }
    const Drain clean = bitbang_run(0x55, 6, -1, 0, 0, false);
    print(serial, "  six banged 0x55 at 9600: frames=", clean.frames, " fe=", clean.fe, " last ", hex(clean.last), crlf);
    judge("a frame from the peer's edge list is a clean 0x55, six times",
          clean.frames == 6u && clean.fe == 0u && clean.ne == 0u && clean.last == 0x55u);
    (void)link_command_mode();

    // One stretched cell, the peer's engine then this end's pull-banger on
    // the same wire (the peer standing off, its pads floating).
    static constexpr int8_t pcts[] = {10, 20, 30, 40, 50, 60, 70};
    int8_t onset_engine = 0;
    for (const int8_t p : pcts) {
        const Drain d = bitbang_run(0x00, 4, 5, p, 0, false);
        if ((d.fe != 0u || d.frames != 4u) && onset_engine == 0) onset_engine = p;
        (void)link_command_mode();
    }
    int8_t onset_pull = 0;
    link::Params so{};
    so.ms = 2000;
    uint8_t sp[link::params_size] = {};
    link::put_params(sp, so);
    const bool off = command(link::Op::standby, sp, link::params_size);
    for (const int8_t p : pcts) {
        if (!off) break;
        (void)w_open(UartFormat{}, 9600);
        WRx::input(PinPull::up);
        wait_ms(1);
        w_flush();
        // DATAR holds one frame: each is read before the next is banged.
        uint8_t frames = 0, fe = 0;
        for (uint8_t i = 0; i < 4u; ++i) {
            pull_bang(0x00, 5, p);
            const Drain one = collect(1);
            frames = static_cast<uint8_t>(frames + one.frames);
            fe = static_cast<uint8_t>(fe + one.fe);
        }
        if ((fe != 0u || frames != 4u) && onset_pull == 0) onset_pull = p;
    }
    WRx::release();
    wait_ms(2000);   // the stand-off runs out; the peer's search resumes
    print(serial, "  one stretched cell breaks the frame from +", onset_engine, " % (the peer's engine), +",
          onset_pull, " % (this end's pull-banger)", crlf);
    judge("one stretched cell survives far past the rate tolerance (30 % and more) from the peer's engine",
          onset_engine == 0 || onset_engine >= 30);
    judge("and this end's own pull-banger on the same wire breaks the frame within one step of it",
          off && near(onset_engine, onset_pull, 10));
    (void)ensure_link();

    const Drain lin = bitbang_run(0x55, 1, -1, 0, 13, true);
    print(serial, "  a 13-bit break then 0x55: LBD ", lin.lbd, ", frames=", lin.frames, " last ", hex(lin.last), crlf);
    judge("a LIN break ahead of a frame: the break detected and the frame after it received",
          lin.lbd && lin.last == 0x55u);
    (void)link_command_mode();
    w_release();
}

// ---------------------------------------------------------------------------
// 6 - USART2's receive engine under the peer's repeated blast
// ---------------------------------------------------------------------------
using LinkRing = Uart<2, P, 256, 64, NoDmaEngine, DmaRxEngine<7>, wire_column>;
volatile bool ring_active = false;

void t6_engine() {
    if (!ready()) {
        w_release();
        return;
    }
    Dma::open();
    static constexpr uint32_t rates[] = {1'000'000, 2'000'000, 3'000'000};
    for (const uint32_t rate : rates) {
        // About a second of gapless frames: blocks of 10000, repeated with
        // no gap, the counting pattern running on across them.
        const uint16_t block = 10'000;
        const uint16_t repeats = static_cast<uint16_t>(rate / 10u / block);
        const uint32_t total = static_cast<uint32_t>(block) * repeats;
        link::Cfg c{};
        c.rate = rate;
        link::Params a{};
        a.cfg = c;
        a.count = block;
        a.aux16 = repeats;
        a.aux8 = 0;
        a.ms = 3000;
        a.value = 0x30;
        a.pattern = link::pattern_counting;
        const bool acted = peer_act(link::Op::blast, a);
        u2_isr = &LinkRing::isr;
        ring_active = true;
        const bool up = acted && LinkRing::init(clock, rate);
        WRx::input(PinPull::up);
        Pfic::enable(dma_channel_irq(7));
        uint32_t got = 0;
        uint8_t expect = 0x30;
        bool broken = false;
        uint8_t wrong = 0;
        const uint32_t t0 = Ticker::millis();
        while (up && got < total && !broken && Ticker::millis() - t0 < 2500u) {
            const std::span<const uint8_t> run = LinkRing::read_span();
            uint32_t n = 0;
            for (const uint8_t b : run) {
                if (b != expect) {
                    wrong = b;
                    broken = true;
                    break;
                }
                ++expect;
                ++n;
            }
            got += n;
            LinkRing::consume(n);
        }
        const uint16_t ore = LinkRing::hw_overruns();
        const uint16_t fe = LinkRing::frame_errors();
        const uint16_t faults = LinkRing::dma_faults();
        Pfic::disable(dma_channel_irq(7));
        LinkRing::release();
        ring_active = false;
        u2_isr = nullptr;
        link::Report r{};
        const bool have = peer_report(r, 5);
        print(serial, "  ", rate, ": ", got, " of ", total, " bytes in sequence");
        if (broken) {
            print(serial, " - BROKEN, ", hex(wrong), " read");
        }
        print(serial, ", ORE ", ore, ", FE ", fe, ", faults ", faults, "; the peer sent ", r.count, " (mod 65536)",
              crlf);
        judge(num(rate), " baud: a second of the peer's blast through the receive engine, every byte in sequence",
              up && have && got == total && !broken && ore == 0u && fe == 0u && faults == 0u &&
                  r.count == static_cast<uint16_t>(total));
    }
    (void)link_command_mode();
    w_release();
}

// ---------------------------------------------------------------------------
// 7 - half duplex on PB0: a second chip's frames, and the wired AND
// ---------------------------------------------------------------------------
void t7_half_duplex() {
    if (!ready()) {
        w_release();
        return;
    }
    // drive_rxd: the peer drives the wire this end's TX pad sits on, from
    // its receive pad; HDSEL listens there through the AF mux, the pad an
    // open drain that never pulls (TE off).
    bool heard_all = true;
    for (const uint32_t rate : {9600u, 115'200u}) {
        link::Params a{};
        a.cfg.rate = rate;
        a.count = 4;
        a.value = 0x5A;
        a.ms = 400;
        const bool acted = peer_act(link::Op::drive_rxd, a);
        bool up = acted && w_open(UartFormat{}, rate);
        U2::transmitter(false);
        up = up && U2::half_duplex(true);
        WTx::function(PinDrive::open_drain);
        w_flush();
        const Drain d = collect(static_cast<uint16_t>(rate == 9600u ? 80u : 40u));
        (void)U2::half_duplex(false);
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  HDSEL on PB0 at ", rate, ": ", d.frames, " frames, last ", hex(d.last), ", fe ", d.fe,
              "; the peer drove ", r.count, crlf);
        heard_all = heard_all && up && d.frames == 4u && d.fe == 0u && d.last == 0x5Au;
        (void)link_command_mode();
    }
    judge("HDSEL on PB0 hears the four frames a second chip drives onto that wire, at 9600 and 115200", heard_all);

    // collide: this end sends 0xA5 then 0xF0 back to back, its pad an open
    // drain; the peer, once 0xA5 is in, pulls 0x3C into the next frame as
    // an emulated open drain - and its own receiver reads the wire: the AND.
    {
        link::Params a{};
        a.cfg.rate = 9600;
        a.value = 0x3C;
        a.aux16 = 15;   // the peer's RXNE comes about 0.6 bit into the stop: wait for the next start
        a.ms = 400;
        const bool acted = peer_act(link::Op::collide, a);
        bool up = acted && w_open(UartFormat{}, 9600);
        up = up && U2::half_duplex(true);
        WTx::function(PinDrive::open_drain);
        wait_for_peer_window();
        w_flush();
        if (up) {
            put_w(0xA5);
            put_w(0xF0);
            wait_tc();
        }
        (void)U2::half_duplex(false);
        link::Report r{};
        (void)peer_report(r, 5);
        print(serial, "  collide at 9600: the peer heard ", r.count, " frames, the second ", hex(r.aux16),
              " (0xF0 AND 0x3C = 0x30), ferr ", r.ferr, crlf);
        judge("THE WIRED AND: the peer's frame pulled into this end's 0xF0 on an open-drain wire reads 0x30",
              up && r.count == 2u && r.aux16 == 0x30u);
    }
    (void)link_command_mode();
    w_release();
}

// ---------------------------------------------------------------------------
// 8 - mute and IrDA on the wire
// ---------------------------------------------------------------------------
void t8_modes() {
    if (!ready()) {
        w_release();
        return;
    }
    // 1. Mute woken by an address mark: 9-bit words, this node 5.
    {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = link::bits9_low;
        link::Params a{};
        a.cfg = c;
        a.value = 0x10;
        a.groups = 3;
        const uint8_t pairs[6] = {3, 2, 5, 3, 3, 2};
        const bool acted = peer_act(link::Op::send_mpcm, a, pairs, 6);
        bool up = acted && dut_link(c);
        up = up && U2::mute_mode(MuteConfig{MuteWake::address_mark, 5}) && U2::mute();
        const Drain d = collect(40);
        print(serial, "  address mark: frames=", d.frames, " first ", hex(d.first), " last ", hex(d.last), crlf);
        link::Report r{};
        (void)peer_report(r, 0);
        judge("a muted receiver wakes on its address alone: the address frame and its three data",
              up && d.frames == 4u && d.first == 0x105u && d.last == 0x14u);
        U2::unmute();
        (void)link_command_mode();
    }
    // 2. Mute woken by an idle line: three runs of four, 3 ms apart; the
    //    receiver muted after the first frame wakes on the idle after the
    //    first run.
    {
        link::Cfg c{};
        c.rate = 9600;
        link::Params a{};
        a.cfg = c;
        a.count = 4;
        a.aux16 = 3;
        a.aux8 = 30;
        a.ms = 400;
        a.pattern = link::pattern_counting;
        a.value = 0;
        const bool acted = peer_act(link::Op::blast, a);
        bool up = acted && dut_link(c) && U2::mute_mode(MuteConfig{MuteWake::idle_line, 0});
        uint16_t seen[12] = {};
        uint8_t n = 0;
        const uint32_t t0 = Ticker::ticks();
        while (Ticker::ticks() - t0 < 60u) {
            if (!U2::rx_ready()) continue;
            const uint16_t w = U2::read_word();
            if (n < 12u) seen[n++] = w;
            if (n == 1u) up = up && U2::mute();
        }
        print(serial, "  idle line: ", n, " frames:");
        for (uint8_t i = 0; i < n; ++i) print(serial, " ", seen[i]);
        print(serial, crlf);
        bool expected = n == 9u && seen[0] == 0u;
        for (uint8_t i = 1; expected && i < 9u; ++i) expected = seen[i] == static_cast<uint16_t>(i + 3u);
        link::Report r{};
        (void)peer_report(r, 0);
        judge("muted after the first frame, the receiver drops the rest of the run and wakes on the idle line",
              up && expected);
        U2::unmute();
        (void)link_command_mode();
    }
    // 3. IrDA, normal mode, between two WCH SIR codecs, both ways.
    {
        link::Cfg c{};
        c.rate = 115'200;
        c.mode = link::Mode::ircom;
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = bound_ms(115'200, 24);
        const bool acted = peer_act(link::Op::echo, a);
        const bool up = acted && dut_link(c);
        const uint16_t bad = up ? echo_mismatches(8, 0xFFu) : 8u;
        link::Report r{};
        const bool have = peer_report(r, 5);
        print(serial, "  IrDA at 115200: wrong=", bad, " peer count=", r.count, " ferr=", r.ferr, crlf);
        judge("IrDA's SIR frames echoed by a second WCH codec, every one equal",
              up && bad == 0u && have && r.count == 8u && r.ferr == 0u);
        U2::irda_off();
        (void)link_command_mode();
    }
    (void)link_command_mode();
    w_release();
}

}  // namespace two

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
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() {
    if (two::ring_active) {   // the two-board key's ring on column 4
        (void)two::LinkRing::dma_isr();
        return;
    }
    if (DmaUart::dma_isr()) {   // the receive ring's lap marks: an edge, as the USART's
        u2_edge_at = cycles_now();
        u2_edges = u2_edges + 1u;
    }
}
#endif   // BRIO_CH32_HAS_USART2

int main() {
    if constexpr (two_board_image) {
        crystal_ok = SysClock::init();
    }
    const bool clock_ok = crystal_ok || HsiClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
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
    bench.letter('q', "errors under the receive engine: breaks in a stream, nothing stolen", tq_errors);
    bench.letter('r', "tx_idle() against the last stop bit on the jumper", tr_tx_idle);
    bench.letter('s', "the burst edge from the vector, nothing polled", ts_edge);
    bench.letter('t', "the rate verbs: set_baud(), can_baud(), min_hz_for(), release()", tt_rate_verbs);
    bench.letter('u', "an overrun at a burst's tail under the receive engine: no storm", tu_overrun_tail);
    bench.letter('y', "host-assisted (brio stress): the console's own error counters", ty_host, false);
    bench.second_letter('1', "the CH32X035 peer: the rendezvous, its HSI calibrated against this crystal", two::t1_link);
    bench.second_letter('2', "the echo ladder 9600..3 Mbaud, each rate timed by the far chip", two::t2_rates);
    bench.second_letter('3', "the frame matrix across the wire, the odd stops' lengths", two::t3_frames);
    bench.second_letter('4', "the receiver's errors from the calibrated peer: PE, FE onset, glitches, ORE, LIN", two::t4_errors);
    bench.second_letter('5', "the peer's engine shapes against this end's pull-banger", two::t5_shapes);
    bench.second_letter('6', "the receive engine on channel 7 under the peer's blast, 1..3 Mbaud", two::t6_engine);
    bench.second_letter('7', "half duplex on PB0: drive_rxd heard, the wired AND", two::t7_half_duplex);
    bench.second_letter('8', "mute by address and by idle line, IrDA between two WCH codecs", two::t8_modes);
#endif
    bench.letter('k', "THE SYNCHRONOUS MODE on the console's USART1: CK counted by TIM2 off PD4, no wire", tk_synchronous);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", crystal_ok ? "PLL48 on the 24 MHz crystal" : (clock_ok ? "PLL48 on the HSI (NO CRYSTAL)" : "FAILED"),
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
