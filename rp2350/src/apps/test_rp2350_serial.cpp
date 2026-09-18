// test_rp2350_serial - the reference bench suite for the RP2350's UART
// (datasheet 12.1, the ARM PL011 r1p5 behind rp2350/uart.hpp and the IP
// stratum brio/pl011/): the resource `Pl011<n>` and the transport `Uart`
// beyond the console personality, on the second instance while the
// console keeps the first - and ON BOTH ARCHITECTURES from one source.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS UART1 ON GP4 (TX) / GP5 (RX), the console UART0 on
// GP0/GP1 through a Debug Probe's bridge. NOTHING TO WIRE for `z`: the
// PL011 has a LOOP-BACK (UARTCR.LBE) that feeds the transmitter into the
// receiver with the RX pad ignored, so every format, every rate of the
// ladder, the FIFOs, the receive timeout, a break and an overrun are
// measured on the instance alone.
//
// WHAT THIS CHIP ADDS TO THE RP2040'S SAME BLOCK, and what letter f is
// for: a SECOND FUNCTION COLUMN. The pads march in groups of four - TX,
// RX, CTS, RTS - and here the flow-control pads carry DATA TOO, under
// function 11: GP6 is UART1's transmitter as surely as GP4 is, and GP7
// its receiver. Twelve transmit pads and twelve receive pads per instance
// on the QFN-80, against the RP2040's four. Letter f drives UART1 out of
// the alternate pads and watches the pad itself.
//
// THE RULER IS THE PLATFORM TIMER (rp2350/mtime.hpp): the crystal's
// microseconds, for every latency and every rate on the loop.
//
// What is exercised, letter by letter:
//   a  the facts: the pin table's BOTH columns as the driver states them,
//      the divisor arithmetic against the registers (81 + 24/64 at
//      150 MHz for 115200), the floor and the ceiling of the rate, the
//      block's reset state, what init() leaves in the registers, the
//      refusals (a ninth bit, a third stop, a rate past clk_peri / 16)
//   b  THE INSTRUMENT: the loop-back proven, then every frame format
//      (5..8 bits, none/even/odd parity, 1 or 2 stop bits: 24 of them)
//      byte-exact on it at 115200
//   c  the baud ladder on the loop-back, from the floor (144 baud at
//      150 MHz) to the ceiling (9375000 = clk_peri / 16), each rung
//      byte-exact and timed against the frame count - plus the receive
//      timeout's 32 bit periods when the count is not a whole number of
//      FIFO levels
//   d  the FIFOs: a single byte delivered by the receive TIMEOUT against
//      a burst delivered by the level (the 16th byte), each timed; a
//      break sent and counted as BE with no byte delivered; an overrun
//      provoked with the line masked (48 frames into a 32-deep FIFO),
//      counted as OE with exactly the FIFO's depth delivered
//   e  bulk traffic: 4096 bytes through write_bulk/read_bulk on the loop
//      at each of five rungs to 3 Mbaud, the highest one that survives
//      whole reported, and the interrupts per hundred bytes counted there
//   f  THE SECOND FUNCTION COLUMN: UART1 moved onto GP6/GP7 under
//      function 11, the pads read back, THE TRANSMITTER WATCHED ON ITS
//      OWN PAD (an idle line is high, a break pulls it low, clearing it
//      lets it back up - read through the bank, which always reads the
//      pad whoever owns it), and the block byte-exact on the loop there
//
//   p  (by name only) THE WIRE against the peer board: 512 bytes out on
//      GP4, their echo read on GP5 byte-exact, no error counted
//   s  (by name only) THE ERROR ATTRIBUTION on the wire: this board
//      listens at 7E1 to the peer's 8N1 frames - the eighth data bit
//      lands where a parity bit is expected, so about half the frames are
//      parity errors, counted and dropped, and no frame error. An ECHO
//      cannot show this, since an echo returns the bits it was given, so
//      the peer SENDS for this letter: GP21 is a mode line into the peer,
//      low for echo and high for send
//   y  (by name only, brio stress) the CONSOLE's own ladder, host-fed:
//      115200 to 3 Mbaud through the probe's bridge, the bytes verified
//      on the board; the rungs to 921600 are verdicts, the ones above
//      measure the bridge and are printed
//   w  (by name only, brio stress) WHETHER THE BRIDGE HONOURS A PARITY
//      SETTING: the host announces 8E1 against the console's 8N1, and an
//      even-parity frame whose parity bit is low is a frame error here -
//      if the bridge sends what it was asked. The letter judges only that
//      the bytes arrived and PRINTS the attribution; the console's own
//      error counting is proven on the wire by s and on the loop by d
//
// The peer letters want the other board running its own fixed-role peer
// firmware, which needs no console of its own; the wire is this board's
// GP4 to the peer's RX and the peer's TX to this board's GP5, with GP21
// into the peer's mode line and a common ground.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;
using P = Rp2350Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins, 512, 256>;
constexpr Serial serial;

// The instrument: UART1 on GP4/GP5, the cross link's pins, under the
// FIRST function column.
constexpr UartPins instrument_pins{
    .tx = {4, PinFunction::uart},
    .rx = {5, PinFunction::uart},
};
using Instrument = Uart<1, instrument_pins, 512, 512>;
using U1 = Instrument::Resource;

// The same instance on the SECOND column: GP6 is UART1's transmitter and
// GP7 its receiver under function 11, the pads that carry CTS and RTS
// under function 2.
constexpr UartPins alt_pins{
    .tx = {6, PinFunction::uart_alt},
    .rx = {7, PinFunction::uart_alt},
};
using AltInstrument = Uart<1, alt_pins, 512, 512>;

using Led = Pin<25>;
using ModeLine = Pin<21>;   // into the peer: low = echo, high = send

TestBench<Serial, 16> bench;

volatile uint32_t u1_interrupts = 0;

/// UART1 wears TWO transport types in this suite - one instance on two
/// different pin sets - and they share the instance's one interrupt line.
/// Exactly one is up at a time, and this says which: a transport's
/// handler drains the FIFO into ITS OWN rings, so the wrong one would
/// swallow the other's traffic.
volatile bool alt_owns_u1 = false;

uint32_t us_now() { return Mtime::micros(); }

void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

void console_drain() {
    const uint32_t t0 = us_now();
    while (!Serial::tx_idle() && us_now() - t0 < 500'000u) {
    }
}

/// The pattern both ends of every stream generate: a 32-bit xorshift, low
/// byte per step, seeded 0x12345678 - the same as brio stress's and the
/// peer firmware's.
struct Lfsr {
    uint32_t state = 0x12345678u;
    void reset() { state = 0x12345678u; }
    uint8_t next() {
        uint32_t s = state;
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        state = s;
        return static_cast<uint8_t>(s & 0xFFu);
    }
};

constexpr uint8_t mask_for(UartBits bits) {
    return static_cast<uint8_t>((1u << static_cast<uint8_t>(bits)) - 1u);
}

/// Send `count` pattern bytes on a transport and read them back, masked
/// to `bits`, within `budget_us`. Returns how many came back exact before
/// the first wrong one or the deadline; sets `took_us` to the time from
/// the first write to the last byte read.
template <typename Port>
uint32_t round_trip(uint32_t count, UartBits bits, uint32_t budget_us,
                    uint32_t* took_us = nullptr) {
    const uint8_t mask = mask_for(bits);
    Lfsr tx;
    Lfsr rx;
    uint32_t sent = 0;
    uint32_t good = 0;
    bool wrong = false;
    const uint32_t t0 = us_now();
    uint32_t t_last = t0;
    while (good < count && !wrong && us_now() - t0 < budget_us) {
        while (sent < count) {
            uint8_t chunk[64];
            uint32_t n = 0;
            Lfsr probe{tx.state};
            while (n < sizeof chunk && sent + n < count) {
                chunk[n++] = static_cast<uint8_t>(probe.next() & mask);
            }
            const uint32_t queued = Port::write_bulk(std::span<const uint8_t>(chunk, n));
            for (uint32_t i = 0; i < queued; ++i) {
                (void)tx.next();
            }
            sent += queued;
            if (queued < n) {
                break;
            }
        }
        uint8_t got[64];
        const uint32_t n = Port::read_bulk(got);
        for (uint32_t i = 0; i < n && !wrong; ++i) {
            if (got[i] != (rx.next() & mask)) {
                wrong = true;
            } else {
                ++good;
                t_last = us_now();
            }
        }
    }
    if (took_us != nullptr) {
        *took_us = t_last - t0;
    }
    return good;
}

bool instrument_up(uint32_t baud, const UartFormat& fmt = {}, bool loop = true) {
    if (!Instrument::init(clock, baud, fmt)) {
        return false;
    }
    return !loop || Instrument::loopback(true);
}

const char* fmt_name(const UartFormat& f) {
    // bits, parity, stops as "8N1" - built on a small static so the
    // verdict names carry it.
    static char name[4];
    name[0] = static_cast<char>('0' + static_cast<uint8_t>(f.bits));
    name[1] = f.parity == UartParity::none ? 'N' : f.parity == UartParity::even ? 'E' : 'O';
    name[2] = static_cast<char>('0' + f.stop_bits);
    name[3] = '\0';
    return name;
}

// =============================================================================
// a - the facts
// =============================================================================
void ta_facts() {
    // The first column, as the RP2040 has it over a longer bank.
    static_assert(uart_tx_pin(1, 4) && uart_rx_pin(1, 5) && uart_tx_pin(0, 0) &&
                  uart_rx_pin(0, 1));
    static_assert(!uart_tx_pin(1, 0) && !uart_rx_pin(0, 5) && !uart_tx_pin(0, 1));
    // THE SECOND COLUMN, which the RP2040 has not: the third and fourth
    // pads of a group are that instance's transmitter and receiver under
    // function 11.
    static_assert(uart_tx_pin(1, 6) && uart_rx_pin(1, 7) && uart_tx_pin(0, 2) &&
                  uart_rx_pin(0, 3));
    static_assert(uart_pad_function(4) == PinFunction::uart &&
                  uart_pad_function(6) == PinFunction::uart_alt);
    bench.verdict("the groups of four cycle UART0, UART1, UART1, UART0 over the whole "
                  "bank: UART1 transmits on 4, 6, 8, 10, 20, 22 ... and receives on 5, 7, "
                  "9, 11, 21, 23 ...",
                  uart_tx_pin(1, 8) && uart_tx_pin(1, 10) && uart_tx_pin(1, 20) &&
                      uart_tx_pin(1, 22) && uart_rx_pin(1, 9) && uart_rx_pin(1, 11) &&
                      uart_rx_pin(1, 21) && !uart_tx_pin(1, 12) && !uart_rx_pin(1, 13));
    bench.verdict("and a pad is named under ITS OWN column: GP6 under function 2 is "
                  "UART1's CTS and not its transmitter, so that pin set is refused",
                  uart_pins_valid(1, alt_pins) &&
                      !uart_pins_valid(1, {.tx = {6, PinFunction::uart},
                                           .rx = {7, PinFunction::uart}}) &&
                      !uart_pins_valid(1, {.tx = {4, PinFunction::uart_alt},
                                           .rx = {5, PinFunction::uart}}));

    const auto d = uart_divisor(SysClock::pclk_hz, 115200);
    print(serial, "  115200 at ", SysClock::pclk_hz, " Hz: divisor ", d ? d->integer : 0u,
          " + ", d ? d->fraction : 0u, "/64 -> ",
          d ? uart_actual_baud(SysClock::pclk_hz, *d) : 0u, " baud", crlf);
    bench.verdict("the divisor at this chip's 150 MHz clk_peri is 81 + 24/64, which is "
                  "115207 baud - 60 ppm of the rate asked for",
                  d && d->integer == 81u && d->fraction == 24u &&
                      uart_actual_baud(SysClock::pclk_hz, *d) == 115207u);
    const auto ceiling = uart_divisor(SysClock::pclk_hz, 9'375'000);
    bench.verdict("the ceiling is clk_peri / 16 = 9375000 baud (divisor 1 + 0/64) and the "
                  "floor 144 baud; 143 and 9500000 are refused, and the generator's reach "
                  "runs a little past the nominal ceiling before the integer part rounds "
                  "to zero",
                  ceiling && ceiling->integer == 1u && ceiling->fraction == 0u &&
                      uart_divisor(SysClock::pclk_hz, 144).has_value() &&
                      !uart_divisor(SysClock::pclk_hz, 143).has_value() &&
                      !uart_divisor(SysClock::pclk_hz, 9'500'000).has_value() &&
                      uart_divisor(SysClock::pclk_hz, 9'448'818).has_value());
    bench.verdict("a ninth data bit and a third stop bit are not formats",
                  !uart_format_valid({.bits = static_cast<UartBits>(9)}) &&
                      !uart_format_valid({.stop_bits = 3}) &&
                      uart_format_valid({.bits = UartBits::five, .stop_bits = 2}));

    const bool reset_ok = U1::reset();
    print(serial, "  UART1 after reset: FR=", hex(U1::flags()), " CR enabled=", U1::enabled(),
          " IMSC=", hex(U1::interrupts()), " IBRD=", U1::divisor().integer, crlf);
    bench.verdict("the block's reset state: both FIFOs empty, disabled, no interrupt "
                  "enabled, divisor 0",
                  reset_ok &&
                      (U1::flags() & (UartFlag::rx_empty | UartFlag::tx_empty)) ==
                          (UartFlag::rx_empty | UartFlag::tx_empty) &&
                      !U1::enabled() && U1::interrupts() == 0u && U1::divisor().integer == 0u);
    bench.verdict("and the block has this chip's 32-deep FIFOs (12.1: 32x8 transmit, "
                  "32x12 receive)",
                  U1::fifo_depth == 32u);

    const bool up = Instrument::init(clock, 115200);
    print(serial, "  after init(): IBRD=", U1::divisor().integer, " FBRD=",
          U1::divisor().fraction, " FIFOs=", U1::fifos_enabled(), " IMSC=",
          hex(U1::interrupts()), " levels rx=", static_cast<uint8_t>(U1::rx_fifo_level()),
          " tx=", static_cast<uint8_t>(U1::tx_fifo_level()), crlf);
    bench.verdict("init() writes the divisor, enables the FIFOs at half/eighth, arms RX "
                  "and the receive timeout, enables the UART",
                  up && U1::divisor().integer == 81u && U1::divisor().fraction == 24u &&
                      U1::fifos_enabled() && U1::enabled() &&
                      U1::interrupts() == (UartInterrupt::rx | UartInterrupt::rx_timeout) &&
                      U1::rx_fifo_level() == UartFifoLevel::half &&
                      U1::tx_fifo_level() == UartFifoLevel::eighth);
    bench.verdict("the pads went to the UART under the first column, out of the ISOLATION "
                  "this chip's pads come up in",
                  Pin<4>::function() == PinFunction::uart &&
                      Pin<5>::function() == PinFunction::uart && !Pin<4>::isolated() &&
                      !Pin<5>::isolated());
    bench.verdict("can_baud: 9375000 yes, 9500000 no, 144 yes, 143 no",
                  Instrument::can_baud(SysClock::pclk_hz, 9'375'000) &&
                      !Instrument::can_baud(SysClock::pclk_hz, 9'500'000) &&
                      Instrument::can_baud(SysClock::pclk_hz, 144) &&
                      !Instrument::can_baud(SysClock::pclk_hz, 143));
    Instrument::release();
    bench.verdict("release() puts the block back into reset and the pads back to theirs - "
                  "which on this chip means ISOLATED again",
                  !U1::released() && Pin<4>::function() == PinFunction::none &&
                      Pin<4>::isolated());
}

// =============================================================================
// b - the instrument, and every frame format
// =============================================================================
void tb_loopback_formats() {
    bench.verdict("the instrument comes up with the loop-back on", instrument_up(115200));
    bench.verdict("UARTCR.LBE reads set", U1::loopback());
    const uint8_t hello[4] = {'b', 'r', 'i', 'o'};
    (void)Instrument::write_bulk(hello);
    spin_us(2000);
    uint8_t back[8] = {};
    const uint32_t n = Instrument::read_bulk(back);
    print(serial, "  4 bytes in, ", n, " back: ", static_cast<char>(back[0]),
          static_cast<char>(back[1]), static_cast<char>(back[2]), static_cast<char>(back[3]),
          crlf);
    bench.verdict("what the transmitter sends, the receiver gets, with the RX pad ignored",
                  n == 4u && back[0] == 'b' && back[1] == 'r' && back[2] == 'i' &&
                      back[3] == 'o');

    constexpr UartBits widths[] = {UartBits::five, UartBits::six, UartBits::seven,
                                   UartBits::eight};
    constexpr UartParity parities[] = {UartParity::none, UartParity::even, UartParity::odd};
    for (UartBits bits : widths) {
        bool all = true;
        print(serial, "  ", static_cast<uint8_t>(bits), " bits:");
        for (uint8_t stops = 1; stops <= 2; ++stops) {
            for (UartParity parity : parities) {
                const UartFormat f{.bits = bits, .parity = parity, .stop_bits = stops};
                const bool set = Instrument::set_format(f);
                Instrument::clear_errors();
                const uint32_t good = set ? round_trip<Instrument>(48, bits, 100'000u) : 0u;
                const bool ok = set && good == 48u && Instrument::frame_errors() == 0u &&
                                Instrument::parity_errors() == 0u;
                print(serial, " ", fmt_name(f), ok ? " ok" : " BAD");
                all = all && ok;
            }
        }
        print(serial, crlf);
        bench.verdict("every format of this width is byte-exact on the loop, no error "
                      "flagged",
                      all);
    }
    (void)Instrument::set_format({});
}

// =============================================================================
// c - the baud ladder
// =============================================================================
void tc_ladder() {
    if (!instrument_up(115200)) {
        bench.verdict("the instrument", false);
        return;
    }
    constexpr uint32_t rungs[] = {144, 300, 9600, 115200, 921600, 2'000'000, 4'000'000,
                                  9'375'000};
    for (uint32_t baud : rungs) {
        const bool set = Instrument::set_baud(SysClock::pclk_hz, baud);
        const uint32_t actual = Instrument::actual_baud(SysClock::pclk_hz);
        const uint32_t count = baud < 1000u ? 8u : 64u;
        // 8N1: ten bit periods a frame, at the rate the divisor really
        // gives - and the receive timeout's 32 bit periods when the last
        // bytes are not a whole FIFO level (the level delivers every 16).
        const uint32_t bit_periods = count * 10u + (count % 16u != 0u ? 32u : 0u);
        const uint32_t expect_us =
            static_cast<uint32_t>((static_cast<uint64_t>(bit_periods) * 1'000'000u) / actual);
        uint32_t took = 0;
        Instrument::clear_errors();
        const uint32_t good = set ? round_trip<Instrument>(count, UartBits::eight,
                                                           expect_us * 2u + 200'000u, &took)
                                  : 0u;
        print(serial, "  ", baud, " baud (", actual, " real): ", good, "/", count,
              " bytes in ", took, " us (", expect_us, " us of frames",
              count % 16u != 0u ? " and timeout)" : ")", crlf);
        // At least the frames' time LESS ONE BIT PERIOD, because a
        // receiver has the byte before the last stop bit has finished
        // elapsing; at most that plus the timeout, 10 % and a millisecond.
        // The one-bit allowance is not slack: at 150 MHz this chip's
        // divider hits 9600, 1 Mbaud, 2, 3 and 4 Mbaud EXACTLY, and an
        // exact rate leaves none of the rounding margin an inexact one
        // hides the difference in.
        const uint32_t frames_us =
            static_cast<uint32_t>((static_cast<uint64_t>(count) * 10u * 1'000'000u) / actual);
        const uint32_t bit_us = 1'000'000u / actual + 1u;
        bench.verdict("this rung is byte-exact and takes its frames' time: at least, less "
                      "the bit the receiver does not wait out, and at most the timeout, "
                      "10 % and a millisecond over",
                      set && good == count && took + bit_us >= frames_us &&
                          took <= expect_us + expect_us / 10u + 1000u &&
                          Instrument::frame_errors() == 0u && Instrument::hw_overruns() == 0u);
    }
    bench.verdict("set_baud refuses 143 and 9500000",
                  !Instrument::set_baud(SysClock::pclk_hz, 143) &&
                      !Instrument::set_baud(SysClock::pclk_hz, 9'500'000));
    (void)Instrument::set_baud(SysClock::pclk_hz, 115200);
}

// =============================================================================
// d - the FIFOs, a break, an overrun
// =============================================================================
void td_fifos() {
    if (!instrument_up(115200)) {
        bench.verdict("the instrument", false);
        return;
    }
    // One frame at 115207 baud is 86.8 us; the receive timeout is 32 bit
    // periods, 277.8 us, after the last frame.
    constexpr uint32_t frame_us = 87;
    uint8_t b = 0;
    const uint32_t t0 = us_now();
    (void)Instrument::write_byte(0xA5);
    while (!Instrument::read_byte(b) && us_now() - t0 < 10'000u) {
    }
    const uint32_t single = us_now() - t0;
    print(serial, "  one byte: delivered ", single, " us after the write (a frame ",
          frame_us, " us + the 32-bit receive timeout 278 us)", crlf);
    bench.verdict("a lone byte arrives by the receive timeout: after one frame plus 32 "
                  "bit periods, within 100 us",
                  b == 0xA5u && single >= frame_us + 278u - 20u &&
                      single <= frame_us + 278u + 100u);

    uint8_t burst[16];
    for (uint8_t i = 0; i < 16u; ++i) {
        burst[i] = static_cast<uint8_t>(0x30u + i);
    }
    const uint32_t t1 = us_now();
    (void)Instrument::write_bulk(burst);
    while (!Instrument::read_byte(b) && us_now() - t1 < 20'000u) {
    }
    const uint32_t first = us_now() - t1;
    uint32_t got = 1;
    while (got < 16u && us_now() - t1 < 20'000u) {
        if (Instrument::read_byte(b)) {
            ++got;
        }
    }
    print(serial, "  a burst of 16: the first byte delivered ", first,
          " us after the write, all ", got, " within ", us_now() - t1, " us", crlf);
    bench.verdict("a burst of the level's size is delivered by the LEVEL: the first byte "
                  "only when the sixteenth has landed (after 15 frames at least), no "
                  "timeout waited",
                  got == 16u && first >= 15u * frame_us && first <= 17u * frame_us + 20u);

    // THE BREAK: TX low for well over a frame, looped into the receiver.
    Instrument::clear_errors();
    U1::break_send(true);
    spin_us(1000);
    U1::break_send(false);
    spin_us(1000);
    uint8_t junk[8];
    const uint32_t after_break = Instrument::read_bulk(junk);
    print(serial, "  a 1 ms break: BE counted ", Instrument::break_errors(), " time(s), FE ",
          Instrument::frame_errors(), ", bytes delivered ", after_break, crlf);
    bench.verdict("a break is counted as BE and delivers no byte",
                  Instrument::break_errors() >= 1u && after_break == 0u);

    // THE OVERRUN: 48 frames into the 32-deep receive FIFO with the line
    // masked - the resource's own write, the transport's handler asleep.
    Instrument::clear_errors();
    (void)Instrument::set_baud(SysClock::pclk_hz, 921600);
    Irq::disable(U1::irq());
    for (uint8_t i = 0; i < 32u; ++i) {
        U1::write_data(i);
    }
    spin_us(400);   // 32 frames at 921600 = 347 us
    for (uint8_t i = 32; i < 48u; ++i) {
        U1::write_data(i);
    }
    spin_us(400);
    Irq::enable(U1::irq());
    Irq::set_pending(U1::irq());
    spin_us(200);
    uint8_t drained[64];
    const uint32_t delivered = Instrument::read_bulk(drained);
    bool in_order = delivered == 32u;
    for (uint32_t i = 0; i < delivered && in_order; ++i) {
        in_order = drained[i] == i;
    }
    print(serial, "  48 frames into the 32-deep FIFO, the line masked: OE counted ",
          Instrument::hw_overruns(), ", ", delivered, " bytes delivered", crlf);
    bench.verdict("the overrun is counted as OE and exactly the FIFO's depth is "
                  "delivered, in order",
                  Instrument::hw_overruns() >= 1u && in_order);
    (void)Instrument::set_baud(SysClock::pclk_hz, 115200);
}

// =============================================================================
// e - bulk traffic at 3 Mbaud
// =============================================================================
// A LONG RUN, not the sixty-four bytes letter c times: the handler has to
// empty a 32-deep FIFO faster than the wire fills it, for four thousand
// bytes together, with the code running out of the flash through the
// interface the bootrom left set up (03h serial reads at CLKDIV 12,
// datasheet 5.1.4 - there is no second-stage bootloader on this chip to
// make it faster, and no chapter here reprograms the QMI yet). The sweep
// says where that stops being true; the verdicts stand on the rung the
// sweep reaches.
void te_bulk() {
    constexpr uint32_t rungs[] = {460800, 921600, 1'000'000, 2'000'000, 3'000'000};
    uint32_t highest_exact = 0;
    uint32_t top_irqs = 0;
    uint32_t top_took = 0;
    for (uint32_t baud : rungs) {
        if (!instrument_up(baud)) {
            bench.verdict("the instrument at this rung", false);
            continue;
        }
        Instrument::clear_errors();
        u1_interrupts = 0;
        uint32_t took = 0;
        const uint32_t good = round_trip<Instrument>(4096, UartBits::eight, 2'000'000u, &took);
        const uint32_t irqs = u1_interrupts;
        const bool clean = good == 4096u && Instrument::hw_overruns() == 0u &&
                           Instrument::rx_overruns() == 0u && Instrument::frame_errors() == 0u;
        print(serial, "  4096 bytes at ", baud, " baud: ", good, " exact in ", took, " us (",
              static_cast<uint32_t>((static_cast<uint64_t>(good) * 1'000'000u) /
                                    (took ? took : 1u)),
              " bytes/s), ", irqs, " interrupts, OE ", Instrument::hw_overruns(), crlf);
        if (clean) {
            highest_exact = baud;
            top_irqs = irqs;
            top_took = took;
        }
    }
    print(serial, "  the highest rung a 4096-byte run survives whole: ", highest_exact,
          " baud, at ", (top_irqs * 100u) / 4096u, " interrupts per hundred bytes", crlf);
    bench.verdict("a 4096-byte run is byte-exact all the way to 3 Mbaud, with nothing "
                  "overrun and no frame lost: the handler keeps ahead of the wire even "
                  "running out of the flash the bootrom left at its slowest setting",
                  highest_exact >= 3'000'000u);
    // ON A LOOP-BACK the receive FIFO is fed at exactly the rate the
    // transmit FIFO empties, so neither ever runs far ahead of its trigger
    // level and the batching is nothing like the FIFO's depth. What the
    // number below measures is that pairing, not the block's best.
    bench.verdict("the FIFOs still batch the work: under one interrupt per three bytes at "
                  "the top rung, the two directions sharing one line",
                  top_irqs != 0u && top_irqs * 3u < 4096u);
    bench.verdict("and the wire's time is what it took: 40960 bit periods at 3 Mbaud is "
                  "13653 us, within 10 %",
                  top_took >= 13'000u && top_took <= 15'100u);
    Instrument::release();
}

// =============================================================================
// f - the second function column
// =============================================================================
void tf_second_column() {
    Instrument::release();
    alt_owns_u1 = true;   // the line is this transport's from here
    if (!AltInstrument::init(clock, 115200)) {
        bench.verdict("UART1 on its alternate pads", false);
        return;
    }
    bench.verdict("the same instance came up on GP6/GP7, the pads that carry its CTS and "
                  "RTS under the first column",
                  Pin<6>::function() == PinFunction::uart_alt &&
                      Pin<7>::function() == PinFunction::uart_alt);
    bench.verdict("and both pads are out of isolation, the receive one pulled up as this "
                  "chip's pads come up pulled DOWN",
                  !Pin<6>::isolated() && !Pin<7>::isolated());

    // THE PAD ITSELF is the witness: the bank always reads the pad,
    // whoever owns it, so an enabled transmitter idling high and a break
    // pulling it low prove that function 11 really routes the UART there.
    spin_us(2000);
    const bool idle_high = Pin<6>::read();
    U1::break_send(true);
    spin_us(2000);
    const bool break_low = !Pin<6>::read();
    U1::break_send(false);
    spin_us(2000);
    const bool back_high = Pin<6>::read();
    print(serial, "  GP6 as UART1's transmitter under function 11: idle reads ",
          idle_high ? "1" : "0", ", during a break ", break_low ? "0" : "1", ", after it ",
          back_high ? "1" : "0", crlf);
    bench.verdict("the transmitter really reaches the alternate pad: the line idles HIGH, "
                  "a break pulls it LOW, and clearing the break lets it back up - read "
                  "through the bank, on the pad the UART owns",
                  idle_high && break_low && back_high);

    AltInstrument::clear_errors();
    (void)AltInstrument::loopback(true);
    uint32_t took = 0;
    const uint32_t good = round_trip<AltInstrument>(256, UartBits::eight, 200'000u, &took);
    print(serial, "  256 bytes round the loop with the instance on its alternate pads: ",
          good, " exact in ", took, " us", crlf);
    bench.verdict("and the block is the same block there: 256 bytes byte-exact, no error",
                  good == 256u && AltInstrument::frame_errors() == 0u &&
                      AltInstrument::hw_overruns() == 0u);

    AltInstrument::release();
    alt_owns_u1 = false;
    bench.verdict("released, the alternate pads are isolated again with no owner",
                  Pin<6>::function() == PinFunction::none && Pin<6>::isolated() &&
                      Pin<7>::isolated());
}

// =============================================================================
// p, s - the two-board letters (by name)
// =============================================================================
void tp_wire() {
    ModeLine::output(false);   // the peer echoes while this is low
    spin_us(20'000);
    if (!instrument_up(115200, {}, false)) {
        bench.verdict("the instrument on the wire", false);
        return;
    }
    Instrument::clear_errors();
    uint32_t took = 0;
    const uint32_t good = round_trip<Instrument>(512, UartBits::eight, 2'000'000u, &took);
    print(serial, "  512 bytes to the peer and back on the cross link: ", good, " exact in ",
          took, " us; FE ", Instrument::frame_errors(), " PE ", Instrument::parity_errors(),
          " BE ", Instrument::break_errors(), " OE ", Instrument::hw_overruns(), crlf);
    bench.verdict("every byte comes back exact from the peer, no error counted",
                  good == 512u && Instrument::frame_errors() == 0u &&
                      Instrument::break_errors() == 0u && Instrument::hw_overruns() == 0u);
    Instrument::release();
    (void)ModeLine::release();
}

void ts_wire_errors() {
    if (!instrument_up(115200, {.bits = UartBits::seven, .parity = UartParity::even},
                       false)) {
        bench.verdict("the instrument at 7E1", false);
        return;
    }
    Instrument::clear_errors();
    print(serial, "  listening at 7E1; the peer is asked for its 64 frames at 8N1 (up to "
          "6 s) ...", crlf);
    ModeLine::output(false);
    spin_us(50'000);
    ModeLine::set();           // the rising edge is the peer's cue to send
    const uint32_t t0 = us_now();
    uint32_t accepted = 0;
    while (accepted + Instrument::parity_errors() + Instrument::frame_errors() < 64u &&
           us_now() - t0 < 6'000'000u) {
        uint8_t in[64];
        accepted += Instrument::read_bulk(in);
    }
    ModeLine::clear();
    print(serial, "  ", accepted, " bytes accepted, PE ", Instrument::parity_errors(),
          " FE ", Instrument::frame_errors(), " BE ", Instrument::break_errors(), crlf);
    bench.verdict("the peer's eighth data bit lands where this end expects parity: parity "
                  "errors counted and those frames dropped, no frame error",
                  Instrument::parity_errors() >= 8u && Instrument::frame_errors() == 0u &&
                      accepted + Instrument::parity_errors() == 64u);
    Instrument::release();
    (void)ModeLine::release();
}

// =============================================================================
// y, w - the host-assisted letters (brio stress), by name
// =============================================================================
//
// The board prints one "HOST op mode baud format window count" line, the
// script moves its own port to that rate and frame, runs the op for LESS
// than the window and goes quiet before the board speaks again
// (cli/bench/stress.py).

uint32_t host_window_t0 = 0;
uint32_t host_window_ms = 0;

void host_announce(const char* op, uint32_t baud, const char* fmt, uint32_t window_ms,
                   uint32_t count) {
    print(serial, "HOST ", op, " 0 ", baud, " ", fmt, " ", window_ms, " ", count, crlf);
    console_drain();
    host_window_t0 = us_now();
    host_window_ms = window_ms;
    spin_us(50'000);
}

void host_settle() {
    spin_us(20'000);
    (void)Serial::set_baud(SysClock::pclk_hz, 115200);
    const uint32_t due = (host_window_ms + 900u) * 1000u;
    while (us_now() - host_window_t0 < due) {
    }
    uint8_t junk[64];
    while (Serial::read_bulk(junk) != 0u) {
    }
    Serial::clear_errors();
}

/// One sink leg: the host pumps the pattern at `baud`, this end verifies.
/// Returns the bytes matched before the first wrong one.
uint32_t host_sink(uint32_t baud, uint32_t window_ms, uint32_t* received, bool* wrong) {
    host_announce("sink", baud, "8N1", window_ms, 0);
    (void)Serial::set_baud(SysClock::pclk_hz, baud);
    Serial::clear_errors();
    Lfsr rx;
    uint32_t got = 0;
    uint32_t good = 0;
    *wrong = false;
    while (us_now() - host_window_t0 < window_ms * 1000u) {
        uint8_t buf[64];
        const uint32_t n = Serial::read_bulk(buf);
        for (uint32_t i = 0; i < n; ++i) {
            ++got;
            if (!*wrong && buf[i] == rx.next()) {
                ++good;
            } else {
                *wrong = true;
            }
        }
    }
    *received = got;
    return good;
}

void ty_console_ladder() {
    constexpr uint32_t rungs[] = {115200, 460800, 921600, 1'000'000, 2'000'000, 3'000'000};
    for (uint32_t baud : rungs) {
        uint32_t got = 0;
        bool wrong = false;
        const uint32_t good = host_sink(baud, 1200, &got, &wrong);
        const uint8_t fe = Serial::frame_errors();
        const uint8_t oe = Serial::hw_overruns();
        const uint8_t ro = Serial::rx_overruns();
        host_settle();
        print(serial, "  ", baud, " baud from the host: ", got, " bytes, ", good, " exact",
              wrong ? " then WRONG" : "", "; FE ", fe, " OE ", oe, " ring overruns ", ro,
              crlf);
        if (baud <= 921600u) {
            bench.verdict("this rung of the console is byte-exact through the probe's "
                          "bridge",
                          got > 1000u && !wrong && fe == 0u && oe == 0u && ro == 0u);
        }
    }
}

void tw_console_errors() {
    host_announce("burst", 115200, "8E1", 1000, 0);
    Serial::clear_errors();
    uint32_t got = 0;
    while (us_now() - host_window_t0 < 1'000'000u) {
        uint8_t buf[64];
        got += Serial::read_bulk(buf);
    }
    const uint8_t fe = Serial::frame_errors();
    const uint8_t pe = Serial::parity_errors();
    host_settle();
    print(serial, "  the host announced 8E1 against the console's 8N1: ", got,
          " bytes accepted, FE ", fe, " PE ", pe, " - the bridge ",
          fe > 0u ? "HONOURS the parity setting" : "sends 8N1 regardless of it", crlf);
    bench.verdict("the host's burst arrived through the bridge (the attribution above is "
                  "the bridge's finding, not a verdict)",
                  got > 1000u && pe == 0u);
}

void banner() {
    print(serial, crlf, "test_rp2350_serial - the RP2350 UART (datasheet 12.1, the ARM "
          "PL011) on ",
          core_kind == CoreKind::hazard3 ? "RISC-V Hazard3" : "Arm Cortex-M33",
          ", the instrument UART1 on GP4/GP5, clk_peri=", SysClock::pclk_hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_uart1() {
    u1_interrupts = u1_interrupts + 1;
    if (alt_owns_u1) {
        (void)AltInstrument::isr();
    } else {
        (void)Instrument::isr();
    }
}
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the facts: both pin columns, the divisor, the reset state", ta_facts);
    bench.letter('b', "THE INSTRUMENT: the loop-back, every frame format",
                 tb_loopback_formats);
    bench.letter('c', "the baud ladder on the loop-back, floor to ceiling", tc_ladder);
    bench.letter('d', "the FIFOs: timeout vs level, a break, an overrun", td_fifos);
    bench.letter('e', "bulk traffic on the loop, five rungs to 3 Mbaud", te_bulk);
    bench.letter('f', "THE SECOND FUNCTION COLUMN: UART1 on GP6/GP7", tf_second_column);
    bench.letter('p', "the wire against the peer, 512 bytes", tp_wire, false);
    bench.letter('s', "the error attribution on the wire: 7E1 against the peer's 8N1",
                 ts_wire_errors, false);
    bench.letter('y', "HOST: the console's ladder through the probe's bridge (brio stress)",
                 ty_console_ladder, false);
    bench.letter('w', "HOST: the console at 8N1 fed 8E1 (brio stress)", tw_console_errors,
                 false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " ruler=", ruler_ok ? "1us" : "FAILED", " tick=",
                    tick_ok ? "1kHz" : "FAILED", brio::crlf);
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
