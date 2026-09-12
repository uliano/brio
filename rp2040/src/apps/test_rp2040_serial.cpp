// test_rp2040_serial - the reference bench suite for the RP2040's UART
// (datasheet 4.2, the ARM PL011 behind rp2040/uart.hpp): the resource
// `Pl011<n>` and the transport `Uart` beyond the console personality,
// on the second instance while the console keeps the first.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS UART1 ON GP4 (TX) / GP5 (RX), the console UART0 on
// GP0/GP1 through the Debug Probe's bridge. NOTHING TO WIRE for `z`: the
// PL011 has a LOOP-BACK (UARTCR.LBE) that feeds the transmitter into
// the receiver with the RX pad ignored, so every format, every rate of
// the ladder, the FIFOs, the receive timeout, a break and an overrun
// are measured on the instance alone. The two letters that need a
// second RP2040 use the cross link (this board's GP4 to the other's
// GP5 and back, a common ground); the two that need the host use the
// console itself, with `brio stress` at the other end.
//
// THE RULER IS THE SYSTEM TIMER (rp2040/timer.hpp): the crystal's
// microseconds, for every latency and every rate on the loop.
//
// What is exercised, letter by letter:
//   a  the facts: table 279's pin bonding as the driver states it, the
//      divisor arithmetic against the registers (67 + 52/64 at 125 MHz
//      for 115200, the datasheet's own), the floor and the ceiling of
//      the rate, the block's reset state, what init() leaves in the
//      registers, the refusals (a ninth bit, a third stop, a rate above
//      clk_peri / 16)
//   b  THE INSTRUMENT: the loop-back proven, then every frame format
//      (5..8 bits, none/even/odd parity, 1 or 2 stop bits: 24 of them)
//      byte-exact on it at 115200
//   c  the baud ladder on the loop-back, from the floor (120 baud at
//      125 MHz) to the ceiling (7.8125 Mbaud = clk_peri / 16), each rung
//      byte-exact and timed against the frame count - plus the receive
//      timeout's 32 bit periods when the count is not a whole number of
//      FIFO levels (a run of 8 is delivered by the timeout, not the
//      level: at 120 baud that is 267 ms of the measure)
//   d  the FIFOs: a single byte delivered by the receive TIMEOUT (32 bit
//      periods after it) against a burst delivered by the level (the
//      16th byte), each timed; a break sent and counted as BE with no
//      byte delivered; an overrun provoked with the line masked (48
//      frames into a 32-deep FIFO), counted as OE with exactly the
//      FIFO's depth delivered
//   e  bulk traffic: 4096 bytes at 3 Mbaud through write_bulk/read_bulk
//      on the loop, byte-exact, the interrupts per byte counted
//
//   q  (by name only) THE PEER: UART1 echoes whatever it receives, at
//      115200 8N1, for thirty seconds after the prompt returns - run it
//      on the other board first
//   p  (by name only) the wire against the peer: 512 bytes sent on GP4,
//      their echo read on GP5 byte-exact, no error counted
//   r  (by name only) THE PEER'S WRONG FRAMES: three seconds after the
//      prompt returns, UART1 sends 64 pattern bytes at 115200 8N1 - run
//      it on the other board first
//   s  (by name only) the error attribution on the wire: this board
//      listens at 7E1 to the peer's 8N1 frames - the eighth data bit
//      lands where a parity bit is expected, so about half the frames
//      are parity errors, counted and dropped, and no frame error (an
//      ECHO could not show this: an echo returns the same bits, parity
//      included, which is why r sends rather than echoes)
//   y  (by name only, brio stress) the CONSOLE's own ladder, host-fed:
//      115200 to 3 Mbaud through the Debug Probe's bridge, the bytes
//      verified on the board; the rungs to 921600 are verdicts, the
//      ones above measure the bridge and are printed
//   w  (by name only, brio stress) WHETHER THE BRIDGE HONOURS A PARITY
//      SETTING: the host announces 8E1 against the console's 8N1, and
//      an even-parity frame whose parity bit is low is a frame error
//      here - if the bridge sends what it was asked. The Debug Probe's
//      bridge sends 8N1 regardless (measured: every frame accepted, FE
//      zero), so the letter judges only that the bytes arrived and
//      PRINTS the attribution; the console's own error counting is
//      proven on the wire by s and on the loop by d
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <span>

#include "rp2040/clock.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;
using P = Rp2040Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins, 512, 256>;
constexpr Serial serial;

// The instrument: UART1 on GP4/GP5, the cross link's pins.
constexpr UartPins instrument_pins{
    .tx = {4, PinFunction::uart},
    .rx = {5, PinFunction::uart},
};
using Instrument = Uart<1, instrument_pins, 512, 512>;
using U1 = Instrument::Resource;

using Led = Pin<25>;

TestBench<Serial, 16> bench;

volatile uint32_t u1_interrupts = 0;

uint32_t us_now() { return Timer::now_low(); }

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

/// The pattern both ends of every stream generate: a 32-bit xorshift,
/// low byte per step, seeded 0x12345678 - the same as brio stress's.
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

/// Send `count` pattern bytes on the instrument and read them back,
/// masked to `bits`, within `budget_us`. Returns how many came back
/// exact before the first wrong one or the deadline; sets `took_us` to
/// the time from the first write to the last byte read.
uint32_t round_trip(uint32_t count, UartBits bits, uint32_t budget_us, uint32_t* took_us = nullptr) {
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
            uint32_t peek = tx.state;
            Lfsr probe{peek};
            while (n < sizeof chunk && sent + n < count) {
                chunk[n++] = static_cast<uint8_t>(probe.next() & mask);
            }
            const uint32_t queued = Instrument::write_bulk(std::span<const uint8_t>(chunk, n));
            for (uint32_t i = 0; i < queued; ++i) {
                (void)tx.next();
            }
            sent += queued;
            if (queued < n) {
                break;
            }
        }
        uint8_t got[64];
        const uint32_t n = Instrument::read_bulk(got);
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

// -----------------------------------------------------------------------------
void ta_facts() {
    static_assert(uart_tx_pin(1, 4) && uart_rx_pin(1, 5) && uart_tx_pin(0, 0) && uart_rx_pin(0, 1));
    static_assert(!uart_tx_pin(1, 0) && !uart_rx_pin(0, 5) && !uart_tx_pin(0, 2));
    bench.verdict("table 279: UART1 transmits on 4, 8, 20, 24 and receives on 5, 9, 21, 25",
                  uart_tx_pin(1, 8) && uart_tx_pin(1, 20) && uart_tx_pin(1, 24) &&
                      uart_rx_pin(1, 9) && uart_rx_pin(1, 21) && uart_rx_pin(1, 25) &&
                      !uart_tx_pin(1, 12) && !uart_rx_pin(1, 13));
    const auto d = uart_divisor(SysClock::pclk_hz, 115200);
    print(serial, "  115200 at ", SysClock::pclk_hz, " Hz: divisor ", d ? d->integer : 0u, " + ",
          d ? d->fraction : 0u, "/64 -> ", d ? uart_actual_baud(SysClock::pclk_hz, *d) : 0u, " baud",
          crlf);
    bench.verdict("the divisor is the datasheet's own 67 + 52/64 (115207 baud)",
                  d && d->integer == 67u && d->fraction == 52u &&
                      uart_actual_baud(SysClock::pclk_hz, *d) == 115207u);
    const auto ceiling = uart_divisor(SysClock::pclk_hz, 7'812'500);
    const auto floor = uart_divisor(SysClock::pclk_hz, 120);
    bench.verdict("the ceiling is clk_peri / 16 = 7812500 baud (divisor 1 + 0/64) and the "
                  "floor 120 baud; 119 and 7900000 are refused (a rate within half a "
                  "sixty-fourth of the ceiling rounds onto it)",
                  ceiling && ceiling->integer == 1u && ceiling->fraction == 0u && floor &&
                      !uart_divisor(SysClock::pclk_hz, 119) &&
                      !uart_divisor(SysClock::pclk_hz, 7'900'000) &&
                      uart_divisor(SysClock::pclk_hz, 7'812'501).has_value());
    bench.verdict("a ninth data bit and a third stop bit are not formats",
                  !uart_format_valid({.bits = static_cast<UartBits>(9)}) &&
                      !uart_format_valid({.stop_bits = 3}) && uart_format_valid({.bits = UartBits::five, .stop_bits = 2}));

    const bool reset_ok = U1::reset();
    print(serial, "  UART1 after reset: FR=", hex(U1::flags()), " CR enabled=", U1::enabled(),
          " IMSC=", hex(U1::interrupts()), " IBRD=", U1::divisor().integer, crlf);
    bench.verdict("the block's reset state: both FIFOs empty, disabled, no interrupt "
                  "enabled, divisor 0",
                  reset_ok && (U1::flags() & (UartFlag::rx_empty | UartFlag::tx_empty)) ==
                                  (UartFlag::rx_empty | UartFlag::tx_empty) &&
                      !U1::enabled() && U1::interrupts() == 0u && U1::divisor().integer == 0u);

    const bool up = Instrument::init(clock, 115200);
    print(serial, "  after init(): IBRD=", U1::divisor().integer, " FBRD=", U1::divisor().fraction,
          " FIFOs=", U1::fifos_enabled(), " IMSC=", hex(U1::interrupts()), " levels rx=",
          static_cast<uint8_t>(U1::rx_fifo_level()), " tx=", static_cast<uint8_t>(U1::tx_fifo_level()),
          crlf);
    bench.verdict("init() writes the divisor, enables the FIFOs at half/eighth, arms RX and "
                  "the receive timeout, enables the UART",
                  up && U1::divisor().integer == 67u && U1::divisor().fraction == 52u &&
                      U1::fifos_enabled() && U1::enabled() &&
                      U1::interrupts() == (UartInterrupt::rx | UartInterrupt::rx_timeout) &&
                      U1::rx_fifo_level() == UartFifoLevel::half &&
                      U1::tx_fifo_level() == UartFifoLevel::eighth);
    bench.verdict("the pads went to the UART function",
                  Pin<4>::function() == PinFunction::uart && Pin<5>::function() == PinFunction::uart);
    bench.verdict("can_baud: 7812500 yes, 8000000 no, 120 yes, 100 no",
                  Instrument::can_baud(SysClock::pclk_hz, 7'812'500) &&
                      !Instrument::can_baud(SysClock::pclk_hz, 8'000'000) &&
                      Instrument::can_baud(SysClock::pclk_hz, 120) &&
                      !Instrument::can_baud(SysClock::pclk_hz, 100));
    Instrument::release();
    bench.verdict("release() puts the block back into reset and the pads to their reset state",
                  !U1::released() && Pin<4>::function() == PinFunction::none);
}

// -----------------------------------------------------------------------------
void tb_loopback_formats() {
    bench.verdict("the instrument comes up with the loop-back on", instrument_up(115200));
    bench.verdict("UARTCR.LBE reads set", U1::loopback());
    const uint8_t hello[4] = {'b', 'r', 'i', 'o'};
    (void)Instrument::write_bulk(hello);
    spin_us(2000);
    uint8_t back[8] = {};
    const uint32_t n = Instrument::read_bulk(back);
    print(serial, "  4 bytes in, ", n, " back: ", static_cast<char>(back[0]), static_cast<char>(back[1]),
          static_cast<char>(back[2]), static_cast<char>(back[3]), crlf);
    bench.verdict("what the transmitter sends, the receiver gets, with the RX pad ignored",
                  n == 4u && back[0] == 'b' && back[1] == 'r' && back[2] == 'i' && back[3] == 'o');

    constexpr UartBits widths[] = {UartBits::five, UartBits::six, UartBits::seven, UartBits::eight};
    constexpr UartParity parities[] = {UartParity::none, UartParity::even, UartParity::odd};
    for (UartBits bits : widths) {
        bool all = true;
        print(serial, "  ", static_cast<uint8_t>(bits), " bits:");
        for (uint8_t stops = 1; stops <= 2; ++stops) {
            for (UartParity parity : parities) {
                const UartFormat f{.bits = bits, .parity = parity, .stop_bits = stops};
                const bool set = Instrument::set_format(f);
                Instrument::clear_errors();
                const uint32_t good = set ? round_trip(48, bits, 100'000u) : 0u;
                const bool ok = set && good == 48u && Instrument::frame_errors() == 0u &&
                                Instrument::parity_errors() == 0u;
                print(serial, " ", fmt_name(f), ok ? " ok" : " BAD");
                all = all && ok;
            }
        }
        print(serial, crlf);
        bench.verdict("every format of this width is byte-exact on the loop, no error flagged", all);
    }
    (void)Instrument::set_format({});
}

// -----------------------------------------------------------------------------
void tc_ladder() {
    if (!instrument_up(115200)) {
        bench.verdict("the instrument", false);
        return;
    }
    constexpr uint32_t rungs[] = {120, 300, 9600, 115200, 921600, 2'000'000, 4'000'000, 7'812'500};
    for (uint32_t baud : rungs) {
        const bool set = Instrument::set_baud(SysClock::pclk_hz, baud);
        const uint32_t actual = Instrument::actual_baud(SysClock::pclk_hz);
        const uint32_t count = baud < 1000u ? 8u : 64u;
        // 8N1: ten bit periods a frame, at the rate the divisor really
        // gives - and the receive timeout's 32 bit periods when the last
        // bytes are not a whole FIFO level (the level delivers every 16).
        const uint32_t bit_periods = count * 10u + (count % 16u != 0u ? 32u : 0u);
        const uint32_t expect_us = static_cast<uint32_t>(
            (static_cast<uint64_t>(bit_periods) * 1'000'000u) / actual);
        uint32_t took = 0;
        Instrument::clear_errors();
        const uint32_t good = set ? round_trip(count, UartBits::eight, expect_us * 2u + 200'000u, &took) : 0u;
        print(serial, "  ", baud, " baud (", actual, " real): ", good, "/", count, " bytes in ", took,
              " us (", expect_us, " us of frames", count % 16u != 0u ? " and timeout)" : ")", crlf);
        // At least the frames' time (a loop cannot be early); at most
        // that plus the timeout, 10 % and a millisecond of slack.
        const uint32_t frames_us = static_cast<uint32_t>(
            (static_cast<uint64_t>(count) * 10u * 1'000'000u) / actual);
        bench.verdict("this rung is byte-exact and takes its frames' time: at least, and at "
                      "most the timeout, 10 % and a millisecond over",
                      set && good == count && took >= frames_us &&
                          took <= expect_us + expect_us / 10u + 1000u &&
                          Instrument::frame_errors() == 0u && Instrument::hw_overruns() == 0u);
    }
    bench.verdict("set_baud refuses 119 and 8000000",
                  !Instrument::set_baud(SysClock::pclk_hz, 119) &&
                      !Instrument::set_baud(SysClock::pclk_hz, 8'000'000));
    (void)Instrument::set_baud(SysClock::pclk_hz, 115200);
}

// -----------------------------------------------------------------------------
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
    print(serial, "  one byte: delivered ", single, " us after the write (a frame ", frame_us,
          " us + the 32-bit receive timeout 278 us)", crlf);
    bench.verdict("a lone byte arrives by the receive timeout: after one frame plus 32 bit "
                  "periods, within 100 us",
                  b == 0xA5u && single >= frame_us + 278u - 20u && single <= frame_us + 278u + 100u);

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
    print(serial, "  a burst of 16: the first byte delivered ", first, " us after the write, all ",
          got, " within ", us_now() - t1, " us", crlf);
    bench.verdict("a burst of the level's size is delivered by the LEVEL: the first byte only "
                  "when the sixteenth has landed (after 15 frames at least), no timeout waited",
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
    Nvic::disable(U1::irq());
    for (uint8_t i = 0; i < 32u; ++i) {
        U1::write_data(i);
    }
    spin_us(400);   // 32 frames at 921600 = 347 us
    for (uint8_t i = 32; i < 48u; ++i) {
        U1::write_data(i);
    }
    spin_us(400);
    Nvic::enable(U1::irq());
    Nvic::set_pending(U1::irq());
    spin_us(200);
    uint8_t drained[64];
    const uint32_t delivered = Instrument::read_bulk(drained);
    bool in_order = delivered == 32u;
    for (uint32_t i = 0; i < delivered && in_order; ++i) {
        in_order = drained[i] == i;
    }
    print(serial, "  48 frames into the 32-deep FIFO, the line masked: OE counted ",
          Instrument::hw_overruns(), ", ", delivered, " bytes delivered", crlf);
    bench.verdict("the overrun is counted as OE and exactly the FIFO's depth is delivered, in "
                  "order",
                  Instrument::hw_overruns() >= 1u && in_order);
    (void)Instrument::set_baud(SysClock::pclk_hz, 115200);
}

// -----------------------------------------------------------------------------
void te_bulk() {
    if (!instrument_up(3'000'000)) {
        bench.verdict("the instrument at 3 Mbaud", false);
        return;
    }
    Instrument::clear_errors();
    u1_interrupts = 0;
    uint32_t took = 0;
    const uint32_t good = round_trip(4096, UartBits::eight, 500'000u, &took);
    const uint32_t irqs = u1_interrupts;
    print(serial, "  4096 bytes at 3 Mbaud on the loop: ", good, " exact in ", took, " us (",
          static_cast<uint32_t>((static_cast<uint64_t>(good) * 1'000'000u) / (took ? took : 1u)),
          " bytes/s), ", irqs, " interrupts = ", (irqs * 100u) / 4096u, " per hundred bytes", crlf);
    bench.verdict("4096 bytes round the loop byte-exact at 3 Mbaud with no error counted",
                  good == 4096u && Instrument::hw_overruns() == 0u && Instrument::rx_overruns() == 0u &&
                      Instrument::frame_errors() == 0u);
    bench.verdict("the FIFOs batch the work: under one interrupt per four bytes",
                  irqs * 4u < 4096u);
    bench.verdict("and the wire's time is what it took: 40960 bit periods = 13653 us, within 10 %",
                  took >= 13'000u && took <= 15'100u);
}

// -----------------------------------------------------------------------------
// The two-board letters (by name): the peer's echo lives in the main loop.

uint32_t peer_until = 0;
uint32_t peer_send_at = 0;   // r: when the 64 frames go out (0 = never)

void peer_service() {
    if (peer_send_at != 0u && static_cast<int32_t>(us_now() - peer_send_at) >= 0) {
        peer_send_at = 0;
        Lfsr tx;
        uint8_t out[64];
        for (uint8_t& b : out) {
            b = tx.next();
        }
        (void)Instrument::write_bulk(out);
    }
    if (peer_until == 0u) {
        return;
    }
    if (static_cast<int32_t>(us_now() - peer_until) >= 0) {
        peer_until = 0;
        Instrument::release();
        return;
    }
    if (peer_send_at != 0u) {
        return;   // the sender's window: nothing to echo
    }
    uint8_t buf[64];
    const uint32_t n = Instrument::read_bulk(buf);
    if (n != 0u) {
        (void)Instrument::write_bulk(std::span<const uint8_t>(buf, n));
    }
}

void tr_peer_sender() {
    const bool up = instrument_up(115200, {}, false);
    peer_send_at = us_now() + 3'000'000u;
    if (peer_send_at == 0u) {
        peer_send_at = 1;
    }
    peer_until = peer_send_at + 5'000'000u;
    print(serial, "  UART1 sends 64 pattern bytes at 115200 8N1 on GP4 in three seconds", crlf);
    bench.verdict("the peer is up", up);
}

void tq_peer() {
    const bool up = instrument_up(115200, {}, false);
    Instrument::clear_errors();
    peer_until = us_now() + 30'000'000u;
    if (peer_until == 0u) {
        peer_until = 1;
    }
    print(serial, "  UART1 echoes on GP4 what arrives on GP5 at 115200 8N1 for the next 30 s",
          crlf);
    bench.verdict("the peer is up", up);
}

void tp_wire() {
    if (!instrument_up(115200, {}, false)) {
        bench.verdict("the instrument on the wire", false);
        return;
    }
    Instrument::clear_errors();
    uint32_t took = 0;
    const uint32_t good = round_trip(512, UartBits::eight, 2'000'000u, &took);
    print(serial, "  512 bytes to the peer and back on the cross link: ", good, " exact in ", took,
          " us; FE ", Instrument::frame_errors(), " PE ", Instrument::parity_errors(), " BE ",
          Instrument::break_errors(), " OE ", Instrument::hw_overruns(), crlf);
    bench.verdict("every byte comes back exact from the peer, no error counted",
                  good == 512u && Instrument::frame_errors() == 0u && Instrument::break_errors() == 0u &&
                      Instrument::hw_overruns() == 0u);
    Instrument::release();
}

void ts_wire_errors() {
    if (!instrument_up(115200, {.bits = UartBits::seven, .parity = UartParity::even}, false)) {
        bench.verdict("the instrument at 7E1", false);
        return;
    }
    Instrument::clear_errors();
    print(serial, "  listening at 7E1 for the peer's 64 frames at 8N1 (up to 6 s) ...", crlf);
    const uint32_t t0 = us_now();
    uint32_t accepted = 0;
    while (accepted + Instrument::parity_errors() + Instrument::frame_errors() < 64u &&
           us_now() - t0 < 6'000'000u) {
        uint8_t in[64];
        accepted += Instrument::read_bulk(in);
    }
    print(serial, "  ", accepted, " bytes accepted, PE ", Instrument::parity_errors(), " FE ",
          Instrument::frame_errors(), " BE ", Instrument::break_errors(), crlf);
    bench.verdict("the peer's eighth data bit lands where this end expects parity: parity "
                  "errors counted and those frames dropped, no frame error",
                  Instrument::parity_errors() >= 8u && Instrument::frame_errors() == 0u &&
                      accepted + Instrument::parity_errors() == 64u);
    Instrument::release();
}

// -----------------------------------------------------------------------------
// The host-assisted letters (brio stress), by name.
//
// The board prints one "HOST op mode baud format window count" line,
// the script moves its own port to that rate and frame, runs the op
// for LESS than the window and goes quiet before the board speaks
// again (cli/bench/stress.py).

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
              wrong ? " then WRONG" : "", "; FE ", fe, " OE ", oe, " ring overruns ", ro, crlf);
        if (baud <= 921600u) {
            bench.verdict("this rung of the console is byte-exact through the probe's bridge",
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
    print(serial, "  the host announced 8E1 against the console's 8N1: ", got, " bytes accepted, FE ",
          fe, " PE ", pe, " - the bridge ", fe > 0u ? "HONOURS the parity setting" :
          "sends 8N1 regardless of it", crlf);
    bench.verdict("the host's burst arrived through the bridge (the attribution above is the "
                  "bridge's finding, not a verdict)",
                  got > 1000u && pe == 0u);
}

void banner() {
    print(serial, crlf, "test_rp2040_serial - the RP2040 UART (datasheet 4.2, PL011), the instrument "
          "UART1 on GP4/GP5, clk_peri=", SysClock::pclk_hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_uart1() {
    u1_interrupts = u1_interrupts + 1;
    (void)Instrument::isr();
}
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the facts: pins, divisor, reset state, refusals", ta_facts);
    bench.letter('b', "THE INSTRUMENT: the loop-back, every frame format", tb_loopback_formats);
    bench.letter('c', "the baud ladder on the loop-back, floor to ceiling", tc_ladder);
    bench.letter('d', "the FIFOs: timeout vs level, a break, an overrun", td_fifos);
    bench.letter('e', "bulk traffic at 3 Mbaud on the loop", te_bulk);
    bench.letter('q', "PEER: echo on UART1 for 30 s (run on the other board first)", tq_peer, false);
    bench.letter('p', "the wire against the peer, 512 bytes", tp_wire, false);
    bench.letter('r', "PEER: 64 frames at 8N1 on UART1 in three seconds (the other board first)", tr_peer_sender, false);
    bench.letter('s', "the error attribution on the wire: listening at 7E1 to the peer's 8N1", ts_wire_errors, false);
    bench.letter('y', "HOST: the console's ladder through the probe's bridge (brio stress)", ty_console_ladder, false);
    bench.letter('w', "HOST: the console at 8N1 fed 8E1 (brio stress)", tw_console_errors, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED",
                    " timer=", timer_ok ? "1us" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        peer_service();
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
