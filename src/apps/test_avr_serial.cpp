// test_avr_serial - the USART test SUITE for the AVR DA/DB target, in
// two halves.
//
// SINGLE BOARD (a..i, `z`): instance and route handling (including the
// pinless NONE route and the teardown), the whole frame-format matrix
// through the internal loop-back, the receive FIFO's overflow, the
// multiprocessor filter, the TXC/DRE semantics, an ELECTRICAL check of
// the fractional baud generator (the start bit measured by a TCB
// through the event system), the clock rebase, auto-baud and Host SPI.
//
// TWO BOARDS (j..u and w, `y`): board B runs `usart_peer` and is driven
// IN BAND over the very link under test (protocol: usart_link.hpp) -
// the baud and frame matrices cross the wire, errors are injected on
// purpose, a cycle-counted bit-banger makes waveforms no clean UART can
// produce, a foreign clock feeds auto-baud, the synchronous roles run
// on a real XCK, RS-485's XDIR guard time is measured, IRCOM's pulses
// are proven on the wire, and the loop-back receiver is asked whether
// it can hear an external driver on its own TXD pad.
//
// Reference test of avrdx/usart.hpp (docs/avrdx/usart.md): keep it
// passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800 - USART2 is never reconfigured here.
// The RTC PIT drives brio::Ticker: every two-board bound is milliseconds.
//
// Wires: the single-board half needs none - everything that needs a
// loop-back runs on USART4 at its default position (TXD PE0, RXD PE1,
// XCK PE2) with LBME, and the internal loop-back is taken at the TXD
// PAD, so a pinless route receives nothing (bench finding, and what
// usart.hpp now refuses). The two-board half needs the campaign wiring
// A.PE0-B.PE1, A.PE1-B.PE0, A.PE2-B.PE2, GND-GND, with `usart_peer`
// flashed on board B. Test `w` alone wants a single PE0-PE0 wire
// instead and is therefore NOT part of `y`.
// USART0's default route is NOT usable on this board: TXD would be PA0,
// the 24 MHz crystal pin. The per-instance loop-back smoke therefore
// uses USART0 ALT1 (PA4/PA5), USART1 default (PC0/PC1) and USART3 ALT1
// (PB4/PB5); on this desk PC0 and PB4 are traffic LEDs (harmless) and
// PA4 doubles as SPI0 MOSI and traffic button 2 - do not hold a button
// down while test b runs.
//
// Commands: ? for the menu, z = all single-board, y = all two-board.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

#include "usart_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

using U0 = Usart<0>;
using U1 = Usart<1>;
using U3 = Usart<3>;
using U4 = Usart<4>;

using TxPin = Pin<'E', 0>;                 // USART4 TXD, default route
using RxPin = Pin<'E', 1>;                 // USART4 RXD
using XckPin = Pin<'E', 2>;                // USART4 XCK
using ChTx = EventChannel<4>;              // PORTE/PORTF pin events: channels 4-5

// The start-bit meter: a TCB measuring the LOW pulse on the TX line.
using T0 = Tcb<0>;
using Meter = PulseWidthMeter<T0>;
volatile uint16_t last_width = 0;
volatile uint16_t captures = 0;

// The transport task, used only by the rebase test (its ISRs are bound
// unconditionally; they are harmless while USART4 is configured as a
// bare resource, because the resource never enables those interrupts).
using U4Tx = Uart<4, Route::def, 32, 32>;
using DynClock = DynamicClock<SysClock, Serial, U4Tx, Meter>;

uint8_t passed = 0, failed = 0;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
void verdict(const char* a, const char* b, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", a, b, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

/// Everything back to a known quiet state: USART0/1/3/4 released (pins
/// handed to PORT), the meter and its channels off.
void quiesce() {
    U0::release();
    U1::release();
    U3::release();
    U4::release();
    T0::disable();
    T0::enable_capt_interrupt(false);
    ChTx::off();
    EventChannel<5>::off();
    captures = 0;
    last_width = 0;
}

/// USART4 in internal loop-back on its default route: the transmitter
/// feeds the receiver through the TXD pad (PE0 also carries the frames
/// out to the neighbour board's input, which is harmless).
bool lbme4(const UsartFormat& f, uint32_t baud, UsartRxMode rx = UsartRxMode::normal,
           bool mpcm = false) {
    const uint16_t r = usart_baud_reg(SysClock::hz, baud, usart_samples(UsartMode::async, rx));
    if (r == 0) return false;
    return U4::init({.route = UsartRoute::def, .bits = f.bits, .parity = f.parity,
                     .two_stop = f.two_stop, .rx_mode = rx, .baud = r,
                     .loop_back = true, .multiprocessor = mpcm});
}

/// One frame out and back through the loop-back.
std::optional<UsartFrame> roundtrip(uint16_t v) {
    if (!U4::send(v)) return {};
    return U4::wait();
}

/// "5N1", "9LE2": the frame format as a label.
char fmt_buf[8];
const char* fmt_name(const UsartFormat& f) {
    uint8_t i = 0;
    fmt_buf[i++] = static_cast<char>('0' + usart_data_bits(f.bits));
    if (usart_is_9bit(f.bits)) {
        fmt_buf[i++] = f.bits == UsartBits::nine_low_first ? 'L' : 'H';
    }
    fmt_buf[i++] = f.parity == UsartParity::none ? 'N'
                 : (f.parity == UsartParity::even ? 'E' : 'O');
    fmt_buf[i++] = f.two_stop ? '2' : '1';
    fmt_buf[i] = 0;
    return fmt_buf;
}

// ---- a: instances, routes, teardown ------------------------------------------

/// Every instance this package has, pinless: the peripheral runs with
/// PORTMUX at NONE, which is what lets the matrix below touch USART0,
/// USART1 and USART3 on a board whose PA0 carries a crystal.
template <typename U>
void pinless_smoke(const char* name) {
    const bool ok = U::init({.route = UsartRoute::none,
                             .baud = usart_baud_reg(SysClock::hz, 115'200u)});
    verdict(name, " pinless init", ok);
    verdict(name, " PORTMUX reads NONE", U::routed() == UsartRoute::none);
    verdict(name, " both directions enabled", U::rx_enabled() && U::tx_enabled());
    U::disable();
    verdict(name, " disable() stops both", !U::rx_enabled() && !U::tx_enabled());
    U::release();
}

void ta_instances() {
    print(serial, "a instances and routes (USART0/1/3 pinless, USART4 routed and released)", crlf);
    quiesce();
    pinless_smoke<U0>("USART0");
    pinless_smoke<U1>("USART1");
    pinless_smoke<U3>("USART3");
    pinless_smoke<U4>("USART4");

    // USART4's default route claims PE0 (TXD, driven) and PE1 (RXD, input).
    const bool ok = U4::init({.route = UsartRoute::def,
                              .baud = usart_baud_reg(SysClock::hz, 115'200u)});
    verdict("USART4 default route init", ok);
    verdict("PORTMUX reads DEFAULT", U4::routed() == UsartRoute::def);
    verdict("TXD PE0 driven as an output", TxPin::is_output());
    verdict("RXD PE1 left an input", !RxPin::is_output());
    verdict("the route table names PE0/PE1/PE2/PE3",
            U4::txd(UsartRoute::def).port == 'E' && U4::txd(UsartRoute::def).pin == 0 &&
            U4::rxd(UsartRoute::def).pin == 1 && U4::xck(UsartRoute::def).pin == 2 &&
            U4::xdir(UsartRoute::def).pin == 3);
    verdict("this package has no USART4 ALT1", !U4::has_route(UsartRoute::alt1));
    U4::release();
    verdict("release() routes back to NONE", U4::routed() == UsartRoute::none);
    verdict("release() hands PE0 back as an input", !TxPin::is_output());
    quiesce();
}

// ---- b: the frame-format matrix in loop-back ---------------------------------

/// Send a few patterns that exercise every bit of the character size
/// (the ninth included) and check they come back untouched and clean.
bool frame_ok(const UsartFormat& f) {
    const uint8_t bits = usart_data_bits(f.bits);
    const uint16_t mask = static_cast<uint16_t>((1u << bits) - 1u);
    const uint16_t patterns[5] = {
        static_cast<uint16_t>(0x000),
        static_cast<uint16_t>(mask),
        static_cast<uint16_t>(0x155u & mask),
        static_cast<uint16_t>(0x0AAu & mask),
        static_cast<uint16_t>(0x101u & mask),
    };
    for (uint16_t p : patterns) {
        const auto got = roundtrip(p);
        if (!got || !got->clean() || got->data != p) return false;
    }
    return true;
}

/// One instance, one route, 8N1 through its own TXD pad.
template <typename U>
bool smoke(UsartRoute route) {
    if (!U::init({.route = route, .baud = usart_baud_reg(SysClock::hz, 115'200u),
                  .loop_back = true})) {
        return false;
    }
    for (uint8_t k = 0; k < 4; ++k) {
        const uint8_t v = static_cast<uint8_t>(0x5A + k);
        if (!U::send(v)) return false;
        const auto got = U::wait();
        if (!got || !got->clean() || got->data != v) return false;
    }
    U::release();
    return true;
}

void tb_frames() {
    print(serial, "b frame formats in loop-back on USART4 (default route, 115200)", crlf);
    quiesce();
    const UsartBits sizes[6] = {UsartBits::five, UsartBits::six, UsartBits::seven,
                                UsartBits::eight, UsartBits::nine_low_first,
                                UsartBits::nine_high_first};
    const UsartParity pars[3] = {UsartParity::none, UsartParity::even, UsartParity::odd};
    for (UsartBits b : sizes) {
        for (UsartParity p : pars) {
            for (uint8_t s = 0; s < 2; ++s) {
                const UsartFormat f{.bits = b, .parity = p, .two_stop = s != 0};
                if (!lbme4(f, 115'200u)) { verdict("init ", fmt_name(f), false); continue; }
                verdict("USART4 ", fmt_name(f), frame_ok(f));
            }
        }
    }
    // The other instances get an 8N1 pass: same code path, other
    // silicon, and a real TXD pad each (see the header for the routes).
    verdict("USART0 ALT1 8N1 loop-back", smoke<U0>(UsartRoute::alt1));
    verdict("USART1 default 8N1 loop-back", smoke<U1>(UsartRoute::def));
    verdict("USART3 ALT1 8N1 loop-back", smoke<U3>(UsartRoute::alt1));
    quiesce();
}

// ---- c: the receive FIFO overflows -------------------------------------------

void tc_overflow() {
    print(serial, "c BUFOVF: two buffered frames plus the shift register, then loss", crlf);
    quiesce();
    if (!lbme4({}, 115'200u)) { verdict("init", false); quiesce(); return; }
    // Six frames go out back to back with nothing read.
    for (uint8_t i = 0; i < 6; ++i) {
        (void)U4::send(static_cast<uint16_t>(0xA0 + i));
    }
    (void)U4::wait_line_idle();
    delay_us(clock, 500);

    UsartFrame got[8];
    uint8_t cnt = 0;
    while (U4::rxc_flag() && cnt < 8) {
        got[cnt++] = U4::receive();
    }
    print(serial, "  drained ", cnt, " frames:");
    for (uint8_t i = 0; i < cnt; ++i) {
        print(serial, " ", hex(got[i].data), got[i].overflow ? "(OVF)" : "");
    }
    print(serial, crlf);
    verdict("three frames survive (two buffered + the shifter)", cnt == 3);
    verdict("the two buffered frames are the two oldest",
            cnt == 3 && got[0].data == 0xA0 && got[1].data == 0xA1);
    // Bench: the shift register is NOT frozen at the third frame - it
    // keeps taking new ones while the buffer stays full, so the third
    // read is the LAST frame on the line, not the third one sent.
    verdict("the third is the last frame received, not the third sent",
            cnt == 3 && got[2].data == 0xA5);
    verdict("BUFOVF marks that frame and only that one",
            cnt == 3 && got[2].overflow && !got[0].overflow && !got[1].overflow);

    // Draining is the recovery: the next frame is clean.
    const auto again = roundtrip(0x5A);
    verdict("clean reception again after draining",
            again && again->clean() && again->data == 0x5A);
    quiesce();
}

// ---- d: the multiprocessor filter --------------------------------------------

void td_mpcm() {
    print(serial, "d MPCM: 9-bit address frames pass, data frames are dropped", crlf);
    quiesce();
    const UsartFormat f9{.bits = UsartBits::nine_low_first};
    if (!lbme4(f9, 115'200u, UsartRxMode::normal, /*mpcm=*/true)) {
        verdict("init", false); quiesce(); return;
    }
    (void)U4::send(0x055);                       // bit 8 = 0: a data frame
    (void)U4::wait_line_idle();
    delay_us(clock, 300);
    verdict("a data frame is filtered out", !U4::rxc_flag());

    (void)U4::send(0x133);                       // bit 8 = 1: an address frame
    const auto addr = U4::wait();
    verdict("the address frame passes", addr && addr->data == 0x133);

    U4::multiprocessor(false);                   // "this client is addressed"
    const auto data = roundtrip(0x055);
    verdict("data frames arrive once MPCM is cleared", data && data->data == 0x055);

    U4::multiprocessor(true);                    // back to listening for addresses
    (void)U4::send(0x066);
    (void)U4::wait_line_idle();
    delay_us(clock, 300);
    verdict("filtered again after re-arming MPCM", !U4::rxc_flag());
    quiesce();
}

// ---- e: what DRE and TXC actually mean ---------------------------------------

/// Spin until `f()` or the budget runs out; returns the iterations used
/// (budget + 1 means it never happened).
template <typename F>
uint32_t spin_until(F f, uint32_t budget = 400'000u) {
    for (uint32_t i = 0; i <= budget; ++i) {
        if (f()) return i;
    }
    return budget + 1;
}

void te_txc_dre() {
    print(serial, "e DRE = TXDATA accepts, TXC = the line went idle (9600)", crlf);
    quiesce();
    if (!lbme4({}, 9600u)) { verdict("init", false); quiesce(); return; }
    verdict("DREIF is set on an idle transmitter", U4::dre_flag());
    U4::clear_txc();
    verdict("TXCIF cleared by a plain write-one store", !U4::txc_flag());

    // No printing between the writes and the measurement: a console line
    // at 460800 lasts longer than a frame at 9600 and every flag would
    // already have settled.
    //
    // THREE frames are queued, not two: the transmit path is TXDATA, the
    // buffer and the shift register (27.3.2.3), and with only two loaded
    // the "TXDATA is full" state lasts a few dozen CPU cycles - long
    // enough to read once, far too short to survive an interrupt landing
    // in the middle. With all three slots taken DREIF stays low for a
    // whole frame time and the measurement is a fact rather than a race.
    // send() waits for DREIF BEFORE each write, so every frame really
    // lands in its own slot; three bare stores to TXDATA in a row are a
    // race - the second can arrive before the first has been handed on
    // and simply overwrite it.
    (void)U4::send(0xAA);                        // straight through to the shifter
    (void)U4::send(0x55);                        // into the buffer
    (void)U4::send(0x3C);                        // into TXDATA: all three slots full
    delay_us(clock, 20);                         // the last write lands
    const bool dre_busy = !U4::dre_flag();
    const bool txc_busy = !U4::txc_flag();
    const uint32_t to_dre = spin_until([] { return U4::dre_flag(); });
    const bool txc_at_dre = U4::txc_flag();
    const uint32_t to_txc = spin_until([] { return U4::txc_flag(); });

    verdict("DREIF low with three frames queued", dre_busy);
    verdict("TXCIF still low while shifting", txc_busy);
    print(serial, "  spins to DREIF: ", to_dre, ", then to TXCIF: ", to_txc, crlf);
    verdict("DREIF returns while the line is still busy", to_dre > 0 && !txc_at_dre);
    verdict("TXCIF only once the last frame has left", to_txc > 0);

    const auto a = U4::poll();
    const auto b = U4::poll();
    const auto c = U4::poll();
    verdict("all three frames arrived",
            a && b && c && a->data == 0xAA && b->data == 0x55 && c->data == 0x3C);
    U4::clear_txc();
    quiesce();
}

// ---- f: the baud generator, measured on the wire ------------------------------
// TXD is driven out on PE0 and read back through the event system into
// a TCB in pulse-width mode: with a stream of 0xFF frames the start bit
// is the only low pulse on the line, so the measured low time IS one
// bit time. INVEN is deliberately NOT used (it would invert the
// physical line, not just the measurement): the TCB's own edge
// selection measures the low pulse.

void clear_captures() {
    cli();
    captures = 0;
    last_width = 0;
    sei();
}

bool wait_captures(uint16_t n, uint16_t ms) {
    for (uint16_t i = 0; i < ms; ++i) {
        if (captures >= n) return true;
        delay_us(clock, 1000);
    }
    return captures >= n;
}

void measure_baud(uint32_t baud, bool clk2x) {
    const uint8_t s = usart_samples(UsartMode::async,
                                    clk2x ? UsartRxMode::clk2x : UsartRxMode::normal);
    const uint16_t reg = usart_baud_reg(SysClock::hz, baud, s);
    if (reg == 0) { verdict("baud not expressible", false); return; }
    // The receiver stays off: PE1 hangs on the neighbour board's input.
    const bool ok = U4::init({.route = UsartRoute::def,
                              .rx_mode = clk2x ? UsartRxMode::clk2x : UsartRxMode::normal,
                              .baud = reg, .rx = false});
    if (!ok) { verdict("init", false); return; }
    Meter::init(clock, ChTx{}, TcbClock::div1, /*low=*/true);
    clear_captures();
    for (uint16_t i = 0; i < 200 && captures < 8; ++i) {
        (void)U4::send(0xFF);
    }
    (void)wait_captures(8, 200);
    const uint16_t w = last_width;
    // Nominal bit time in tenths of a CLK_PER tick: S x BAUD / 64.
    const uint32_t exp10 = (static_cast<uint32_t>(reg) * s * 10u) / 64u;
    const uint32_t real = usart_actual_baud(SysClock::hz, reg, s);
    print(serial, "  ", baud, clk2x ? " (CLK2X)" : "", ": BAUD=", reg,
          " actual=", real, " bit=", exp10 / 10u, ".", exp10 % 10u,
          " ticks, measured=", w, crlf);
    verdict("start bit within 2 ticks of the divisor",
            captures >= 8 && near(static_cast<int32_t>(w) * 10, static_cast<int32_t>(exp10), 20));
    T0::enable_capt_interrupt(false);
    T0::disable();
    U4::release();
}

void tf_baud_on_the_wire() {
    print(serial, "f baud generator measured on PE0 (start bit -> EvPin -> TCB0 PW meter)", crlf);
    quiesce();
    ChTx::source(EvPin<TxPin>{});
    measure_baud(9600u, false);
    measure_baud(115'200u, false);
    measure_baud(460'800u, false);
    measure_baud(1'000'000u, false);
    measure_baud(460'800u, true);
    quiesce();
}

// ---- g: the clock rebase under traffic ---------------------------------------

void tg_rebase() {
    print(serial, "g rebase 24 -> 12 -> 24 MHz with USART4 among the clock's users", crlf);
    quiesce();
    ChTx::source(EvPin<TxPin>{});
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());
    U4Tx::init(DynClock{}, 115'200u);
    U4::loop_back(true);                          // TXD still drives PE0 for the meter
    Meter::init(DynClock{}, ChTx{}, TcbClock::div1, /*low=*/true);

    // 0xFF frames only: the start bit is then the ONLY low pulse on the
    // line, so what the meter reads is exactly one bit time.
    auto exchange = []() {
        (void)U4Tx::write_byte(0xFF);
        for (uint32_t i = 0; i < 400'000u; ++i) {
            uint8_t b;
            if (U4Tx::read_byte(b)) return b == 0xFF;
        }
        return false;
    };

    clear_captures();
    bool clean24 = true;
    for (uint8_t i = 0; i < 8; ++i) clean24 = clean24 && exchange();
    verdict("loop-back clean at 24 MHz", clean24);
    (void)wait_captures(4, 100);
    const uint16_t t24 = last_width;
    const uint32_t us24 = Meter::us(t24);

    verdict("switch to 12 MHz", DynClock::set(12'000'000u));
    clear_captures();
    bool clean12 = true;
    for (uint8_t i = 0; i < 8; ++i) clean12 = clean12 && exchange();
    verdict("loop-back clean at 12 MHz", clean12);
    (void)wait_captures(4, 200);
    const uint16_t t12 = last_width;
    const uint32_t us12 = Meter::us(t12);

    print(serial, "  24 MHz: ", t24, " ticks = ", us24, " us; 12 MHz: ", t12,
          " ticks = ", us12, " us", crlf);
    verdict("ticks follow CLK_PER (halved +-2)", near(t12 * 2, t24, 4));
    verdict("the bit time stands still (+-1 us)", near(static_cast<int32_t>(us12),
                                                       static_cast<int32_t>(us24), 1));
    verdict("back to 24 MHz", DynClock::set(24'000'000u));
    clear_captures();
    bool back = true;
    for (uint8_t i = 0; i < 8; ++i) back = back && exchange();
    verdict("loop-back clean again at 24 MHz", back);
    (void)wait_captures(4, 100);
    verdict("bit time restored (+-2 ticks)", near(last_width, t24, 2));

    U4::enable_rxc_interrupt(false);
    quiesce();
}

// ---- h: auto-baud in loop-back ------------------------------------------------
// GENAUTO with WFB accepts a break of any length, so the transmitter's
// own 0x00 frame can play the break and the following 0x55 the sync
// field. Then the deliberate failure: a sync field measured outside the
// accepted 0x0064..0xFFFF window sets ISFIF, and errata 2.16.3 says the
// receiver stays dead until RXEN is toggled - which is exactly what
// AutoBaud::recover() does.

using Ab = AutoBaud<4, UsartRoute::def>;

/// The instance in an auto-baud mode WITH the loop-back on from the
/// first store: configuring it in two steps would leave the receiver
/// listening to the floating RXD pin for a moment, and a noise edge
/// there eats the armed WFB (observed on this bench).
bool autobaud_init(uint32_t baud, bool lin) {
    const uint16_t r = usart_baud_reg(SysClock::hz, baud);
    if (r == 0) return false;
    if (!U4::init({.route = UsartRoute::def,
                   .rx_mode = lin ? UsartRxMode::linauto : UsartRxMode::genauto,
                   .baud = r, .loop_back = true})) {
        return false;
    }
    U4::flush_rx();
    U4::clear_break();
    U4::clear_isf();
    return true;
}

/// A break frame (0x00: nine low bit times) followed by a sync
/// character, sent by this instance's own transmitter into its own
/// receiver. Short of a real LIN break, but enough for GENAUTO once
/// WFB is armed.
void send_break_and_sync(uint8_t sync) {
    (void)U4::send(0x00);
    (void)U4::send(sync);
    (void)U4::wait_line_idle();
    delay_us(clock, 4000);
}

/// A REAL break: the transmitter is switched off (TXD goes back to
/// PORT), the pin is held low by hand for `bits` bit times and then
/// released, which is the only way a single board can produce the 12+
/// low bit times LINAUTO insists on. Then the sync character.
void manual_break_and_sync(uint8_t sync, uint32_t bit_us, uint8_t bits) {
    U4::enable_tx(false);                        // the pad is PORT's again
    TxPin::set();
    TxPin::output();
    delay_us(clock, bit_us * 2);
    TxPin::clear();
    delay_us(clock, bit_us * bits);
    TxPin::set();
    delay_us(clock, bit_us * 2);
    U4::enable_tx(true);                         // the USART overrides it again
    (void)U4::send(sync);
    (void)U4::wait_line_idle();
    delay_us(clock, 4000);
}

void th_autobaud() {
    print(serial, "h auto-baud (GENAUTO) in loop-back and the ISFIF recovery", crlf);
    quiesce();
    const uint16_t nominal = usart_baud_reg(SysClock::hz, 19'200u);
    verdict("GENAUTO init at 19200", autobaud_init(19'200u, /*lin=*/false));
    Ab::arm_break();                             // WFB is write-only: no readback
    send_break_and_sync(0x55);
    const uint16_t measured = Ab::measured_baud_reg();
    print(serial, "  nominal BAUD=", nominal, " after the sync field BAUD=", measured,
          " (", Ab::measured_baud(SysClock::hz), " baud), BDF=", Ab::break_detected() ? 1 : 0,
          ", ISFIF=", Ab::sync_error() ? 1 : 0, crlf);
    verdict("a break plus sync field was detected", Ab::break_detected());
    verdict("no sync-field error", !Ab::sync_error());
    verdict("the measured BAUD is the sender's within 1 %",
            near(measured, nominal, nominal / 100));
    // Whatever the counter's scale, transmitter and receiver share BAUD,
    // so the link must still round-trip after the update.
    U4::flush_rx();
    const auto after = roundtrip(0xC3);
    verdict("the link still round-trips at the measured rate",
            after && after->clean() && after->data == 0xC3);

    // LINAUTO insists the sync character be 0x55 (27.3.3.2.5): anything
    // else sets ISFIF, which is the deterministic way to reach errata
    // 2.16.3 from a loop-back.
    print(serial, "  LINAUTO with a sync character that is not 0x55", crlf);
    (void)autobaud_init(19'200u, /*lin=*/true);
    manual_break_and_sync(0x53, 1'000'000u / 19'200u + 1u, 16);
    const bool isf = Ab::sync_error();
    print(serial, "  ISFIF=", isf ? 1 : 0, ", BAUD=", Ab::measured_baud_reg(), crlf);
    verdict("ISFIF on an inconsistent sync field", isf);
    if (isf) {
        // Errata 2.16.3: the receiver is dead and clearing the flag alone
        // does not revive it.
        U4::clear_isf();
        U4::rx_mode(UsartRxMode::normal);
        U4::baud_reg(nominal);
        U4::flush_rx();
        (void)U4::send(0x3C);
        (void)U4::wait_line_idle();
        delay_us(clock, 2000);
        verdict("clearing ISFIF alone leaves the receiver dead", !U4::rxc_flag());
        Ab::recover();
        U4::flush_rx();
        (void)U4::send(0x3C);
        const auto alive = U4::wait();
        verdict("RXEN toggled: the receiver is back", alive && alive->data == 0x3C);
    }
    quiesce();
}

// ---- i: Host SPI mode ---------------------------------------------------------
// Compile-complete and exercised through the internal loop-back only:
// MOSI (TXD) is fed back to the receiver, so a transfer must return
// what it sent. The electrical side (a real client on XCK/TXD/RXD)
// belongs to the SPI campaign.

using Mspi = MspiHost<4, UsartRoute::def>;

bool mspi_pass(bool lsb_first, bool trailing) {
    if (!Mspi::init(clock, 1'000'000u, {.lsb_first = lsb_first, .sample_trailing = trailing})) {
        return false;
    }
    U4::loop_back(true);
    const uint8_t patterns[4] = {0x00, 0xFF, 0x5A, 0xA5};
    for (uint8_t p : patterns) {
        const auto got = Mspi::transfer(p);
        if (!got || *got != p) return false;
    }
    return true;
}

void ti_mspi() {
    print(serial, "i Host SPI mode on USART4 (XCK PE2, MOSI looped back internally)", crlf);
    quiesce();
    verdict("XCK PE2 is bonded on this package", U4::xck(UsartRoute::def).bonded);
    verdict("MSB first, sample leading", mspi_pass(false, false));
    verdict("XCK PE2 driven as an output", XckPin::is_output());
    verdict("LSB first, sample leading", mspi_pass(true, false));
    verdict("MSB first, sample trailing", mspi_pass(false, true));
    verdict("LSB first, sample trailing", mspi_pass(true, true));
    verdict("the SPI rate is CLK_PER / (2 x BAUD[15:6])",
            U4::actual_baud(SysClock::hz) == 1'000'000u);
    quiesce();
}

// ==============================================================================
//  THE TWO-BOARD HALF (j .. u, w): board B runs `usart_peer` and is driven
//  IN BAND over the very link under test - the protocol is
//  src/apps/usart_link.hpp. Command mode is async 8N1 at 115200 on both
//  boards; every command that changes the link carries a frame count and a
//  millisecond deadline after which the peer restores command mode by
//  itself, so a test that loses the thread recovers by retrying instead of
//  hanging.
// ==============================================================================

using XdirPin = Pin<'E', 3>;               // USART4 XDIR: DUT-local, not wired
using Ch2 = EventChannel<5>;               // the second PORTE/PORTF channel

// The tasks the two-board half puts on the wire.
using Irda = IrdaLink<4, UsartRoute::def>;
using Bus = Rs485<4, UsartRoute::def>;
using SyncH = SyncHost<4, UsartRoute::def>;
using SyncC = SyncClient<4, UsartRoute::def>;
using Line = OneWire<4, UsartRoute::def>;

link::Decoder dec;
const uint8_t no_payload[1] = {0};

/// Which wiring this desk has (usart_link.hpp Topology). Found once by
/// ensure_link() and then reused: the crossed full-duplex pair the
/// campaign assumes, or a single wire between the two TXD pads.
link::Topology topo = link::Topology::full_duplex;
bool topo_known = false;
bool link_quiet = false;      ///< suppress the failure dump while probing
bool shared_line() { return topo == link::Topology::shared; }

void put4(uint8_t b) { (void)U4::send(b, 200'000u); }

/// Half-duplex turnaround guard. RXCIF is raised at the MIDDLE of the
/// stop bit - half a bit BEFORE the sender's TXCIF - so an end that
/// answers at once starts its start bit while the other end is still
/// transmitting with its receiver off, and the answer's first bits are
/// swallowed (measured: two bits lost at 2400 baud, the receiver then
/// locking onto a later low). Wait four bit times, derived from the
/// BAUD register in force, before taking the line.
void turnaround_guard() {
    const uint32_t bit_cycles =
        (static_cast<uint32_t>(U4::baud_reg()) * U4::samples()) / 64u;
    delay_cycles(4u * (bit_cycles ? bit_cycles : 1u));
}

/// Take the shared wire: receiver off (LBME would otherwise decode our
/// own echo), pin driven, transmitter on. On the full-duplex wiring only
/// the guard and the flag clear are left.
void line_talk() {
    turnaround_guard();
    if (shared_line()) {
        U4::enable_rx(false);
        TxPin::set();
        TxPin::output();
        U4::enable_tx(true);
    }
    U4::clear_txc();            // so line_listen waits for THIS burst
}

/// Give it back to the pull-up and listen again. It OWNS the wait for
/// the line to go idle: TXCIF is write-one-to-clear, so a caller that
/// waits first and then calls this would spin out its whole budget with
/// the receiver still off - and miss the answer.
void line_listen(bool wait = true) {
    if (wait) (void)U4::wait_line_idle(200'000u);
    if (!shared_line()) return;
    U4::enable_tx(false);
    TxPin::input();
    pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    U4::flush_rx();
    U4::enable_rx(true);
}

void xck_invert(bool on) {
    volatile uint8_t& c = pinctrl_of('E', 2);
    if (on) c |= PORT_INVEN_bm;
    else c &= static_cast<uint8_t>(~PORT_INVEN_bm);
}

/// Command mode, the state everything returns to.
bool link_command_mode() {
    xck_invert(false);
    const bool ok = U4::init({.route = UsartRoute::def,
                              .baud = usart_baud_reg(SysClock::hz, link::command_baud),
                              .tx = !shared_line(),
                              .loop_back = shared_line()});
    if (shared_line()) {
        TxPin::input();
        pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    } else {
        pinctrl_of('E', 1) |= PORT_PULLUPEN_bm;
    }
    U4::flush_rx();
    U4::clear_txc();
    dec.reset();
    return ok;
}

uint8_t raw_seen[12];
uint8_t raw_n = 0;
uint16_t pin_edges = 0;
bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    dec.reset();
    raw_n = 0;
    pin_edges = 0;
    // Which pad the answer arrives on: the DUT's own TXD pad on a shared
    // line, its RXD on the crossed pair. Counting edges there separates
    // "the peer never answered" from "we were not listening".
    bool prev = shared_line() ? TxPin::read() : RxPin::read();
    while (Ticker::millis() - t0 < ms) {
        const bool now = shared_line() ? TxPin::read() : RxPin::read();
        if (now != prev) { ++pin_edges; prev = now; }
        if (!U4::rxc_flag()) continue;
        const UsartFrame f = U4::receive();
        if (raw_n < 12) {
            raw_seen[raw_n++] = f.clean() ? static_cast<uint8_t>(f.data) : 0xEE;
        }
        if (!f.clean()) { dec.reset(); continue; }
        if (dec.feed(static_cast<uint8_t>(f.data)) == link::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    U4::flush_rx();
    line_talk();
    link::write_frame(put4, op, p, len);
    line_listen();
    link::Frame f;
    if (!recv_frame(f, ms)) return false;
    return f.op == link::Op::ack && f.len == 2 && f.data[0] == link::byte_of(op);
}

/// The recovery guarantee in action: three attempts, each separated by a
/// quiet interval longer than the peer's own resynchronization timeout.
bool command(link::Op op, const uint8_t* p = no_payload, uint8_t len = 0,
             uint16_t ms = 80) {
    uint8_t first_n = 0;
    uint16_t first_edges = 0;
    uint8_t first_seen[12];
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len, ms)) return true;
        if (k == 0) {
            first_n = raw_n;
            first_edges = pin_edges;
            for (uint8_t i = 0; i < raw_n; ++i) first_seen[i] = raw_seen[i];
        }
        (void)link_command_mode();
        delay_us(clock, 70'000);
    }
    if (link_quiet) return false;
    // What a lost command looks like matters more than the fact of it:
    // edges on the line with no decoded byte means the receiver was not
    // listening; no edges at all means the peer never answered.
    print(serial, "    LINK FAILURE op ", hex(link::byte_of(op)), ": line edges=",
          first_edges, ", decoded ", first_n, " byte(s):");
    for (uint8_t i = 0; i < first_n; ++i) print(serial, " ", hex(first_seen[i]));
    print(serial, " (CTRLA=", hex(U4::regs().CTRLA), " CTRLB=", hex(U4::regs().CTRLB),
          " BAUD=", U4::baud_reg(), ")", crlf);
    (void)link_command_mode();
    delay_us(clock, 70'000);
    return false;
}

bool query(link::Op op, link::Frame& data, uint16_t ms = 80) {
    if (!command(op, no_payload, 0, ms)) return false;
    return recv_frame(data, ms);
}

/// Find the peer, and with it the desk's wiring: ping in the topology
/// last used, then in the other one. The peer alternates its own
/// command-mode configuration meanwhile, so one of the two dwells lands.
bool ensure_link() {
    if (topo_known) {
        (void)link_command_mode();
        if (command(link::Op::ping)) return true;
    }
    // The SHARED topology is probed FIRST, and on purpose: in it the DUT
    // only drives the line while it is transmitting, so a desk that turns
    // out to be one shared wire is never subjected to two push-pull
    // transmitters holding it at once. The full-duplex configuration idles
    // its TXD high forever, which on a shared desk would short every
    // answer the peer tries to send.
    const link::Topology order[2] = {link::Topology::shared,
                                     link::Topology::full_duplex};
    link_quiet = true;
    for (link::Topology t : order) {
        topo = t;
        (void)link_command_mode();
        for (uint8_t k = 0; k < 2; ++k) {
            if (command(link::Op::ping)) {
                topo_known = true;
                link_quiet = false;
                return true;
            }
        }
    }
    link_quiet = false;
    topo_known = false;
    return false;
}

bool peer_ident(link::Ident& d) {
    link::Frame f;
    if (!query(link::Op::ident, f) || f.op != link::Op::ident_data ||
        f.len != link::ident_size) {
        return false;
    }
    d = link::get_ident(f.data);
    return true;
}

/// Ask for the report of the last action, after giving the peer's bound
/// time to expire.
bool peer_report(link::Report& r, uint16_t wait_ms) {
    (void)link_command_mode();
    if (wait_ms) delay_us(clock, static_cast<uint32_t>(wait_ms) * 1000u);
    for (uint8_t k = 0; k < 4; ++k) {
        link::Frame f;
        if (query(link::Op::report, f) && f.op == link::Op::report_data &&
            f.len == link::report_size) {
            r = link::get_report(f.data);
            return true;
        }
        delay_us(clock, 40'000);
    }
    return false;
}

/// Command an action and wait out the rendezvous: the peer reconfigures
/// as soon as its acknowledgement has left the line and starts its own
/// traffic link::settle_ms + 2 ms after that, so the DUT has this window
/// to take up the other end of the new configuration.
bool peer_act(link::Op op, const link::Params& a,
              const uint8_t* extra = nullptr, uint8_t n = 0) {
    uint8_t p[link::max_payload] = {};
    link::put_params(p, a);
    for (uint8_t i = 0; i < n; ++i) p[link::params_size + i] = extra[i];
    if (!command(op, p, static_cast<uint8_t>(link::params_size + n))) return false;
    delay_us(clock, static_cast<uint32_t>(link::settle_ms) * 1000u);
    return true;
}

// ---- the DUT's own end of the link --------------------------------------------

UsartBits dut_bits(uint8_t n) {
    switch (n) {
        case 5: return UsartBits::five;
        case 6: return UsartBits::six;
        case 7: return UsartBits::seven;
        case link::bits9_low: return UsartBits::nine_low_first;
        case link::bits9_high: return UsartBits::nine_high_first;
        default: return UsartBits::eight;
    }
}
UsartParity dut_parity(uint8_t n) {
    return n == 1 ? UsartParity::even : (n == 2 ? UsartParity::odd : UsartParity::none);
}
uint16_t mask_of(uint8_t bits) {
    const uint8_t n = bits >= link::bits9_low ? 9 : bits;
    return static_cast<uint16_t>((1u << n) - 1u);
}

/// Configure USART4 as the DUT's end. `c.mode` names the DUT's OWN role,
/// so a test hands the peer one Cfg and itself the mirrored one.
bool dut_link(const link::Cfg& c, bool rx = true, bool tx = true) {
    UsartConfig u{};
    u.route = UsartRoute::def;
    // On a shared line the receiver listens to the TXD pad and the
    // transmitter is taken up only for the turnaround.
    const bool one_line = shared_line();
    switch (c.mode) {
        case link::Mode::sync_client:
        case link::Mode::sync_host: u.mode = UsartMode::sync; break;
        case link::Mode::ircom: u.mode = UsartMode::ircom; break;
        default: u.mode = UsartMode::async; break;
    }
    u.bits = dut_bits(c.bits);
    u.parity = dut_parity(c.parity);
    u.two_stop = c.stop >= 2;
    u.rx_mode = (c.flags & link::flag_clk2x) ? UsartRxMode::clk2x : UsartRxMode::normal;
    u.sync_client = c.mode == link::Mode::sync_client;
    u.multiprocessor = (c.flags & link::flag_mpcm) != 0;
    u.tx_pulse = c.txpl;
    u.rx_pulse = c.rxpl;
    u.rx = rx;
    u.tx = one_line && rx ? false : tx;
    u.loop_back = one_line && rx;
    const uint8_t s = usart_samples(u.mode, u.rx_mode);
    u.baud = usart_baud_reg(SysClock::hz, c.rate, s);
    if (u.baud == 0) {
        if (u.sync_client) u.baud = 64;
        else return false;
    }
    if (!U4::init(u)) return false;
    xck_invert((c.flags & link::flag_invert_xck) != 0);
    if (one_line && rx) {
        TxPin::input();
        pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    }
    U4::flush_rx();
    return true;
}

/// Milliseconds a frame count needs at a rate, generously (four times
/// the frame time plus a fixed floor) - never a magic constant.
uint16_t bound_ms(uint32_t rate, uint16_t frames) {
    const uint32_t bits = 12u * static_cast<uint32_t>(frames);
    const uint32_t ms = (bits * 1000u * 4u) / (rate ? rate : 1u);
    const uint32_t v = ms + 80u;
    return v > 4000u ? 4000u : static_cast<uint16_t>(v);
}

/// Lock-step round trip against the peer's ECHO: one frame out, its
/// echo back, compared.
/// The peer opens its window link::settle_ms + 2 ms after acknowledging
/// and flushes whatever the DUT's reconfiguration left on the line; this
/// is how long the DUT waits before transmitting, so its first frame is
/// never the thing that gets flushed.
void wait_for_peer_window() { delay_us(clock, 6000); }

uint16_t echo_first_sent = 0, echo_first_got = 0, echo_first_flags = 0;
uint16_t echo_mismatches(uint8_t bits, uint16_t n) {
    const uint16_t mask = mask_of(bits);
    uint16_t bad = 0;
    wait_for_peer_window();
    echo_first_sent = echo_first_got = echo_first_flags = 0;
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = static_cast<uint16_t>(0x5A + i * 37u) & mask;
        line_talk();
        if (!U4::send(v, 400'000u)) { line_listen(); ++bad; continue; }
        line_listen();
        const auto got = U4::wait(400'000u);
        if (!got || !got->clean() || got->data != v) {
            if (bad == 0) {
                echo_first_sent = v;
                echo_first_got = got ? got->data : 0xFFFF;
                echo_first_flags = got ? static_cast<uint16_t>(
                    (got->frame_error ? 1u : 0u) | (got->parity_error ? 2u : 0u) |
                    (got->overflow ? 4u : 0u)) : 0x8000u;
            }
            ++bad;
        }
    }
    return bad;
}

/// Decimal for a verdict label (one call per expression).
char num_buf[12];
const char* num(uint32_t v) {
    uint8_t i = 11;
    num_buf[i] = 0;
    if (v == 0) num_buf[--i] = '0';
    while (v != 0 && i != 0) {
        num_buf[--i] = static_cast<char>('0' + (v % 10u));
        v /= 10u;
    }
    return &num_buf[i];
}

/// Drain the receiver and report what came in.
struct Drain {
    uint16_t frames = 0;
    uint16_t ferr = 0;
    uint16_t perr = 0;
    uint16_t ovf = 0;
    uint16_t last = 0;
    uint16_t sum = 0;
};
/// Read frames for a whole window instead of draining at the end: the
/// receive FIFO holds three, so anything longer must be collected live.
Drain collect(uint16_t ms) {
    Drain d{};
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        if (!U4::rxc_flag()) continue;
        const UsartFrame f = U4::receive();
        ++d.frames;
        if (f.frame_error) ++d.ferr;
        if (f.parity_error) ++d.perr;
        if (f.overflow) ++d.ovf;
        d.last = f.data;
        d.sum = static_cast<uint16_t>(d.sum + f.data);
    }
    return d;
}

Drain drain(uint16_t max = 64) {
    Drain d{};
    while (U4::rxc_flag() && d.frames < max) {
        const UsartFrame f = U4::receive();
        ++d.frames;
        if (f.frame_error) ++d.ferr;
        if (f.parity_error) ++d.perr;
        if (f.overflow) ++d.ovf;
        d.last = f.data;
        d.sum = static_cast<uint16_t>(d.sum + f.data);
    }
    return d;
}

/// Wait for the line to fall quiet again, bounded.
void quiet(uint16_t ms) { delay_us(clock, static_cast<uint32_t>(ms) * 1000u); }

/// Auto-baud on the DUT, configured in ONE store: on a shared line the
/// loop-back must be there from the first write, because a receiver that
/// listens to a floating pin even for a moment loses the armed WFB (the
/// single-board half measured exactly that).
bool dut_autobaud(uint32_t fallback, bool lin, UsartAbWindow w = UsartAbWindow::wdw0) {
    const uint16_t r = usart_baud_reg(SysClock::hz, fallback);
    if (r == 0) return false;
    const bool ok = U4::init({.route = UsartRoute::def,
                              .rx_mode = lin ? UsartRxMode::linauto : UsartRxMode::genauto,
                              .baud = r,
                              .tx = !shared_line(),
                              .loop_back = shared_line(),
                              .ab_window = w});
    if (shared_line()) {
        TxPin::input();
        pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    }
    U4::flush_rx();
    U4::clear_break();
    U4::clear_isf();
    return ok;
}

/// IRCOM on the DUT: the IrdaLink task on the crossed pair, the resource
/// on a shared line (the task has no loop-back knob, and here the
/// receiver must listen to the pad).
bool dut_ircom(uint8_t txpl, uint8_t rxpl, uint32_t rate = 115'200) {
    if (!shared_line()) return Irda::init(clock, rate, txpl, rxpl);
    const uint16_t r = usart_baud_reg(SysClock::hz, rate);
    if (r == 0) return false;
    const bool ok = U4::init({.route = UsartRoute::def, .mode = UsartMode::ircom,
                              .baud = r, .tx = false, .loop_back = true,
                              .tx_pulse = txpl, .rx_pulse = rxpl});
    TxPin::input();
    pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    U4::flush_rx();
    return ok;
}

/// Point the pulse-width meter at whichever pad the peer's traffic
/// arrives on: the DUT's RXD on the crossed pair, its own TXD pad on a
/// shared line.
void watch_incoming() {
    if (shared_line()) ChTx::source(EvPin<TxPin>{});
    else ChTx::source(EvPin<RxPin>{});
    Meter::init(clock, ChTx{}, TcbClock::div1, /*low=*/true);
}

// ---- v: the wiring probe -------------------------------------------------------
// Not part of `y`: it needs board B's console command '2' started at the
// same time, and it is the answer to "is the desk wired the way the
// campaign assumes?" - a question no in-band protocol can ask when the
// wires are the thing that is broken. Each board drives its own PE0 with
// a slow square wave (the DUT at 5 Hz, the peer at 7) and counts the
// edges on the pins it only listens to.

constexpr uint16_t probe_ms = 6000;
constexpr uint16_t probe_half_ms[4] = {250, 125, 62, 31};

void probe_drive() {
    PORTE.DIRSET = 0x0Fu;
    for (uint16_t t = 0; t < probe_ms; ++t) {
        uint8_t out = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            if ((t / probe_half_ms[i]) & 1u) out = static_cast<uint8_t>(out | (1u << i));
        }
        PORTE.OUT = static_cast<uint8_t>((PORTE.OUT & 0xF0u) | out);
        delay_us(clock, 1000);
    }
    PORTE.DIRCLR = 0x0Fu;
}

void probe_listen(uint16_t* edges) {
    PORTE.DIRCLR = 0x0Fu;
    for (uint8_t i = 0; i < 4; ++i) pinctrl_of('E', i) |= PORT_PULLUPEN_bm;
    uint8_t prev = static_cast<uint8_t>(PORTE.IN & 0x0Fu);
    for (uint16_t t = 0; t < probe_ms; ++t) {
        delay_us(clock, 1000);
        const uint8_t now = static_cast<uint8_t>(PORTE.IN & 0x0Fu);
        const uint8_t ch = static_cast<uint8_t>(now ^ prev);
        for (uint8_t i = 0; i < 4; ++i) {
            if (ch & (1u << i)) ++edges[i];
        }
        prev = now;
    }
    for (uint8_t i = 0; i < 4; ++i) pinctrl_of('E', i) &= static_cast<uint8_t>(~PORT_PULLUPEN_bm);
}

int8_t probe_source(uint16_t edges) {
    if (edges < 8) return -1;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t want = probe_ms / probe_half_ms[i];
        if (edges * 4u >= want * 3u && edges * 3u <= want * 4u) return static_cast<int8_t>(i);
    }
    return -2;
}

void tv_wiring() {
    print(serial, "v wiring probe - start board B's console command '2' now: this end "
                  "listens 6 s, then drives 6 s", crlf);
    quiesce();
    uint16_t edges[4] = {0, 0, 0, 0};
    probe_listen(edges);
    probe_drive();
    bool crossed = true;
    for (uint8_t i = 0; i < 4; ++i) {
        const int8_t src = probe_source(edges[i]);
        print(serial, "  A.PE", i, ": ", edges[i], " edges -> ");
        if (src >= 0) print(serial, "B.PE", static_cast<uint8_t>(src), crlf);
        else if (src == -1) print(serial, "nothing", crlf);
        else print(serial, "several drivers or noise", crlf);
        const int8_t want = i == 0 ? 1 : (i == 1 ? 0 : (i == 2 ? 2 : -1));
        if (src != want) crossed = false;
    }
    verdict("the desk carries the campaign wiring (PE0-PE1, PE1-PE0, PE2-PE2)", crossed);
    quiesce();
    (void)link_command_mode();
}

// ---- j: the rendezvous ---------------------------------------------------------

void tj_link() {
    print(serial, "j rendezvous: ping, IDENT, a bad checksum, a desync, the peer's clock",
          crlf);
    quiesce();
    const bool found = ensure_link();
    print(serial, "  topology: ", shared_line() ? "SHARED LINE (one wire between the two "
                                                  "TXD pads, half duplex)"
                                                : "full duplex (the crossed pair)", crlf);
    verdict("command mode on USART4 (8N1, 115200)", found);
    verdict("the peer answers a ping", found && command(link::Op::ping));

    link::Ident d{};
    const bool got = peer_ident(d);
    verdict("IDENT answered", got);
    if (got) {
        print(serial, "  peer label='", d.label, "' clock=", d.xtal ? "XTAL" : "OSCHF",
              " sanity=", hex(d.sanity), " fw=", hex(d.version), crlf);
        const char* want = "brio-b";
        bool same = true;
        for (uint8_t i = 0; i < 7; ++i) same = same && d.label[i] == want[i];
        verdict("the peer is brio-b", same);
        verdict("the peer runs usart_peer", d.sanity == link::ident_sanity);
        if (!d.xtal) {
            print(serial, "  NOTE: board B's 24 MHz crystal did not start - its rates come "
                          "from OSCHF", crlf);
        }
    } else {
        verdict("the peer is brio-b", false);
        verdict("the peer runs usart_peer", false);
    }

    // A frame whose checksum does not check out must be refused, not obeyed.
    U4::flush_rx();
    line_talk();
    put4(link::magic);
    put4(link::byte_of(link::Op::ping));
    put4(0);
    put4(0x00);                                   // not the checksum
    line_listen();
    link::Frame bad;
    verdict("a bad checksum comes back as a NAK",
            recv_frame(bad, 120) && bad.op == link::Op::nak);

    // Raw garbage, the quiet interval, then the link must be there again.
    line_talk();
    for (uint8_t i = 0; i < 12; ++i) put4(static_cast<uint8_t>(0x11u * i));
    line_listen();
    quiet(120);
    (void)link_command_mode();
    verdict("the link recovers after a desync", command(link::Op::ping));

    // The peer's clock, measured in the DUT's crystal time: a stream of
    // 0xFF frames at 9600 whose only low pulse is the start bit.
    link::Params a{};
    a.cfg.rate = 9600;
    a.count = 24;
    a.ms = bound_ms(9600, 24);
    a.value = 0xFF;
    a.pattern = link::pattern_fixed;
    const bool sending = peer_act(link::Op::send, a);
    verdict("the peer accepts SEND", sending);
    watch_incoming();                   // only now: command mode runs at 115200
    clear_captures();
    (void)wait_captures(8, 400);
    const uint16_t w = last_width;
    T0::enable_capt_interrupt(false);
    T0::disable();
    const int32_t nominal = static_cast<int32_t>(SysClock::hz / 9600u);
    const int32_t err_ppm = nominal ? ((static_cast<int32_t>(w) - nominal) * 10000L) / nominal : 0;
    print(serial, "  peer start bit at 9600: ", w, " DUT ticks (nominal ", nominal,
          "), so board B's clock runs ", err_ppm <= 0 ? "+" : "-",
          static_cast<uint32_t>(err_ppm >= 0 ? err_ppm : -err_ppm) / 100u, ".",
          static_cast<uint32_t>(err_ppm >= 0 ? err_ppm : -err_ppm) % 100u,
          " % against the DUT's crystal", crlf);
    verdict("the peer's bit time is within 3 % of the DUT's",
            captures >= 8 && near(w, nominal, nominal / 33));
    link::Report r{};
    verdict("the peer reports the 24 frames it sent",
            peer_report(r, 20) && r.count == 24);
    quiesce();
    (void)link_command_mode();
}

// ---- k: the baud matrix, both directions --------------------------------------

struct RateCase { uint32_t rate; bool clk2x; };

void tk_baud() {
    print(serial, "k baud matrix across the wire (8N1, echoed by the peer)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    // 300 baud is NOT in the list: at CLK_PER = 24 MHz it would need
    // BAUD = 320000, five times what the register holds - the slowest
    // rate this clock can express is 1465 baud.
    const RateCase cases[] = {
        {2400, false}, {9600, false}, {115'200, false}, {230'400, false},
        {460'800, false}, {921'600, false}, {1'000'000, false}, {2'000'000, true},
    };
    for (const RateCase& rc : cases) {
        link::Cfg c{};
        c.rate = rc.rate;
        c.flags = rc.clk2x ? link::flag_clk2x : 0;
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = bound_ms(rc.rate, 16);
        if (!peer_act(link::Op::echo, a)) {
            verdict(num(rc.rate), " baud: the peer refused ECHO", false);
            (void)link_command_mode();
            continue;
        }
        const bool up = dut_link(c);
        const uint16_t bad = up ? echo_mismatches(8, 8) : 8;
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(serial, "  ", rc.rate, rc.clk2x ? " (CLK2X)" : "", ": BAUD=",
              usart_baud_reg(SysClock::hz, rc.rate, rc.clk2x ? 8 : 16),
              " echoes wrong=", bad, " peer count=", have ? r.count : 0,
              " ferr=", have ? r.ferr : 0, " perr=", have ? r.perr : 0,
              " ovf=", have ? r.ovf : 0, crlf);
        if (bad != 0) {
            print(serial, "    first mismatch: sent ", hex(echo_first_sent), " got ",
                  hex(echo_first_got), " flags ", hex(echo_first_flags), crlf);
        }
        verdict(num(rc.rate), " baud: every echo matches", up && bad == 0);
        verdict(num(rc.rate), " baud: the peer counted 8 clean frames",
                have && r.count == 8 && r.ferr == 0 && r.perr == 0 && r.ovf == 0);
    }
    quiesce();
    (void)link_command_mode();
}

// ---- l: the frame matrix across the wire --------------------------------------

struct FmtCase { uint8_t bits; uint8_t parity; uint8_t stop; const char* name; };

void tl_frames_cross() {
    print(serial, "l frame formats across the wire at 115200 (echoed by the peer)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    const FmtCase cases[] = {
        {5, 0, 1, "5N1"}, {6, 2, 2, "6O2"}, {7, 1, 1, "7E1"},
        {8, 2, 2, "8O2"}, {8, 1, 2, "8E2"},
        {link::bits9_low, 0, 1, "9LN1"}, {link::bits9_high, 0, 1, "9HN1"},
        {link::bits9_low, 1, 1, "9LE1"},
    };
    for (const FmtCase& fc : cases) {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = fc.bits;
        c.parity = fc.parity;
        c.stop = fc.stop;
        link::Params a{};
        a.cfg = c;
        a.count = 6;
        a.ms = bound_ms(115'200, 20);
        if (!peer_act(link::Op::echo, a)) {
            verdict(fc.name, " across the wire: the peer refused", false);
            (void)link_command_mode();
            continue;
        }
        const bool up = dut_link(c);
        const uint16_t bad = up ? echo_mismatches(fc.bits, 6) : 6;
        link::Report r{};
        const bool have = peer_report(r, 10);
        verdict(fc.name, " across the wire", up && bad == 0 && have && r.count == 6 &&
                                             r.ferr == 0 && r.perr == 0);
        if (!(up && bad == 0)) {
            print(serial, "    ", fc.name, ": wrong=", bad, " peer count=",
                  have ? r.count : 0, " ferr=", have ? r.ferr : 0,
                  " perr=", have ? r.perr : 0, crlf);
        }
    }

    // SBMODE is a TRANSMITTER setting: the receiver never looks past the
    // first stop bit, so the two asymmetric pairings must both be clean.
    for (uint8_t k = 0; k < 2; ++k) {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.stop = k == 0 ? 2 : 1;
        link::Cfg dc = pc;
        dc.stop = k == 0 ? 1 : 2;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 20);
        const bool sent = peer_act(link::Op::echo, a);
        const bool up = sent && dut_link(dc);
        const uint16_t bad = up ? echo_mismatches(8, 6) : 6;
        link::Report r{};
        const bool have = peer_report(r, 10);
        verdict(k == 0 ? "DUT 1 stop bit receives the peer's 2"
                       : "DUT 2 stop bits received by the peer's 1",
                up && bad == 0 && have && r.count == 6 && r.ferr == 0);
    }
    quiesce();
    (void)link_command_mode();
}

// ---- m: errors injected from the other board ----------------------------------

void tm_errors() {
    print(serial, "m injected errors: parity, baud mismatch, overflow, a break", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }

    // 1. The peer transmits with odd parity into a receiver set to even.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.parity = 2;                                  // odd
        link::Cfg dc = pc;
        dc.parity = 1;                                  // even
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 12);
        a.value = 0x5A;
        a.pattern = link::pattern_counting;
        const bool sent = peer_act(link::Op::send, a);
        const bool up = sent && dut_link(dc);
        const Drain d = collect(bound_ms(115'200, 12));
        print(serial, "  odd into even: frames=", d.frames, " perr=", d.perr,
              " ferr=", d.ferr, " last=", hex(d.last), crlf);
        verdict("every frame of a wrong parity flags PERR",
                up && d.frames == 6 && d.perr == 6);
        verdict("the data of a PERR frame is still readable",
                d.frames == 6 && d.last == 0x5F);
        (void)link_command_mode();
    }

    // 2. The same frames through the Uart TASK: counted and dropped.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.parity = 2;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 12);
        a.value = 0x5A;
        a.aux16 = 40;              // Uart::init holds TX idle for 10 ms first
        const bool sent = peer_act(link::Op::send, a);
        U4Tx::init(clock, 115'200u);
        if (shared_line()) {
            U4::enable_tx(false);
            U4::loop_back(true);
            TxPin::input();
            pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
        }
        U4::frame({.bits = UsartBits::eight, .parity = UsartParity::even});
        U4::enable_rxc_interrupt(true);
        // The reconfiguration itself moved the pad, and a moved pad is a
        // start bit to a loop-back receiver: let that settle, throw it
        // away, and only then start counting.
        quiet(4);
        uint8_t b, delivered = 0;
        while (U4Tx::read_byte(b)) {
        }
        U4Tx::clear_errors();
        U4::flush_rx();
        delivered = 0;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < bound_ms(115'200, 12)) {
            if (U4Tx::read_byte(b)) ++delivered;
        }
        const uint8_t pe = U4Tx::parity_errors();
        const bool any = delivered != 0;
        U4::enable_rxc_interrupt(false);
        print(serial, "  Uart task: parity_errors=", pe, " bytes delivered=", delivered,
              crlf);
        verdict("the Uart task counts the parity errors", sent && pe == 6);
        verdict("and delivers none of those bytes", !any);
        (void)link_command_mode();
    }

    // 3. Where FERR really begins: the peer sends 0x00 (the frame whose
    //    stop bit has the least margin) at a rate deliberately off.
    {
        const int8_t errs[] = {0, 2, 4, 5, 6, 8, -2, -4, -5, -6, -8};
        int16_t first_pos = 0, first_neg = 0;
        for (int8_t e : errs) {
            const uint32_t rate =
                static_cast<uint32_t>(115'200L + (115'200L * e) / 100L);
            link::Cfg pc{};
            pc.rate = rate;
            link::Cfg dc{};
            dc.rate = 115'200;
            link::Params a{};
            a.cfg = pc;
            a.count = 8;
            a.ms = bound_ms(rate, 16);
            a.value = 0x00;
            a.pattern = link::pattern_fixed;
            if (!peer_act(link::Op::send, a)) { (void)link_command_mode(); continue; }
            (void)dut_link(dc);
            const Drain d = collect(bound_ms(rate, 16));
            print(serial, "  ", e >= 0 ? "+" : "-", e >= 0 ? e : -e, " %: frames=",
                  d.frames, " ferr=", d.ferr, crlf);
            if (d.ferr != 0) {
                if (e > 0 && first_pos == 0) first_pos = e;
                if (e < 0 && first_neg == 0) first_neg = e;
            }
            if (e == 0) verdict("no error at the nominal rate", d.frames == 8 && d.ferr == 0);
            (void)link_command_mode();
        }
        print(serial, "  FERR onset: +", first_pos, " %, ", first_neg, " % (table 27-4 "
              "gives -4.19/+4.14 % for D = 9)", crlf);
        verdict("a rate error inside the operational range is clean",
                first_pos == 0 || first_pos >= 4);
        verdict("a rate error outside it does flag FERR",
                first_pos != 0 || first_neg != 0);
    }

    // 4. A flood into a receiver that reads late: BUFOVF on the right frame.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = bound_ms(115'200, 16);
        a.value = 0xB0;
        a.pattern = link::pattern_counting;
        const bool sent = peer_act(link::Op::blast, a);
        (void)dut_link(pc);
        quiet(bound_ms(115'200, 16));       // read LATE on purpose: the FIFO must overflow
        const Drain d = drain();
        print(serial, "  blast of 8: drained=", d.frames, " ovf=", d.ovf,
              " last=", hex(d.last), crlf);
        verdict("only three frames survive a flood", sent && d.frames == 3);
        verdict("the last of them is the newest and carries BUFOVF",
                d.frames == 3 && d.ovf == 1 && d.last == 0xB7);
        (void)link_command_mode();
    }

    // 5. A break on the wire: a normal receiver sees FERR with data zero.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 20;                                   // bit times held low
        a.ms = 200;
        const bool sent = peer_act(link::Op::brk, a);
        (void)dut_link(pc);
        const Drain d = collect(120);
        print(serial, "  break of 20 bit times: frames=", d.frames, " ferr=", d.ferr,
              " last=", hex(d.last), crlf);
        verdict("a break arrives as FERR with data 0x00",
                sent && d.frames >= 1 && d.ferr >= 1 && d.last == 0x00);
        (void)link_command_mode();
    }
    quiesce();
    (void)link_command_mode();
}

// ---- n: waveforms only a bit-banger can make ----------------------------------

/// One bit-bang action and what the DUT made of it.
Drain bitbang_run(uint32_t rate, uint8_t value, uint16_t frames, int8_t cell, int8_t pct,
                  uint16_t break_bits, const link::Cfg& dut_cfg, uint16_t extra_ms = 0) {
    link::Params a{};
    a.cfg.rate = rate;
    a.count = frames;
    a.ms = bound_ms(rate, static_cast<uint16_t>(frames * 6 + break_bits));
    a.value = value;
    a.cell = cell;
    a.pct = pct;
    a.aux16 = break_bits;
    if (!peer_act(link::Op::bitbang, a)) return Drain{};
    (void)dut_link(dut_cfg);
    return collect(static_cast<uint16_t>(a.ms + extra_ms));
}

void tn_waveforms() {
    print(serial, "n bit-banged waveforms: glitches, a rate error, a distorted cell, ABW",
          crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    link::Cfg base{};
    base.rate = 9600;

    // 1. Sub-bit low pulses on an idle line: where does a start bit start
    //    being believed? The DUT measures each pulse it sees through the
    //    event system, so the width is a fact and not a claim.
    {
        const uint16_t widths[] = {6, 30, 150, 625, 1250, 1562, 1875, 2500};
        uint16_t first_accepted = 0;
        for (uint16_t w : widths) {
            link::Params a{};
            a.cfg.rate = 9600;
            a.count = 4;
            a.ms = 300;
            a.aux16 = w;
            a.aux8 = 3;                                  // milliseconds... microseconds
            a.aux8 = 200;                                // gap between pulses, us
            if (!peer_act(link::Op::glitch, a)) { (void)link_command_mode(); continue; }
            (void)dut_link(base);
            const Drain d = collect(80);
            print(serial, "  low pulse of ", w, " CPU cycles: frames=", d.frames,
                  " ferr=", d.ferr, crlf);
            if (d.frames != 0 && first_accepted == 0) first_accepted = w;
            if (w == 6) verdict("a 6-cycle glitch is not a start bit", d.frames == 0);
            if (w == 625) verdict("a quarter-bit glitch is not a start bit", d.frames == 0);
            if (w == 2500) verdict("a full-bit low IS taken as a start bit", d.frames != 0);
            (void)link_command_mode();
        }
        print(serial, "  narrowest accepted low pulse: ", first_accepted,
              " CPU cycles of a 2500-cycle bit", crlf);
        verdict("the start-bit threshold sits between a half and a whole bit",
                first_accepted >= 1250 && first_accepted <= 2500);
    }

    // 2. A UNIFORM rate error, generated by counting cycles instead of by
    //    a baud register: this is the operational range of 27.3.3.2.3
    //    measured directly, free of the divisor's quantization.
    {
        const int8_t errs[] = {0, 2, 3, 4, 5, 6, -2, -3, -4, -5, -6};
        int16_t pos = 0, neg = 0;
        for (int8_t e : errs) {
            const uint32_t rate = static_cast<uint32_t>(9600L + (9600L * e) / 100L);
            const Drain d = bitbang_run(rate, 0x00, 6, -1, 0, 0, base);
            print(serial, "  ", e >= 0 ? "+" : "-", e >= 0 ? e : -e, " %: frames=",
                  d.frames, " ferr=", d.ferr, crlf);
            if (d.ferr != 0) {
                if (e > 0 && pos == 0) pos = e;
                if (e < 0 && neg == 0) neg = e;
            }
            if (e == 0) verdict("a bit-banged frame at the nominal rate is clean",
                                d.frames == 6 && d.ferr == 0);
            (void)link_command_mode();
        }
        print(serial, "  bit-banged FERR onset: +", pos, " %, ", neg, " %", crlf);
        verdict("the measured operational range brackets the data sheet's +-4 %",
                (pos == 0 || pos >= 4) && (neg == 0 || neg <= -4));
    }

    // 3. ONE cell stretched: not a rate error at all - a single boundary
    //    displaced, which the stop bit's sample survives until the shift
    //    reaches half a cell.
    {
        const int8_t pcts[] = {10, 20, 30, 40, 50, 60, 70};
        int16_t onset = 0;
        for (int8_t p : pcts) {
            const Drain d = bitbang_run(9600, 0x00, 4, 5, p, 0, base);
            if (d.ferr != 0 && onset == 0) onset = p;
            print(serial, "  cell 5 +", p, " %: frames=", d.frames, " ferr=", d.ferr, crlf);
            (void)link_command_mode();
        }
        print(serial, "  one stretched cell breaks the frame at +", onset, " %", crlf);
        verdict("a single stretched cell survives far past the rate tolerance",
                onset == 0 || onset >= 30);
    }

    // 4. The LIN auto-baud window: the sync field's bit pair is what ABW
    //    measures, so distorting one of its cells is the only honest probe.
    {
        const UsartAbWindow windows[4] = {UsartAbWindow::wdw1, UsartAbWindow::wdw0,
                                          UsartAbWindow::wdw2, UsartAbWindow::wdw3};
        const char* names[4] = {"WDW1 (32+-5)", "WDW0 (32+-6)", "WDW2 (32+-7)",
                                "WDW3 (32+-8)"};
        const int8_t pcts[] = {12, 16, 20, 24, 28, 32, 36, 40, 44};
        for (uint8_t wi = 0; wi < 4; ++wi) {
            int16_t last_ok = 0, first_bad = 0;
            for (int8_t p : pcts) {
                link::Params a{};
                a.cfg.rate = 9600;
                a.count = 1;
                a.ms = 300;
                a.value = 0x55;
                a.cell = 2;
                a.pct = p;
                a.aux16 = 20;                            // a real LIN break
                if (!peer_act(link::Op::bitbang, a)) { (void)link_command_mode(); continue; }
                (void)dut_autobaud(9600u, true, windows[wi]);
                quiet(120);
                const bool bdf = Ab::break_detected();
                const bool isf = Ab::sync_error();
                if (bdf && !isf) last_ok = p;
                else if (first_bad == 0) first_bad = p;
                Ab::recover();
                (void)link_command_mode();
            }
            print(serial, "  ", names[wi], ": accepted up to +", last_ok,
                  " %, refused from +", first_bad, " %", crlf);
            verdict(names[wi], " has a measurable acceptance edge",
                    last_ok != 0 || first_bad != 0);
        }
    }
    quiesce();
    (void)link_command_mode();
}

// ---- o: auto-baud against a foreign sender ------------------------------------

void to_autobaud_foreign() {
    print(serial, "o auto-baud against board B, a genuinely foreign clock", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    const uint32_t rates[] = {9600, 57'600, 230'400, 123'456};
    for (uint32_t rate : rates) {
        for (uint8_t kind = 0; kind < 2; ++kind) {
            link::Params a{};
            a.cfg.rate = rate;
            a.count = 0;
            a.ms = bound_ms(rate, 32);
            a.value = 0x55;                              // the sync character
            a.aux16 = 20;                                // break, in bit times
            a.aux8 = 4;                                  // payload frames
            if (!peer_act(link::Op::autobaud_tx, a)) { (void)link_command_mode(); continue; }
            // LINAUTO is CONSTRAINED: ABW compares the sync field against the
            // BAUD already in force, so its fallback has to start near the
            // incoming rate (10 % away here, inside WDW0). GENAUTO has no
            // such tie and starts from a wholly unrelated 19200.
            const uint32_t fallback = kind ? rate * 9u / 10u : 19'200u;
            const bool up = dut_autobaud(fallback, kind != 0);
            if (!kind) Ab::arm_break();
            // BDF is cleared by the next DATA frame, and the payload follows
            // the sync field immediately: latch it as it happens.
            bool bdf = false, isf = false;
            uint16_t measured = 0;
            Drain d{};
            const uint32_t t0 = Ticker::millis();
            while (Ticker::millis() - t0 < static_cast<uint32_t>(a.ms) + 60u) {
                if (!bdf && Ab::break_detected()) {
                    bdf = true;
                    measured = Ab::measured_baud_reg();
                }
                if (Ab::sync_error()) isf = true;
                if (!U4::rxc_flag()) continue;
                const UsartFrame f = U4::receive();
                ++d.frames;
                if (f.frame_error) ++d.ferr;
                d.last = f.data;
            }
            if (measured == 0) measured = Ab::measured_baud_reg();
            const uint16_t own = usart_baud_reg(SysClock::hz, rate);
            const int32_t delta = own ? ((static_cast<int32_t>(measured) -
                                          static_cast<int32_t>(own)) * 10000L) / own
                                      : 0;
            print(serial, "  ", rate, kind ? " LINAUTO" : " GENAUTO", ": BAUD measured=",
                  measured, " own=", own, " delta=", delta >= 0 ? "+" : "-",
                  static_cast<uint32_t>(delta >= 0 ? delta : -delta) / 100u, ".",
                  static_cast<uint32_t>(delta >= 0 ? delta : -delta) % 100u,
                  " % BDF=", bdf ? 1 : 0, " ISFIF=", isf ? 1 : 0, " payload=", d.frames,
                  " ferr=", d.ferr, crlf);
            verdict(num(rate), kind ? " LINAUTO learns the foreign rate"
                                    : " GENAUTO learns the foreign rate",
                    up && bdf && !isf && own != 0 && near(measured, own, own / 20));
            verdict(num(rate), kind ? " LINAUTO then reads the payload"
                                    : " GENAUTO then reads the payload",
                    d.frames >= 3 && d.ferr == 0);
            Ab::recover();
            (void)link_command_mode();
        }
    }

    // The constraint itself: a sync field 100 % away from the BAUD in force
    // is refused by LINAUTO and accepted by GENAUTO. Same wire, same sender.
    {
        link::Params a{};
        a.cfg.rate = 9600;
        a.ms = bound_ms(9600, 32);
        a.value = 0x55;
        a.aux16 = 20;
        a.aux8 = 2;
        const bool sent = peer_act(link::Op::autobaud_tx, a);
        (void)dut_autobaud(19'200u, /*lin=*/true);
        bool bdf = false;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < static_cast<uint32_t>(a.ms) + 60u) {
            if (Ab::break_detected()) bdf = true;
        }
        print(serial, "  LINAUTO at BAUD 19200 against a 9600 sync field: BDF=",
              bdf ? 1 : 0, " BAUD=", Ab::measured_baud_reg(), " ISFIF=",
              Ab::sync_error() ? 1 : 0, crlf);
        verdict("LINAUTO refuses a sync field outside the window of the BAUD in force",
                sent && !bdf);
        Ab::recover();
        (void)link_command_mode();
    }

    // A sync character that is not 0x55 in LINAUTO: ISFIF, the erratum's
    // recovery, and then a retry that works.
    {
        link::Params a{};
        a.cfg.rate = 9600;
        a.ms = bound_ms(9600, 32);
        a.value = 0x53;
        a.aux16 = 20;
        a.aux8 = 2;
        const bool sent = peer_act(link::Op::autobaud_tx, a);
        (void)dut_autobaud(9600u, true);           // the sender's own rate
        bool bdf1 = false;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < static_cast<uint32_t>(a.ms) + 60u) {
            if (Ab::break_detected()) bdf1 = true;
        }
        const bool isf = Ab::sync_error();
        print(serial, "  LINAUTO with sync character 0x53: ISFIF=", isf ? 1 : 0, " BDF=",
              bdf1 ? 1 : 0, " BAUD=", Ab::measured_baud_reg(), crlf);
        verdict("a sync character that is not 0x55 is refused", sent && isf && !bdf1);
        Ab::recover();
        (void)link_command_mode();

        a.value = 0x55;
        const bool again = peer_act(link::Op::autobaud_tx, a);
        (void)dut_autobaud(9600u, true);
        bool bdf2 = false, isf2 = false;
        const uint32_t t1 = Ticker::millis();
        while (Ticker::millis() - t1 < static_cast<uint32_t>(a.ms) + 60u) {
            if (Ab::break_detected()) bdf2 = true;
            if (Ab::sync_error()) isf2 = true;
        }
        verdict("after recover() the next sync field is learned", again && bdf2 && !isf2);
        Ab::recover();
        (void)link_command_mode();
    }

    // The register's documented floor (0x0064) and ceiling, probed from
    // outside: a sync field too fast, and one too slow to fit BAUD.
    {
        link::Params a{};
        a.cfg.rate = 1'200'000;                          // BAUD would be 80, below 0x64
        a.ms = 200;
        a.value = 0x55;
        a.aux16 = 20;
        a.aux8 = 2;
        const bool sent = peer_act(link::Op::autobaud_tx, a);
        (void)dut_autobaud(19'200u, false);
        Ab::arm_break();
        bool bdf3 = false;
        uint16_t reg3 = 0;
        const uint32_t t2 = Ticker::millis();
        while (Ticker::millis() - t2 < 200u) {
            if (!bdf3 && Ab::break_detected()) { bdf3 = true; reg3 = Ab::measured_baud_reg(); }
        }
        if (reg3 == 0) reg3 = Ab::measured_baud_reg();
        print(serial, "  sync at 1.2 Mbaud (BAUD 80, below the documented 0x0064 floor): "
              "BAUD=", reg3, " BDF=", bdf3 ? 1 : 0,
              " ISFIF=", Ab::sync_error() ? 1 : 0, crlf);
        verdict("a sync field below the documented BAUD floor is answered either way",
                sent);
        Ab::recover();
        (void)link_command_mode();
    }
    quiesce();
    (void)link_command_mode();
}

// ---- p: the multiprocessor filter across the wire ------------------------------

void tp_mpcm_cross() {
    print(serial, "p MPCM: the peer addresses two clients, the DUT filters on one", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    const uint8_t groups[4] = {0x41, 3, 0x42, 3};

    // Nine data bits: the ninth marks the address frame.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.bits = link::bits9_low;
        link::Cfg dc = pc;
        dc.flags = link::flag_mpcm;
        link::Params a{};
        a.cfg = pc;
        a.ms = bound_ms(115'200, 24);
        a.value = 0x10;                                  // the data run starts here
        a.groups = 2;
        const bool sent = peer_act(link::Op::send_mpcm, a, groups, 4);
        const bool up = sent && dut_link(dc);
        quiet(bound_ms(115'200, 24));
        // Filtering: only the two address frames may have got through.
        const Drain first = drain();
        print(serial, "  filtered: frames=", first.frames, " last=", hex(first.last), crlf);
        verdict("with MPCM on, only the address frames arrive",
                up && first.frames == 2 && first.last == 0x142);
        (void)link_command_mode();
    }

    // The same traffic with the filter opened after the first address.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.bits = link::bits9_low;
        link::Cfg dc = pc;
        dc.flags = link::flag_mpcm;
        link::Params a{};
        a.cfg = pc;
        a.ms = bound_ms(115'200, 24);
        a.value = 0x10;
        a.groups = 2;
        const bool sent = peer_act(link::Op::send_mpcm, a, groups, 4);
        const bool up = sent && dut_link(dc);
        uint16_t addr = 0, data = 0, second = 0;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 60) {
            if (!U4::rxc_flag()) continue;
            const UsartFrame f = U4::receive();
            if (f.data == 0x141 && addr == 0) {
                addr = f.data;
                U4::multiprocessor(false);               // this client is addressed
            } else if (addr != 0 && second == 0 && (f.data & 0x100u) == 0) {
                ++data;
                if (data == 3) {
                    U4::multiprocessor(true);            // done: listen for addresses
                }
            } else if ((f.data & 0x100u) != 0 && addr != 0) {
                second = f.data;
            }
        }
        print(serial, "  addressed: addr=", hex(addr), " data frames=", data,
              " next address=", hex(second), crlf);
        verdict("the addressed client sees its own data frames",
                up && addr == 0x141 && data == 3);
        verdict("and catches the next group's address after re-arming",
                second == 0x142);
        (void)link_command_mode();
    }

    // The 5..8-bit flavour: the sender must use nine bits, because a
    // transmitter with five to eight data bits can only ever send ones in
    // the stop positions - the receiver reads the sender's ninth bit AS
    // its own first stop bit.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.bits = link::bits9_low;
        link::Cfg dc{};
        dc.rate = 115'200;
        dc.bits = 8;
        dc.stop = 2;
        dc.flags = link::flag_mpcm;
        link::Params a{};
        a.cfg = pc;
        a.ms = bound_ms(115'200, 24);
        a.value = 0x10;
        a.groups = 2;
        const bool sent = peer_act(link::Op::send_mpcm, a, groups, 4);
        const bool up = sent && dut_link(dc);
        quiet(bound_ms(115'200, 24));
        const Drain d = drain();
        print(serial, "  8-bit + 2 stop, MPCM: frames=", d.frames, " last=", hex(d.last),
              " ferr=", d.ferr, crlf);
        verdict("the first stop bit filters exactly like the ninth data bit",
                up && d.frames == 2 && d.last == 0x42);
        (void)link_command_mode();
    }
    quiesce();
    (void)link_command_mode();
}

// ---- q: the synchronous roles on a real XCK ------------------------------------

void tq_sync() {
    print(serial, "q synchronous USART on PE2: host, client, and the client's ceiling",
          crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    if (shared_line()) {
        print(serial, "  SKIPPED: a synchronous link needs XCK wired across (A.PE2-B.PE2) "
                      "AND data on the crossed pair; this desk has one shared wire", crlf);
        return;
    }
    const uint32_t xcks[] = {100'000, 1'000'000};

    // The DUT drives XCK, the peer follows and sinks.
    for (uint32_t xck : xcks) {
        for (uint8_t inv = 0; inv < 2; ++inv) {
            link::Cfg pc{};
            pc.mode = link::Mode::sync_client;
            pc.rate = xck;
            pc.flags = inv ? link::flag_invert_xck : 0;
            link::Params a{};
            a.cfg = pc;
            a.count = 8;
            a.ms = bound_ms(xck, 40);
            if (!peer_act(link::Op::sink, a)) { (void)link_command_mode(); continue; }
            const bool up = SyncH::init(clock, xck);
            SyncH::invert_xck(inv != 0);
            wait_for_peer_window();
            uint16_t sum = 0;
            for (uint8_t i = 0; i < 8; ++i) {
                const uint16_t v = static_cast<uint16_t>(0x30 + i);
                (void)U4::send(v, 400'000u);
                sum = static_cast<uint16_t>(sum + v);
            }
            (void)U4::wait_line_idle();
            SyncH::invert_xck(false);
            link::Report r{};
            const bool have = peer_report(r, 10);
            print(serial, "  host ", xck, " Hz", inv ? " INVEN" : "", ": peer count=",
                  have ? r.count : 0, " sum=", hex(have ? r.sum : 0), " want=", hex(sum),
                  " ferr=", have ? r.ferr : 0, crlf);
            verdict(num(xck), inv ? " Hz host, inverted XCK" : " Hz host, XCK as it comes",
                    up && have && r.count == 8 && r.sum == sum && r.ferr == 0);
            (void)link_command_mode();
        }
    }

    // Roles reversed: the peer drives XCK, the DUT is the client.
    for (uint32_t xck : xcks) {
        link::Cfg pc{};
        pc.mode = link::Mode::sync_host;
        pc.rate = xck;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = bound_ms(xck, 60);
        a.value = 0x60;
        if (!peer_act(link::Op::send, a)) { (void)link_command_mode(); continue; }
        const bool up = SyncC::init();
        quiet(static_cast<uint16_t>(a.ms + 40));
        const Drain d = drain();
        uint16_t want = 0;
        for (uint8_t i = 0; i < 8; ++i) want = static_cast<uint16_t>(want + 0x60 + i);
        print(serial, "  client at ", xck, " Hz: frames=", d.frames, " sum=", hex(d.sum),
              " want=", hex(want), " ferr=", d.ferr, crlf);
        verdict(num(xck), " Hz client takes the peer's clock",
                up && d.frames == 8 && d.sum == want && d.ferr == 0);
        (void)link_command_mode();
    }

    // Above the client's CLK_PER/4: the ceiling is supposed to be real.
    {
        const uint32_t xck = 12'000'000;                 // CLK_PER / 2, twice the ceiling
        link::Cfg pc{};
        pc.mode = link::Mode::sync_host;
        pc.rate = xck;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = 200;
        a.value = 0x60;
        const bool sent = peer_act(link::Op::send, a);
        (void)SyncC::init();
        quiet(240);
        const Drain d = drain();
        uint16_t want = 0;
        for (uint8_t i = 0; i < 8; ++i) want = static_cast<uint16_t>(want + 0x60 + i);
        const bool clean = d.frames == 8 && d.sum == want && d.ferr == 0;
        print(serial, "  client at 12 MHz (ceiling is CLK_PER/4 = 6 MHz): frames=",
              d.frames, " sum=", hex(d.sum), " want=", hex(want), " ferr=", d.ferr,
              clean ? "  <-- CLEAN: the data sheet's ceiling did not bite" : "", crlf);
        verdict("XCK at twice the client's ceiling does not come through cleanly",
                sent && !clean);
        (void)link_command_mode();
    }
    quiesce();
    (void)link_command_mode();
}

// ---- r: RS-485 and the XDIR guard time -----------------------------------------

void tr_rs485() {
    print(serial, "r RS-485: the peer receives while a TCB measures XDIR on PE3", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    const uint32_t rates[] = {9600, 115'200};
    for (uint32_t rate : rates) {
        link::Cfg pc{};
        pc.rate = rate;
        link::Params a{};
        a.cfg = pc;
        a.count = 4;
        a.ms = bound_ms(rate, 24);
        if (!peer_act(link::Op::sink, a)) { (void)link_command_mode(); continue; }
        const bool up = Bus::init(clock, rate);
        wait_for_peer_window();
        Ch2::source(EvPin<XdirPin>{});
        Meter::init(clock, Ch2{}, TcbClock::div1, /*high pulse=*/false);
        clear_captures();
        uint16_t sum = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            const uint16_t v = static_cast<uint16_t>(0x70 + i);
            (void)Bus::send(v, 400'000u);
            sum = static_cast<uint16_t>(sum + v);
            (void)Bus::wait_line_idle(400'000u);
            delay_us(clock, 400);
        }
        const uint16_t w = last_width;
        const uint16_t caught = captures;
        T0::enable_capt_interrupt(false);
        T0::disable();
        Ch2::off();
        link::Report r{};
        const bool have = peer_report(r, 10);
        // XDIR leads the start bit by one baud clock and falls after the
        // stop bit: guard + 10 bits of an 8N1 frame.
        const uint32_t bit_ticks = SysClock::hz / rate;
        const uint32_t want = bit_ticks * (Bus::guard_bits + 10u);
        print(serial, "  ", rate, " baud: XDIR high ", w, " ticks (expected ", want,
              " = ", Bus::guard_bits, " guard + 10 frame bits), pulses=", caught,
              ", peer count=", have ? r.count : 0, " sum=", hex(have ? r.sum : 0),
              " want=", hex(sum), " ferr=", have ? r.ferr : 0, crlf);
        verdict(num(rate), " baud: the peer receives every RS-485 frame",
                up && have && r.count == 4 && r.sum == sum && r.ferr == 0);
        verdict(num(rate), " baud: XDIR is guard + frame long",
                caught >= 1 && near(w, static_cast<int32_t>(want),
                                    static_cast<int32_t>(bit_ticks / 2 + 4)));
        (void)link_command_mode();
    }
    quiesce();
    (void)link_command_mode();
}

// ---- s: IRCOM on the wire -------------------------------------------------------

void ts_irda() {
    print(serial, "s IRCOM across the wire at 115200 (the mode's IrDA ceiling)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }

    struct IrCase { uint8_t txpl; uint8_t rxpl; const char* name; bool expect; };
    const IrCase cases[] = {
        {0x00, 0x00, "3/16 pulses both ways", true},
        {0x3C, 0x00, "a fixed 60-cycle pulse", true},
    };
    for (const IrCase& ic : cases) {
        link::Cfg pc{};
        pc.mode = link::Mode::ircom;
        pc.rate = 115'200;
        pc.txpl = ic.txpl;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 24);
        if (!peer_act(link::Op::echo, a)) {
            verdict(ic.name, "", false);
            (void)link_command_mode();
            continue;
        }
        const bool up = dut_ircom(ic.txpl, ic.rxpl);
        const uint16_t bad = up ? echo_mismatches(8, 6) : 6;
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(serial, "  ", ic.name, ": wrong=", bad, " peer count=", have ? r.count : 0,
              crlf);
        verdict(ic.name, ic.expect ? " works" : " blocks reception",
                up && (ic.expect ? bad == 0 : bad == 6));
    }

    // The RXPL filter, swept rather than guessed: at 115200 on a 24 MHz
    // CLK_PER a bit is 208 peripheral clocks and the 3/16 pulse is 39 of
    // them, so where reception stops says what unit RXPL counts in.
    {
        const uint8_t rxpls[] = {0, 6, 20, 40, 60, 100, 127};
        uint16_t first_block = 0;
        for (uint8_t rx : rxpls) {
            link::Cfg pc{};
            pc.mode = link::Mode::ircom;
            pc.rate = 115'200;
            link::Params a{};
            a.cfg = pc;
            a.count = 6;
            a.ms = bound_ms(115'200, 24);
            if (!peer_act(link::Op::echo, a)) { (void)link_command_mode(); continue; }
            const bool up = dut_ircom(0x00, rx);
            const uint16_t bad = up ? echo_mismatches(8, 6) : 6;
            print(serial, "  RXPL ", rx, " (", rx + 1, " samples needed): wrong=", bad,
                  crlf);
            if (bad == 6 && first_block == 0) first_block = rx;
            (void)link_command_mode();
        }
        print(serial, "  the 3/16 pulse (39 CLK_PER of a 208-clock bit) is rejected from "
              "RXPL ", first_block, crlf);
        verdict("the RXPL filter does reject the 3/16 pulse somewhere in 0..127",
                first_block != 0);
    }

    // TXPL = 0xFF turns the coder off. Is the line then plain async? Ask a
    // NORMAL asynchronous receiver on the other board.
    {
        link::Cfg pc{};
        pc.rate = 115'200;                               // the peer stays ASYNC
        link::Cfg dc{};
        dc.mode = link::Mode::ircom;
        dc.rate = 115'200;
        dc.txpl = 0xFF;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 24);
        const bool sent = peer_act(link::Op::echo, a);
        const bool up = sent && dut_ircom(0xFF, 0x00);
        const uint16_t bad = up ? echo_mismatches(8, 6) : 6;
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(serial, "  TXPL=0xFF against a plain async peer: DUT->peer count=",
              have ? r.count : 0, " ferr=", have ? r.ferr : 0, ", peer->DUT wrong=", bad,
              crlf);
        // TXPL is a TRANSMITTER knob: with the coder off the DUT's frames
        // are plain asynchronous and a normal receiver reads them. The
        // RECEIVER is a different matter and the next verdict measures it.
        verdict("with the coder off the IRCOM TRANSMITTER is plain asynchronous",
                up && have && r.count == 6 && r.ferr == 0);
        verdict("but its RECEIVER still expects pulses, so plain async is not readable",
                bad != 0);
        (void)link_command_mode();
    }

    // And the same interop the other way round: a coded transmitter into a
    // plain receiver must NOT be readable, or the coding means nothing.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Cfg dc{};
        dc.mode = link::Mode::ircom;
        dc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 24);
        const bool sent = peer_act(link::Op::sink, a);
        const bool up = sent && dut_ircom(0x00, 0x00);
        uint16_t sum = 0;
        wait_for_peer_window();
        line_talk();
        for (uint8_t i = 0; i < 6; ++i) {
            const uint16_t v = static_cast<uint16_t>(0x41 + i);
            (void)U4::send(v, 400'000u);
            sum = static_cast<uint16_t>(sum + v);
        }
        line_listen();
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(serial, "  3/16 pulses into a plain async receiver: peer count=",
              have ? r.count : 0, " sum=", hex(have ? r.sum : 0), " want=", hex(sum),
              " ferr=", have ? r.ferr : 0, crlf);
        verdict("a pulse-coded transmitter is not readable as plain async",
                up && have && !(r.count == 6 && r.sum == sum && r.ferr == 0));
        (void)link_command_mode();
    }
    quiesce();
    (void)link_command_mode();
}

// ---- t: a clock rebase under real traffic ---------------------------------------

void tt_rebase_traffic() {
    print(serial, "t 24 -> 12 -> 24 MHz mid-stream, the peer counting every byte", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    link::Cfg pc{};
    pc.rate = 115'200;
    link::Params a{};
    a.cfg = pc;
    a.count = 96;
    a.ms = 2000;
    const bool sent = peer_act(link::Op::sink, a);
    verdict("the peer accepts a 96-frame SINK", sent);
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());
    U4Tx::init(DynClock{}, 115'200u);

    uint16_t sum = 0;
    auto stream = [&sum](uint8_t from, uint8_t to) {
        for (uint16_t v = from; v < to; ++v) {
            for (uint32_t s = 0; s < 400'000u; ++s) {
                if (U4Tx::write_byte(static_cast<uint8_t>(v))) break;
            }
            sum = static_cast<uint16_t>(sum + v);
        }
    };
    stream(0, 32);
    verdict("switch to 12 MHz under traffic", DynClock::set(12'000'000u));
    stream(32, 64);
    verdict("back to 24 MHz under traffic", DynClock::set(24'000'000u));
    stream(64, 96);
    for (uint32_t s = 0; s < 800'000u && !U4Tx::tx_idle(); ++s) {
    }
    delay_us(clock, 20'000);
    U4::enable_rxc_interrupt(false);

    link::Report r{};
    const bool have = peer_report(r, 40);
    print(serial, "  peer saw count=", have ? r.count : 0, " sum=", hex(have ? r.sum : 0),
          " want=", hex(sum), " ferr=", have ? r.ferr : 0, " perr=", have ? r.perr : 0,
          " ovf=", have ? r.ovf : 0, crlf);
    verdict("every byte survived the two clock switches",
            have && r.count == 96 && r.sum == sum);
    verdict("and not one of them was corrupted",
            have && r.ferr == 0 && r.perr == 0 && r.ovf == 0);
    quiesce();
    (void)link_command_mode();
}

// ---- u: does a loop-back receiver hear the PAD? ---------------------------------
// The open question of the single-board half: LBME connects the receiver
// to the TXD PAD, not to the shifter (a pinless route receives nothing).
// If it really is the pad, then a driver on the other board pulling that
// pad should be heard - which is what makes one-wire collision detection
// electrically real. The DUT joins with ODME so its own pad is held by
// the pull-up only: no contention, no short.

void tu_lbme_pad() {
    print(serial, "u LBME: can a loop-back receiver hear an EXTERNAL driver on TXD?", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    link::Params a{};
    a.cfg.rate = 9600;
    a.count = 4;
    a.ms = 400;
    a.value = 0x5A;
    a.cell = -1;
    const bool sent = peer_act(link::Op::drive_rxd, a);
    verdict("the peer accepts DRIVE_RXD (bit-bang on its own RXD pad)", sent);
    // Open drain: PE0 stays an input held high by its pull-up (errata
    // 2.16.1), so the peer's push-pull driver meets only the pull-up.
    const bool up = U4::init({.route = UsartRoute::def,
                              .baud = usart_baud_reg(SysClock::hz, 9600u),
                              .loop_back = true, .open_drain = true});
    verdict("DUT in loop-back with the open-drain pin discipline", up);
    verdict("errata 2.16.1: TXD is left an INPUT, not driven high", !TxPin::is_output());
    quiet(500);
    const Drain d = drain();
    print(serial, "  external driver on the TXD pad: frames=", d.frames, " last=",
          hex(d.last), " ferr=", d.ferr, crlf);
    const bool heard = d.frames >= 3 && d.last == 0x5A && d.ferr == 0;
    print(serial, heard ? "  FINDING: the loop-back receiver DOES hear the pad - one-wire "
                          "collision detection is electrically real"
                        : "  FINDING: the loop-back receiver does NOT hear an external "
                          "driver on the pad",
          crlf);
    verdict("the probe produced a definite answer", d.frames == 0 || heard);
    (void)link_command_mode();

    // The complementary half: the DUT's own transmission always comes
    // back through the same path (this one is purely internal - on the
    // crossed wiring the peer cannot hear PE0 at all).
    verdict("DUT one-wire init", U4::init({.route = UsartRoute::def,
                                           .baud = usart_baud_reg(SysClock::hz, 9600u),
                                           .loop_back = true, .open_drain = true}));
    bool echoed = true;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint8_t v = static_cast<uint8_t>(0xC0 + i);
        (void)U4::send(v, 400'000u);
        const auto back = U4::wait(400'000u);
        echoed = echoed && back && back->clean() && back->data == v;
    }
    verdict("an open-drain transmitter reads its own frames back", echoed);
    quiesce();
    (void)link_command_mode();
}

// ---- w: the one-wire bus (needs the PE0-PE0 jumper) -----------------------------

void tw_onewire() {
    print(serial, "w one-wire on a SHARED line - REQUIRES THE TWO TXD PADS JUMPERED "
                  "(A.PE0 - B.PE0)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer is reachable", false); return; }
    if (!shared_line()) {
        print(serial, "  SKIPPED: this desk carries the crossed full-duplex pair, which "
                      "has no shared line for two transmitters to share", crlf);
        return;
    }
    print(serial, "  (this desk carries the shared wire: the test runs)", crlf);
    link::Params a{};
    a.cfg.rate = 19'200;
    a.count = 4;
    a.ms = 600;
    verdict("the peer joins the one-wire line", peer_act(link::Op::onewire, a));
    verdict("DUT one-wire init", Line::init(clock, 19'200u));
    bool echoed = true, answered = true;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint8_t v = static_cast<uint8_t>(0xA5 + i);
        Line::talk();
        (void)Line::send(v, 400'000u);
        echoed = echoed && Line::echo_matches(v, 400'000u);
        (void)Line::listen(400'000u);
        const auto back = U4::wait(400'000u);
        answered = answered && back && back->clean() &&
                   static_cast<uint8_t>(back->data) == static_cast<uint8_t>(v ^ 0xFFu);
    }
    verdict("every frame comes back through the DUT's own echo path", echoed);
    verdict("the peer answers half duplex on the same wire", answered);

    // The collision: the peer transmits into the DUT's frame, and the
    // open-drain AND of the two makes the echo differ from what was sent.
    (void)link_command_mode();
    link::Params c{};
    c.cfg.rate = 19'200;
    c.ms = 600;
    c.value = 0x00;
    // The peer only knows a frame started once it has received all of
    // it, so it cannot collide with the frame that triggered it: it
    // collides with the NEXT one. 150 us into a 520 us frame at 19200.
    c.aux16 = 150;
    const bool armed = peer_act(link::Op::collide, c);
    (void)Line::init(clock, 19'200u);
    wait_for_peer_window();                              // or the trigger arrives too early
    Line::talk();
    (void)Line::send(0xA5, 400'000u);                    // the trigger
    const bool trigger_clean = Line::echo_matches(0xA5, 400'000u);
    (void)Line::send(0xFF, 400'000u);                    // collided with
    const bool mismatch = !Line::echo_matches(0xFF, 400'000u);
    (void)Line::listen(400'000u);
    verdict("the trigger frame itself is echoed clean", armed && trigger_clean);
    verdict("a simultaneous transmitter shows up as an echo mismatch", armed && mismatch);
    quiesce();
    (void)link_command_mode();
}

// ---- the menu ------------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_instances}, {'b', tb_frames}, {'c', tc_overflow}, {'d', td_mpcm},
    {'e', te_txc_dre}, {'f', tf_baud_on_the_wire}, {'g', tg_rebase},
    {'h', th_autobaud}, {'i', ti_mspi},
    {'j', tj_link}, {'k', tk_baud}, {'l', tl_frames_cross}, {'m', tm_errors},
    {'n', tn_waveforms}, {'o', to_autobaud_foreign}, {'p', tp_mpcm_cross},
    {'q', tq_sync}, {'r', tr_rs485}, {'s', ts_irda}, {'t', tt_rebase_traffic},
    {'u', tu_lbme_pad}, {'v', tv_wiring}, {'w', tw_onewire},
};
constexpr char single_board[] = "abcdefghi";
constexpr char two_board[] = "jklmnopqrstu";

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_serial, one board: a instances | b frames | c overflow | "
                  "d mpcm | e txc/dre | f baud on the wire | g rebase | h auto-baud | "
                  "i host spi", crlf);
    print(serial, "  two boards (needs usart_peer on board B): j link | k baud matrix | "
                  "l frame matrix | m errors | n waveforms | o auto-baud foreign | "
                  "p mpcm | q sync roles | r rs-485 | s ircom | t rebase | u lbme pad",
          crlf);
    print(serial, "  v wiring probe (with board B's '2') | w one-wire (needs the "
                  "PE0-PE0 jumper); neither is in y", crlf);
    print(serial, "  z = all single-board, y = all two-board", crlf);
}

/// The single-board half runs on the DUT's own loop-back, which on a
/// SHARED-line desk is a wire the peer is sitting on too: it can
/// recognize a run of test bytes as a command frame and answer, driving
/// the line in the middle of a measurement. Ask it to let go first. On
/// the crossed wiring, and with no peer at all, this costs one ping.
uint8_t stand_off_buf[link::max_payload];
const uint8_t* stand_off_payload(uint16_t ms) {
    link::Params a{};
    a.cfg.apply = 0;
    a.ms = ms;
    link::put_params(stand_off_buf, a);
    return stand_off_buf;
}

void ask_peer_to_stand_off(uint16_t ms) {
    if (!ensure_link()) return;
    if (command(link::Op::standby, stand_off_payload(ms), link::params_size)) {
        print(serial, "  (board B asked to stay quiet for ", ms, " ms)", crlf);
    }
}

/// End it early: the single-board half is over, the peer may answer again.
void end_peer_stand_off() {
    (void)link_command_mode();
    (void)command(link::Op::standby, stand_off_payload(0), link::params_size);
}

void run_set(const char* keys) {
    uint16_t tp = 0, tf = 0;
    for (const char* k = keys; *k != 0; ++k) {
        for (const Test& t : tests) {
            if (t.key == *k) { run(t.fn); tp += passed; tf += failed; }
        }
    }
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(USART4_RXC_vect) { (void)U4Tx::rxc(); }
ISR(USART4_DRE_vect) { U4Tx::dre(); }
ISR(TCB0_INT_vect) {
    last_width = Meter::width_ticks();
    ++captures;
}
ISR(RTC_PIT_vect) { Ticker::pit(); }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_serial - USART test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF",
          " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'z' || c == 'Z') {
            ask_peer_to_stand_off(65'000);
            run_set(single_board);
            end_peer_stand_off();
        }
        else if (c == 'y' || c == 'Y') { run_set(two_board); }
        // 'w' is a SET of its own (the shared-wire block, outside y/z):
        // close it with the "ALL:" line so bench.py run can judge it.
        else if (c == 'w' || c == 'W') { run_set("w"); }
        else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
