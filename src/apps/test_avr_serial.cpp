// test_avr_serial - the USART test SUITE for the AVR DA/DB target,
// single-board half: instance and route handling (including the
// pinless NONE route and the teardown), the whole frame-format matrix
// through the internal loop-back, the receive FIFO's overflow, the
// multiprocessor filter, the TXC/DRE semantics, an ELECTRICAL check of
// the fractional baud generator (the start bit measured by a TCB
// through the event system), the clock rebase, auto-baud and Host SPI.
// Reference test of avrdx/usart.hpp (docs/avrdx/usart.md): keep it
// passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800 - USART2 is never reconfigured here.
//
// Wires: none needed. Everything that needs a loop-back runs on USART4
// at its default position (TXD PE0, RXD PE1, XCK PE2) with LBME: the
// internal loop-back is taken at the TXD PAD, so a pinless route
// receives nothing (bench finding, and what usart.hpp now refuses).
// PE0/PE1/PE2 are jumpered to the second bench board, which must be
// holding them as inputs - flash it with family_probe first.
// USART0's default route is NOT usable on this board: TXD would be PA0,
// the 24 MHz crystal pin. The per-instance loop-back smoke therefore
// uses USART0 ALT1 (PA4/PA5), USART1 default (PC0/PC1) and USART3 ALT1
// (PB4/PB5); on this desk PC0 and PB4 are traffic LEDs (harmless) and
// PA4 doubles as SPI0 MOSI and traffic button 2 - do not hold a button
// down while test b runs.
//
// Commands: ? | a instances | b frames | c overflow | d mpcm
// | e txc/dre | f baud on the wire | g rebase | h auto-baud | i host spi
// | z all

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

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
/// handed to PORT), the meter and its channel off.
void quiesce() {
    U0::release();
    U1::release();
    U3::release();
    U4::release();
    T0::disable();
    T0::enable_capt_interrupt(false);
    ChTx::off();
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
    print(serial, "e DRE = the buffer accepts, TXC = the line went idle (9600)", crlf);
    quiesce();
    if (!lbme4({}, 9600u)) { verdict("init", false); quiesce(); return; }
    verdict("DREIF is set on an idle transmitter", U4::dre_flag());
    U4::clear_txc();
    verdict("TXCIF cleared by a plain write-one store", !U4::txc_flag());

    // No printing between the writes and the measurement: a console
    // line at 460800 lasts longer than a frame at 9600 and every flag
    // would already have settled.
    U4::transmit(0xAA);                          // straight through to the shifter
    U4::transmit(0x55);                          // fills TXDATA and the buffer
    const bool dre_busy = !U4::dre_flag();
    const bool txc_busy = !U4::txc_flag();
    const uint32_t to_dre = spin_until([] { return U4::dre_flag(); });
    const bool txc_at_dre = U4::txc_flag();
    const uint32_t to_txc = spin_until([] { return U4::txc_flag(); });

    verdict("DREIF low with two frames queued", dre_busy);
    verdict("TXCIF still low while shifting", txc_busy);
    print(serial, "  spins to DREIF: ", to_dre, ", then to TXCIF: ", to_txc, crlf);
    verdict("DREIF returns while the line is still busy", to_dre > 0 && !txc_at_dre);
    verdict("TXCIF only once the last frame has left", to_txc > 0);

    const auto a = U4::poll();
    const auto b = U4::poll();
    verdict("both frames arrived", a && b && a->data == 0xAA && b->data == 0x55);
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

// ---- the menu ------------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_instances}, {'b', tb_frames}, {'c', tc_overflow}, {'d', td_mpcm},
    {'e', te_txc_dre}, {'f', tf_baud_on_the_wire}, {'g', tg_rebase},
    {'h', th_autobaud}, {'i', ti_mspi},
};

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_serial: a instances | b frames | c overflow | d mpcm | "
                  "e txc/dre | f baud on the wire | g rebase | h auto-baud | "
                  "i host spi | z all", crlf);
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

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
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
            uint16_t tp = 0, tf = 0;
            for (const Test& t : tests) { run(t.fn); tp += passed; tf += failed; }
            print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
        } else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
