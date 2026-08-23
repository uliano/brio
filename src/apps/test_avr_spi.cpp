// test_avr_spi - the SPI test SUITE for the AVR DA/DB target.
//
// SINGLE BOARD (a..j, `z`): route handling and teardown (including the
// pinless NONE route and the two refusals the package and the errata
// impose), an ELECTRICAL measurement of all seven bit rates (the SPI's
// own SCK event through a TCB frequency meter), the data path with the
// MISO pin driven by this board's own PORT, the four transfer modes'
// idle levels, the write-collision flag and its documented clear
// sequence, the buffer mode's four flags and their clear disciplines,
// the host demotion an SS pin can force, the two ISR bodies, the clock
// rebase with an SCK ceiling, and the transfer engine (SpiHost) with
// both its completion styles.
//
// Reference test of avrdx/spi.hpp (docs/avrdx/spi.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800 - USART2 is never reconfigured here.
//
// Wires: NONE of its own. Everything runs on SPI0 ALT1 (MOSI PE0, MISO
// PE1, SCK PE2, SS PE3) with the board answering itself: MISO and SS are
// driven by this board's own PORT while the SPI reads them (and the SS
// pin's INVEN fakes the external driver that would demote the host), and
// MOSI is observed through a pin event and an edge counter. All four PE
// pins carry the campaign's board-to-board link, so board B MUST BE
// INERT while this suite runs - flash `family_probe` there, it leaves
// PORTE as inputs. A peer that drives any of the four fights this board
// pin for pin.
// SPI0's DEFAULT route (PA4-PA7) is NEVER used here: on this desk those
// pins are cabled to a 3.3 V display module and an MCP3550, and the desk
// runs at 5 V. SPI1 is exercised on route NONE only - its pin positions
// (PC0-PC3 / PC4-PC7) are the traffic LEDs of this bench.
//
// Commands: ? for the menu, z = all.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/spi.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "kernel/active_object.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

using S0 = Spi<0>;
using S1 = Spi<1>;

using Mosi = Pin<'E', 0>;
using Miso = Pin<'E', 1>;
using Sck = Pin<'E', 2>;
using Ss = Pin<'E', 3>;

// The SCK meter: the SPI's own SCK event (a level generator, legal on
// every channel) into a TCB measuring the period between rising edges.
using ChSck = EventChannel<0>;
using T0 = Tcb<0>;
using SckMeter = FrequencyMeter<T0>;
// The MOSI edge counter: a PORTE pin event (channels 4-5) counted by a
// second TCB.
using ChMosi = EventChannel<4>;
using T1 = Tcb<1>;
using MosiCount = PulseCounter<T1>;

// The transfer engine and the arbiter that rides it (util/spi_bus.hpp).
using Host = SpiHost<0, SpiRoute::alt1>;
using Bus = SpiBus<Host, AvrPlatform>;
static_assert(ActiveObject<Bus>,
              "the SpiBus/BusMaster stratum must still fit over this engine");

using DynClock = DynamicClock<SysClock, Serial, Host, SckMeter>;

// ---- ISR state ---------------------------------------------------------------

volatile uint16_t captures = 0;
volatile uint16_t min_ticks = 0xFFFF;
volatile uint16_t last_ticks = 0;

volatile bool engine_mode = false;      // the SPI vector belongs to the engine
volatile bool engine_done = false;
volatile bool buffer_isr = false;       // which INTFLAGS layout the ISR must use
volatile uint16_t isr_count = 0;
volatile uint8_t isr_flags = 0;
volatile uint8_t isr_data = 0;
volatile uint8_t isr_bytes[12];
volatile uint8_t isr_flag_log[12];
volatile uint8_t isr_idx = 0;

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

/// Everything back to a known quiet state: both instances released
/// (pins handed to PORT), the meters and their channels off, PE1/PE3
/// back to plain inputs, the SPI vector back to the counting body.
void quiesce() {
    S0::release();
    S1::release();
    T0::enable_capt_interrupt(false);
    T0::disable();
    T1::disable();
    ChSck::off();
    ChMosi::off();
    Miso::input();
    Miso::clear();
    Ss::input();
    Ss::set();
    Ss::pullup(false);
    Ss::invert(false);
    engine_mode = false;
    engine_done = false;
    buffer_isr = false;
    isr_count = 0;
    captures = 0;
    min_ticks = 0xFFFF;
}

/// SPI0 as a host on ALT1, with the knobs this suite varies.
bool host(SpiClock c, SpiMode m = SpiMode::mode0, bool buffered = false, bool ssd = true) {
    return S0::init({.route = SpiRoute::alt1,
                     .role = SpiRole::host,
                     .mode = m,
                     .clock = c,
                     .client_select_disable = ssd,
                     .buffer_mode = buffered});
}

/// Drive the MISO pin from this board's own PORT: what the SPI shifts
/// in is then a level this suite chose. The peripheral only READS the
/// pin in host mode (28.2.2) - but the direction OVERRIDE that makes
/// MISO an input is latched when the SPI is ENABLED: a PORT.DIRSET
/// while it is running has no effect on the pad (bench). So the level
/// is set with the instance briefly disabled, and it sticks.
void miso_level(bool high) {
    const bool was_enabled = S0::enabled();
    S0::enable(false);
    if (high) Miso::set(); else Miso::clear();
    Miso::output();
    if (was_enabled) S0::enable(true);
}

/// One polled host byte at the resource level, bounded.
std::optional<uint8_t> xfer(uint8_t out) { return S0::transfer(out, 200'000u); }

// ---- a: routes, pins and the refusals ----------------------------------------

void ta_routes() {
    print(serial, "a routes: PORTMUX, pin claim and teardown, the package and errata "
                  "refusals", crlf);
    quiesce();

    verdict("SPI0 ALT1 host init", host(SpiClock::div16));
    verdict("PORTMUX reads ALT1", S0::routed() == SpiRoute::alt1);
    verdict("route() remembers it", S0::route() == SpiRoute::alt1);
    verdict("MOSI PE0 driven by the SPI", Mosi::is_output());
    verdict("SCK PE2 driven by the SPI", Sck::is_output());
    verdict("MISO PE1 left an input", !Miso::is_output());
    verdict("SS PE3 untouched (SSD set)", !Ss::is_output());
    verdict("the instance is enabled and a host", S0::enabled() && S0::is_host());

    S0::release();
    verdict("release routes back to NONE", S0::routed() == SpiRoute::none);
    verdict("release hands MOSI back", !Mosi::is_output());
    verdict("release hands SCK back", !Sck::is_output());
    verdict("release disables the instance", !S0::enabled());

    // The pinless route: an instance with no pins still runs.
    verdict("SPI0 pinless host init", S0::init({.route = SpiRoute::none}));
    verdict("no pin claimed on the pinless route", !Mosi::is_output() && !Sck::is_output());
    S0::release();
    verdict("SPI1 pinless host init", S1::init({.route = SpiRoute::none}));
    verdict("SPI1 PORTMUX reads NONE", S1::routed() == SpiRoute::none);
    S1::release();

    // What must be refused at run time (the compile-time twins live in
    // test/family/neg/).
    verdict("SPI1 ALT2 refused on a 48-pin part (DB errata 2.11.1)",
            !S1::init({.route = SpiRoute::alt2}));
    verdict("SPI0 ALT2 refused (PORTG is 64-pin)", !S0::init({.route = SpiRoute::alt2}));
    verdict("a pinless host watching SS refused (DA errata 2.10.1)",
            !S0::init({.route = SpiRoute::none, .client_select_disable = false}));
    verdict("a pinless client refused", !S0::init({.route = SpiRoute::none,
                                                   .role = SpiRole::client}));
    verdict("BUFWR without BUFEN refused", !S0::init({.route = SpiRoute::alt1,
                                                      .role = SpiRole::client,
                                                      .buffer_wait = true}));

    // The client's pin picture is the mirror image of the host's.
    verdict("SPI0 ALT1 client init", S0::init({.route = SpiRoute::alt1,
                                               .role = SpiRole::client}));
    verdict("a client drives MISO only",
            Miso::is_output() && !Mosi::is_output() && !Sck::is_output() && !Ss::is_output());
    verdict("a client is not a host", !S0::is_host() && S0::role() == SpiRole::client);
    S0::release();
    verdict("release hands MISO back", !Miso::is_output());

    // A host that watches its SS pin takes it as an input WITH the
    // pull-up: a floating SS would demote it at random.
    verdict("SPI0 ALT1 host with SSD = 0", host(SpiClock::div16, SpiMode::mode0, false, false));
    verdict("SS PE3 an input", !Ss::is_output());
    verdict("SS PE3 pulled up", (pinctrl_of('E', 3) & PORT_PULLUPEN_bm) != 0);
    S0::release();
    verdict("release clears the pull-up it turned on",
            (pinctrl_of('E', 3) & PORT_PULLUPEN_bm) == 0);
    quiesce();
}

// ---- b: the seven bit rates, measured ----------------------------------------

void measure_rate(SpiClock c) {
    if (!host(c)) { verdict("host init", false); return; }
    miso_level(true);
    SckMeter::init(clock, ChSck{}, TcbClock::div1);
    cli();
    captures = 0;
    min_ticks = 0xFFFF;
    sei();
    // A stream of bytes: the meter sees one period per SCK cycle, plus
    // the long gap between bytes - so the MINIMUM period is the rate
    // (the first two captures are dropped: arming a capture input reads
    // a spurious edge, a TCB finding of the timer campaign).
    for (uint8_t i = 0; i < 64; ++i) {
        (void)xfer(0xAA);
    }
    T0::enable_capt_interrupt(false);
    const uint16_t m = min_ticks;
    const uint16_t got = captures;
    const uint32_t expected = spi_division(c);
    print(serial, "  CLK_PER/", expected, " = ", spi_sck_hz(SysClock::hz, c),
          " Hz: captures=", got, " min period=", m, " ticks", crlf);
    verdict("the SCK period is CLK_PER/", got >= 8 && m == expected ? "division (exact)"
                                                                    : "division",
            got >= 8 && m == expected);
    S0::release();
    T0::disable();
}

void tb_rates() {
    print(serial, "b the seven bit rates on SCK (the SPI's own SCK event -> TCB0 "
                  "frequency meter, full period)", crlf);
    quiesce();
    ChSck::source(S0::SckEvent{});
    measure_rate(SpiClock::div2);
    measure_rate(SpiClock::div4);
    measure_rate(SpiClock::div8);
    measure_rate(SpiClock::div16);
    measure_rate(SpiClock::div32);
    measure_rate(SpiClock::div64);
    measure_rate(SpiClock::div128);
    quiesce();
}

// ---- c: the data path --------------------------------------------------------

void tc_data() {
    print(serial, "c data path: MISO driven by this board's PORT, MOSI counted "
                  "through a pin event", crlf);
    quiesce();
    verdict("host init", host(SpiClock::div16));

    miso_level(true);
    bool all_ff = true;
    for (uint8_t i = 0; i < 8; ++i) {
        const auto v = xfer(0x00);
        all_ff = all_ff && v && *v == 0xFF;
    }
    verdict("a MISO held high reads 0xFF on every byte", all_ff);

    miso_level(false);
    bool all_00 = true;
    for (uint8_t i = 0; i < 8; ++i) {
        const auto v = xfer(0xFF);
        all_00 = all_00 && v && *v == 0x00;
    }
    verdict("a MISO held low reads 0x00 on every byte", all_00);

    // The pin-direction override the levels above ran into, stated as
    // its own verdict: while the SPI is enabled a host's MISO pad
    // ignores PORT.DIR - the override is latched at ENABLE.
    Miso::input();
    Miso::set();
    Miso::output();                       // DIRSET under a running SPI
    delay_us(clock, 20);
    const bool ignored = !Miso::read();
    S0::enable(false);
    delay_us(clock, 20);
    const bool drives_when_off = Miso::read();
    S0::enable(true);
    delay_us(clock, 20);
    const bool keeps_driving = Miso::read();
    print(serial, "  FINDING: MISO DIRSET under a running host is ",
          ignored ? "IGNORED" : "honoured", ", the same DIR with the SPI disabled ",
          drives_when_off ? "drives" : "does not drive",
          ", and it survives the next enable: ", keeps_driving, crlf);
    verdict("a host's MISO direction is overridden while the SPI runs", ignored);
    verdict("the override is latched at enable, not enforced continuously",
            drives_when_off && keeps_driving);

    // MOSI: a stream of 0xAA is four rising edges per byte. A stream of
    // 0x00 is not silent - see the finding below: the line parks HIGH
    // between transfers, so every byte of zeros costs one rising edge.
    miso_level(false);
    ChMosi::source(EvPin<Mosi>{});
    MosiCount::init(ChMosi{});
    MosiCount::reset();
    delay_us(clock, 1000);
    verdict("no MOSI edges while the bus is idle", MosiCount::count() == 0);
    MosiCount::reset();
    for (uint8_t i = 0; i < 8; ++i) (void)xfer(0xAA);
    const uint16_t edges_aa = MosiCount::count();
    MosiCount::reset();
    for (uint8_t i = 0; i < 8; ++i) (void)xfer(0x00);
    const uint16_t edges_00 = MosiCount::count();
    print(serial, "  MOSI rising edges: 8 x 0xAA = ", edges_aa, " (expect 32), 8 x 0x00 = ",
          edges_00, " (expect 8: the return to the idle level)", crlf);
    verdict("MOSI carries the pattern (0xAA: four rises a byte)", edges_aa == 32);
    verdict("a stream of zeros costs exactly one edge a byte", edges_00 == 8);

    // What MOSI shows BETWEEN transfers: neither the last bit sent nor
    // anything of the byte received - the line parks HIGH.
    miso_level(true);
    (void)xfer(0x00);
    delay_us(clock, 20);
    const bool after_rx_high = Mosi::read();
    miso_level(false);
    (void)xfer(0x00);
    delay_us(clock, 20);
    const bool after_rx_low = Mosi::read();
    print(serial, "  FINDING: after sending 0x00, MOSI sits at ", after_rx_high,
          " with a high MISO and at ", after_rx_low,
          " with a low one, while PORT.OUT for the pin is ",
          (PORTE.OUT & Mosi::mask) != 0,
          " - the host parks MOSI HIGH between transfers", crlf);
    verdict("MOSI parks high between transfers, whatever was sent or received",
            after_rx_high && after_rx_low);

    // A caveat of the self-driven-MISO technique, measured here so that
    // nobody trusts a byte read while MOSI toggles: with MISO held HIGH
    // and a toggling pattern on MOSI, what comes back is the PATTERN,
    // not 0xFF - the two halves of the desk's crossed pair run side by
    // side for 20 cm into an inert board. A MISO held LOW is immune
    // (0x00 comes back whatever is sent), so every byte-level check in
    // this suite either holds MISO low or sends a constant.
    miso_level(true);
    const auto quiet = xfer(0x00);
    const auto toggling = xfer(0xF0);
    print(serial, "  FINDING: MISO high, sending 0x00 reads ", hex(quiet ? *quiet : 0),
          " but sending 0xF0 reads ", hex(toggling ? *toggling : 0),
          " - a toggling MOSI couples into a MISO held high on this desk", crlf);
    verdict("a constant MISO is read exactly (0x00 sent -> 0xFF back)",
            quiet && *quiet == 0xFF);

    // Bit order: an edge count cannot tell MSb-first from LSb-first (a
    // reversed word has exactly as many rising edges in a repeated
    // stream), and a loop-back would be order-symmetric too. What is
    // checked here is the register plumbing and that the path still
    // carries data; the wire proof belongs to the two-board half.
    S0::lsb_first(true);
    verdict("DORD set reads back", S0::lsb_first());
    miso_level(true);
    const auto lsb = xfer(0xC1);
    verdict("the path still carries data with DORD set", lsb && *lsb == 0xFF);
    miso_level(false);
    MosiCount::reset();
    for (uint8_t i = 0; i < 8; ++i) (void)xfer(0xAA);
    const uint16_t edges_lsb = MosiCount::count();
    print(serial, "  8 x 0xAA with DORD set = ", edges_lsb,
          " rising edges: a bit-order proof needs a second board", crlf);
    verdict("an LSb-first stream still moves the same number of edges",
            edges_lsb == edges_aa);
    S0::lsb_first(false);
    verdict("DORD cleared reads back", !S0::lsb_first());
    quiesce();
}

// ---- d: the four transfer modes ----------------------------------------------

void check_mode(SpiMode m) {
    if (!host(SpiClock::div64, m)) { verdict("host init", false); return; }
    miso_level(true);
    const auto v = xfer(0x5A);
    delay_us(clock, 50);
    const bool idle = Sck::read();
    print(serial, "  mode ", static_cast<uint8_t>(m), ": CPOL=", spi_cpol(m),
          " CPHA=", spi_cpha(m), " SCK idle ", idle ? "high" : "low", crlf);
    verdict("SCK idles at CPOL", idle == spi_cpol(m));
    verdict("the mode reads back", S0::mode() == m);
    verdict("the transfer completed", v.has_value());
    S0::release();
}

void td_modes() {
    print(serial, "d the four transfer modes: SCK's idle level is CPOL (CPHA needs a "
                  "second board)", crlf);
    quiesce();
    check_mode(SpiMode::mode0);
    check_mode(SpiMode::mode1);
    check_mode(SpiMode::mode2);
    check_mode(SpiMode::mode3);
    quiesce();
}

// ---- e: the write collision --------------------------------------------------

void te_wrcol() {
    print(serial, "e normal mode: WRCOL and the two clear disciplines of the layout",
          crlf);
    quiesce();
    verdict("host init at CLK_PER/128", host(SpiClock::div128));
    miso_level(true);

    S0::write(0x5A);              // starts a 42.7 us byte
    S0::write(0x3C);              // ignored, and WRCOL is set
    verdict("a write during a transfer sets WRCOL", S0::write_collision());
    for (uint32_t i = 0; i < 200'000u && !S0::if_flag(); ++i) {}
    verdict("the transfer itself completed (IF)", S0::if_flag());
    const uint8_t f = S0::flags();
    const uint8_t d = S0::read();      // INTFLAGS then DATA: the documented sequence
    verdict("the collided transfer's data is unharmed", d == 0xFF);
    verdict("both flags were up before the sequence",
            (f & SPI_IF_bm) != 0 && (f & SPI_WRCOL_bm) != 0);
    verdict("the read-then-DATA sequence clears IF and WRCOL",
            !S0::if_flag() && !S0::write_collision());

    // The other documented path: a plain store of one bit. It clears IF
    // and, per the register description, NOTHING else - WRCOL has no
    // write-one-to-clear path at all.
    S0::write(0x5A);
    S0::write(0x3C);
    for (uint32_t i = 0; i < 200'000u && !S0::if_flag(); ++i) {}
    const bool both_up = S0::if_flag() && S0::write_collision();
    S0::clear_if();
    const bool if_gone = !S0::if_flag();
    const bool wrcol_stays = S0::write_collision();
    print(serial, "  FINDING: after a W1C store to IF alone, WRCOL ",
          wrcol_stays ? "SURVIVES (only the read-then-DATA sequence clears it)"
                      : "is cleared too", crlf);
    verdict("W1C on IF clears IF", both_up && if_gone);
    verdict("W1C on IF leaves WRCOL alone", both_up && wrcol_stays);
    S0::clear_flags_by_data_access();
    verdict("the sequence then clears WRCOL", !S0::write_collision());
    quiesce();
}

// ---- f: buffer mode ----------------------------------------------------------

void tf_buffer() {
    print(serial, "f buffer mode: the four flags, the two transmit levels, BUFOVF",
          crlf);
    quiesce();
    verdict("host init in buffer mode", host(SpiClock::div128, SpiMode::mode0, true));
    verdict("BUFEN reads back", S0::buffer_mode());
    miso_level(true);

    // Every flag below is SAMPLED first and judged afterwards: a
    // verdict line takes a millisecond on the console, and two bytes at
    // CLK_PER/128 are gone in 85 us.
    const bool dre_idle = S0::dre_flag();
    const bool txc_idle = S0::txc_flag();
    S0::write(0x11);              // straight into the shifter
    const bool dre_after_one = S0::dre_flag();
    const bool txc_after_one = S0::txc_flag();
    S0::write(0x22);              // into the transmit buffer
    const bool dre_after_two = S0::dre_flag();
    const bool txc_after_two = S0::txc_flag();
    print(serial, "  idle: DREIF=", dre_idle, " TXCIF=", txc_idle,
          "; after one write: DREIF=", dre_after_one, " TXCIF=", txc_after_one,
          "; after two: DREIF=", dre_after_two, " TXCIF=", txc_after_two, crlf);
    verdict("DREIF is up on an idle transmitter", dre_idle);
    verdict("init leaves TXCIF clear", !txc_idle);
    verdict("the first write leaves room (DREIF up)", dre_after_one);
    verdict("the second write fills the buffer (DREIF down)", !dre_after_two);
    verdict("TXCIF stays down while the two bytes are in flight", !txc_after_two);

    // A W1C store to DREIF must NOT clear it: DREIF follows the DATA
    // register, never a store to INTFLAGS (28.5.5).
    for (uint32_t i = 0; i < 200'000u && !S0::dre_flag(); ++i) {}
    const bool dre_back = S0::dre_flag();
    S0::regs().INTFLAGS = SPI_DREIF_bm;
    const bool dre_survives = S0::dre_flag();
    print(serial, "  FINDING: a W1C store to DREIF ",
          dre_survives ? "leaves it set (it follows DATA alone)" : "CLEARS it", crlf);
    verdict("DREIF comes back when the buffer empties", dre_back);
    verdict("a store to DREIF does not clear it", dre_survives);

    for (uint32_t i = 0; i < 200'000u && !S0::txc_flag(); ++i) {}
    verdict("TXCIF rises when shifter and buffer are both empty", S0::txc_flag());
    S0::clear_txc();
    verdict("TXCIF is write-one-to-clear", !S0::txc_flag());

    // The receive side: two buffers plus the shifter. A third byte with
    // nothing drained sits in the shifter, and the chapter is explicit
    // that BUFOVF is not raised until the NEXT transfer starts.
    verdict("RXCIF is up with the two received bytes unread", S0::rxc_flag());
    S0::clear_txc();
    S0::write(0x33);
    for (uint32_t i = 0; i < 200'000u && !S0::txc_flag(); ++i) {}
    const bool ovf_third = S0::overflow_flag();
    S0::clear_txc();
    S0::write(0x44);
    for (uint32_t i = 0; i < 200'000u && !S0::txc_flag(); ++i) {}
    const bool ovf_fourth = S0::overflow_flag();
    print(serial, "  BUFOVF after the third undrained byte: ", ovf_third,
          ", after the fourth: ", ovf_fourth, crlf);
    verdict("the third undrained byte waits in the shifter (no BUFOVF yet)",
            !ovf_third);
    verdict("the next transfer raises BUFOVF", ovf_fourth);
    S0::clear_overflow();
    verdict("BUFOVF is write-one-to-clear", !S0::overflow_flag());
    for (uint8_t i = 0; i < 4; ++i) (void)S0::read();
    verdict("draining DATA clears RXCIF", !S0::rxc_flag());
    S0::clear_txc();

    // BUFWR is a client bit: the chapter says it does not affect Host
    // mode (28.3.2.1.2), and the flag sequence is the check.
    verdict("host init with BUFWR set", S0::init({.route = SpiRoute::alt1,
                                                  .role = SpiRole::host,
                                                  .clock = SpiClock::div128,
                                                  .buffer_mode = true,
                                                  .buffer_wait = true}));
    verdict("BUFWR reads back", S0::buffer_wait());
    const bool dre0 = S0::dre_flag();
    S0::write(0x44);
    const bool dre1 = S0::dre_flag();
    S0::write(0x55);
    const bool dre2 = S0::dre_flag();
    verdict("BUFWR does not change the host's two transmit levels",
            dre0 && dre1 && !dre2);
    for (uint32_t i = 0; i < 400'000u && !S0::txc_flag(); ++i) {}
    verdict("both bytes went out", S0::txc_flag());
    quiesce();
}

// ---- g: host demotion --------------------------------------------------------

void tg_demotion() {
    print(serial, "g host demotion: an SS pin seen low takes Host mode away (SSD = 0)",
          crlf);
    quiesce();
    verdict("host init with SSD = 0", host(SpiClock::div128, SpiMode::mode0, false, false));
    verdict("SSD reads back clear", !S0::client_select_disabled());
    verdict("a host with SS pulled high stays a host", S0::is_host() && !S0::demoted());

    // Table 28-2: an SS pin configured as an OUTPUT never demotes,
    // whatever level it is driven to - the pin is then the
    // application's to use. This board proves it on its own pin.
    Ss::clear();
    Ss::output();
    delay_us(clock, 20);
    const bool demoted_by_output = S0::demoted();
    Ss::input();
    delay_us(clock, 20);
    print(serial, "  FINDING: SS driven LOW as an OUTPUT demotes the host: ",
          demoted_by_output, " (table 28-2: an output SS keeps the host activated)",
          crlf);
    verdict("an SS pin driven low as an output does not demote", !demoted_by_output);

    // What does demote is an INPUT seen low. With no second driver on
    // this desk the pin's own INVEN provides it: the pad stays an input
    // held high by its pull-up, and the peripheral sees a low.
    Ss::invert(true);
    delay_us(clock, 20);
    const bool demoted = S0::demoted();
    const bool flagged = S0::if_flag();
    Ss::invert(false);
    verdict("an SS input seen low clears MASTER", demoted);
    verdict("the demotion raises IF (normal layout)", flagged);
    delay_us(clock, 20);
    verdict("MASTER does not come back by itself", !S0::is_host());
    S0::restore_host();
    verdict("restore_host re-arms Host mode", S0::is_host() && !S0::demoted());
    verdict("restore_host cleared the flag", !S0::if_flag());

    // Mid-transfer: a byte at CLK_PER/128 lasts 42.7 us.
    miso_level(true);
    S0::write(0x5A);
    delay_us(clock, 10);
    Ss::invert(true);
    delay_us(clock, 60);
    const bool demoted_mid = S0::demoted();
    Ss::invert(false);
    verdict("SS seen low mid-byte demotes the host too", demoted_mid);
    S0::restore_host();
    verdict("and the host recovers", S0::is_host());
    // The demotion made this instance a CLIENT for a moment, and a
    // client owns the MISO pad: re-establish the suite's level (the
    // helper cycles ENABLE, which is what re-latches the pin roles).
    miso_level(true);
    const auto after = xfer(0xA5);
    verdict("transfers work again after a recovery", after && *after == 0xFF);

    // The buffer layout reports the same event on SSIF.
    verdict("host init in buffer mode with SSD = 0",
            S0::init({.route = SpiRoute::alt1, .role = SpiRole::host,
                      .clock = SpiClock::div128, .client_select_disable = false,
                      .buffer_mode = true}));
    Ss::invert(true);
    delay_us(clock, 20);
    const bool demoted_buf = S0::demoted();
    const bool ssif = S0::ss_flag();
    Ss::invert(false);
    verdict("an SS input seen low demotes a buffer-mode host", demoted_buf);
    verdict("the demotion raises SSIF (buffer layout)", ssif);
    S0::restore_host();
    verdict("restore_host clears SSIF and re-arms", S0::is_host() && !S0::ss_flag());

    // With SSD set the pin is nobody's business.
    verdict("host init with SSD = 1", host(SpiClock::div128));
    Ss::pullup(true);
    Ss::invert(true);
    delay_us(clock, 20);
    const bool ssd_immune = S0::is_host() && !S0::demoted();
    Ss::invert(false);
    Ss::pullup(false);
    verdict("SSD = 1 ignores an SS pin seen low", ssd_immune);
    miso_level(true);
    const auto v = xfer(0x00);
    verdict("and keeps transferring", v && *v == 0xFF);
    quiesce();
}

// ---- h: the ISR bodies -------------------------------------------------------

void th_interrupts() {
    print(serial, "h the two ISR bodies on the one SPI vector", crlf);
    quiesce();
    verdict("host init", host(SpiClock::div64));
    miso_level(true);
    buffer_isr = false;
    isr_count = 0;
    S0::enable_interrupt(true);
    S0::write(0x11);
    for (uint32_t i = 0; i < 200'000u && isr_count == 0; ++i) {}
    verdict("one interrupt for one byte (normal layout)", isr_count == 1);
    verdict("the body hands the byte over", isr_data == 0xFF);
    verdict("the body saw IF", (isr_flags & SPI_IF_bm) != 0);
    verdict("the body's read cleared IF", !S0::if_flag());

    isr_count = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        S0::write(i);
        for (uint32_t k = 0; k < 200'000u && isr_count <= i; ++k) {}
    }
    verdict("one interrupt per byte over a burst", isr_count == 16);
    S0::enable_interrupt(false);

    // Buffer mode: RXC drains the FIFO, TXC is written one - the two
    // halves of take_buffer().
    // init() re-runs the pin setup, which hands MISO back to PORT as an
    // input: drive it again for the buffer-mode half.
    verdict("host init in buffer mode", host(SpiClock::div64, SpiMode::mode0, true));
    miso_level(true);
    buffer_isr = true;
    isr_count = 0;
    isr_idx = 0;
    S0::enable_rxc_interrupt(true);
    S0::enable_txc_interrupt(true);
    for (uint8_t i = 0; i < 8; ++i) {
        const uint16_t before = isr_count;
        // A CONSTANT byte on MOSI: a toggling one couples into the MISO
        // this board holds high (test c's finding) and the received
        // bytes would not be a clean 0xFF any more.
        S0::write(0x00);
        for (uint32_t k = 0; k < 200'000u && isr_count == before; ++k) {}
    }
    delay_us(clock, 500);
    const uint16_t n = isr_count;
    print(serial, "  buffer-mode interrupts for 8 bytes: ", n,
          " (one RXC each, plus the TXC of every idle gap)", crlf);
    verdict("the buffer body fires and stays quiet afterwards", n >= 8 && n <= 24);
    const uint16_t settled = isr_count;
    delay_us(clock, 2000);
    verdict("no interrupt storm: the body clears what it must",
            isr_count == settled);
    print(serial, "  last buffer-mode ISR flags=", hex(isr_flags), " data=",
          hex(isr_data), crlf);
    print(serial, "  bytes received under the interrupt:");
    for (uint8_t i = 0; i < isr_idx; ++i) print(serial, " ", hex(isr_bytes[i]));
    print(serial, crlf);
    verdict("the last byte received is what MISO carried", isr_data == 0xFF);
    S0::enable_rxc_interrupt(false);
    S0::enable_txc_interrupt(false);
    buffer_isr = false;
    quiesce();
}

// ---- i: the clock rebase with an SCK ceiling ---------------------------------

uint16_t measure_now() {
    cli();
    captures = 0;
    min_ticks = 0xFFFF;
    sei();
    for (uint8_t i = 0; i < 32; ++i) {
        (void)S0::transfer(0xAA, 200'000u);
    }
    return min_ticks;
}

void ti_rebase() {
    print(serial, "i rebase 24 -> 12 -> 24 MHz: the engine re-picks the division that "
                  "honours its SCK ceiling", crlf);
    quiesce();
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());

    constexpr uint32_t ceiling = 1'500'000u;
    verdict("engine init with a 1.5 MHz ceiling", Host::init(DynClock{}, ceiling));
    verdict("at 24 MHz the ceiling resolves to CLK_PER/16",
            Host::ceiling_clock() && *Host::ceiling_clock() == SpiClock::div16);

    ChSck::source(S0::SckEvent{});
    SckMeter::init(DynClock{}, ChSck{}, TcbClock::div1);
    miso_level(true);
    // The engine's own division is in force after init(); the resource's
    // polled transfer is the traffic.
    const uint16_t t24 = measure_now();
    print(serial, "  24 MHz: min period ", t24, " ticks = ",
          t24 ? 24'000'000u / t24 : 0, " Hz", crlf);
    verdict("SCK is CLK_PER/16 at 24 MHz", t24 == 16);

    verdict("switch to 12 MHz", DynClock::set(12'000'000u));
    verdict("the ceiling now resolves to CLK_PER/8",
            Host::ceiling_clock() && *Host::ceiling_clock() == SpiClock::div8);
    verdict("a request faster than the ceiling is clamped",
            Host::sck_hz(SpiClock::div2) <= ceiling);
    // Re-arm the peripheral at the re-picked division (init() is what an
    // application calls after a switch; the engine keeps the ceiling).
    verdict("engine re-init at 12 MHz", Host::init(DynClock{}, ceiling));
    miso_level(true);
    const uint16_t t12 = measure_now();
    print(serial, "  12 MHz: min period ", t12, " ticks = ",
          t12 ? 12'000'000u / t12 : 0, " Hz", crlf);
    verdict("SCK is CLK_PER/8 at 12 MHz", t12 == 8);
    verdict("the ceiling holds in Hz across the switch",
            near(static_cast<int32_t>(t24 ? 24'000'000u / t24 : 0),
                 static_cast<int32_t>(t12 ? 12'000'000u / t12 : 0), 1000));

    verdict("back to 24 MHz", DynClock::set(24'000'000u));
    verdict("engine re-init at 24 MHz", Host::init(DynClock{}, ceiling));
    const uint16_t back = measure_now();
    verdict("SCK is CLK_PER/16 again", back == 16);
    T0::enable_capt_interrupt(false);
    quiesce();
}

// ---- j: the transfer engine (SpiHost) ----------------------------------------

uint8_t eng_rx[8];
const uint8_t eng_cmd[3] = {0x11, 0x22, 0x33};

bool run_engine(const Host::Request& r, uint32_t spins = 400'000u) {
    engine_done = false;
    engine_mode = true;
    if (Host::start(r)) {
        engine_mode = false;
        return true;                       // completed synchronously
    }
    for (uint32_t i = 0; i < spins && !engine_done; ++i) {}
    engine_mode = false;
    return engine_done;
}

void tj_engine() {
    print(serial, "j the transfer engine: descriptors, both completion styles, CS and "
                  "DC, the pinless refusal", crlf);
    quiesce();
    verdict("engine init on ALT1", Host::init(clock));
    verdict("the engine took the route", S0::routed() == SpiRoute::alt1);
    miso_level(true);
    Ss::set();
    Ss::output();                          // PE3 is the chip select here (not wired)

    // Polled bulk read.
    for (uint8_t i = 0; i < 8; ++i) eng_rx[i] = 0;
    bool ok = run_engine({Ss::ref(), {}, nullptr, 0, nullptr, eng_rx, 8, {},
                          SpiClock::div16, SpiMode::mode0, true, 0});
    bool all_ff = ok;
    for (uint8_t i = 0; i < 8; ++i) all_ff = all_ff && eng_rx[i] == 0xFF;
    verdict("a polled request completes synchronously and fills rx", all_ff);
    verdict("the engine released CS", Ss::read());

    // The ISR pump.
    for (uint8_t i = 0; i < 8; ++i) eng_rx[i] = 0;
    ok = run_engine({Ss::ref(), {}, nullptr, 0, nullptr, eng_rx, 8, {},
                     SpiClock::div64, SpiMode::mode0, false, 0});
    bool pumped = ok;
    for (uint8_t i = 0; i < 8; ++i) pumped = pumped && eng_rx[i] == 0xFF;
    verdict("the ISR pump moves the same bytes", pumped);
    verdict("the pump released CS", Ss::read());

    // A command phase with a DC pin: the D/C line ends HIGH (the data
    // phase), and the command bytes go out on MOSI - counted with MISO
    // held LOW, so the line falls back to zero between bytes (the
    // received byte's MSb is what MOSI idles at - test c).
    miso_level(false);
    ChMosi::source(EvPin<Mosi>{});
    MosiCount::init(ChMosi{});
    MosiCount::reset();
    ok = run_engine({{}, Ss::ref(), eng_cmd, 3, nullptr, nullptr, 0, {},
                     SpiClock::div16, SpiMode::mode0, true, 0});
    // 0x11, 0x22, 0x33 back to back from a line that PARKS HIGH between
    // bytes (test c). 0x11 = 0,0,0,1,0,0,0,1: two rises, ends high, the
    // park adds none. 0x22 = 0,0,1,0,0,0,1,0: two rises, ends low, the
    // park adds one. 0x33 = 0,0,1,1,0,0,1,1: two rises, ends high.
    // Seven in all.
    const uint16_t cmd_edges = MosiCount::count();
    print(serial, "  command phase 0x11 0x22 0x33: ", cmd_edges,
          " MOSI rising edges (expect 7)", crlf);
    verdict("the command phase reaches the wire", ok && cmd_edges == 7);
    verdict("DC ends in the data phase (high)", Ss::read());

    // cs_setup_us and the degenerate request.
    ok = run_engine({Ss::ref(), {}, nullptr, 0, nullptr, eng_rx, 1, {},
                     SpiClock::div16, SpiMode::mode3, true, 10});
    verdict("a request with a chip-select setup delay still runs", ok);
    verdict("a mode-3 request leaves SCK idle high", Sck::read());
    ok = Host::start({{}, {}, nullptr, 0, nullptr, nullptr, 0, {}});
    verdict("a zero-length request completes on the spot without the wire", ok);

    // The ceiling clamps a too-fast request.
    verdict("engine re-init with a 400 kHz ceiling", Host::init(clock, 400'000u));
    verdict("the ceiling resolved to CLK_PER/64",
            Host::ceiling_clock() && *Host::ceiling_clock() == SpiClock::div64);
    ChSck::source(S0::SckEvent{});
    SckMeter::init(clock, ChSck{}, TcbClock::div1);
    cli();
    captures = 0;
    min_ticks = 0xFFFF;
    sei();
    (void)run_engine({Ss::ref(), {}, nullptr, 0, nullptr, eng_rx, 8, {},
                      SpiClock::div4, SpiMode::mode0, true, 0});
    const uint16_t clamped = min_ticks;
    print(serial, "  a CLK_PER/4 request under a 400 kHz ceiling ran at CLK_PER/",
          clamped, crlf);
    verdict("the engine slowed the request to its ceiling", clamped == 64);
    T0::enable_capt_interrupt(false);

    Host::release();
    verdict("the engine's release hands the pins back",
            !Mosi::is_output() && !Sck::is_output());
    Ss::input();
    quiesce();
}

// ---- the menu ----------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_routes}, {'b', tb_rates}, {'c', tc_data}, {'d', td_modes},
    {'e', te_wrcol}, {'f', tf_buffer}, {'g', tg_demotion}, {'h', th_interrupts},
    {'i', ti_rebase}, {'j', tj_engine},
};
constexpr char single_board[] = "abcdefghij";

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void run_set(const char* keys) {
    uint16_t tp = 0, tf = 0;
    for (const char* k = keys; *k != 0; ++k) {
        for (const Test& t : tests) {
            if (t.key != *k) continue;
            run(t.fn);
            tp += passed;
            tf += failed;
        }
    }
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
}

void help() {
    print(serial, "test_avr_spi: a routes | b bit rates | c data path | d modes | "
                  "e write collision | f buffer mode | g host demotion | "
                  "h isr bodies | i rebase | j engine", crlf);
    print(serial, "  z = all. NO WIRES of its own, but the desk's PORTE link is wired "
                  "straight through: board B MUST BE INERT (flash family_probe there) or "
                  "it drives the very pins this suite measures", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

ISR(SPI0_INT_vect) {
    if (engine_mode) {
        if (Host::isr()) engine_done = true;
        return;
    }
    if (buffer_isr) {
        isr_flags = S0::take_buffer();      // TXCIF/SSIF/BUFOVF written one
        const uint8_t d = S0::read();       // RXCIF follows the data
        if (isr_flags & SPI_RXCIF_bm) isr_data = d;
        if (isr_idx < 12) { isr_bytes[isr_idx] = d; isr_flag_log[isr_idx] = isr_flags; ++isr_idx; }
    } else {
        const auto r = S0::take_normal();   // INTFLAGS then DATA: the clear sequence
        isr_flags = r.flags;
        isr_data = r.data;
    }
    ++isr_count;
}

ISR(TCB0_INT_vect) {
    const uint16_t t = SckMeter::period_ticks();
    last_ticks = t;
    ++captures;
    if (captures > 2 && t < min_ticks) min_ticks = t;
}

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_spi - SPI test suite (board ", board,
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
        else if (c == 'z' || c == 'Z') { run_set(single_board); }
        else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
