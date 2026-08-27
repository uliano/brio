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
// TWO BOARDS (k..s, `y`): the same four pins with board B running
// `spi_peer` as a real client, driven IN BAND over the bus under test
// (src/apps/spi_link.hpp) - the client matrix (four transfer modes, both
// bit orders, all three buffering regimes), the rates against the
// client's CLK_PER/6 ceiling and above it, deliberate CPOL/CPHA/DORD
// mismatches, the select wire raised mid-byte, a client that never
// drains, a REAL host demotion driven by the other board, the USART's
// own Host SPI mode against this peripheral, and a clock rebase under
// two-board traffic.
//
// Reference test of avrdx/spi.hpp (docs/avrdx/spi.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800 - USART2 is never reconfigured here.
//
// Wires: NONE of its own. Everything runs on SPI0 ALT1 (MOSI PE0, MISO
// PE1, SCK PE2, SS PE3), which is also the desk's board-to-board link
// (A.PEn - B.PEn, n = 0..3, straight across). In the SINGLE-board half
// the board answers itself: MISO and SS are driven by its own PORT while
// the SPI reads them (and the SS pin's INVEN fakes the external driver
// that would demote the host), and MOSI is observed through a pin event
// and an edge counter. That half needs the other board to keep off all
// four wires, and `spi_peer` does: it is a DARK listener that drives
// MISO only for one answer window, after a frame that checked out. So
// both halves run with the peer attached - `z` scores the same with
// board B inert (`family_probe`) and with `spi_peer` on it.
// SPI0's DEFAULT route (PA4-PA7) is NEVER used here: on this desk those
// pins are cabled to a 3.3 V display module and an MCP3550, and the desk
// runs at 5 V. SPI1 is exercised on route NONE only - its pin positions
// (PC0-PC3 / PC4-PC7) are the traffic LEDs of this bench.
//
// Commands: ? for the menu, z = the single-board half, y = the two-board
// half.

// build: monitor_speed = 460800

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

#include "spi_link.hpp"

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
    bool ok = run_engine({Ss::ref(), {}, {}, 0, {}, lend<Lease::reply>(eng_rx), 8, {},
                          SpiClock::div16, SpiMode::mode0, true, 0});
    bool all_ff = ok;
    for (uint8_t i = 0; i < 8; ++i) all_ff = all_ff && eng_rx[i] == 0xFF;
    verdict("a polled request completes synchronously and fills rx", all_ff);
    verdict("the engine released CS", Ss::read());

    // The ISR pump.
    for (uint8_t i = 0; i < 8; ++i) eng_rx[i] = 0;
    ok = run_engine({Ss::ref(), {}, {}, 0, {}, lend<Lease::reply>(eng_rx), 8, {},
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
    ok = run_engine({{}, Ss::ref(), lend<Lease::reply>(eng_cmd), 3, {}, {}, 0, {},
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
    ok = run_engine({Ss::ref(), {}, {}, 0, {}, lend<Lease::reply>(eng_rx), 1, {},
                     SpiClock::div16, SpiMode::mode3, true, 10});
    verdict("a request with a chip-select setup delay still runs", ok);
    verdict("a mode-3 request leaves SCK idle high", Sck::read());
    ok = Host::start({{}, {}, {}, 0, {}, {}, 0, {}});
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
    (void)run_engine({Ss::ref(), {}, {}, 0, {}, lend<Lease::reply>(eng_rx), 8, {},
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

// ==============================================================================
//  THE TWO-BOARD HALF (k .. s, `y`): board B runs `spi_peer` and is driven IN
//  BAND over the very bus under test - the protocol is src/apps/spi_link.hpp.
//  The command channel is SPI mode 0, MSb first, CLK_PER/32, with PE3 as a
//  plain GPIO chip select (this end runs with SSD set, so the peripheral
//  leaves the pin alone). One exchange is three phases: the command frame in
//  ONE select window, a settling pause while the peer prepares its answer,
//  then a second window in which this end clocks dummies until its own
//  decoder is satisfied. Every action carries a byte count and a millisecond
//  deadline after which the peer restores its dark command-mode client by
//  itself, so a test that loses the thread recovers by retrying.
//
//  These tests NEVER drive PE0, PE1 or PE2 from PORT and never call
//  miso_level(): the other board is on the far end of all four wires.
// ==============================================================================

const uint8_t no_payload[1] = {0};
spilink::Decoder dec;
bool link_quiet = false;              ///< suppress the failure dump while probing

/// The command channel's division, and the one place it is named.
constexpr SpiClock command_clock = SpiClock::div32;
static_assert(spi_division(command_clock) == spilink::command_division,
              "the command channel's SpiClock and spi_link.hpp must agree");

SpiMode mode_of(uint8_t m) {
    switch (m & 0x03u) {
        case 1: return SpiMode::mode1;
        case 2: return SpiMode::mode2;
        case 3: return SpiMode::mode3;
        default: return SpiMode::mode0;
    }
}

/// The inter-byte pause that lets the peer's polled loop turn a byte
/// around, and the pause between two select windows. Both are timed off
/// the CONSTEXPR 24 MHz clock, so a rebase to 12 MHz makes them twice as
/// long in real time - slower, never shorter.
void gap() { delay_us(clock, spilink::gap_us); }
void settle() { delay_us(clock, static_cast<uint32_t>(spilink::settle_ms) * 1000u); }

void cs_assert() { Ss::clear(); }
void cs_release() { Ss::set(); }

/// SPI0 as the command channel's host, and PE3 as its chip select.
bool link_command_mode() {
    Ss::invert(false);
    Ss::pullup(false);
    Ss::set();
    Ss::output();
    const bool ok = S0::init({.route = SpiRoute::alt1,
                              .role = SpiRole::host,
                              .mode = SpiMode::mode0,
                              .clock = command_clock,
                              .client_select_disable = true});
    dec.reset();
    return ok;
}

void put_link(uint8_t b) {
    (void)S0::transfer(b, 200'000u);
    gap();
}

void send_frame(spilink::Op op, const uint8_t* p, uint8_t len) {
    cs_assert();
    gap();
    spilink::write_frame(put_link, op, p, len);
    cs_release();
}

uint8_t raw_seen[16];
uint8_t raw_n = 0;

/// One answer window: dummies at the command pace until the decoder is
/// satisfied or the budget runs out. What was clocked is kept for the
/// failure dump - "the peer said nothing" and "the peer said something
/// this end could not parse" are different faults.
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    bool got = false;
    cs_assert();
    gap();
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        const auto v = S0::transfer(0x00, 200'000u);
        gap();
        if (!v) break;
        if (raw_n < 16) raw_seen[raw_n++] = *v;
        if (dec.feed(*v) == spilink::Decoder::Result::frame) {
            out = dec.frame();
            got = true;
            break;
        }
    }
    cs_release();
    return got;
}

bool command_once(spilink::Op op, const uint8_t* p, uint8_t len) {
    send_frame(op, p, len);
    settle();
    spilink::Frame f;
    if (!recv_frame(f)) return false;
    return f.op == spilink::Op::ack && f.len == 2 && f.data[0] == spilink::byte_of(op);
}

/// The recovery guarantee in action: three attempts, each separated by
/// longer than the peer's own answer-window bound, so a peer that was
/// serving into nothing is certainly dark again before the retry.
bool command(spilink::Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    uint8_t first_n = 0;
    uint8_t first_seen[16];
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len)) return true;
        if (k == 0) {
            first_n = raw_n;
            for (uint8_t i = 0; i < raw_n; ++i) first_seen[i] = raw_seen[i];
        }
        (void)link_command_mode();
        delay_us(clock, 400'000u);
    }
    if (link_quiet) return false;
    print(serial, "    LINK FAILURE op ", hex(spilink::byte_of(op)),
          ": the first answer window carried");
    if (first_n == 0) print(serial, " nothing");
    for (uint8_t i = 0; i < first_n; ++i) print(serial, " ", hex(first_seen[i]));
    print(serial, crlf, "      board B must be running `spi_peer`; its console '0' forces "
                        "the dark client back.", crlf);
    (void)link_command_mode();
    return false;
}

bool query(spilink::Op op, spilink::Frame& data) {
    if (!command(op)) return false;
    settle();
    return recv_frame(data);
}

bool peer_ident(spilink::Ident& d) {
    spilink::Frame f;
    if (!query(spilink::Op::ident, f) || f.op != spilink::Op::ident_data ||
        f.len != spilink::ident_size) {
        return false;
    }
    d = spilink::get_ident(f.data);
    return true;
}

/// Ask for the report of the last action. The caller has already given
/// the peer's bound time to expire.
bool peer_report(spilink::Report& r) {
    (void)link_command_mode();
    for (uint8_t k = 0; k < 4; ++k) {
        spilink::Frame f;
        if (query(spilink::Op::report, f) && f.op == spilink::Op::report_data &&
            f.len == spilink::report_size) {
            r = spilink::get_report(f.data);
            return true;
        }
        delay_us(clock, 60'000u);
    }
    return false;
}

bool peer_act(spilink::Op op, const spilink::Params& a) {
    uint8_t p[spilink::params_size];
    spilink::put_params(p, a);
    if (!command(op, p, spilink::params_size)) return false;
    settle();
    return true;
}

bool ensure_link() {
    link_quiet = true;
    (void)link_command_mode();
    for (uint8_t k = 0; k < 3; ++k) {
        if (command(spilink::Op::ping)) {
            link_quiet = false;
            return true;
        }
    }
    link_quiet = false;
    print(serial, "  the peer did not answer. Board B must be running `spi_peer`; its "
                  "console '0' forces it back to the dark client.", crlf);
    return false;
}

// ---- the exchange, the workhorse of this half ---------------------------------

struct Exchange {
    spilink::Cfg cfg{};                     ///< what the CLIENT becomes
    SpiMode host_mode = SpiMode::mode0;     ///< and what THIS end runs
    bool host_lsb = false;
    SpiClock rate = SpiClock::div32;
    uint16_t count = 8;
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint16_t ms = 250;
};

constexpr uint8_t max_exchange = 16;
uint8_t xrx[max_exchange];

/// Command an exchange, then be the host of it. The rx bytes land in
/// `xrx`; what they SHOULD be is verify_rx's business, because the
/// answer's alignment depends on the client's buffering regime.
bool do_exchange(const Exchange& e) {
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    a.flags = e.flags;
    // A bit-order mismatch is not a shrug: told about it, the client
    // checks the exact bit-reverse of what this end sent, and this end
    // checks the exact bit-reverse of what the client answered.
    if (e.host_lsb != (e.cfg.dord != 0)) a.flags |= spilink::flag_expect_reversed;
    if (!peer_act(spilink::Op::exchange, a)) return false;
    if (!S0::init({.route = SpiRoute::alt1,
                   .role = SpiRole::host,
                   .mode = e.host_mode,
                   .clock = e.rate,
                   .lsb_first = e.host_lsb,
                   .client_select_disable = true})) {
        return false;
    }
    Ss::set();
    Ss::output();
    settle();
    for (uint8_t i = 0; i < max_exchange; ++i) xrx[i] = 0xEE;
    spilink::Stream out(e.pattern, e.seed_a);
    cs_assert();
    gap();
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        const auto v = S0::transfer(out.next(), 200'000u);
        xrx[i] = v ? *v : 0xEE;
        gap();
    }
    cs_release();
    (void)link_command_mode();
    return true;
}

struct Verify {
    uint16_t mism = 0;
    uint8_t idx = 0xFF;
    uint8_t got = 0;
    uint8_t exp = 0;
    uint8_t dummy = 0;
    bool leads = false;    ///< buffer mode without BUFWR: a dummy comes first
};

/// What the host read back, against what the client's regime says it
/// should be. In buffer mode WITHOUT BUFWR the client's first write went
/// into the transmit buffer and the shift register's leftover leads, so
/// byte 0 is a DUMMY - measured, never assumed - and the answer stream
/// is one place late.
Verify verify_rx(const Exchange& e) {
    Verify v{};
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    v.leads = e.cfg.regime == spilink::regime_buffer;
    if (v.leads) v.dummy = xrx[0];
    for (uint16_t i = v.leads ? 1u : 0u; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(
            e.pattern, e.seed_b, v.leads ? static_cast<uint16_t>(i - 1) : i);
        if (reversed) exp = spilink::bit_reverse(exp);
        if (xrx[i] != exp) {
            if (v.mism == 0) {
                v.idx = static_cast<uint8_t>(i);
                v.got = xrx[i];
                v.exp = exp;
            }
            ++v.mism;
        }
    }
    return v;
}

/// An exchange plus its report, judged: exact in BOTH directions.
bool exchange_exact(const Exchange& e, Verify& v, spilink::Report& r) {
    const bool ran = do_exchange(e);
    const bool rep = ran && peer_report(r);
    v = verify_rx(e);
    return ran && rep && v.mism == 0 && r.mism == 0 && r.count == e.count;
}

/// Everything a failed exchange knows: the two mismatch counts, the
/// first offender each end saw, and the whole byte row this end read
/// against the row the client's regime says it should have read.
void dump_exchange(const Exchange& e, const Verify& v, const spilink::Report& r) {
    print(serial, "    host mism=", v.mism, " (first idx ", v.idx, " got ", hex(v.got),
          " exp ", hex(v.exp), "), client count=", r.count, " mism=", r.mism,
          " (first idx ", r.idx, " got ", hex(r.got), " exp ", hex(r.exp),
          ") flags=", hex(r.flags), crlf);
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    print(serial, "    read:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        print(serial, " ", hex(xrx[i]));
    }
    print(serial, crlf, "    want:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        if (v.leads && i == 0) {
            print(serial, " --");
            continue;
        }
        uint8_t exp = spilink::pattern_value(
            e.pattern, e.seed_b, v.leads ? static_cast<uint16_t>(i - 1) : i);
        if (reversed) exp = spilink::bit_reverse(exp);
        print(serial, " ", hex(exp));
    }
    print(serial, crlf);
}

// ---- k: the bring-up ------------------------------------------------------------

bool same_label(const char* a, const char* b) {
    for (uint8_t i = 0; i < 8; ++i) {
        if (a[i] != b[i]) return false;
        if (a[i] == 0) return true;
    }
    return true;
}

void tk_bringup() {
    print(serial, "k two boards: the command channel over SPI0 ALT1, the peer's identity, "
                  "the nak and the recovery", crlf);
    quiesce();
    const bool up = ensure_link();
    verdict("the peer answers a ping", up);
    if (!up) return;

    spilink::Ident d{};
    const bool got = peer_ident(d);
    char label[9] = {};
    for (uint8_t i = 0; i < 8; ++i) label[i] = d.label[i];
    print(serial, "  peer: label='", label, "' xtal=", d.xtal, " sanity=", hex(d.sanity),
          " fw=", hex(d.version), crlf);
    verdict("ident comes back", got);
    verdict("the sanity byte names spi_peer", got && d.sanity == spilink::ident_sanity);
    verdict("the peer is board brio-b", got && same_label(label, "brio-b"));
    verdict("the peer's 24 MHz crystal started", got && d.xtal == 1);

    // A frame with an op the protocol knows and a checksum that does
    // not match: nak'ed by name, and the channel usable straight after.
    const uint8_t op_byte = spilink::byte_of(spilink::Op::ping);
    const uint8_t good = spilink::checksum(op_byte, 0, no_payload);
    cs_assert();
    gap();
    put_link(spilink::magic);
    put_link(op_byte);
    put_link(0);
    put_link(static_cast<uint8_t>(good ^ 0xFFu));
    cs_release();
    settle();
    spilink::Frame nf;
    const bool nak = recv_frame(nf) && nf.op == spilink::Op::nak && nf.len == 2 &&
                     nf.data[0] == op_byte;
    verdict("a frame with a broken checksum is nak'ed by name", nak);
    verdict("and the channel still works right afterwards", command(spilink::Op::ping));

    // A frame that stops half way: the peer's reassembly must time out
    // on the quiet wire instead of eating the next command.
    cs_assert();
    gap();
    put_link(spilink::magic);
    put_link(spilink::byte_of(spilink::Op::ident));
    cs_release();
    delay_us(clock, 200'000u);
    verdict("a truncated frame does not eat the next command",
            command(spilink::Op::ping));
    quiesce();
}

// ---- l: the client matrix -------------------------------------------------------

const char* const regime_name[3] = {"normal", "buffer", "buffer+BUFWR"};
const char* const combo_name[8] = {"mode 0 MSb, ", "mode 0 LSb, ", "mode 1 MSb, ",
                                   "mode 1 LSb, ", "mode 2 MSb, ", "mode 2 LSb, ",
                                   "mode 3 MSb, ", "mode 3 LSb, "};

void tl_matrix() {
    print(serial, "l the client matrix: 4 transfer modes x 2 bit orders x 3 buffering "
                  "regimes at CLK_PER/32, exact BOTH ways", crlf);
    quiesce();
    if (!ensure_link()) {
        verdict("the peer answers a ping", false);
        return;
    }
    uint8_t dummy_seen = 0;
    uint8_t dummy_n = 0;
    uint8_t dummy_nonzero = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        for (uint8_t dord = 0; dord < 2; ++dord) {
            for (uint8_t reg = 0; reg < 3; ++reg) {
                Exchange e{};
                e.cfg.mode = m;
                e.cfg.dord = dord;
                e.cfg.regime = reg;
                e.host_mode = mode_of(m);
                e.host_lsb = dord != 0;
                e.count = 8;
                e.seed_a = static_cast<uint8_t>(0x11 + m * 16 + dord * 5 + reg);
                e.seed_b = static_cast<uint8_t>(0x83 + m * 16 + dord * 5 + reg);
                Verify v{};
                spilink::Report r{};
                const bool ok = exchange_exact(e, v, r);
                if (v.leads) {
                    dummy_seen = v.dummy;
                    ++dummy_n;
                    if (v.dummy != 0) ++dummy_nonzero;
                }
                if (!ok) dump_exchange(e, v, r);
                verdict(combo_name[m * 2 + dord], regime_name[reg], ok);
            }
        }
    }
    print(serial, "  FINDING: in buffer mode WITHOUT BUFWR the answer stream is led by a "
                  "DUMMY byte - the shift register's leftover. Over the ", dummy_n,
          " combinations that used the regime, ", dummy_nonzero,
          " were non-zero; the last one measured was ", hex(dummy_seen),
          " (a fresh init leaves the shifter clear)", crlf);
    verdict("the leading dummy is the shift register a fresh init cleared",
            dummy_n == 8 && dummy_nonzero == 0);
    quiesce();
}

// ---- m: the rates, and the client's ceiling -------------------------------------

struct RateCase {
    SpiClock c;
    const char* name;
};

void tm_rates() {
    print(serial, "m the rates against a real client: inside the CLK_PER/6 ceiling, and "
                  "above it", crlf);
    quiesce();
    if (!ensure_link()) {
        verdict("the peer answers a ping", false);
        return;
    }
    static const RateCase inside[5] = {{SpiClock::div8, "CLK_PER/8 (3 MHz)"},
                                       {SpiClock::div16, "CLK_PER/16 (1.5 MHz)"},
                                       {SpiClock::div32, "CLK_PER/32 (750 kHz)"},
                                       {SpiClock::div64, "CLK_PER/64 (375 kHz)"},
                                       {SpiClock::div128, "CLK_PER/128 (187.5 kHz)"}};
    for (const RateCase& rc : inside) {
        Exchange e{};
        e.rate = rc.c;
        e.count = 12;
        e.seed_a = 0x21;
        e.seed_b = 0x9B;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        if (!ok) dump_exchange(e, v, r);
        verdict("exact at ", rc.name, ok);
    }

    // The client's ceiling is CLK_PER/6 = 4 MHz on a 24 MHz peer
    // (errata clarification 3.7.3). Above it the bytes must be
    // corrupted, and this has to SEE the corruption: a test that cannot
    // observe its own failure signal passes vacuously.
    static const RateCase above[2] = {{SpiClock::div4, "CLK_PER/4 (6 MHz)"},
                                      {SpiClock::div2, "CLK_PER/2 (12 MHz)"}};
    for (const RateCase& rc : above) {
        Exchange e{};
        e.rate = rc.c;
        e.count = 12;
        e.seed_a = 0x35;
        e.seed_b = 0xC7;
        Verify v{};
        spilink::Report r{};
        const bool ran = do_exchange(e);
        const bool rep = ran && peer_report(r);
        v = verify_rx(e);
        print(serial, "  FINDING: ", rc.name, " is above the client's CLK_PER/6 ceiling: "
              "host mism=", v.mism, "/", e.count, " (first idx ", v.idx, " got ",
              hex(v.got), " exp ", hex(v.exp), "), client count=", r.count, " mism=",
              r.mism, "/", e.count, crlf);
        verdict("the exchange is positively corrupted at ", rc.name,
                ran && rep && (v.mism > 0 || r.mism > 0));
    }

    Exchange back{};
    back.count = 12;
    back.seed_a = 0x4D;
    back.seed_b = 0xE1;
    Verify v{};
    spilink::Report r{};
    const bool ok = exchange_exact(back, v, r);
    if (!ok) dump_exchange(back, v, r);
    verdict("CLK_PER/32 is exact again after the over-speed runs", ok);
    quiesce();
}

// ---- n: deliberate mismatches ----------------------------------------------------

void tn_mismatch() {
    print(serial, "n deliberate mismatches: CPOL alone, CPHA alone, and the bit order as "
                  "an EXACT two-way reversal", crlf);
    quiesce();
    if (!ensure_link()) {
        verdict("the peer answers a ping", false);
        return;
    }

    // CPOL only: this end mode 0, the client mode 2 - same sampling
    // edge in each end's own frame of reference, opposite idle level.
    Exchange cpol{};
    cpol.cfg.mode = 2;
    cpol.host_mode = SpiMode::mode0;
    cpol.count = 12;
    cpol.seed_a = 0x19;
    cpol.seed_b = 0x71;
    Verify v1{};
    spilink::Report r1{};
    bool ran = do_exchange(cpol);
    bool rep = ran && peer_report(r1);
    v1 = verify_rx(cpol);
    print(serial, "  FINDING: host mode 0 against a mode-2 client (CPOL apart): host "
                  "mism=", v1.mism, "/", cpol.count, " first got ", hex(v1.got), " exp ",
          hex(v1.exp), ", client mism=", r1.mism, "/", r1.count, crlf);
    verdict("a CPOL mismatch positively corrupts the data",
            ran && rep && (v1.mism > 0 || r1.mism > 0));

    // CPHA only: mode 0 against mode 1.
    Exchange cpha{};
    cpha.cfg.mode = 1;
    cpha.host_mode = SpiMode::mode0;
    cpha.count = 12;
    cpha.seed_a = 0x2B;
    cpha.seed_b = 0x8D;
    Verify v2{};
    spilink::Report r2{};
    ran = do_exchange(cpha);
    rep = ran && peer_report(r2);
    v2 = verify_rx(cpha);
    print(serial, "  FINDING: host mode 0 against a mode-1 client (CPHA apart): host "
                  "mism=", v2.mism, "/", cpha.count, " first got ", hex(v2.got), " exp ",
          hex(v2.exp), ", client mism=", r2.mism, "/", r2.count, crlf);
    verdict("a CPHA mismatch positively corrupts the data",
            ran && rep && (v2.mism > 0 || r2.mism > 0));

    // DORD: this is the one mismatch with an EXACT answer - each end
    // reads the other's bytes bit-reversed, and both check it.
    Exchange dord{};
    dord.cfg.dord = 1;
    dord.host_lsb = false;
    dord.count = 12;
    dord.seed_a = 0x3D;
    dord.seed_b = 0xA9;
    Verify v3{};
    spilink::Report r3{};
    ran = do_exchange(dord);
    rep = ran && peer_report(r3);
    v3 = verify_rx(dord);
    print(serial, "  a MSb-first host against an LSb-first client: host mism=", v3.mism,
          "/", dord.count, ", client mism=", r3.mism, "/", r3.count,
          " - both sides checked the exact bit-reverse", crlf);
    if (v3.mism != 0 || r3.mism != 0) dump_exchange(dord, v3, r3);
    verdict("a bit-order mismatch is an exact reversal, host side",
            ran && rep && v3.mism == 0);
    verdict("and client side", ran && rep && r3.mism == 0 && r3.count == dord.count);

    Exchange back{};
    back.count = 8;
    back.seed_a = 0x5F;
    back.seed_b = 0xB7;
    Verify v4{};
    spilink::Report r4{};
    const bool ok = exchange_exact(back, v4, r4);
    if (!ok) dump_exchange(back, v4, r4);
    verdict("a matched exchange is exact again", ok);
    quiesce();
}

// ---- o: the select wire dropped mid-byte -----------------------------------------

void to_ss_midbyte() {
    print(serial, "o the select wire raised MID-BYTE at CLK_PER/128: what each end keeps",
          crlf);
    quiesce();
    if (!ensure_link()) {
        verdict("the peer answers a ping", false);
        return;
    }
    spilink::Params a{};
    a.cfg.regime = spilink::regime_normal;
    a.count = 8;
    a.ms = 300;
    a.seed_a = 0x61;
    a.seed_b = 0xB3;
    a.pattern = spilink::pattern_counting;
    if (!peer_act(spilink::Op::exchange, a)) {
        verdict("the peer took the exchange", false);
        return;
    }
    if (!S0::init({.route = SpiRoute::alt1,
                   .role = SpiRole::host,
                   .mode = SpiMode::mode0,
                   .clock = SpiClock::div128,
                   .client_select_disable = true})) {
        verdict("host init at CLK_PER/128", false);
        return;
    }
    Ss::set();
    Ss::output();
    settle();
    uint8_t rx[3] = {};
    cs_assert();
    gap();
    for (uint8_t i = 0; i < 3; ++i) {
        const auto v = S0::transfer(spilink::pattern_value(a.pattern, a.seed_a, i), 200'000u);
        rx[i] = v ? *v : 0xEE;
        gap();
    }
    // A byte lasts 42.7 us at CLK_PER/128: start the fourth and take the
    // select wire away in the middle of it.
    S0::write(spilink::pattern_value(a.pattern, a.seed_a, 3));
    delay_us(clock, 21);
    cs_release();
    // This end is the clock: its own byte finishes whatever SS does.
    for (uint32_t i = 0; i < 200'000u && !S0::if_flag(); ++i) {}
    const bool host_finished = S0::if_flag();
    const uint8_t host_last = S0::read();

    delay_us(clock, 400'000u);            // let the peer's deadline expire
    spilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  FINDING: 3 clean bytes then SS raised mid-byte 4. The host read ",
          hex(rx[0]), " ", hex(rx[1]), " ", hex(rx[2]), " then ", hex(host_last),
          " where the client's loaded answer was ",
          hex(spilink::pattern_value(a.pattern, a.seed_b, 3)),
          " - the pad stops driving at the SS edge and the rest of the byte is the "
          "released line; the client counted ", r.count, " byte(s), mism=", r.mism, crlf);
    verdict("the host's own byte completes whatever the select wire does", host_finished);
    verdict("the client kept the three complete bytes", rep && r.count == 3);
    verdict("and they are exact", rep && r.mism == 0);
    verdict("the interrupted byte never reached the client",
            rep && r.count == 3 && (r.flags & spilink::report_timed_out) != 0);

    Exchange back{};
    back.count = 8;
    back.seed_a = 0x77;
    back.seed_b = 0xC3;
    Verify v{};
    spilink::Report r2{};
    const bool ok = exchange_exact(back, v, r2);
    if (!ok) dump_exchange(back, v, r2);
    verdict("a clean exchange proves both ends recovered", ok);
    quiesce();
}

// ---- p: what a client that never drains keeps -------------------------------------

/// Stream `count` bytes with NO inter-byte gap into a client that has
/// been told not to drain. Returns the report.
bool sink_burst(uint8_t regime, uint8_t seed, uint16_t count, uint8_t flags,
                spilink::Report& r) {
    spilink::Params a{};
    a.cfg.regime = regime;
    a.count = count;
    a.ms = 250;
    a.seed_a = seed;
    a.flags = flags;
    a.pattern = spilink::pattern_counting;
    if (!peer_act(spilink::Op::sink_slow, a)) return false;
    if (!S0::init({.route = SpiRoute::alt1,
                   .role = SpiRole::host,
                   .mode = SpiMode::mode0,
                   .clock = command_clock,
                   .client_select_disable = true})) {
        return false;
    }
    Ss::set();
    Ss::output();
    settle();
    cs_assert();
    gap();
    for (uint16_t i = 0; i < count; ++i) {
        (void)S0::transfer(static_cast<uint8_t>(seed + i), 200'000u);   // gapless
    }
    cs_release();
    delay_us(clock, 350'000u);
    return peer_report(r);
}

void tp_loss() {
    print(serial, "p a client that never drains: the normal-mode survivor, buffer mode's "
                  "BUFOVF, and the client's write collision", crlf);
    quiesce();
    if (!ensure_link()) {
        verdict("the peer answers a ping", false);
        return;
    }

    spilink::Report r{};
    bool rep = sink_burst(spilink::regime_normal, 0x40, 8, 0, r);
    print(serial, "  FINDING: normal mode, 8 gapless bytes 0x40..0x47, DATA never read: "
                  "the client retained ", r.count, " byte(s) = ", hex(r.aux0), " ",
          hex(r.aux1), " ", hex(r.aux2), " ", hex(r.aux3), "; INTFLAGS ", hex(r.sum),
          " ever raised, ", hex(r.got), " before the drain and ", hex(r.exp), " after",
          crlf);
    verdict("a normal-mode client keeps exactly one of the eight", rep && r.count == 1);
    verdict("and it is the LAST byte: a new one overwrites the unread one",
            rep && r.count >= 1 && r.aux0 == 0x47);

    rep = sink_burst(spilink::regime_buffer, 0x50, 8, 0, r);
    const bool ovf_idle_tx = rep && (r.flags & spilink::report_bufovf) != 0;
    print(serial, "  FINDING: buffer mode, 8 gapless bytes 0x50..0x57, DATA never read and "
                  "the transmitter IDLE: the client retained ", r.count, " byte(s) = ",
          hex(r.aux0), " ", hex(r.aux1), " ", hex(r.aux2), " ", hex(r.aux3),
          "; INTFLAGS ", hex(r.sum), " ever raised, ", hex(r.got),
          " before the drain and ", hex(r.exp), " after, BUFOVF=", ovf_idle_tx, crlf);
    verdict("a buffer-mode client keeps three: the two-deep FIFO plus the shifter",
            rep && r.count == 3);
    verdict("the FIFO holds the FIRST two bytes of the burst",
            rep && r.count >= 2 && r.aux0 == 0x50 && r.aux1 == 0x51);
    verdict("and the shifter holds the LAST", rep && r.count >= 3 && r.aux2 == 0x57);

    // 28.5.5 puts a condition on the flag: "If there is no transmit
    // data, the Buffer Overflow will not be set before the start of a
    // new serial transfer." The same flood with the client's
    // TRANSMITTER kept fed is the experiment that isolates it.
    spilink::Report rf{};
    const bool repf = sink_burst(spilink::regime_buffer, 0x60, 8, spilink::flag_feed_tx, rf);
    const bool ovf_fed_tx = repf && (rf.flags & spilink::report_bufovf) != 0;
    print(serial, "  FINDING: the same flood with the client's TRANSMITTER FED: retained ",
          rf.count, " byte(s) = ", hex(rf.aux0), " ", hex(rf.aux1), " ", hex(rf.aux2),
          " ", hex(rf.aux3), "; INTFLAGS ", hex(rf.sum), " ever raised, ", hex(rf.got),
          " before the drain, BUFOVF=", ovf_fed_tx, crlf);
    verdict("BUFOVF marks the loss when the client has transmit data", ovf_fed_tx);
    verdict("and stays clear when its transmitter is idle (28.5.5)", !ovf_idle_tx);

    // The client's own write collision, and the BOUNDARY it turns on. A
    // write to DATA is a collision only while the shifter is running; in
    // the gap between two bytes it is an ordinary write that replaces
    // the answer already loaded. Both sides are walked in one burst of
    // constant streams (host 0x5A, client 0xA5) with the client writing
    // a marker over its own answer at each - so which write was obeyed
    // and which ignored is legible straight off the wire.
    Exchange e{};
    e.cfg.regime = spilink::regime_normal;
    e.count = 8;
    e.pattern = spilink::pattern_fixed;
    e.seed_a = 0x5A;
    e.seed_b = 0xA5;
    e.flags = spilink::flag_wrcol;
    spilink::Report rw{};
    const bool ran = do_exchange(e);
    const bool repw = ran && peer_report(rw);
    const uint8_t gap_pos = spilink::wrcol_gap_at + 1;
    const uint8_t hold_pos = spilink::wrcol_hold_at + 1;
    print(serial, "  FINDING: the client wrote ", hex(spilink::wrcol_marker),
          " over its own answer twice - in the GAP after byte ", spilink::wrcol_gap_at,
          " and INSIDE the transfer after byte ", spilink::wrcol_hold_at, "; WRCOL=",
          (rw.flags & spilink::report_wrcol) != 0, ", the host read");
    for (uint8_t i = 0; i < e.count; ++i) print(serial, " ", hex(xrx[i]));
    print(serial, crlf);
    bool others_ok = ran && repw;
    for (uint8_t i = 0; i < e.count; ++i) {
        if (i == gap_pos) continue;
        others_ok = others_ok && xrx[i] == e.seed_b;
    }
    verdict("a client's write INSIDE a transfer sets WRCOL",
            repw && (rw.flags & spilink::report_wrcol) != 0);
    verdict("and is ignored: the answer loaded before it goes out intact",
            ran && repw && xrx[hold_pos] == e.seed_b);
    verdict("a write in the inter-byte GAP is no collision at all: it is obeyed",
            ran && repw && xrx[gap_pos] == spilink::wrcol_marker);
    verdict("nothing else in the burst is disturbed", others_ok);
    verdict("the client received the whole burst either way",
            repw && rw.count == e.count && rw.mism == 0);

    // The other half of the same coin: a client that MISSES its load
    // entirely. In normal mode the shift register is shared between the
    // two directions, so what goes out next is whatever came in last -
    // with two constant streams (host 0x5A, client 0xA5) that is
    // unmistakable, and the answer resumes one place late afterwards.
    Exchange sk{};
    sk.cfg.regime = spilink::regime_normal;
    sk.count = 8;
    sk.pattern = spilink::pattern_fixed;
    sk.seed_a = 0x5A;
    sk.seed_b = 0xA5;
    sk.flags = spilink::flag_skip_write;
    spilink::Report rs{};
    const bool sran = do_exchange(sk);
    const bool srep = sran && peer_report(rs);
    print(serial, "  FINDING: the client answered 0xA5 to every 0x5A but skipped its load "
                  "after byte ", spilink::skip_at, "; the host read");
    for (uint8_t i = 0; i < sk.count; ++i) print(serial, " ", hex(xrx[i]));
    print(serial, crlf);
    bool echoed = sran && srep && xrx[spilink::skip_at + 1] == sk.seed_a;
    bool rest_ok = sran && srep;
    for (uint8_t i = 0; i < sk.count; ++i) {
        if (i == spilink::skip_at + 1) continue;
        rest_ok = rest_ok && xrx[i] == sk.seed_b;
    }
    verdict("a client that misses its load sends back the byte it just received",
            echoed);
    verdict("and every other byte of that burst is the client's own answer", rest_ok);
    verdict("the client still received the whole burst",
            srep && rs.count == sk.count && rs.mism == 0);
    quiesce();
}

// ---- q: a REAL host demotion -------------------------------------------------------

void tq_demotion() {
    print(serial, "q a REAL host demotion: board B drives the shared select wire low",
          crlf);
    quiesce();
    if (!ensure_link()) {
        verdict("the peer answers a ping", false);
        return;
    }
    spilink::Params a{};
    a.aux8 = 60;          // milliseconds after the ack window before it takes the wire
    a.aux16 = 20'000;     // and microseconds to hold it low - long enough to be caught
    a.ms = 250;           // still held when this end reacts
    if (!peer_act(spilink::Op::ss_pulse, a)) {
        verdict("the peer took ss_pulse", false);
        return;
    }
    // This end becomes a host that WATCHES its SS pin: SSD = 0, the pin
    // an input held up by its own pull-up, no INVEN anywhere. The low
    // that demotes it comes from the other board - which is what the
    // single-board half could only fake.
    Ss::invert(false);
    Ss::input();
    const bool init_ok = S0::init({.route = SpiRoute::alt1,
                                   .role = SpiRole::host,
                                   .mode = SpiMode::mode0,
                                   .clock = command_clock,
                                   .client_select_disable = false});
    verdict("host with SSD = 0 on the shared select wire", init_ok);
    verdict("it starts out a host", init_ok && S0::is_host() && !S0::demoted());

    uint8_t flags_at = 0;
    bool demoted = false;
    for (uint16_t i = 0; i < 2000 && !demoted; ++i) {
        if (S0::demoted()) {
            demoted = true;
            flags_at = S0::flags();
            break;
        }
        delay_us(clock, 200);
    }
    print(serial, "  FINDING: the peer's low on PE3 ", demoted ? "DEMOTED" : "did NOT demote",
          " this host; INTFLAGS at the moment = ", hex(flags_at), " (IF=",
          (flags_at & SPI_IF_bm) != 0, "), MASTER now ", S0::is_host(), crlf);
    verdict("a real second driver on SS clears MASTER", demoted);
    verdict("the demotion raises IF (normal layout)", (flags_at & SPI_IF_bm) != 0);
    verdict("MASTER does not come back by itself", !S0::is_host());

    // Re-arming is not a thing an application can do whenever it likes:
    // the demotion follows the LEVEL on the pin, so a host that writes
    // MASTER back while the other one is still holding the wire is
    // simply demoted again. This end reacts within a fraction of the
    // peer's hold, so the wire really is still low here.
    const bool still_low = !Ss::read();
    S0::restore_host();
    const bool armed_while_low = S0::is_host();
    uint16_t waited = 0;
    for (; waited < 400 && !Ss::read(); ++waited) delay_us(clock, 1000);
    const bool released = Ss::read();
    print(serial, "  FINDING: with the peer still holding SS low, restore_host left MASTER ",
          armed_while_low, "; the wire was released ", waited, " ms later", crlf);
    verdict("this end reacted while the wire was still held", still_low);
    verdict("restore_host does not stick while the wire is still held low",
            still_low && !armed_while_low);
    verdict("the peer released the wire on its own bound", released);
    S0::restore_host();
    verdict("restore_host re-arms Host mode once the wire is free",
            S0::is_host() && !S0::demoted());
    verdict("restore_host cleared the flag", !S0::if_flag());

    (void)link_command_mode();
    delay_us(clock, 150'000u);
    Exchange back{};
    back.count = 8;
    back.seed_a = 0x0F;
    back.seed_b = 0xF0;
    Verify v{};
    spilink::Report r{};
    const bool ok = exchange_exact(back, v, r);
    if (!ok) dump_exchange(back, v, r);
    verdict("an exchange after the recovery is exact", ok);
    quiesce();
}

// ---- r: the USART's own Host SPI against a real client -----------------------------

using Mspi = MspiHost<4, UsartRoute::def>;

struct MspiCase {
    uint8_t mode;         ///< bit 0 = UCPHA, bit 1 = the SCK inversion
    uint8_t dord;         ///< UDORD
    const char* name;
};

bool run_mspi_case(const MspiCase& mc, uint32_t sck_hz, Exchange& shape, Verify& v,
                   spilink::Report& r) {
    constexpr uint16_t lead_ms = 25;
    constexpr uint16_t count = 12;
    const uint8_t seed_a = static_cast<uint8_t>(0x91 + mc.mode * 4 + mc.dord);
    const uint8_t seed_b = static_cast<uint8_t>(0x3B + mc.mode * 4 + mc.dord);

    spilink::Params a{};
    a.cfg.mode = mc.mode;
    a.cfg.dord = mc.dord;
    a.cfg.regime = spilink::regime_normal;
    a.count = count;
    a.ms = 800;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = lead_ms;
    if (!peer_act(spilink::Op::mspi, a)) return false;

    // Hand PORTE over: the SPI lets go of MOSI/SCK, the select wire goes
    // back to an input (the peer holds it up and selects ITSELF with
    // INVEN - Host SPI mode has no client select), and USART4 takes
    // TXD PE0, RXD PE1, XCK PE2.
    S0::release();
    Ss::invert(false);
    Ss::input();
    if (!Mspi::init(clock, sck_hz,
                    {.lsb_first = mc.dord != 0,
                     .sample_trailing = (mc.mode & 0x01u) != 0,
                     .invert_sck = (mc.mode & 0x02u) != 0})) {
        (void)link_command_mode();
        return false;
    }
    delay_us(clock, (static_cast<uint32_t>(lead_ms) + 10u) * 1000u);
    for (uint8_t i = 0; i < max_exchange; ++i) xrx[i] = 0xEE;
    for (uint16_t i = 0; i < count && i < max_exchange; ++i) {
        const auto b = Mspi::transfer(spilink::pattern_value(a.pattern, seed_a, i), 200'000u);
        xrx[i] = b ? *b : 0xEE;
        delay_us(clock, spilink::gap_us);
    }
    delay_us(clock, 10'000u);             // the peer clears its INVEN at the count bound
    Mspi::release();
    (void)link_command_mode();

    shape = Exchange{};                   // only to describe the expected stream
    shape.cfg.dord = mc.dord;
    shape.host_lsb = mc.dord != 0;
    shape.cfg.regime = spilink::regime_normal;
    shape.count = count;
    shape.seed_a = seed_a;
    shape.seed_b = seed_b;
    shape.pattern = a.pattern;
    v = verify_rx(shape);
    return peer_report(r);
}

void tr_mspi() {
    print(serial, "r the USART's own Host SPI (MspiHost on USART4) against a real SPI "
                  "client - usart.md's deferred electrical check", crlf);
    quiesce();
    if (!ensure_link()) {
        verdict("the peer answers a ping", false);
        return;
    }
    static const MspiCase cases[5] = {
        {0, 0, "UCPHA=0 MSb (SPI mode 0)"},   {1, 0, "UCPHA=1 MSb (SPI mode 1)"},
        {2, 0, "inverted SCK MSb (mode 2)"},  {3, 0, "inverted SCK UCPHA=1 (mode 3)"},
        {0, 1, "UCPHA=0 LSb (UDORD)"},
    };
    for (const MspiCase& mc : cases) {
        Verify v{};
        spilink::Report r{};
        Exchange shape{};
        const bool rep = run_mspi_case(mc, 750'000u, shape, v, r);
        const bool ok = rep && v.mism == 0 && r.mism == 0 && r.count == 12;
        if (!ok) dump_exchange(shape, v, r);
        verdict("Host SPI exact on the wire: ", mc.name, ok);
    }
    Exchange back{};
    back.count = 8;
    back.seed_a = 0x6B;
    back.seed_b = 0xD5;
    Verify v{};
    spilink::Report r{};
    const bool ok = exchange_exact(back, v, r);
    if (!ok) dump_exchange(back, v, r);
    verdict("SPI0 takes PORTE back and the bus is exact again", ok);
    quiesce();
}

// ---- s: a rebase under two-board traffic --------------------------------------------

bool rebase_step(uint32_t hz, const char* what) {
    Exchange e{};
    e.count = 12;
    e.seed_a = static_cast<uint8_t>(hz / 1'000'000u);
    e.seed_b = static_cast<uint8_t>(0xC0 + (hz / 1'000'000u));
    Verify v{};
    spilink::Report r{};
    const bool ok = exchange_exact(e, v, r);
    if (!ok) dump_exchange(e, v, r);
    verdict("the command channel and the client are exact ", what, ok);

    ChSck::source(S0::SckEvent{});
    SckMeter::init(DynClock{}, ChSck{}, TcbClock::div1);
    const uint16_t t = measure_now();
    T0::enable_capt_interrupt(false);
    T0::disable();
    ChSck::off();
    print(serial, "  ", what, ": CLK_PER = ", hz, " Hz, SCK period ", t,
          " CLK_PER ticks = ", t ? hz / t : 0, " Hz", crlf);
    const bool tracked = t == spilink::command_division;
    verdict("the division still reads CLK_PER/32 ", what, tracked);
    (void)link_command_mode();
    return ok && tracked;
}

void ts_rebase() {
    print(serial, "s 24 -> 12 -> 24 MHz under two-board traffic: the command channel is a "
                  "DIVISION of CLK_PER and follows it", crlf);
    quiesce();
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());
    if (!ensure_link()) {
        verdict("the peer answers a ping", false);
        return;
    }
    (void)rebase_step(24'000'000u, "at 24 MHz");
    verdict("switch to 12 MHz", DynClock::set(12'000'000u));
    (void)rebase_step(12'000'000u, "at 12 MHz");
    verdict("back to 24 MHz", DynClock::set(24'000'000u));
    (void)rebase_step(24'000'000u, "back at 24 MHz");
    quiesce();
}

// ---- the menu ----------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_routes}, {'b', tb_rates}, {'c', tc_data}, {'d', td_modes},
    {'e', te_wrcol}, {'f', tf_buffer}, {'g', tg_demotion}, {'h', th_interrupts},
    {'i', ti_rebase}, {'j', tj_engine},
    {'k', tk_bringup}, {'l', tl_matrix}, {'m', tm_rates}, {'n', tn_mismatch},
    {'o', to_ss_midbyte}, {'p', tp_loss}, {'q', tq_demotion}, {'r', tr_mspi},
    {'s', ts_rebase},
};
constexpr char single_board[] = "abcdefghij";
constexpr char two_board[] = "klmnopqrs";

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
                  "h isr bodies | i rebase | j engine    -> z = all of a..j", crlf);
    print(serial, "  two boards: k bring-up | l client matrix | m rates and the client "
                  "ceiling | n mismatches | o SS mid-byte | p undrained client | "
                  "q real demotion | r Host SPI | s rebase    -> y = all of k..s", crlf);
    print(serial, "  NO WIRES of its own: the desk's PORTE link (A.PEn - B.PEn, n = 0..3) "
                  "is SPI0 ALT1 straight across. Board B runs `spi_peer`, which stays DARK "
                  "- it drives MISO only for one answer window - so z passes with the peer "
                  "attached", crlf);
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
        if (isr_idx < 12) { isr_bytes[isr_idx] = d; isr_flag_log[isr_idx] = isr_flags; isr_idx = isr_idx + 1; }
    } else {
        const auto r = S0::take_normal();   // INTFLAGS then DATA: the clear sequence
        isr_flags = r.flags;
        isr_data = r.data;
    }
    isr_count = isr_count + 1;
}

ISR(TCB0_INT_vect) {
    const uint16_t t = SckMeter::period_ticks();
    last_ticks = t;
    captures = captures + 1;
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
        else if (c == 'y' || c == 'Y') { run_set(two_board); }
        else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
