// spi_peer - a scriptable SPI CLIENT for the other end of a two-board
// bus test: the far board is the bus host and drives this one IN BAND
// over the very wires under test, every command travelling as a
// checksummed frame in the same traffic.
//
// The wire format is spi_link.hpp, included by relative path: one file
// is the single source of truth for every board that speaks it,
// whatever its architecture. What this silicon makes of it:
//
//  - THE RUN IS ONE AHEAD, ON THE BOUNDARY. A byte written to DATA
//    reaches the shifter three SCK cycles later, and those cycles elapse
//    only while SCK runs (32.6.2.6.2), so an answer written in the
//    host's inter-byte gap arrives one character late. Instead: the
//    FIRST answer goes in through CTRLB.PLOADEN while SS is still high,
//    the SECOND parks in DATA at once, and every character boundary
//    (DRE) loads the next - inside the first five SCK cycles of the
//    character in flight. That is the driver's polled run
//    (SpiClient::answer and ::exchange), placed in SRAM here; serve()
//    and run_exchange() are each one call of it.
//
//  - THE POLLED RUN IS THE ONLY SERVE. This stratum drives the DMAC for
//    one user alone, the Uart's transmitter, because erratum 1.10.4
//    (DS80000740S) corrupts the live write-back descriptor under
//    concurrent channel triggers (docs/samc21/dmac.md). So
//    spilink::spare_polled_pump is accepted and changes nothing (bit
//    0x04 of aux2, the engined serve, is never set here);
//    spilink::spare_answer_only serves an exchange's answers on the
//    answer run, which reads nothing.
//
//  - THE BUFFERING REGIMES COLLAPSE. This client is always buffered two
//    deep and has no knob for anything else; PLOADEN plays for the
//    first character the role a write-before-clock flag plays where one
//    exists. spilink::Cfg::regime is therefore MAPPED, not translated:
//    regime_buffer_wait runs the preloaded one-ahead pump and delivers
//    the exact rx[i] = P_B(i) alignment that regime promises; the other
//    two run without preload, which here means the shifter's leftover
//    leads (spi_link.hpp's table, its second row) - what a host that
//    commands them reads is test_avr_spi l's measurement.
//
//  - THE PROTOCOL CARRIES OPTIONS THIS SILICON HAS NOT GOT. flag_wrcol
//    and flag_feed_tx name conditions chapter 32 does not have and are
//    ignored; Op::mspi asks for a USART in host-SPI mode and is answered
//    report_cfg_failed - this SERCOM is not a USART while it is an SPI.
//    Everything else of the repertoire is served: ping/ident/report,
//    exchange, sink_slow, ss_pulse, host_burst.
//
// THE DARK LISTENER discipline: the command-mode client never drives
// MISO (drive_output = false), the
// answer line wakes only for one answer window after a frame that
// CHECKED OUT, an unknown op is dropped in silence, and a bad checksum
// is nak'ed only while ENGAGED. The select pad carries NO pull while
// the SERCOM owns it: under PMUXEN, PINCFG.PULLEN on a SERCOM input
// enables a pull-DOWN only, whatever OUT says (31.5.1 states it for the
// USART; measured here on the SPI client's SS pad - the line read low
// within a millisecond of PULLEN with OUT set, and held high for 300 ms
// with no pull). A pull-down would SELECT this client whenever the host
// leaves its end an input, so the host's end holds the line: an AVR
// host watching its SS (SSD = 0) pulls it up itself, every other host
// here drives its chip select.
//
// Link: SERCOM1 function C, to the host board -
//   PA16 = PAD[0]  MOSI (DI here: the client's DOPO row is 0x1)
//   PA17 = PAD[1]  SS (input, no pull: the host's end holds it)
//   PA18 = PAD[2]  MISO (DO, driven only to answer)
//   PA19 = PAD[3]  SCK
// straight through to a second SAM C21's same pads, or to an
// AVR128DA48's SPI0 ALT1 as PE0, PE3, PE1 and PE2 - the rows
// test_samc_spi's header explains. The same four pads carry this board
// as the bus HOST for Op::host_burst - DOPO row 0x3, chip select by
// PORT on PA17.
//
// Console: SERCOM5 PB30/PB31 at 115200, observability only.
//   ? help | i status and counters | 0 back to the dark client | 3 trace
//
// The ident label is the die serial's first word in hex - this family's
// identity is factory-programmed (no USERROW label to read); ident.xtal
// reports whether the board's 24 MHz crystal started (probed once at
// boot: it is the ruler every rate on this link is measured against).
//
// Placement (docs/samc21/platform.md, "A handler in SRAM"): the two run
// wrappers in `.fast_text`, measured - from the flash the exchange run
// missed characters at the AVR host's 12 MHz pace (docs/samc21/spi.md);
// the vectors stay in the flash (the console and the tick, neither on a
// run's path).
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "samc21/clock.hpp"
#include "samc21/nvic.hpp"
#include "samc21/nvm.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "samc21/spi.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same
// file.
#include "../../../avrdx/src/apps/spi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using spilink::Op;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Console = Uart<5, console_pads>;
constexpr Console console;

// ---------------------------------------------------------------------------
// The link's pads, both roles (the test_samc_spi layouts, sides swapped)
// ---------------------------------------------------------------------------

constexpr uint8_t link_sercom = 1;

/// This board as the CLIENT: DOPO row 0x1 (DO = MISO on PAD[2]).
constexpr SpiPads client_pads{
    .data_out = SercomPad::pad2,
    .sck = SercomPad::pad3,
    .ss = SercomPad::pad1,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 18, PinFunction::c},
    .sck_pin = {'A', 19, PinFunction::c},
    .ss_pin = {'A', 17, PinFunction::c},
    .data_in_pin = {'A', 16, PinFunction::c},
};

/// This board as the HOST (Op::host_burst only): DOPO row 0x3.
constexpr SpiPads host_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad3,
    .ss = SercomPad::pad1,
    .data_in = SercomPad::pad2,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 19, PinFunction::c},
    .ss_pin = {'A', 17, PinFunction::c},
    .data_in_pin = {'A', 18, PinFunction::c},
};

static_assert(MUX_PA16C_SERCOM1_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA17C_SERCOM1_PAD1 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA18C_SERCOM1_PAD2 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA19C_SERCOM1_PAD3 == static_cast<uint8_t>(PinFunction::c));
static_assert(spi_dopo_for(client_pads.data_out, client_pads.sck, client_pads.ss).value() == 1);
static_assert(spi_dopo_for(host_pads.data_out, host_pads.sck, host_pads.ss).value() == 3);

using Client = SpiClient<link_sercom, client_pads>;
using Raw = Spi<link_sercom>;

using MosiPin = Pin<'A', 16>;
using SsPin = Pin<'A', 17>;
using MisoPin = Pin<'A', 18>;
using SckPin = Pin<'A', 19>;

constexpr uint16_t firmware_version = 0x0204;   ///< see spi_link.hpp's Ident

bool crystal_ok = false;
bool trace = false;

spilink::Decoder decoder;
spilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, serves = 0;
uint32_t last_byte_ms = 0;
uint32_t engaged_until = 0;

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }

// ---- microsecond waits ----------------------------------------------------------
//
// An APP-LOCAL spin, calibrated once against the SysTick millisecond
// at boot (the test_samc_spi link_hold precedent). samc21/delay.hpp was
// born AFTER this app and could replace it; the calibrated spin stays
// because it is proven at the bench and the swap would re-verify the
// whole suite for cosmetics.

volatile uint32_t spins_per_ms = 4000;   ///< blind default, overwritten at boot

void spin(uint32_t n) {
    for (volatile uint32_t i = 0; i < n; i = i + 1) {
    }
}

void calibrate_spin() {
    // Wait for a tick edge, then count how many 1000-spin blocks fit
    // in 32 ms. The result is only used for gap-scale waits (tens of
    // microseconds), where a few per cent of error is nothing.
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() == t0) {
    }
    const uint32_t t1 = Ticker::millis();
    uint32_t blocks = 0;
    while (Ticker::millis() - t1 < 32u) {
        spin(1000);
        ++blocks;
    }
    spins_per_ms = (blocks * 1000u) / 32u;
    if (spins_per_ms < 100u) spins_per_ms = 100u;
}

void delay_us(uint32_t us) {
    spin((us * spins_per_ms) / 1000u + 1u);
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

// ---- the client's standing configurations ---------------------------------------

/// The select pad while the SERCOM owns it: NO pull. PULLEN under
/// PMUXEN is a pull-DOWN on a SERCOM input (31.5.1, measured on this
/// pad: see the header), and a pull-down selects this client; the
/// host's end holds the line. Stated after every re-init, so a pull a
/// GPIO phase left behind never survives into the muxed pad.
void ss_unpulled() { SsPin::pull(PinPull::none); }

/// DARK: mode 0, MSb first, MISO NOT driven. Everything returns here.
bool go_dark() {
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .lsb_first = false,
                                         .preload = false,
                                         .drive_output = false});
    ss_unpulled();
    decoder.reset();
    return ok;
}

/// The answer window's configuration: preload on and MISO driven, so
/// byte 0 of the window is already the answer frame's magic.
bool arm_answer() {
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .lsb_first = false,
                                         .preload = true,
                                         .drive_output = true});
    ss_unpulled();
    return ok;
}

SpiMode mode_of(uint8_t m) {
    switch (m & 0x03u) {
        case 1: return SpiMode::mode1;
        case 2: return SpiMode::mode2;
        case 3: return SpiMode::mode3;
        default: return SpiMode::mode0;
    }
}

/// What a command asked this client to become. See the header: only
/// regime_buffer_wait has an exact counterpart on this silicon, and it
/// is the one the SAM suite commands.
bool apply_cfg(const spilink::Cfg& c, bool drive_output = true) {
    const bool preload = c.regime == spilink::regime_buffer_wait;
    const bool ok = Client::init(clock, {.mode = mode_of(c.mode),
                                         .lsb_first = c.dord != 0,
                                         .preload = preload,
                                         .drive_output = drive_output});
    ss_unpulled();
    return ok;
}

// ---- answering ------------------------------------------------------------------

void wait_ss_high(uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Client::selected() && Ticker::millis() - t0 < ms) {
    }
}

// ---- the runs, in SRAM -----------------------------------------------------------
//
// Every window this peer serves is ONE CALL of the driver's polled run
// (SpiClient::answer / exchange): the answers written on DRE inside the
// first five SCK cycles of each character (32.6.2.6.2's three-cycle rule,
// samc21/spi.hpp), nothing judged or sampled inside the burst. The two
// wrappers are placed in .fast_text, the runs inlined into them, so a turn
// runs from SRAM and not from the flash behind its two wait states
// (docs/samc21/platform.md, "A handler in SRAM"; the boundary each
// placement reaches is docs/samc21/spi.md's).

using Run = Client::Run;

/// A run's spin budget for `ms` milliseconds: one spin is an idle poll of
/// some eight core cycles at 48 MHz (the listing), so this is a bound of
/// the right order, not a clock.
constexpr uint32_t spins_for_ms(uint32_t ms) { return ms * (48'000u / 8u); }

[[gnu::noinline, gnu::section(".fast_text")]] Run answer_run(const uint8_t* out, uint16_t n,
                                                             uint32_t spins) {
    return Client::answer(std::span<const uint8_t>(out, n), spins);
}

[[gnu::noinline, gnu::section(".fast_text")]] Run exchange_run(const uint8_t* out, uint8_t* in,
                                                               uint16_t n, uint32_t spins) {
    return Client::exchange(std::span<const uint8_t>(out, n), std::span<uint8_t>(in, n), spins);
}

/// Serve ONE answer window: the frame padded with zeros to the most the
/// host clocks (spilink::answer_bytes), handed to the answer run - buf[0]
/// through PLOADEN into the shifter, buf[1] parked in DATA, every later
/// answer on its boundary.
///
/// THE WINDOW ENDS WHEN THE HOST CLOSES IT. A host may clock all of
/// spilink::answer_bytes, or stop as soon as its decoder holds the frame
/// and raise the select (the avrdx DUT does): a window still being served
/// then would swallow the NEXT window's traffic - the data frame behind
/// an ack, the exchange behind its command - as dummies. The run ends on
/// the client's TXC, set when SS is pulled high (32.8.6); the select PIN
/// is never waited on.
void serve(const uint8_t* buf, uint8_t n) {
    wait_ss_high(50);                     // the command window must close first
    if (!arm_answer()) {
        (void)go_dark();
        return;
    }
    uint8_t padded[spilink::answer_bytes] = {};
    for (uint8_t i = 0; i < n && i < spilink::answer_bytes; ++i) padded[i] = buf[i];
    (void)answer_run(padded, spilink::answer_bytes, spins_for_ms(spilink::serve_ms));
    ++serves;
    (void)go_dark();
}

void answer(Op op, const uint8_t* p, uint8_t len) {
    uint8_t buf[spilink::max_payload + 4];
    uint8_t n = 0;
    spilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof buf) buf[n++] = b;
        },
        op, p, len);
    serve(buf, n);
}

void ack(Op op, uint8_t sum, bool good) {
    const uint8_t p[2] = {spilink::byte_of(op), sum};
    answer(good ? Op::ack : Op::nak, p, 2);
}

// ---- accounting -----------------------------------------------------------------

void account(spilink::Report& r, uint8_t got, uint8_t expected) {
    if (got != expected) {
        if (r.mism == 0) {
            r.idx = static_cast<uint8_t>(r.count);
            r.got = got;
            r.exp = expected;
        }
        ++r.mism;
    }
    r.sum = static_cast<uint16_t>(r.sum + got);
    ++r.count;
}

struct Streams {
    spilink::Stream in{};
    spilink::Stream out{};
    bool reversed = false;

    explicit Streams(const spilink::Params& a)
        : in(a.pattern, a.seed_a),
          out(a.pattern, a.seed_b),
          reversed((a.flags & spilink::flag_expect_reversed) != 0) {}

    uint8_t next_expected() {
        const uint8_t v = in.next();
        return reversed ? spilink::bit_reverse(v) : v;
    }
};

// ---- the actions ----------------------------------------------------------------

/// The workhorse: the host sends P_A(i), this client answers P_B(i) -
/// one call of the driver's exchange run (the header: preloaded and one
/// ahead under regime_buffer_wait, the leftover leading otherwise), or,
/// when the command sets spilink::spare_answer_only, of its answer run,
/// which reads nothing: the host judges its read-back alone and this
/// report counts the answers loaded.
///
/// THE BURST CARRIES NOTHING BUT BYTES. A loop that steps the two LFSRs,
/// compares and tallies INSIDE itself slips from byte 2 on at
/// back-to-back characters above 1 MHz - the two preloads exact, then the
/// reload missing the three-SCK-cycle window every time. So the answer
/// stream is PRECOMPUTED, the received bytes only STORED, and every
/// judgement happens after the burst - and the select is not sampled
/// inside it either: aux1 is the milliseconds from the window's opening
/// to the run's END, aux2 the select read once after it (bit 0) and how
/// the run ended (bit 1: the host closed it), spi_link.hpp's Report.
constexpr uint16_t exchange_cap = 64;
uint8_t x_out[exchange_cap];
uint8_t x_in[exchange_cap];

spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    const uint16_t n = a.count < exchange_cap ? a.count : exchange_cap;
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    {
        spilink::Stream out(a.pattern, a.seed_b);
        for (uint16_t i = 0; i < n; ++i) x_out[i] = out.next();
    }
    const bool answer_only = (a.spare & spilink::spare_answer_only) != 0;

    const uint32_t t0 = Ticker::millis();
    const Run run = answer_only ? answer_run(x_out, n, spins_for_ms(a.ms))
                                : exchange_run(x_out, x_in, n, spins_for_ms(a.ms));
    const uint32_t dt = Ticker::millis() - t0;
    r.aux1 = run.count == 0 ? 255u : (dt > 254u ? 254u : static_cast<uint8_t>(dt));
    if (Client::selected()) r.aux2 |= 0x01;
    if (run.end == Client::RunEnd::closed) r.aux2 |= 0x02;
    r.aux0 = Client::flags();
    if (Client::overflow()) r.flags |= spilink::report_bufovf;

    if (answer_only) {
        r.count = run.count;   // answers loaded; nothing was received
    } else {
        // The judgement, after the wire has gone quiet.
        Streams s(a);
        for (uint16_t i = 0; i < run.count; ++i) {
            account(r, x_in[i], s.next_expected());
        }
    }
    if (r.count < a.count) r.flags |= spilink::report_timed_out;
    return r;
}

/// The loss semantics: DATA is never read for the whole burst; the
/// flags are OR'ed DURING it (a flag that comes and goes is invisible
/// afterwards), and what the two-level buffer retained - and in what
/// order - is counted at the end. The burst's end is the DEADLINE the
/// command carries: with the select read demoted to telemetry there is
/// no other edge to trust.
spilink::Report run_sink_slow(const spilink::Params& a) {
    spilink::Report r{};
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    Client::write(0x00);                  // something harmless for the host to read
    uint8_t seen = 0;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        seen = static_cast<uint8_t>(seen | Client::flags());
        if (Client::overflow()) r.flags |= spilink::report_bufovf;
    }
    r.flags |= spilink::report_timed_out;   // by construction: the deadline IS the exit
    r.sum = seen;
    r.got = Client::flags();
    uint8_t vals[4] = {};
    uint8_t retained = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        const auto v = Client::poll();
        if (!v) break;
        if (retained < 4) vals[retained] = static_cast<uint8_t>(*v);
        ++retained;
    }
    r.exp = Client::flags();
    r.count = retained;
    r.aux0 = vals[0];
    r.aux1 = vals[1];
    r.aux2 = vals[2];
    r.aux3 = vals[3];
    return r;
}

/// The second driver on the shared select wire. The SPI is handed back
/// first, so nothing of this board is on the other three wires.
spilink::Report run_ss_pulse(const spilink::Params& a) {
    spilink::Report r{};
    Client::release();
    if (a.aux8) wait_ms(a.aux8);
    SsPin::clear();
    SsPin::output();
    delay_us(a.aux16 ? a.aux16 : 1000u);
    SsPin::input(PinPull::up);
    r.aux0 = 1;
    r.count = 1;
    return r;
}

/// THE ROLES INVERT: this end becomes the bus HOST for a bounded burst
/// (the DUT's own CLIENT half needs a clock, and there is nobody else
/// on this wire). The choreography is the AVR peer's: the ack is
/// already served, both boards count the same lead-in milliseconds,
/// and at their end the DUT is a client with its first answer
/// preloaded. The select wire is driven low from PORT for the WHOLE
/// burst - one transaction, the thing hardware SS cannot frame.
///
/// aux16 is "the instrument's SCK division of its own clock" - and this
/// instrument's clock is 48 MHz, not the AVR's 24: the same division
/// names twice the rate. BAUD = div/2 - 1 (table 30-2's synchronous
/// row); the gap_us pacing between characters is kept, so the DUT's
/// pump is paced per character exactly as the AVR peer paced it.
spilink::Report run_host_burst(const spilink::Params& a) {
    spilink::Report r{};

    // Hand the client back first; the pull-up holds the select wire
    // high from PORT while the pins change hands. Client::release()
    // ALSO RELEASES THE CLOCKS (the GCLK channel and the APB mask), so
    // they are re-established before one register of the host role is
    // written - a host configured into a clockless block has a SYNCBUSY
    // of zeros that READS like success while every store is dropped:
    // the burst then "runs" and moves nothing.
    Client::release();
    SsPin::input(PinPull::up);
    Raw::bus_clock(true);
    if (!Raw::core_clock(0) || !Raw::reset()) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }

    const uint32_t lead = a.aux8 ? a.aux8 : 20u;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < lead) {
    }

    uint16_t div = a.aux16 ? a.aux16 : spilink::command_division;
    if (div < 2u) div = 2u;
    if (div > 512u) div = 512u;
    SpiConfig hc = spi_role_probe(host_pads, SpiRole::host);
    hc.mode = mode_of(a.cfg.mode);
    hc.lsb_first = a.cfg.dord != 0;
    hc.baud = static_cast<uint8_t>(div / 2u - 1u);
    if (!Raw::configure(hc) || !Raw::enable(true)) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    // Pads to the SERCOM with SCK settled at its idle level, the chip
    // select still high from PORT (apply-before-select, the CPOL-flip
    // lesson).
    MosiPin::function(PinFunction::c);
    SckPin::function(PinFunction::c);
    MisoPin::function(PinFunction::c, {.input_enable = true});
    SsPin::set();
    SsPin::output();
    Raw::flush_rx();
    delay_us(200);

    spilink::Stream out(a.pattern, a.seed_a);      // what THIS end sends
    spilink::Stream want(a.pattern, a.seed_b);     // what the client owes

    SsPin::clear();                                 // select, for the whole burst
    delay_us(spilink::gap_us);
    while (r.count < a.count && Ticker::millis() - t0 < lead + a.ms) {
        Raw::data(out.next());
        uint32_t spins_left = 200000u;
        while (!Raw::rxc_flag() && spins_left-- != 0u) {
        }
        if (!Raw::rxc_flag()) break;
        account(r, static_cast<uint8_t>(Raw::data()), want.next());
        delay_us(spilink::gap_us);
    }
    SsPin::set();
    r.aux0 = Raw::flags();
    if (r.count < a.count) r.flags |= spilink::report_timed_out;

    // Whatever happened, back to the dark client; the pads change
    // hands under the PORT-held high select.
    Raw::release();
    MosiPin::release();
    SckPin::release();
    MisoPin::release();
    SsPin::input(PinPull::up);
    (void)go_dark();
    return r;
}

// ---- the command handler --------------------------------------------------------

char label8[9] = {};   ///< die-serial word 0 in hex: this board's name

void handle(const spilink::Frame& f) {
    const Op op = f.op;
    ++commands;
    engaged_until = Ticker::millis() + spilink::engage_ms;
    if (trace) {
        print(console, "  [cmd op=", hex(spilink::byte_of(op)), " len=", f.len, "]", crlf);
    }

    if (op == Op::ping) {
        ack(op, f.sum, true);
        return;
    }
    if (op == Op::ident) {
        ack(op, f.sum, true);
        spilink::Ident d{};
        for (uint8_t i = 0; i < 8; ++i) d.label[i] = label8[i];
        d.xtal = crystal_ok ? 1 : 0;
        d.sanity = spilink::ident_sanity;
        d.version = firmware_version;
        uint8_t p[spilink::ident_size];
        spilink::put_ident(p, d);
        answer(Op::ident_data, p, spilink::ident_size);
        return;
    }
    if (op == Op::report) {
        ack(op, f.sum, true);
        uint8_t p[spilink::report_size];
        spilink::put_report(p, last_report);
        answer(Op::report_data, p, spilink::report_size);
        return;
    }

    if (f.len < spilink::params_size) {
        ack(op, f.sum, false);
        ++naks;
        return;
    }
    ack(op, f.sum, true);
    ++actions;

    const spilink::Params a = spilink::get_params(f.data);
    const uint32_t started = Ticker::millis();
    spilink::Report r{};
    switch (op) {
        case Op::exchange: r = run_exchange(a); break;
        case Op::sink_slow: r = run_sink_slow(a); break;
        case Op::ss_pulse: r = run_ss_pulse(a); break;
        case Op::host_burst: r = run_host_burst(a); break;
        // Op::mspi is the AVR's USART-Host-SPI experiment: this SERCOM
        // is not a USART while it is an SPI, and pretending would
        // corrupt the measurement. cfg_failed says "not on this peer".
        default: r.flags |= spilink::report_cfg_failed; break;
    }
    if ((r.flags & spilink::report_cfg_failed) == 0) r.flags |= spilink::report_ran;
    const uint32_t took = Ticker::millis() - started;
    r.ms = took > 255u ? 255u : static_cast<uint8_t>(took);
    r.op = spilink::byte_of(op);
    last_report = r;
    engaged_until = Ticker::millis() + spilink::engage_ms;
    (void)go_dark();
}

// ---- the console ----------------------------------------------------------------

void help() {
    print(console, "spi_peer: ? help | i status | 0 back to the dark client | 3 trace",
          crlf);
}

void status() {
    print(console, "  client SERCOM1 fn C, enabled=", Raw::enabled(),
          " role=", Raw::role() == SpiRole::client ? "client"
                    : Raw::role() == SpiRole::host ? "HOST" : "none",
          " mode=", static_cast<uint8_t>(Raw::mode()), crlf);
    print(console, "  SS pin reads ", SsPin::read() ? "high" : "LOW",
          ", selected=", Client::selected(),
          ", MISO driven=", MisoPin::has_function(), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          " served=", serves, engaged() ? "  (ENGAGED)" : "  (dark, mute)", crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " mism=", last_report.mism, " sum=", hex(last_report.sum),
          " flags=", hex(last_report.flags), " idx=", last_report.idx,
          " got=", hex(last_report.got), " exp=", hex(last_report.exp), crlf);
    print(console, "               aux=", hex(last_report.aux0), " ", hex(last_report.aux1),
          " ", hex(last_report.aux2), " ", hex(last_report.aux3),
          " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void SERCOM5_Handler() { (void)Console::isr(); }

/// The peer never arms a SERCOM1 interrupt (everything here is polled),
/// but an unbound vector is a silent spin - so a stray arm is disarmed
/// rather than looped on.
extern "C" void SERCOM1_Handler() {
    Raw::enable_interrupt(brio::SpiFlag::all, false);
}

int main() {
    SysClock::init();
    Console::init(clock, 115200);
    Ticker::init(clock);
    brio::enable_interrupts();
    calibrate_spin();

    // The board's name: the factory die serial's first word, in hex.
    const uint32_t w0 = brio::DeviceSerial::read().word[0];
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        label8[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }

    // The crystal probe, for ident.xtal (the peer's clock quality is a
    // bench fact the DUT asks about). The oscillator is left running -
    // nothing else here uses it.
    crystal_ok = Xosc::init(XoscConfig{.hz = 24'000'000UL, .startup = 4,
                                       .on_demand = false});

    print(console, crlf, "spi_peer - SPI instrument client (SAM C21 board ", label8,
          ", xtal=", crystal_ok ? "up" : "DOWN", ", fw ", hex(firmware_version), ")",
          crlf);
    print(console, "client SERCOM1 fn C: MOSI PA16, SS PA17 (no pull), MISO PA18, "
                   "SCK PA19; command mode = SPI mode 0, MSb first", crlf);
    print(console, "DARK by default: MISO is driven only for one answer window, after "
                   "a frame that checked out", crlf);
    (void)go_dark();
    last_byte_ms = Ticker::millis();
    help();
    print(console, "> ");

    for (;;) {
        uint8_t c;
        if (Console::read_byte(c)) {
            if (c != '\r' && c != '\n') {
                print(console, static_cast<char>(c), crlf);
                if (c == '?') help();
                else if (c == 'i') status();
                else if (c == '0') {
                    engaged_until = 0;
                    (void)go_dark();
                    print(console, "  dark client restored, engagement dropped", crlf);
                } else if (c == '3') {
                    trace = !trace;
                    print(console, trace ? "  trace on" : "  trace off", crlf);
                } else {
                    print(console, "? for help", crlf);
                }
                print(console, "> ");
            }
        }

        if (decoder.partial() && Ticker::millis() - last_byte_ms > 100u) decoder.reset();

        const auto v = Client::poll();
        if (!v) continue;
        last_byte_ms = Ticker::millis();
        switch (decoder.feed(static_cast<uint8_t>(*v))) {
            case spilink::Decoder::Result::frame:
                if (spilink::is_command(decoder.frame().op)) handle(decoder.frame());
                break;
            case spilink::Decoder::Result::bad_checksum:
                if (spilink::is_command(decoder.pending_op()) && engaged()) {
                    ack(decoder.pending_op(), decoder.frame().sum, false);
                    ++naks;
                }
                break;
            default: break;
        }
    }
}
