// spi_peer - the INSTRUMENT half of the SPI campaign on the SAM C21:
// board D, the scriptable CLIENT that test_samc_spi (board C, the DUT
// and the bus host) drives IN BAND over the very bus both are testing.
//
// A PORT OF avrdx/src/apps/spi_peer.cpp TO THE SECOND ARCHITECTURE,
// over the SAME protocol header (spi_link.hpp, included by relative
// path - one source of truth for the wire format, never a copy). What
// changed is exactly what the silicon changed:
//
//  - THE PUMP RUNS ONE AHEAD. The AVR client loads its answer in the
//    host's inter-byte gap and the shifter takes it at the boundary;
//    this client's DATA write needs THREE SCK CYCLES to reach the
//    shifter, and those cycles elapse only while SCK runs (32.6.2.6.2)
//    - so an answer written in the gap is one character late, every
//    time. The working shape (test_samc_spi letter e, measured): the
//    FIRST answer goes in through CTRLB.PLOADEN while SS is still high,
//    the SECOND parks in DATA at once, and every received character
//    loads the next-PLUS-ONE. Both serve() and run_exchange() are that
//    pump.
//
//  - THE BUFFERING REGIMES COLLAPSE. The AVR's normal/buffer/buffer-
//    wait knobs (CTRLB.BUFEN/BUFWR) do not exist here: this client is
//    always buffered two deep, and PLOADEN plays BUFWR's role for the
//    first character only. spilink::Cfg::regime is therefore mapped,
//    not translated: regime_buffer_wait (the only regime the SAM suite
//    ever commands) runs the preloaded one-ahead pump and delivers the
//    exact rx[i] = P_B(i) alignment that regime promises; the other two
//    run without preload, which on this silicon means the shifter's
//    leftover leads - stated here, exercised nowhere.
//
//  - flag_wrcol AND flag_feed_tx ARE AVR EXPERIMENTS (WRCOL and the
//    28.5.5 BUFOVF clause have no counterpart in ch. 32) and are
//    ignored; Op::mspi (the USART Host SPI client) is answered with
//    report_cfg_failed - this SERCOM is not a USART while it is an SPI.
//    Everything else of the repertoire is served: ping/ident/report,
//    exchange, sink_slow, ss_pulse, host_burst.
//
// THE DARK LISTENER discipline is the AVR peer's, unchanged: the
// command-mode client never drives MISO (drive_output = false), the
// answer line wakes only for one answer window after a frame that
// CHECKED OUT, an unknown op is dropped in silence, and a bad checksum
// is nak'ed only while ENGAGED. The select wire is held up by THIS
// board's internal pull-up at all times - the pull survives PMUXEN
// (28.6.3.2, the EIC campaign's finding: the mux takes the output
// driver, not the pull), so it is re-stated after every re-init.
//
// Link: SERCOM1 function C on the C-D straight-through wires -
//   PA16 = PAD[0]  MOSI (DI here: the client's DOPO row is 0x2)
//   PA17 = PAD[1]  SCK
//   PA18 = PAD[2]  SS (input; this board's pull-up holds it high)
//   PA19 = PAD[3]  MISO (DO, driven only to answer)
// The same four pads carry this board as the bus HOST for
// Op::host_burst - DOPO row 0x0, chip select by PORT on PA18.
//
// Console: SERCOM5 PB30/PB31 at 115200, observability only.
//   ? help | i status and counters | 0 back to the dark client | 3 trace
//
// The ident label is the die serial's first word in hex - this family's
// identity is factory-programmed (no USERROW label to read); ident.xtal
// reports whether the board's 24 MHz crystal started (probed once at
// boot, the test_samc_spi ruler's own arrangement).
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/nvic.hpp"
#include "samc/nvm.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/spi.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"

// THE PROTOCOL IS THE AVR CAMPAIGN'S, AND IT IS NOT COPIED (the
// test_samc_spi ruling: pure encoding, no register, both architectures
// compile the same file).
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

/// This board as the CLIENT: DOPO row 0x2 (DO = MISO on PAD[3]).
constexpr SpiPads client_pads{
    .data_out = SercomPad::pad3,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 19, PinFunction::c},
    .sck_pin = {'A', 17, PinFunction::c},
    .ss_pin = {'A', 18, PinFunction::c},
    .data_in_pin = {'A', 16, PinFunction::c},
};

/// This board as the HOST (Op::host_burst only): DOPO row 0x0.
constexpr SpiPads host_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 17, PinFunction::c},
    .ss_pin = {'A', 18, PinFunction::c},
    .data_in_pin = {'A', 19, PinFunction::c},
};

static_assert(MUX_PA16C_SERCOM1_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA17C_SERCOM1_PAD1 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA18C_SERCOM1_PAD2 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA19C_SERCOM1_PAD3 == static_cast<uint8_t>(PinFunction::c));
static_assert(spi_dopo_for(client_pads.data_out, client_pads.sck, client_pads.ss).value() == 2);
static_assert(spi_dopo_for(host_pads.data_out, host_pads.sck, host_pads.ss).value() == 0);

using Client = SpiClient<link_sercom, client_pads>;
using Raw = Spi<link_sercom>;

using MosiPin = Pin<'A', 16>;
using SckPin = Pin<'A', 17>;
using SsPin = Pin<'A', 18>;
using MisoPin = Pin<'A', 19>;

constexpr uint16_t firmware_version = 0x0201;   ///< 0x01xx = the AVR peer

/// The exchange's DMA engines (channels 0/1 on SERCOM1's triggers).
/// Armed once at boot; the SERCOM re-inits under them freely - the
/// claim binds a DATA address and a trigger code, not a configuration.
using PeerTx = DmaTxEngine<0>;
using PeerRx = DmaRxEngine<1>;
bool dmac_ok = false;

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
// at boot (the test_samc_spi link_hold precedent). samc/delay.hpp was
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

/// The select wire is shared; while this end is (or is becoming) a
/// client, its own pull-up is what keeps the line from floating low and
/// selecting it at random. The pull survives PMUXEN, so this is legal
/// on the muxed pad - and every re-init clears PULLEN, so it is
/// re-stated after each one.
void hold_ss_up() { SsPin::pull(PinPull::up); }

/// DARK: mode 0, MSb first, MISO NOT driven. Everything returns here.
bool go_dark() {
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .lsb_first = false,
                                         .preload = false,
                                         .drive_output = false});
    hold_ss_up();
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
    hold_ss_up();
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
    hold_ss_up();
    return ok;
}

// ---- answering ------------------------------------------------------------------

void wait_ss_high(uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Client::selected() && Ticker::millis() - t0 < ms) {
    }
}

/// Serve ONE answer window, with the one-ahead pump: buf[0] through
/// PLOADEN into the shifter, buf[1] parked in DATA, and every received
/// character loads the next one (a whole character early, so the
/// three-SCK-cycle rule is already paid when its boundary comes). The
/// host clocks spilink::answer_bytes dummies; the tail pads with zeros.
void serve(const uint8_t* buf, uint8_t n) {
    wait_ss_high(50);                     // the command window must close first
    if (!arm_answer()) {
        (void)go_dark();
        return;
    }
    Client::write(buf[0]);                // PLOADEN: straight into the shifter
    Client::write(n > 1 ? buf[1] : 0x00u);   // parked in DATA
    uint8_t idx = 2;
    uint16_t seen = 0;
    const uint32_t t0 = Ticker::millis();
    while (seen < spilink::answer_bytes &&
           Ticker::millis() - t0 < spilink::serve_ms) {
        if (!Client::poll()) continue;
        ++seen;
        Client::write(idx < n ? buf[idx] : 0x00u);
        if (idx < n) ++idx;
    }
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
/// through the one-ahead pump when the regime asks for the exact
/// alignment (see the header). The RXC poll IS the wait for the burst
/// (the run_exchange lesson: the select read is telemetry, never a
/// gate), and the deadline is the only other exit.
///
/// THE HOT LOOP CARRIES NOTHING BUT BYTES. The first version stepped
/// the two LFSRs, compared and tallied INSIDE the loop, and the bench
/// answer was immediate: back-to-back characters above 1 MHz slipped
/// from byte 2 on - the two preloads exact, then the reload missing
/// the three-SCK-cycle window every time. So the answer stream is
/// PRECOMPUTED, the received bytes only STORED, and every judgement
/// happens after the burst. What that buys is a reload path of a few
/// register accesses; what it costs is the burst cap below.
constexpr uint16_t exchange_cap = 64;
uint8_t x_out[exchange_cap];
uint8_t x_in[exchange_cap];

spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    const bool preload = a.cfg.regime == spilink::regime_buffer_wait;
    const uint16_t n = a.count < exchange_cap ? a.count : exchange_cap;
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    {
        spilink::Stream out(a.pattern, a.seed_b);
        for (uint16_t i = 0; i < n; ++i) x_out[i] = out.next();
    }

    // THE DMA SERVE, and it is the default: preload P_B(0) through
    // PLOADEN, hand P_B(1..n-1) to the transmit channel (whose enable
    // under the standing DRE fires the first beat by itself - the SPI
    // host measurement, relied on here for the client too and judged
    // by the host's own read-back), and drain the n received bytes on
    // the receive channel. The reload is then hardware: the polled
    // loop's ~3 MHz boundary is gone and what remains is the silicon's.
    // spare bit 0 (spilink::spare_polled_pump) asks for the polled
    // loop instead, so both boundaries stay measurable.
    if (dmac_ok && preload && (a.flags == 0u || a.flags == spilink::flag_expect_reversed)
        && (a.spare & spilink::spare_polled_pump) == 0u) {
        (void)PeerRx::start(x_in, n);
        Client::write(x_out[0]);          // PLOADEN: straight into the shifter
        if (n > 1) {
            (void)PeerTx::start(x_out + 1, static_cast<uint16_t>(n - 1));
        }
        const uint32_t t0 = Ticker::millis();
        while (!PeerRx::idle() && Ticker::millis() - t0 < a.ms) {
        }
        const bool all_in = PeerRx::idle();
        PeerTx::stop();
        PeerRx::stop();
        PeerTx::arm(Spi<link_sercom>::data_address(),
                    Sercom<link_sercom>::dma_tx_trigger());
        PeerRx::arm(Spi<link_sercom>::data_address(),
                    Sercom<link_sercom>::dma_rx_trigger());
        r.aux0 = Client::flags();
        r.aux1 = all_in ? 0u : 255u;
        r.aux2 = 0x04;                    // marks the DMA serve in the report
        Streams s(a);
        const uint16_t got = all_in ? n : 0u;
        for (uint16_t i = 0; i < got; ++i) {
            account(r, x_in[i], s.next_expected());
        }
        if (r.count < a.count) r.flags |= spilink::report_timed_out;
        if (Client::overflow()) r.flags |= spilink::report_bufovf;
        return r;
    }

    uint16_t queued = 0;
    Client::write(x_out[queued++]);       // b0: the shifter (PLOADEN) or DATA
    if (preload && n > 1) {
        Client::write(x_out[queued++]);   // b1: parked in DATA
    }
    r.aux3 = Client::flags();             // INTFLAG right after the preloads

    const uint32_t t0 = Ticker::millis();
    uint16_t got = 0;
    bool sel_at_first = false;
    bool any_selected = false;
    while (got < n && Ticker::millis() - t0 < a.ms) {
        const auto v = Client::poll();
        if (!v) continue;
        x_in[got++] = static_cast<uint8_t>(*v);
        if (queued < n) Client::write(x_out[queued++]);
        // Telemetry AFTER the reload: the select pin sampled per
        // received byte, never in the reload path.
        const bool sel_now = Client::selected();
        any_selected = any_selected || sel_now;
        if (got == 1) {
            sel_at_first = sel_now;
            const uint32_t dt = Ticker::millis() - t0;
            r.aux1 = dt > 254u ? 254u : static_cast<uint8_t>(dt);
        }
    }
    if (got == 0) r.aux1 = 255;
    r.aux0 = Client::flags();
    if (Client::overflow()) r.flags |= spilink::report_bufovf;

    // The judgement, after the wire has gone quiet.
    Streams s(a);
    for (uint16_t i = 0; i < got; ++i) {
        account(r, x_in[i], s.next_expected());
    }
    if (r.count < a.count) r.flags |= spilink::report_timed_out;
    if (sel_at_first) r.aux2 |= 0x01;
    if (any_selected) r.aux2 |= 0x02;
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
    // written - the first version configured a host into a clockless
    // block, whose SYNCBUSY of zeros READS like success while every
    // store is dropped: the burst then "ran" and moved nothing.
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

/// The engines' completions: nothing to dispatch - the serve loop polls
/// the receive channel - but the interrupts the claim arms must land
/// somewhere that is not Default_Handler's silent spin.
extern "C" void DMAC_Handler() {
    while (brio::Dmac::take_pending()) {
    }
}
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

    // The DMAC block and the exchange's two engines - see run_exchange.
    dmac_ok = brio::Dmac::init();
    if (dmac_ok) {
        PeerTx::arm(brio::Spi<link_sercom>::data_address(),
                    brio::Sercom<link_sercom>::dma_tx_trigger());
        PeerRx::arm(brio::Spi<link_sercom>::data_address(),
                    brio::Sercom<link_sercom>::dma_rx_trigger());
    }

    // The crystal probe, for ident.xtal (the peer's clock quality is a
    // bench fact the DUT asks about). The oscillator is left running -
    // nothing else here uses it.
    crystal_ok = Xosc::init(XoscConfig{.hz = 24'000'000UL, .startup = 4,
                                       .on_demand = false});

    print(console, crlf, "spi_peer - SPI instrument client (SAM C21 board ", label8,
          ", xtal=", crystal_ok ? "up" : "DOWN", ", fw ", hex(firmware_version), ")",
          crlf);
    print(console, "client SERCOM1 fn C: MOSI PA16, SCK PA17, SS PA18 (pulled up), "
                   "MISO PA19; command mode = SPI mode 0, MSb first", crlf);
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
