// spi_peer - the INSTRUMENT half of the SPI campaign: board B, the
// scriptable CLIENT that test_avr_spi (board A, the DUT and the bus
// host) drives IN BAND over the very bus both are testing.
//
// It is deliberately not a kernel app: one blocking loop that shifts
// whatever the host clocks, decodes a command frame
// (src/apps/spi_link.hpp), acknowledges it and then becomes for a
// bounded moment whatever the DUT needs at the other end of the wire -
// a client in any of the four transfer modes, either bit order and all
// three buffering regimes; a client that never drains, so the loss
// semantics can be measured; a second driver on the shared select wire,
// which is the only way to demote a real host; a client for the USART's
// own Host SPI mode. Every action carries a byte count and a
// millisecond deadline, after which the peer restores its dark
// command-mode client BY ITSELF.
//
// THE DARK LISTENER - the one thing to understand before touching this
// file. The four wires are PORTE straight through (A.PEn - B.PEn), and
// they are also the pins the DUT's SINGLE-board half measures: that
// half drives the select wire low and drives MISO from its own PORT.
// So this peer's command-mode client runs with `drive_miso = false` and
// only listens. MISO is driven for exactly one answer window, entered
// only after a frame that CHECKED OUT, and dropped again at once. The
// two gates that keep the single-board half's traffic from ever waking
// the answer line are in spi_link.hpp's header comment: an unknown op
// is dropped in silence, and a bad checksum is nak'ed only while the
// peer is ENGAGED. The proof is that `test_avr_spi z` still scores
// 148/148 with this firmware attached and running.
//
// The select wire is held up by THIS board's pull-up at all times: the
// DUT leaves its own end an input for the demotion test, and a floating
// select line would select this client at random.
//
// Link: SPI0 ALT1 - MOSI PE0, MISO PE1, SCK PE2, SS PE3. The command
// channel is SPI mode 0, MSb first, at whatever the host clocks (it
// uses CLK_PER/32).
//
// Console: USART2 ALT1 (PF4/PF5) at 460800, observability only.
//   ? help | i status and counters | 0 back to the dark client | 3 trace
//
// The banner reports the USERROW board label and whether THIS board's
// 24 MHz crystal started: the peer's clock sets its CLK_PER/6 client
// ceiling, so its quality is itself a bench fact and the DUT asks for it
// with the `ident` command.

// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/spi.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

#include "spi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;
using spilink::Op;

using Console = Uart<2, Route::alt1>;
constexpr Console console;

// The task is the handle; the resource under it is where the raw flag
// and DATA verbs live, and the loss experiments need them (a client
// that must NOT drain cannot go through a poll() that reads DATA).
// Both names are the same instance - SpiClient is a thin handle by
// design, so this is the driver being used as it is, not bent.
using Client = SpiClient<0, SpiRoute::alt1>;
using Raw = Client::Resource;

using SckPin = Pin<'E', 2>;
using SsPin = Pin<'E', 3>;

constexpr uint16_t firmware_version = 0x0100;

bool crystal_ok = false;
bool trace = false;

spilink::Decoder decoder;
spilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, serves = 0;
uint32_t last_byte_ms = 0;
/// A frame that checked out keeps the peer willing to nak for this
/// long. Zero = never engaged, answer nothing but valid frames.
uint32_t engaged_until = 0;

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }

// ---- the client's two standing configurations ---------------------------------

/// The select wire is shared and the DUT leaves its end an input for
/// the demotion test: this board holds it up, always. INVEN is cleared
/// with it - only the Host SPI action turns it on, and it must never
/// outlive that action.
void hold_ss_up() {
    SsPin::invert(false);
    SsPin::pullup(true);
}

/// DARK: normal mode, mode 0, MSb first, MISO NOT driven. Everything
/// returns here.
bool go_dark() {
    const bool ok = Client::init({.mode = SpiMode::mode0,
                                  .lsb_first = false,
                                  .buffer_mode = false,
                                  .buffer_wait = false,
                                  .drive_miso = false});
    hold_ss_up();
    decoder.reset();
    return ok;
}

/// The answer window's configuration: buffer mode WITH BUFWR, so the
/// first write goes straight to the shift register and byte 0 of the
/// window is already the answer frame's magic - no dummy to skip.
bool arm_answer() {
    const bool ok = Client::init({.mode = SpiMode::mode0,
                                  .lsb_first = false,
                                  .buffer_mode = true,
                                  .buffer_wait = true,
                                  .drive_miso = true});
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

/// What a command asked this client to become.
bool apply_cfg(const spilink::Cfg& c, bool drive_miso = true) {
    const bool buffered = c.regime != spilink::regime_normal;
    const bool wait = c.regime == spilink::regime_buffer_wait;
    const bool ok = Client::init({.mode = mode_of(c.mode),
                                  .lsb_first = c.dord != 0,
                                  .buffer_mode = buffered,
                                  .buffer_wait = wait,
                                  .drive_miso = drive_miso});
    hold_ss_up();
    return ok;
}

// ---- answering ------------------------------------------------------------------

void wait_ss_high(uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Client::selected() && Ticker::millis() - t0 < ms) {
    }
}

/// Serve ONE answer window: MISO is driven for exactly as long as the
/// host keeps the select wire low, and never a moment longer. The
/// window the host opens is longer than the frame (it clocks dummies
/// until its own decoder is satisfied), so the tail is padded with
/// zeros.
void serve(const uint8_t* buf, uint8_t n) {
    wait_ss_high(50);                     // the command window must be closed first
    if (!arm_answer()) {
        (void)go_dark();
        return;
    }
    Raw::write(buf[0]);                   // BUFWR: straight into the shift register
    uint8_t idx = 1;
    if (n > 1 && Raw::dre_flag()) {
        Raw::write(buf[1]);               // and this one into the transmit buffer
        idx = 2;
    }
    const uint32_t t0 = Ticker::millis();
    while (!Client::selected() && Ticker::millis() - t0 < spilink::serve_ms) {
    }
    while (Client::selected() && Ticker::millis() - t0 < spilink::serve_ms) {
        if (Raw::dre_flag()) Raw::write(idx < n ? buf[idx++] : 0x00u);
        if (Raw::rxc_flag()) (void)Raw::read();
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

/// The two streams of one action, as running state: what the host is
/// expected to send and what this end answers. They are stepped ONCE
/// per byte - the whole read-check-load turnaround has to fit inside
/// the host's inter-byte gap, and replaying an LFSR from its seed does
/// not stay inside it.
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

/// Spin until the host's clock leaves its idle level - that is, until
/// the next transfer has really started. This is a PASSIVE read of a
/// pin the other board drives and this one never does (SCK is an input
/// on a client, and the campaign's rule against touching PE0/PE2 is
/// about driving them), and it is the only way to write to DATA on a
/// chosen side of the inter-byte boundary rather than a random one. At
/// CLK_PER/32 a byte lasts about 10.7 us and this loop samples every
/// few cycles, so a write that follows a `true` lands early inside the
/// transfer with room to spare. False when the host never clocked.
bool wait_transfer_start(bool idle_level) {
    for (uint16_t k = 0; k < 20000; ++k) {
        if (SckPin::read() != idle_level) return true;
    }
    return false;
}

// ---- the actions ------------------------------------------------------------------

/// The workhorse. The host sends P_A(i) and this client answers P_B(i),
/// both generated from the seeds in the command. What the host READS
/// back depends on the buffering regime, and that asymmetry is one of
/// the things under test - so this end simply emits P_B in order, one
/// write per byte received, and lets the host measure the alignment.
spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    const bool buffered = a.cfg.regime != spilink::regime_normal;
    const bool bufwr = a.cfg.regime == spilink::regime_buffer_wait;
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    Streams s(a);
    // BUFWR loads the shift register directly, so two levels can be
    // filled before the window opens; without it the single write goes
    // into the transmit buffer and the shifter's leftover leads.
    Raw::write(s.out.next());
    if (bufwr) Raw::write(s.out.next());

    const uint32_t t0 = Ticker::millis();
    while (!Client::selected() && Ticker::millis() - t0 < a.ms) {
    }
    const bool cpol_idle = spi_cpol(mode_of(a.cfg.mode));
    const bool collide = !buffered && (a.flags & spilink::flag_wrcol) != 0;
    bool skipped = false;
    while (r.count < a.count && Ticker::millis() - t0 < a.ms) {
        const auto v = Raw::poll();
        if (!v) continue;
        const uint16_t i = r.count;             // the byte just received
        const uint8_t want = s.next_expected();
        if ((a.flags & spilink::flag_skip_write) != 0 && !skipped &&
            i == spilink::skip_at) {
            // Answer NOTHING to this one: the next transfer starts with
            // the shift register still holding what just came in.
            skipped = true;
            account(r, *v, want);
            continue;
        }
        account(r, *v, want);
        Raw::write(s.out.next());               // the answer, loaded in the gap
        if (!collide) continue;

        // THE WRITE-COLLISION BOUNDARY. A write to DATA is a COLLISION
        // only while the shifter is running; in the gap the host leaves
        // between bytes it is an ordinary write that simply replaces the
        // answer just loaded. Which side of that boundary a write lands
        // on cannot be left to the phase between this loop and the
        // host's pacing - that is a coin toss, and it decides whether
        // the burst survives. So each side is entered deliberately, and
        // the marker byte makes the outcome visible to the host.
        if (i == spilink::wrcol_gap_at) {
            // Still inside the gap: this must be ACCEPTED, and the
            // marker must come out in place of the answer above.
            Raw::write(spilink::wrcol_marker);
        } else if (i == spilink::wrcol_hold_at && wait_transfer_start(cpol_idle)) {
            // The next transfer has actually started: this must be
            // IGNORED, the loaded answer must go out intact, and WRCOL
            // must come up. Sampled right here and nowhere else - a read
            // of INTFLAGS followed by an access to DATA is precisely the
            // sequence that clears WRCOL, so every poll() after this
            // point erases the evidence.
            Raw::write(spilink::wrcol_marker);
            if (Raw::write_collision()) r.flags |= spilink::report_wrcol;
        }
    }
    if (r.count < a.count) r.flags |= spilink::report_timed_out;
    r.aux0 = Raw::flags();
    if (!buffered && Raw::write_collision()) r.flags |= spilink::report_wrcol;
    if (buffered && Raw::overflow_flag()) r.flags |= spilink::report_bufovf;
    return r;
}

/// The loss semantics: the host streams with no gap and this end does
/// NOT touch DATA for the whole burst. What is left afterwards, and in
/// what order, is the measurement.
spilink::Report run_sink_slow(const spilink::Params& a) {
    spilink::Report r{};
    const bool buffered = a.cfg.regime != spilink::regime_normal;
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    Raw::write(0x00);                     // something harmless for the host to read
    const uint32_t t0 = Ticker::millis();
    while (!Client::selected() && Ticker::millis() - t0 < a.ms) {
    }
    // DATA is not touched for the whole burst - that is the experiment -
    // but INTFLAGS is sampled and OR'ed, because a flag that comes and
    // goes inside the burst is invisible to a look taken afterwards.
    // Reading INTFLAGS alone clears nothing: every clear in the normal
    // layout needs a DATA access to follow it.
    const bool feed_tx = buffered && (a.flags & spilink::flag_feed_tx) != 0;
    uint8_t seen = 0;
    bool overflowed = false;
    while (Client::selected() && Ticker::millis() - t0 < a.ms) {
        seen = static_cast<uint8_t>(seen | Raw::flags());
        if (buffered && Raw::overflow_flag()) overflowed = true;
        // Writing DATA never clears BUFOVF (only READING it does), so a
        // fed transmitter changes nothing but the condition 28.5.5 puts
        // on the flag itself.
        if (feed_tx && Raw::dre_flag()) Raw::write(0xA5);
    }
    if (Ticker::millis() - t0 >= a.ms) r.flags |= spilink::report_timed_out;
    r.sum = seen;                         // every flag the burst ever raised

    r.got = Raw::flags();                 // and the ones still standing
    if (buffered && (overflowed || Raw::overflow_flag())) r.flags |= spilink::report_bufovf;
    if (!buffered && Raw::write_collision()) r.flags |= spilink::report_wrcol;
    uint8_t vals[4] = {};
    uint8_t retained = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        const auto v = Raw::poll();
        if (!v) break;
        if (retained < 4) vals[retained] = *v;
        ++retained;
    }
    r.exp = Raw::flags();                 // after it
    r.count = retained;
    r.aux0 = vals[0];
    r.aux1 = vals[1];
    r.aux2 = vals[2];
    r.aux3 = vals[3];
    return r;
}

/// The second driver on the shared select wire: what a real multi-host
/// bus does to a host that watches its SS pin. The SPI is handed back
/// first, so nothing of this board is on the other three wires.
spilink::Report run_ss_pulse(const spilink::Params& a) {
    spilink::Report r{};
    const uint8_t cpu = cycles_per_us(SysClock::hz);
    Raw::release();
    if (a.aux8) delay_us_runtime(cpu, static_cast<uint32_t>(a.aux8) * 1000u);
    SsPin::pullup(false);
    SsPin::clear();
    SsPin::output();
    delay_us_runtime(cpu, a.aux16 ? a.aux16 : 1000u);
    SsPin::input();
    hold_ss_up();
    r.aux0 = 1;
    r.count = 1;
    return r;
}

/// The client for the USART's own Host SPI mode (usart.hpp MspiHost).
/// That mode has NO client select - SCK is XCK, MOSI is TXD, MISO is
/// RXD and nothing frames the burst - so this client selects ITSELF
/// with INVEN on its pulled-up SS pin, and both ends wait out the same
/// lead-in instead of a chip-select edge.
spilink::Report run_mspi(const spilink::Params& a) {
    spilink::Report r{};
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    SsPin::invert(true);                  // the pad still reads high; the SPI sees low
    const uint32_t lead = a.aux8 ? a.aux8 : 20u;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < lead) {
    }
    // Whatever the host's hand-over of PE0/PE2 put on the wire is not
    // traffic: drop it, then preload the first real answer.
    while (Raw::poll()) {
    }
    Streams s(a);
    Raw::write(s.out.next());
    while (r.count < a.count && Ticker::millis() - t0 < a.ms) {
        const auto v = Raw::poll();
        if (!v) continue;
        account(r, *v, s.next_expected());
        Raw::write(s.out.next());
    }
    if (r.count < a.count) r.flags |= spilink::report_timed_out;
    r.aux0 = Raw::flags();
    SsPin::invert(false);                 // deselect: never outlive the action
    return r;
}

// ---- the command handler ------------------------------------------------------------

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
        auto board = board_id();
        for (uint8_t i = 0; i < 8 && i < board.size(); ++i) d.label[i] = board[i];
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
        case Op::mspi: r = run_mspi(a); break;
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

// ---- the console --------------------------------------------------------------------

void help() {
    print(console, "spi_peer: ? help | i status | 0 back to the dark client | 3 trace",
          crlf);
}

void status() {
    print(console, "  client SPI0 ALT1, route=", static_cast<uint8_t>(Raw::routed()),
          " enabled=", Raw::enabled(), " host=", Raw::is_host(),
          " buffered=", Raw::buffer_mode(), " bufwr=", Raw::buffer_wait(),
          " mode=", static_cast<uint8_t>(Raw::mode()), " lsb=", Raw::lsb_first(), crlf);
    print(console, "  SS pin reads ", SsPin::read() ? "high" : "LOW",
          ", selected=", Client::selected(), ", MISO driven=", Pin<'E', 1>::is_output(),
          crlf);
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

ISR(USART2_RXC_vect) { (void)Console::rxc(); }
ISR(USART2_DRE_vect) { Console::dre(); }
ISR(RTC_PIT_vect) { Ticker::pit(); }

int main() {
    crystal_ok = SysClock::init();
    Console::init(clock, 460800);
    Ticker::init();
    sei();

    auto board = board_id();
    if (board.empty()) board = "?";
    print(console, crlf, "spi_peer - SPI instrument client (board ", board,
          ", clk=", crystal_ok ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ",
          hex(SYSCFG.REVID), ", fw ", hex(firmware_version), ")", crlf);
    print(console, "client SPI0 ALT1: MOSI PE0, MISO PE1, SCK PE2, SS PE3; command mode "
                   "= SPI mode 0, MSb first", crlf);
    print(console, "DARK by default: MISO is driven only for one answer window, after a "
                   "frame that checked out", crlf);
    if (!crystal_ok) {
        print(console, "WARNING: the 24 MHz crystal did NOT start - this client's "
                       "CLK_PER/6 ceiling is OSCHF's, not the crystal's", crlf);
    }
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

        // A frame that stops half way (a mode the DUT was testing, a
        // window that ended early) must not eat the next one: a quiet
        // wire for longer than any inter-byte gap resets the reassembly.
        if (decoder.partial() && Ticker::millis() - last_byte_ms > 100u) decoder.reset();

        const auto v = Raw::poll();
        if (!v) continue;
        last_byte_ms = Ticker::millis();
        switch (decoder.feed(*v)) {
            case spilink::Decoder::Result::frame:
                // An op this protocol does not know is dropped in
                // silence, whatever its checksum says: the answer line
                // must never wake up for the single-board half's traffic.
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
