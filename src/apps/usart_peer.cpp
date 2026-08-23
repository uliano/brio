// usart_peer - the INSTRUMENT half of the USART campaign: board B, the
// scriptable peer that test_avr_serial (board A, the DUT) drives IN
// BAND over the very link under test.
//
// It is deliberately not a kernel app: one blocking loop that polls the
// link, decodes a command frame (src/apps/usart_link.hpp), acknowledges
// it and then becomes for a bounded moment whatever the DUT needs on
// the other end of the wire - an echo, a silent sink, a generator, a
// flood, a break, a foreign auto-baud sender, a cycle-counted bit-banger
// that a clean UART could never be. Every mode-changing command carries
// a frame count and a millisecond deadline, after which the peer
// restores command mode BY ITSELF: that is the recovery guarantee the
// DUT's tests lean on.
//
// Link: USART4 at its default position - TXD PE0, RXD PE1, XCK PE2. Two
// wirings are supported and the peer FINDS OUT which one the desk has
// (usart_link.hpp Topology): the crossed full-duplex pair (A.PE0-B.PE1,
// A.PE1-B.PE0, A.PE2-B.PE2) or a single wire between the two TXD pads,
// which is the one-wire bus of 27.3.3.2.6. Console command '2' is the
// wiring probe that names every connection outright.
//
// Console: USART2 ALT1 (PF4/PF5) at 460800, observability only.
//   ? help | i status and counters | 0 back to command mode
//   | 1 arm one-wire standby | 2 wiring probe | 3 trace the commands
//
// The stand-off (opcode `standby`) is the other half of sharing a wire:
// while the DUT runs its SINGLE-board half this peer sits on its
// loop-back, so it is told to stay mute for the duration - it keeps
// decoding, it just answers nothing but the command that ends it.
//
// The banner reports the USERROW board label and whether THIS board's
// 24 MHz crystal started: the peer's clock is the reference the DUT's
// auto-baud and error-tolerance tests are measured against, so its
// quality is itself a bench fact.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay_basic.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

#include "usart_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;
using link::Op;

using Console = Uart<2, Route::alt1>;
constexpr Console console;

using Link = Usart<4>;
using LinePin = Pin<'E', 0>;      // TXD pad: the bit-bang and break line
using RxdPin = Pin<'E', 1>;       // RXD pad: DRIVE_RXD drives the DUT's TXD
using XckPin = Pin<'E', 2>;

constexpr uint16_t firmware_version = 0x0201;

bool crystal_ok = false;
/// Which wiring the desk turns out to have (usart_link.hpp Topology).
/// The peer alternates until a command frame arrives, believes that
/// answer while the channel keeps working, and goes back to alternating
/// once it has been silent for link::rediscover_ms. Nothing here is ever
/// latched for good: a desk that gets re-jumpered under two running
/// firmwares has to converge by itself, and a topology that is believed
/// for ever cannot.
link::Topology topo = link::Topology::full_duplex;
bool proven = false;
link::Decoder decoder;
link::Report last_report;
uint32_t commands = 0, naks = 0, actions = 0;
bool one_wire_standby = false;

/// When the acknowledgement of the command being executed left the
/// line. The DUT reconfigures link::settle_ms after that same instant,
/// so every action of this peer waits a little longer before touching
/// the wire: whoever transmits first must be sure the other end is
/// already listening in the new configuration.
uint32_t ack_ms = 0;
uint32_t last_byte_ms = 0;
uint32_t last_frame_ms = 0;   ///< when a whole command frame last decoded
uint32_t last_flip_ms = 0;    ///< when the discovery last changed topology
uint32_t standby_until = 0;   ///< mute until this millisecond (0 = answering)

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(link::settle_ms) + 2u) {
    }
}

// ---- small conversions --------------------------------------------------------

UsartBits bits_of(uint8_t n) {
    switch (n) {
        case 5: return UsartBits::five;
        case 6: return UsartBits::six;
        case 7: return UsartBits::seven;
        case link::bits9_low: return UsartBits::nine_low_first;
        case link::bits9_high: return UsartBits::nine_high_first;
        default: return UsartBits::eight;
    }
}

UsartParity parity_of(uint8_t n) {
    switch (n) {
        case 1: return UsartParity::even;
        case 2: return UsartParity::odd;
        default: return UsartParity::none;
    }
}

uint16_t data_mask(uint8_t bits) {
    const uint8_t n = bits >= link::bits9_low ? 9 : bits;
    return static_cast<uint16_t>((1u << n) - 1u);
}

/// CPU cycles of one bit at a rate - the unit every bit-banged shape
/// here is built from, never a magic number.
uint32_t cycles_per_bit(uint32_t rate) {
    if (rate == 0) return 2500;
    return SysClock::hz / rate;
}

void set_inven(char port, uint8_t pin, bool on) {
    volatile uint8_t& c = pinctrl_of(port, pin);
    if (on) c |= PORT_INVEN_bm;
    else c &= static_cast<uint8_t>(~PORT_INVEN_bm);
}

// ---- the link ------------------------------------------------------------------

/// True while the link is a single wire between the two TXD pads.
bool shared() { return topo == link::Topology::shared; }

/// May the transmitter sit enabled BETWEEN frames? Only on the crossed
/// pair, where this board's TXD wire reaches nothing but the other
/// board's RXD - and only once the topology has been PROVEN by a
/// decoded frame. While discovery is still guessing, the peer drives
/// nothing at all: a probe that idled its TXD high would, on a shared
/// desk, fight the other board's transmitter for as long as it guessed
/// wrong.
bool tx_idle_on() { return !shared() && proven; }

/// Half-duplex turnaround guard. RXCIF is raised at the MIDDLE of the
/// stop bit - half a bit BEFORE the sender's TXCIF - so an end that
/// answers at once starts its start bit while the other end is still
/// transmitting with its receiver off, and the answer's first bits are
/// swallowed (measured: two bits lost at 2400 baud, the receiver then
/// locking onto a later low). Wait four bit times, derived from the
/// BAUD register in force, before taking the line.
void turnaround_guard() {
    const uint32_t bit_cycles =
        (static_cast<uint32_t>(Link::baud_reg()) * Link::samples()) / 64u;
    delay_cycles(4u * (bit_cycles ? bit_cycles : 1u));
}

/// Take the line to answer. On a shared wire that also means muting the
/// receiver, which LBME would otherwise feed with our own echo. The pin
/// and the transmitter are claimed UNCONDITIONALLY: they are already
/// ours whenever tx_idle_on(), and the one case that needs the claim is
/// answering out of a non-driving discovery configuration - where
/// command_mode() deliberately left the transmitter off.
void line_talk() {
    turnaround_guard();
    if (shared()) Link::enable_rx(false);
    PORTE.OUTSET = LinePin::mask;
    PORTE.DIRSET = LinePin::mask;
    Link::enable_tx(true);
    Link::clear_txc();          // so line_listen waits for THIS burst
}

/// Turnaround the other way: let the last frame leave, hand the wire
/// back to the pull-up and listen again. It OWNS the wait for the line
/// to go idle - TXCIF is write-one-to-clear, so a caller that waits too
/// and then calls this would spin out its whole budget deaf.
void line_listen(bool wait = true) {
    if (wait) (void)Link::wait_line_idle(200'000u);
    if (!tx_idle_on()) {
        Link::enable_tx(false);
        PORTE.DIRCLR = LinePin::mask;
        pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    }
    if (shared()) {
        Link::flush_rx();
        Link::enable_rx(true);
    }
}

/// Command mode: async 8N1 at link::command_baud. On the full-duplex
/// wiring both directions are simply on; on a shared line the receiver
/// listens to the TXD pad through LBME and the transmitter is taken up
/// only for the duration of an answer.
bool command_mode() {
    set_inven('E', 2, false);
    const bool ok = Link::init({.route = UsartRoute::def,
                                .baud = usart_baud_reg(SysClock::hz, link::command_baud),
                                .tx = tx_idle_on(),
                                .loop_back = shared()});
    if (shared() || !proven) {
        PORTE.DIRCLR = LinePin::mask;
        pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    } else {
        // The idle level of an input that nobody drives is nobody's
        // business but the pull-up's: a floating RXD invents frames.
        pinctrl_of('E', 1) |= PORT_PULLUPEN_bm;
    }
    Link::flush_rx();
    Link::clear_txc();
    decoder.reset();
    one_wire_standby = false;
    return ok;
}

/// Apply a commanded configuration. False (and the link left as it was
/// found, in command mode) when this silicon cannot express it.
bool apply_cfg(const link::Cfg& c) {
    if (!c.apply) return true;
    UsartConfig u{};
    u.route = UsartRoute::def;
    switch (c.mode) {
        case link::Mode::sync_client:
        case link::Mode::sync_host: u.mode = UsartMode::sync; break;
        case link::Mode::ircom: u.mode = UsartMode::ircom; break;
        default: u.mode = UsartMode::async; break;
    }
    u.bits = bits_of(c.bits);
    u.parity = parity_of(c.parity);
    u.two_stop = c.stop >= 2;
    u.rx_mode = (c.flags & link::flag_clk2x) ? UsartRxMode::clk2x : UsartRxMode::normal;
    u.sync_client = c.mode == link::Mode::sync_client;
    u.multiprocessor = (c.flags & link::flag_mpcm) != 0;
    u.tx_pulse = c.txpl;
    u.rx_pulse = c.rxpl;
    if (shared()) {
        u.loop_back = true;      // the receiver listens to the TXD pad
        u.tx = false;            // the wire is taken only to answer
    }
    const uint8_t s = usart_samples(u.mode, u.rx_mode);
    u.baud = usart_baud_reg(SysClock::hz, c.rate, s);
    if (u.baud == 0) {
        if (u.sync_client) u.baud = 64;         // a client ignores BAUD entirely
        else return false;
    }
    if (!Link::init(u)) return false;
    set_inven('E', 2, (c.flags & link::flag_invert_xck) != 0);
    if (shared()) {
        PORTE.DIRCLR = LinePin::mask;
        pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    }
    Link::flush_rx();
    return true;
}

void put_link(uint8_t b) { (void)Link::send(b, 200'000u); }

void answer(Op op, const uint8_t* p, uint8_t len) {
    line_talk();
    link::write_frame(put_link, op, p, len);
    line_listen();
}

void ack(Op op, uint8_t sum, bool good) {
    const uint8_t p[2] = {link::byte_of(op), sum};
    answer(good ? Op::ack : Op::nak, p, 2);
}

// ---- accounting ----------------------------------------------------------------

void account(link::Report& r, const UsartFrame& f) {
    if (f.frame_error && r.ferr < 255) ++r.ferr;
    if (f.parity_error && r.perr < 255) ++r.perr;
    if (f.overflow && r.ovf < 255) ++r.ovf;
    r.sum = static_cast<uint16_t>(r.sum + f.data);
    ++r.count;
}

// ---- bit-banged shapes (what a clean UART cannot do) ---------------------------
//
// One _delay_loop_2 iteration is four cycles; the port write and the
// loop around it cost about a dozen more, taken off here. The DUT
// measures every shape back through a TCB pulse-width meter, so the
// absolute calibration matters less than the RATIO between a nominal
// cell and a distorted one - and that ratio is exact, both coming from
// this same routine.

[[gnu::always_inline]] inline void bb_wait(uint16_t cycles) {
    _delay_loop_2(cycles > 20 ? static_cast<uint16_t>((cycles - 12) / 4) : 2);
}

/// The width of bit cell `idx` (0 = start, 1..n = data, n+1 = stop),
/// stretched or shrunk by `pct` when it is the chosen one.
uint16_t cell_cycles(uint32_t nominal, int8_t cell, int8_t pct, int8_t idx) {
    int32_t v = static_cast<int32_t>(nominal);
    if (cell == idx && pct != 0) {
        v += static_cast<int32_t>((static_cast<int32_t>(nominal) * pct) / 100);
    }
    if (v < 24) v = 24;
    if (v > 60'000) v = 60'000;
    return static_cast<uint16_t>(v);
}

/// One 8N1 frame, LSB first, on `bm` of PORTE. Interrupts off: this is
/// the only place in the peer where a cycle is a unit of measurement.
/// The ten cell widths are worked out BEFORE the timed section -
/// cell_cycles() multiplies and divides 32-bit values, which inside the
/// loop cost about a hundred cycles a cell and showed up on the DUT as a
/// four to five per cent baud error at 9600.
void bb_frame(uint8_t bm, uint32_t nominal, uint8_t value, int8_t cell, int8_t pct) {
    uint16_t w[10];
    for (int8_t i = 0; i < 10; ++i) w[i] = cell_cycles(nominal, cell, pct, i);
    const uint8_t saved = SREG;
    cli();
    PORTE.OUTCLR = bm;                                    // start bit
    bb_wait(w[0]);
    for (uint8_t i = 0; i < 8; ++i) {
        if (value & (1u << i)) PORTE.OUTSET = bm;
        else PORTE.OUTCLR = bm;
        bb_wait(w[i + 1]);
    }
    PORTE.OUTSET = bm;                                    // stop bit
    bb_wait(w[9]);
    SREG = saved;
}

/// `pulses` low pulses of `width` CPU cycles on an otherwise idle line,
/// `gap_us` apart: the false-start-rejection probe.
void bb_glitch(uint8_t bm, uint16_t pulses, uint16_t width, uint8_t gap_us) {
    // _delay_loop_1 counts in a BYTE (three cycles an iteration), so it
    // tops out around 765 cycles: anything wider goes through the 16-bit
    // loop instead. Below that it is the only way to shape a pulse a few
    // cycles wide.
    const bool fine = width <= 200;
    const uint8_t loops = width >= 6 ? static_cast<uint8_t>((width - 3) / 3) : 1;
    for (uint16_t i = 0; i < pulses; ++i) {
        const uint8_t saved = SREG;
        cli();
        PORTE.OUTCLR = bm;
        if (fine) _delay_loop_1(loops);
        else delay_cycles(width - 8u);
        PORTE.OUTSET = bm;
        SREG = saved;
        delay_us_runtime(cycles_per_us(SysClock::hz), gap_us ? gap_us : 1u);
    }
}

/// Take one PORTE pin away from the USART and hold it idle high.
void claim_line(uint8_t bm) {
    PORTE.OUTSET = bm;
    Link::release();
    PORTE.OUTSET = bm;
    PORTE.DIRSET = bm;
    delay_us(clock, 200);
}

// ---- the actions ----------------------------------------------------------------

link::Report run_echo(const link::Params& a, bool sink) {
    link::Report r{};
    settle();
    // Whatever the DUT's own reconfiguration put on the line while this
    // end was already listening is not traffic: a released pad dips, and
    // a dip is a start bit. The DUT holds off until after this point.
    Link::flush_rx();
    const uint32_t t0 = Ticker::millis();
    while (r.count < a.count) {
        if (Link::rxc_flag()) {
            const UsartFrame f = Link::receive();
            account(r, f);
            if (!sink) {
                line_talk();
                (void)Link::send(f.data, 200'000u);
                line_listen();
            }
            continue;
        }
        if (Ticker::millis() - t0 >= a.ms) {
            r.flags |= link::report_timed_out;
            break;
        }
    }
    line_listen();
    return r;
}

link::Report run_send(const link::Params& a, bool blast) {
    link::Report r{};
    settle();
    // `send` reads aux16 as EXTRA lead-in milliseconds: a DUT that has to
    // stand up an interrupt-driven transport first needs more than the
    // rendezvous window.
    if (a.aux16) delay_us_runtime(cycles_per_us(SysClock::hz),
                                  static_cast<uint32_t>(a.aux16) * 1000u);
    const uint16_t mask = data_mask(a.cfg.bits);
    const uint32_t gap = blast ? 0u : (2u * 1'000'000u) / (a.cfg.rate ? a.cfg.rate : 1u);
    line_talk();
    for (uint16_t i = 0; i < a.count; ++i) {
        const uint16_t v = static_cast<uint16_t>(link::pattern_value(a.pattern, a.value, i)) & mask;
        if (!Link::send(v, 200'000u)) break;
        r.sum = static_cast<uint16_t>(r.sum + v);
        ++r.count;
        if (gap) delay_us_runtime(cycles_per_us(SysClock::hz), gap);
    }
    line_listen();
    return r;
}

/// Address frame (ninth bit set) followed by its data frames, per group.
/// The groups are (address, count) pairs appended to the payload.
link::Report run_send_mpcm(const link::Params& a, const uint8_t* pairs) {
    link::Report r{};
    settle();
    uint8_t seq = a.value;
    line_talk();
    for (uint8_t g = 0; g < a.groups; ++g) {
        const uint16_t addr = static_cast<uint16_t>(0x100u | pairs[2 * g]);
        if (!Link::send(addr, 200'000u)) break;
        r.sum = static_cast<uint16_t>(r.sum + addr);
        ++r.count;
        for (uint8_t k = 0; k < pairs[2 * g + 1]; ++k) {
            const uint16_t v = seq++;                       // ninth bit clear: data
            if (!Link::send(v, 200'000u)) break;
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++r.count;
        }
    }
    line_listen();
    return r;
}

/// TXD held low for `count` bit times, from PORT with the transmitter
/// off: the only way to make a break longer than a character.
link::Report run_break(const link::Params& a) {
    link::Report r{};
    settle();
    const uint32_t bit_cyc = cycles_per_bit(a.cfg.rate);
    line_talk();
    Link::enable_tx(false);
    PORTE.OUTSET = LinePin::mask;
    PORTE.DIRSET = LinePin::mask;
    delay_cycles(bit_cyc * 2u);
    PORTE.OUTCLR = LinePin::mask;
    delay_cycles(bit_cyc * a.count);
    PORTE.OUTSET = LinePin::mask;
    delay_cycles(bit_cyc * 2u);
    Link::enable_tx(true);
    line_listen(/*wait=*/false);        // nothing was transmitted, only a level held
    r.count = a.count;
    r.aux16 = Link::baud_reg();
    return r;
}

/// The foreign sender: a break of `aux16` bit times, a sync character
/// (0x55 or a deliberately wrong one) and `aux8` payload bytes, all at
/// a rate this board computes from ITS OWN crystal.
link::Report run_autobaud_tx(const link::Params& a) {
    link::Report r{};
    settle();
    const uint32_t bit_cyc = cycles_per_bit(a.cfg.rate);
    line_talk();
    Link::enable_tx(false);
    PORTE.OUTSET = LinePin::mask;
    PORTE.DIRSET = LinePin::mask;
    delay_cycles(bit_cyc * 4u);
    PORTE.OUTCLR = LinePin::mask;
    delay_cycles(bit_cyc * (a.aux16 ? a.aux16 : 16u));
    PORTE.OUTSET = LinePin::mask;
    delay_cycles(bit_cyc * 2u);
    Link::enable_tx(true);
    (void)Link::send(a.value, 200'000u);                 // the sync field
    r.sum = a.value;
    ++r.count;
    for (uint8_t k = 0; k < a.aux8; ++k) {
        const uint8_t v = static_cast<uint8_t>(0xA0 + k);
        if (!Link::send(v, 200'000u)) break;
        r.sum = static_cast<uint16_t>(r.sum + v);
        ++r.count;
    }
    r.aux16 = Link::baud_reg();                          // the peer's own divisor
    line_listen();
    return r;
}

link::Report run_bitbang(const link::Params& a, uint8_t bm) {
    link::Report r{};
    const uint32_t nominal = cycles_per_bit(a.cfg.rate);
    claim_line(bm);
    settle();
    if (a.aux16 != 0) {
        // A break field ahead of the frames: the auto-baud probes need
        // the break and the (distorted) sync character to come from the
        // same cycle-counted generator, with no command frame between.
        PORTE.OUTCLR = bm;
        delay_cycles(nominal * a.aux16);
        PORTE.OUTSET = bm;
        delay_cycles(nominal * 2u);
    }
    for (uint16_t i = 0; i < a.count; ++i) {
        bb_frame(bm, nominal, a.value, a.cell, a.pct);
        delay_cycles(nominal * 4u);                      // a clean idle between frames
        r.sum = static_cast<uint16_t>(r.sum + a.value);
        ++r.count;
    }
    r.aux16 = static_cast<uint16_t>(nominal);
    return r;
}

link::Report run_glitch(const link::Params& a, uint8_t bm) {
    link::Report r{};
    claim_line(bm);
    settle();
    bb_glitch(bm, a.count, a.aux16, a.aux8);
    r.count = a.count;
    r.aux16 = a.aux16;
    return r;
}

/// Half duplex on the shared line: the peer joins with LBME + ODME and
/// answers every byte it hears, taking the line only to reply. Needs
/// the PE0-PE0 jumper, not the crossed pair.
link::Report run_onewire(const link::Params& a) {
    link::Report r{};
    UsartConfig u{};
    u.route = UsartRoute::def;
    u.baud = usart_baud_reg(SysClock::hz, a.cfg.rate ? a.cfg.rate : link::command_baud);
    u.loop_back = true;
    u.open_drain = true;
    u.tx = false;                                        // listening first
    if (!Link::init(u)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    settle();
    const uint32_t t0 = Ticker::millis();
    while (r.count < a.count) {
        if (Link::rxc_flag()) {
            const UsartFrame f = Link::receive();
            account(r, f);
            Link::enable_tx(true);                       // talk
            (void)Link::send(static_cast<uint16_t>(f.data ^ 0xFFu), 200'000u);
            (void)Link::wait_line_idle();
            Link::enable_tx(false);                      // listen
            Link::flush_rx();                            // drop our own echo
            continue;
        }
        if (Ticker::millis() - t0 >= a.ms) {
            r.flags |= link::report_timed_out;
            break;
        }
    }
    return r;
}

/// Wait for one trigger frame, then transmit into the DUT's own frame
/// after a commanded delay: the collision the one-wire echo must catch.
link::Report run_collide(const link::Params& a) {
    link::Report r{};
    UsartConfig u{};
    u.route = UsartRoute::def;
    u.baud = usart_baud_reg(SysClock::hz, a.cfg.rate ? a.cfg.rate : link::command_baud);
    u.loop_back = true;
    u.open_drain = true;
    if (!Link::init(u)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    settle();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        if (Link::rxc_flag()) {
            const UsartFrame f = Link::receive();
            account(r, f);
            delay_us_runtime(cycles_per_us(SysClock::hz), a.aux16);
            (void)Link::send(a.value, 200'000u);
            (void)Link::wait_line_idle();
            r.aux16 = a.value;
            return r;
        }
    }
    r.flags |= link::report_timed_out;
    return r;
}

// ---- the command handler --------------------------------------------------------

bool trace = false;

void handle(const link::Frame& f) {
    ++commands;
    const Op op = f.op;
    if (trace) {
        print(console, "  [cmd op=", hex(link::byte_of(op)), " len=", f.len,
              " topo=", shared() ? "sh" : "fd", " proven=", proven ? 1 : 0,
              " standby=", standby_until != 0 ? 1 : 0, "]", crlf);
    }

    // STAND-OFF. On a SHARED line this peer sits on the DUT's own
    // loop-back, so while the DUT runs its single-board half the one
    // thing that must not happen is this board ANSWERING something. A
    // stand-off therefore does not release anything and does not stop
    // decoding: it just makes the peer mute to every command but the one
    // that ends it (`standby` with ms = 0). Muteness is the whole
    // requirement - a listener drives nothing.
    if (op == Op::standby) {
        const uint16_t ms = f.len >= link::params_size ? link::get_params(f.data).ms : 0;
        standby_until = ms ? Ticker::millis() + ms : 0;
        ack(op, f.sum, true);
        return;
    }
    if (standby_until != 0) {
        if (Ticker::millis() < standby_until) return;    // deaf AND mute
        standby_until = 0;
    }

    if (op == Op::ping) {
        ack(op, f.sum, true);
        return;
    }
    if (op == Op::ident) {
        ack(op, f.sum, true);
        link::Ident d{};
        auto board = board_id();
        for (uint8_t i = 0; i < 8 && i < board.size(); ++i) d.label[i] = board[i];
        d.xtal = crystal_ok ? 1 : 0;
        d.sanity = link::ident_sanity;
        d.version = firmware_version;
        uint8_t p[link::ident_size];
        link::put_ident(p, d);
        answer(Op::ident_data, p, link::ident_size);
        return;
    }
    if (op == Op::report) {
        ack(op, f.sum, true);
        uint8_t p[link::report_size];
        link::put_report(p, last_report);
        answer(Op::report_data, p, link::report_size);
        return;
    }

    if (f.len < link::params_size) {
        ack(op, f.sum, false);
        ++naks;
        return;
    }
    ack(op, f.sum, true);
    ack_ms = Ticker::millis();
    ++actions;

    const link::Params a = link::get_params(f.data);
    link::Report r{};
    const uint32_t started = Ticker::millis();

    // The ack has left the line (answer() waits for TXCIF): the link may
    // change now. The DUT waits link::settle_ms before doing the same,
    // so the peer is always ready first.
    const bool bitbanged = op == Op::bitbang || op == Op::glitch || op == Op::drive_rxd;
    const bool own_config = op == Op::onewire || op == Op::collide;
    bool ok = true;
    if (!bitbanged && !own_config) ok = apply_cfg(a.cfg);

    if (!ok) {
        r.flags |= link::report_cfg_failed;
    } else {
        switch (op) {
            case Op::set_link: {
                settle();
                const uint32_t t0 = Ticker::millis();
                while (Ticker::millis() - t0 < a.ms) {
                    if (Link::rxc_flag()) { account(r, Link::receive()); }
                }
                r.flags |= link::report_timed_out;
                break;
            }
            case Op::echo: r = run_echo(a, false); break;
            case Op::sink: r = run_echo(a, true); break;
            case Op::send: r = run_send(a, false); break;
            case Op::blast: r = run_send(a, true); break;
            case Op::send_mpcm: r = run_send_mpcm(a, f.data + link::params_size); break;
            case Op::brk: r = run_break(a); break;
            case Op::autobaud_tx: r = run_autobaud_tx(a); break;
            case Op::bitbang: r = run_bitbang(a, LinePin::mask); break;
            case Op::glitch: r = run_glitch(a, LinePin::mask); break;
            // The DUT's TXD pad is reached through the peer's RXD pin on
            // the crossed wiring and through the peer's own TXD pin on a
            // shared line: either way this drives THE OTHER BOARD's TXD.
            case Op::drive_rxd:
                r = run_bitbang(a, shared() ? LinePin::mask : RxdPin::mask);
                break;
            case Op::onewire: r = run_onewire(a); break;
            case Op::collide: r = run_collide(a); break;
            default: r.flags |= link::report_cfg_failed; break;
        }
        r.flags |= link::report_ran;
    }

    const uint32_t took = Ticker::millis() - started;
    r.ms = took > 255u ? 255u : static_cast<uint8_t>(took);
    r.op = link::byte_of(op);
    last_report = r;
    (void)command_mode();
}

// ---- the console ------------------------------------------------------------------

void help() {
    print(console, "usart_peer: ? help | i status | 0 command mode | 1 one-wire standby "
                  "| 2 wiring probe | 3 trace", crlf);
}

void status() {
    print(console, "  topology=", shared() ? "shared line (PE0-PE0)"
                                           : "full duplex (crossed pair)",
          proven ? " PROVEN" : " searching", crlf);
    print(console, "  link route=", static_cast<uint8_t>(Link::routed()),
          " BAUD=", Link::baud_reg(), " mode=", static_cast<uint8_t>(Link::mode()),
          one_wire_standby ? " (one-wire standby)" : "", crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          standby_until != 0 && Ticker::millis() < standby_until ? "  (STAND-OFF)" : "",
          crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " sum=", hex(last_report.sum), " ferr=", last_report.ferr,
          " perr=", last_report.perr, " ovf=", last_report.ovf,
          " flags=", hex(last_report.flags), " aux=", last_report.aux16,
          " ms=", last_report.ms, crlf);
}

/// Wiring probe. A desk whose jumpers are missing, loose or straight
/// instead of crossed looks exactly like a dead protocol from inside the
/// protocol, so this asks the question the wires themselves answer:
/// PE0..PE3 are driven at four different rates (2, 4, 8, 16 Hz) for six
/// seconds and then listened to for six more, and the EDGE COUNT on a
/// pin names which pin of the other board it is tied to. Run it together
/// with the DUT's own probe (test_avr_serial command 'v'), which does
/// the two phases in the opposite order.
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

/// Which driven pin an edge count names, or -1 when the pin heard nothing.
int8_t probe_source(uint16_t edges) {
    if (edges < 8) return -1;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t want = probe_ms / probe_half_ms[i];
        if (edges * 4u >= want * 3u && edges * 3u <= want * 4u) return static_cast<int8_t>(i);
    }
    return -2;                                   // heard something, but not one driver
}

void wire_probe() {
    Link::release();
    print(console, "  driving PE0..PE3 at 2/4/8/16 Hz for 6 s, then listening 6 s", crlf);
    probe_drive();
    uint16_t edges[4] = {0, 0, 0, 0};
    probe_listen(edges);
    for (uint8_t i = 0; i < 4; ++i) {
        const int8_t src = probe_source(edges[i]);
        print(console, "  B.PE", i, ": ", edges[i], " edges -> ");
        if (src >= 0) print(console, "A.PE", static_cast<uint8_t>(src), crlf);
        else if (src == -1) print(console, "nothing", crlf);
        else print(console, "several or noise", crlf);
    }
    (void)command_mode();
}

void one_wire_standby_mode() {
    UsartConfig u{};
    u.route = UsartRoute::def;
    u.baud = usart_baud_reg(SysClock::hz, link::command_baud);
    u.loop_back = true;
    u.open_drain = true;
    u.tx = false;
    one_wire_standby = Link::init(u);
    print(console, one_wire_standby ? "  one-wire standby on PE0 (needs the PE0-PE0 jumper)"
                                    : "  one-wire config refused",
          crlf);
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
    print(console, crlf, "usart_peer - USART instrument peer (board ", board,
          ", clk=", crystal_ok ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ",
          hex(SYSCFG.REVID), ", fw ", hex(firmware_version), ")", crlf);
    print(console, "link USART4 default: TXD PE0, RXD PE1, XCK PE2; command mode 8N1 @ ",
          link::command_baud, crlf);
    print(console, "waiting for the DUT: the wiring (crossed pair or shared PE0 line) is "
                   "found by alternating", crlf);
    if (!crystal_ok) {
        print(console, "WARNING: the 24 MHz crystal did NOT start - every rate on this "
                       "board comes from OSCHF", crlf);
    }
    (void)command_mode();
    last_frame_ms = Ticker::millis();
    last_flip_ms = last_frame_ms;
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
                    // The manual way back: forget the topology, drop any
                    // stand-off, and start looking again from scratch.
                    proven = false;
                    standby_until = 0;
                    one_wire_standby = false;
                    decoder.reset();
                    (void)command_mode();
                    last_flip_ms = Ticker::millis();
                    print(console, "  command mode, discovery restarted", crlf);
                }
                else if (c == '1') one_wire_standby_mode();
                else if (c == '2') wire_probe();
                else if (c == '3') { trace = !trace; print(console, trace ? "  trace on" : "  trace off", crlf); }
                else print(console, "? for help", crlf);
                print(console, "> ");
            }
        }
        if (one_wire_standby) continue;      // the DUT owns the line now

        const uint32_t now = Ticker::millis();
        if (standby_until != 0 && now >= standby_until) standby_until = 0;

        // RENDEZVOUS, and RE-rendezvous. A topology is believed only
        // while the command channel keeps proving it: once it has been
        // silent for link::rediscover_ms the peer stops believing and
        // goes back to alternating the two wirings until a frame decodes
        // again. That is what lets a desk be re-jumpered under two
        // running firmwares. A stand-off is the one silence that is
        // expected, so it suspends all of this - flipping topology under
        // a DUT that asked for quiet is exactly what the stand-off is
        // there to prevent.
        if (standby_until == 0) {
            if (proven && now - last_frame_ms > link::rediscover_ms) {
                proven = false;
                last_flip_ms = now;
                (void)command_mode();    // stop driving: the wiring is unknown again
            }
            if (!proven && now - last_flip_ms > link::rendezvous_ms) {
                topo = topo == link::Topology::full_duplex ? link::Topology::shared
                                                           : link::Topology::full_duplex;
                (void)command_mode();
                last_flip_ms = now;
            }
        }
        // A frame that stops half way (a desync, a reconfiguration in the
        // middle of it) must not eat the next one: a quiet line for longer
        // than any inter-byte gap resets the reassembly. This is the other
        // half of the recovery guarantee.
        if (decoder.partial() && Ticker::millis() - last_byte_ms > 50u) {
            decoder.reset();
        }
        if (Link::rxc_flag()) {
            const UsartFrame f = Link::receive();
            last_byte_ms = Ticker::millis();
            if (!f.clean()) {
                decoder.reset();
                continue;
            }
            switch (decoder.feed(static_cast<uint8_t>(f.data))) {
                case link::Decoder::Result::frame:
                    proven = true;
                    handle(decoder.frame());
                    last_byte_ms = Ticker::millis();
                    last_frame_ms = last_byte_ms;
                    break;
                case link::Decoder::Result::bad_checksum:
                    ack(decoder.pending_op(), decoder.frame().sum, false);
                    ++naks;
                    break;
                default: break;
            }
        }
    }
}
