// usart_peer - the INSTRUMENT half of a two-board USART test on the SAM
// C21: the scriptable peer test_avr_serial (the DUT) drives IN BAND over
// the very link under test, speaking usart_link.hpp - the one wire format
// every architecture on this link compiles.
//
// Not a kernel app: one blocking loop that polls SERCOM1, decodes a
// command frame, acknowledges it and becomes for a bounded moment what the
// DUT needs on the other end - an echo, a silent sink, a generator, a
// flood, a break, a foreign auto-baud sender, a bit-banger no clean UART
// could be. Every mode-changing command carries a frame count and a
// millisecond deadline, after which the peer restores command mode BY
// ITSELF (the protocol's recovery guarantee).
//
// LINK: SERCOM1 function C on the desk's four wires, the crossed pair -
//   PA16 = PAD[0]  RxD   (from the DUT's TxD)
//   PA18 = PAD[2]  TxD   (to the DUT's RxD; TXPO 0x1)
//   PA19 = PAD[3]  XCK   (the synchronous roles only, beside TxD)
//   PA17 = PAD[1]  unused by the USART (TXPO 0x1 does not take it)
// NO TOPOLOGY IS GUESSED: this desk fixes the crossed pair, so the peer
// is full duplex always and goes onto the DUT's TxD wire (PA16) only
// inside onewire and collide (usart_link.hpp's Topology). A peer that
// guessed "shared" would answer on PA16 while the DUT's full-duplex TxD
// drives it high.
//
// WHAT THIS SILICON MAKES OF THE REPERTOIRE (docs/samc21/sercom.md has the
// chapter's offer, item by item):
//  - every rate from the CRYSTAL: the CPU stays on OSC48M at 48 MHz, but
//    SERCOM1's core clock is GCLK generator 2 on the 24 MHz XOSC, so the
//    reference every BAUD is computed from is the crystal's. Per rate the
//    generator CLOSEST to it of table 30-2's two (sercom_closest_baud():
//    3750 baud is exact on the fractional one, 921600 on the arithmetic
//    one); 16 samples a bit, 8 under flag_clk2x - what 2 Mbaud needs on a
//    24 MHz reference (16x tops at 1.5 Mbaud);
//  - nine-bit frames in both of the protocol's orders map to CHSIZE 9:
//    the AVR's two orders differ in which data register shifts its FIFO,
//    not on the wire;
//  - the synchronous roles on PA19 (CMODE, the internal clock as host
//    with the 8-bit BAUD of 31.6.2.3, the external one as client, CPOL
//    under flag_invert_xck - the AVR's INVEN on XCK is the same edge
//    swap, DS40002247B 27.3.2.3.1 against DS60001479M 31.6.2.3.1);
//  - IrDA at 16x arithmetic (31.6.3.3), RXPL from the Cfg's rxpl. The
//    coder TRANSMITS 3/16 OF A BIT ONLY: a txpl other than 0 (the AVR's
//    fixed-pulse TXPLCTRL) is REFUSED, report_cfg_failed;
//  - flag_mpcm is refused: this SERCOM has no multiprocessor receive mode
//    (the DUT alone filters, and only it sets the flag);
//  - THE WAVEFORM ENGINE for bitbang, glitch, drive_rxd, brk and
//    autobaud_tx's break: TC0+TC1 as one 32-bit counter on the same
//    crystal generator, its two compare channels' match events carried
//    by two asynchronous EVSYS channels to the PORT's event inputs 0
//    and 1 acting CLR and SET on the pad (DS60001479M 28.6.4) - the edge
//    list precomputed, both first edges programmed before the run, the
//    CPU reloading a channel's NEXT compare between its edges. Not a
//    cycle-counted loop: the CPU runs on OSC48M (half a per cent off the
//    crystal), behind flash wait states, and a TC COUNT read costs two
//    READSYNC crossings (samc21/tc.hpp) - no polled loop could time a
//    6-tick glitch. 28.6.4: the SET and CLR actions "will be executed up
//    to three clock cycles after a rising edge" of the event - the same
//    path for both edges (two channels of one TC, two inputs of one
//    PORT), so a width carries the asynchronous crossing's jitter into
//    the PORT clock and no offset: one cycle of 48 MHz each edge, half a
//    24 MHz tick. collide's frame is open drain - the SERCOM has none -
//    so its edges are taken by the CPU on the same compare flags, as DIR
//    toggles with OUT held at 0: drive low, release;
//  - the Report's span on TC2+TC3, a free-running 32-bit count of the
//    crystal's 24 MHz ticks capturing CC0 on a SOFTWARE event (EVSYS
//    channel 0): one store a character, and one read of CC0, which is
//    what acknowledges the capture and makes room for the next (35.6.2.8:
//    a capture arriving while MC0 still stands is dropped).
//
// TIME (usart_link.hpp, TIME IN THE PAYLOAD): every tick this peer reports
// or obeys is a tick of the 24 MHz crystal - the span TC's counts, the
// engine TC's compare values. The CPU's own cycles are never a unit here.
//
// THE DARK DISCIPLINE (usart_link.hpp): while it searches, TxD's pad is
// TAKEN from the SERCOM - a PORT input with no pull - because a disabled
// transmitter still drives its pad at the idle level (CTRLB.TXEN = 0 is no
// release: measured, docs/samc21/sercom.md); and no link pad carries a
// pull - a SERCOM input cannot pull up anyway (31.5.1), so the receive pad
// has none even once proven: the DUT's transmitter idles it high. A frame
// that checks out proves the wiring and TxD's pad goes to the SERCOM;
// rediscover_ms of quiet takes it back. A bad checksum is nak'ed only
// while ENGAGED (engage_ms of the last good frame). The shared wire of
// onewire and collide follows the same rule, harder: there the SERCOM
// only listens on PA16 (its transmitter on PAD[2], not muxed), and every
// bit this board puts on the DUT's wire comes from the waveform engine
// OPEN DRAIN - pulled low or released, never driven high against the
// DUT's own open-drain transmitter.
//
// Console: SERCOM5 PB30/PB31 at 115200, observability only.
//   ? help | i status and counters | 0 back to command mode, wiring
//   unproven | 2 wiring probe | 3 trace the commands | 4 the engine's
//   self-test on PA18 (the far board dark or listening)
//
// The ident label is the die serial's first word in hex (this family has
// no USERROW label); ident.xtal reports whether the 24 MHz crystal
// started - without it generator 2 falls back to OSC48M / 2 and every
// rate on this link is half a per cent off.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc21/clock.hpp"
#include "samc21/delay.hpp"
#include "samc21/evsys.hpp"
#include "samc21/nvic.hpp"
#include "samc21/nvm.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/usart_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using link::Op;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Console = Uart<5, console_pads>;
constexpr Console console;

// ---------------------------------------------------------------------------
// The link
// ---------------------------------------------------------------------------

using Link = Sercom<1>;

using RxdPin = Pin<'A', 16>;    // PAD[0]: RxD, and the one-wire pad
using Pad1Pin = Pin<'A', 17>;   // PAD[1]: not the USART's on this desk
using TxdPin = Pin<'A', 18>;    // PAD[2]: TxD
using XckPin = Pin<'A', 19>;    // PAD[3]: XCK
using LinkPort = Port<'A'>;

static_assert(MUX_PA16C_SERCOM1_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA18C_SERCOM1_PAD2 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA19C_SERCOM1_PAD3 == static_cast<uint8_t>(PinFunction::c));

/// The crossed pair: TxD PAD[2] (TXPO 0x1, XCK beside it on PAD[3]),
/// RxD PAD[0].
constexpr UartPads link_pads{
    .tx = SercomPad::pad2,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 18, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};
static_assert(uart_xck_pad(link_pads.tx) == SercomPad::pad3);

// ---------------------------------------------------------------------------
// The crystal: generator 2, the reference of the link and of both rulers
// ---------------------------------------------------------------------------

constexpr uint8_t gen_xtal = 2;
constexpr uint32_t ref_hz = 24'000'000UL;
static_assert(ref_hz == 24'000'000UL,
              "usart_link.hpp counts time in 24 MHz ticks: the reference must be one");

/// Command mode's BAUD, folded: 115200 on the arithmetic generator.
constexpr SercomBaud command_baud = sercom_closest_baud(ref_hz, link::command_baud).value();
static_assert(command_baud.sampling == UartSampling::x16_arithmetic && command_baud.reg == 60503);

constexpr uint16_t firmware_version = 0x0201;   ///< see usart_link.hpp's Ident

// ---------------------------------------------------------------------------
// The span's ruler: TC2+TC3, capture on a software event
// ---------------------------------------------------------------------------

using Span = Tc<2>;
constexpr uint8_t ev_snap = 0;      ///< software -> TC2's event input

constexpr TcConfig span_cfg{.mode = TcMode::count32, .prescaler = TcPrescaler::div1,
                            .capture_enable = 0x1};

bool span_up() {
    if (!Span::init(gen_xtal) || !Span::configure(span_cfg) ||
        !Span::event_config(span_cfg, TcEventConfig{.action = TcEventAction::off,
                                                    .input_enable = true})) {
        return false;
    }
    if (!Evsys::connect(Span::event_user, ev_snap,
                        EventChannelConfig{.generator = 0, .path = EventPath::asynchronous})) {
        return false;
    }
    return Span::enable(true);
}

/// Both capture stages empty and the flags down, so the first stamp of
/// a run is the run's.
void span_arm() {
    (void)Span::cc32(0);
    (void)Span::cc32(0);
    Span::clear_flags(Span::match_flag(0) | Span::error_flag);
}

// ---------------------------------------------------------------------------
// The waveform engine: TC0+TC1, compare events -> EVSYS -> PORT CLR/SET
// ---------------------------------------------------------------------------

using Engine = Tc<0>;
constexpr uint8_t ev_fall = 1;      ///< TC0 MC0 -> PORT event input 0 (CLR)
constexpr uint8_t ev_rise = 2;      ///< TC0 MC1 -> PORT event input 1 (SET)

constexpr TcConfig engine_cfg{.mode = TcMode::count32, .prescaler = TcPrescaler::div1,
                              .waveform = TcWaveform::normal_frequency};

bool engine_up() {
    if (!Engine::init(gen_xtal) || !Engine::configure(engine_cfg) ||
        !Engine::event_config(engine_cfg, TcEventConfig{.match_out = 0x3})) {
        return false;
    }
    // Table 29-3: the PORT's event users take the asynchronous path only.
    if (!Evsys::connect(LinkPort::event_user(0), ev_fall,
                        EventChannelConfig{.generator = Engine::match_generator(0),
                                           .path = EventPath::asynchronous}) ||
        !Evsys::connect(LinkPort::event_user(1), ev_rise,
                        EventChannelConfig{.generator = Engine::match_generator(1),
                                           .path = EventPath::asynchronous})) {
        return false;
    }
    LinkPort::release_events();
    return Engine::enable(true);
}

/// The edge list: times in 24 MHz ticks from the run's start, strictly
/// increasing, alternating FALL (even index, channel 0) and RISE (odd,
/// channel 1) from an idle-high line, ending on a rise.
constexpr uint16_t edge_most = 256;
uint32_t edges[edge_most];
uint16_t edge_n = 0;
bool edge_overflow = false;

void edges_clear() {
    edge_n = 0;
    edge_overflow = false;
}

/// Append a level change at `t`. The level a run starts from is high, so
/// an edge is a fall at even positions and a rise at odd ones; pushing a
/// level that is already the line's is ignored.
void edge_level(uint32_t t, bool high) {
    const bool line_high = (edge_n & 1u) == 0u;
    if (high == line_high) {
        return;
    }
    if (edge_n >= edge_most) {
        edge_overflow = true;
        return;
    }
    edges[edge_n++] = t;
}

/// The engine's reload floor, in 24 MHz ticks: the shortest interval
/// between two edges of ONE channel (edge i and i + 2) the CPU is given to
/// write the channel's next compare. THE RELOAD, COUNTED IN THE RELEASE
/// LISTING: the flag seen, about fifty instructions to the next poll - the
/// flag's clear, the CC store, two calls down to clock_wait() for the CC's
/// SYNCBUSY (a few 24 MHz cycles), the pad read back and the loop - some
/// 100 cycles at 48 MHz behind the flash's wait states, about 50 ticks; a
/// poll turn while waiting is some twenty instructions with the
/// millisecond bound. The floor leaves four reloads, or a SysTick entry
/// and one.
constexpr uint32_t reload_floor_ticks = 240;   // 10 us

/// The run's lead: from the COUNT read (two READSYNC crossings, about 300
/// CPU cycles) to the first edge, with both first compares written.
constexpr uint32_t lead_floor_ticks = 960;     // 40 us

/// THE ENGINE CHECKS ITSELF: after an edge whose successor is at least
/// this far away, the CPU reads the pad back (PORT IN) and the run is
/// flagged when it is not at the level the edge was for - the event path
/// to the PORT proven on every run, not assumed. A closer successor (a
/// glitch a few ticks wide) may already have moved the pad by the read.
constexpr uint32_t verify_gap_ticks = 120;   // 5 us
uint32_t edges_verified = 0;
uint32_t edges_wrong = 0;

enum class Drive : uint8_t {
    out,   ///< the PORT's CLR/SET event actions on OUT: push-pull, hardware-timed
    dir,   ///< the CPU on each compare flag: DIR set (OUT = 0) and cleared - open drain
};

/// Is the list a waveform the engine can play?
bool edges_playable() {
    if (edge_overflow || edge_n < 2u || (edge_n & 1u) != 0u) {
        return false;
    }
    for (uint16_t i = 1; i < edge_n; ++i) {
        if (edges[i] <= edges[i - 1]) {
            return false;
        }
    }
    for (uint16_t i = 2; i < edge_n; ++i) {
        if (edges[i] - edges[i - 2] < reload_floor_ticks) {
            return false;
        }
    }
    return true;
}

/// Play the list on pin P, `lead` ticks from now. The pad is the caller's
/// to prepare: under PORT, OUT and DIR set (high) for Drive::out; OUT
/// clear and DIR clear (released) for Drive::dir. False when the list is
/// not playable or an edge was not seen in time (a reload missed its
/// window: the run is broken, and the report says so).
template <typename P>
bool play(Drive drive, uint32_t lead = lead_floor_ticks) {
    if (!edges_playable()) {
        return false;
    }
    if (lead < lead_floor_ticks) {
        lead = lead_floor_ticks;
    }
    if (drive == Drive::out) {
        (void)LinkPort::configure_event(0, PortEventConfig{.pin = P::pin_number,
                                                           .action = PortEventAction::clear,
                                                           .enable = true});
        (void)LinkPort::configure_event(1, PortEventConfig{.pin = P::pin_number,
                                                           .action = PortEventAction::set,
                                                           .enable = true});
    }
    const uint32_t base = Engine::count32() + lead;
    (void)Engine::set_cc32(0, base + edges[0]);
    (void)Engine::set_cc32(1, base + edges[1]);
    Engine::clear_flags(Engine::match_flag(0) | Engine::match_flag(1));

    // The wall bound: the whole run plus a margin, on the millisecond.
    const uint32_t run_ms = (lead + edges[edge_n - 1u]) / (ref_hz / 1000u) + 3u;
    const uint32_t t0 = Ticker::millis();
    bool ok = true;
    for (uint16_t i = 0; i < edge_n; ++i) {
        const uint8_t ch = static_cast<uint8_t>(i & 1u);
        const uint8_t flag = Engine::match_flag(ch);
        while ((Engine::flags() & flag) == 0u) {
            if (Ticker::millis() - t0 > run_ms) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            break;
        }
        Engine::clear_flags(flag);
        if (drive == Drive::dir) {
            if (ch == 0u) {
                P::output();          // OUT is 0: drive the wire low
            } else {
                P::input();           // release it to the far end's pull-up
            }
        }
        if (static_cast<uint16_t>(i + 2u) < edge_n) {
            (void)Engine::set_cc32(ch, base + edges[i + 2u]);
        }
        const bool far = static_cast<uint16_t>(i + 1u) >= edge_n ||
                         edges[i + 1u] - edges[i] >= verify_gap_ticks;
        if (drive == Drive::out && far) {
            ++edges_verified;
            if (P::read() != (ch == 1u)) {
                ++edges_wrong;
                ok = false;
            }
        }
    }
    LinkPort::release_events();
    if (drive == Drive::dir) {
        P::input();
    } else {
        P::set();
    }
    return ok;
}

// ---------------------------------------------------------------------------
// Waits and small conversions
// ---------------------------------------------------------------------------

/// Microseconds of any length: delay_us() serves less than a SysTick
/// period, so a longer wait is chunked.
void wait_us(uint32_t us) {
    while (us >= 900u) {
        (void)delay_us(clock, 900u);
        us -= 900u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// 24 MHz ticks of one bit, in sixteenths: the unit every precomputed
/// edge is built from (sixteenths keep a 115200 bit's 208.33 from
/// drifting over a frame; 16 x 24 MHz fits 32 bits with room).
uint32_t bit_q4(uint32_t rate) {
    if (rate == 0u) {
        rate = 9600u;
    }
    return (ref_hz * 16u) / rate;
}

uint32_t q4_ticks(uint32_t q4) { return (q4 + 8u) >> 4; }

/// `n` bits of `q` sixteenths stay inside the engine's 32-bit arithmetic
/// (an hour of 300 baud would not): a run past it is refused.
bool bits_fit(uint32_t q, uint32_t n) { return n == 0u || q <= 0x3FFFFFFFu / n; }

UartBits bits_of(uint8_t n) {
    switch (n) {
        case 5: return UartBits::five;
        case 6: return UartBits::six;
        case 7: return UartBits::seven;
        case link::bits9_low:
        case link::bits9_high: return UartBits::nine;
        default: return UartBits::eight;
    }
}

UartParity parity_of(uint8_t n) {
    switch (n) {
        case 1: return UartParity::even;
        case 2: return UartParity::odd;
        default: return UartParity::none;
    }
}

uint16_t data_mask(uint8_t bits) {
    const uint8_t n = bits >= link::bits9_low ? 9 : bits;
    return static_cast<uint16_t>((1u << n) - 1u);
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

bool crystal_ok = false;
bool engine_ok = false;
bool span_ok = false;
/// A frame has checked out recently: the wiring is proven and the
/// transmitter may idle on PA18. Cleared after rediscover_ms of quiet.
bool proven = false;
link::Decoder decoder;
link::Report last_report;
uint32_t commands = 0, naks = 0, actions = 0;
uint32_t ack_ms = 0;
uint32_t last_byte_ms = 0;
uint32_t last_frame_ms = 0;
uint32_t standby_until = 0;
uint32_t engaged_until = 0;
bool trace = false;
char label8[9] = {};

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(link::settle_ms) + 2u) {
    }
}

// ---------------------------------------------------------------------------
// The link's configurations
// ---------------------------------------------------------------------------

/// The XCK pad: the SERCOM's in the synchronous roles, an input with no
/// pull otherwise (the dark discipline).
void xck_pad(bool sercom, bool host) {
    if (sercom) {
        XckPin::function(PinFunction::c, {.input_enable = !host});
    } else {
        XckPin::release();
        XckPin::input();
    }
}

/// The crossed pair's two pads to the SERCOM, PORT's DIR cleared behind
/// them so a later release leaves the pad an input.
void link_pads_up() {
    RxdPin::function(PinFunction::c, {.input_enable = true});
    TxdPin::function(PinFunction::c);
    LinkPort::dir_clear(RxdPin::mask | TxdPin::mask);
}

bool link_apply(const SercomUartConfig& c) {
    if (!Link::configure(c)) {
        return false;
    }
    if (c.external_clock) {
        // ON AN EXTERNAL CLOCK THE ENABLE'S SECOND HALF WAITS FOR XCK: the
        // directions come up in the clock domain that XCK is, and until the
        // far host clocks no SYNCBUSY.CTRLB clears. The enable's own bit is
        // what can be judged here; a short bound, so the host's first
        // frame is not missed waiting.
        (void)Link::enable(true, 64u);
        return Link::enabled();
    }
    return Link::enable(true);
}

/// TxD's pad given to PORT: an input with no pull, no driver on the wire.
void txd_dark() {
    TxdPin::release();
    TxdPin::input();
}

/// Command mode: async 8N1 at link::command_baud on the crossed pair, TxD's
/// pad the SERCOM's only once the wiring is proven (the dark discipline).
bool command_mode() {
    const bool ok = link_apply({.pads = link_pads, .baud = command_baud.reg,
                                .sampling = command_baud.sampling});
    link_pads_up();
    if (!proven) {
        txd_dark();
    }
    xck_pad(false, false);
    Pad1Pin::release();
    Pad1Pin::input();
    Link::flush_rx();
    decoder.reset();
    return ok;
}

/// A commanded configuration. False when this silicon cannot express it
/// - the caller reports it as a refusal and is back in command mode.
bool apply_cfg(const link::Cfg& c) {
    if (!c.apply) {
        return true;
    }
    if ((c.flags & link::flag_mpcm) != 0u) {
        return false;   // no multiprocessor receive mode in this SERCOM
    }
    if (c.stop != 1u && c.stop != 2u) {
        return false;   // CTRLB.SBMODE: one or two whole stop bits, no half ones
    }
    SercomUartConfig u{.pads = link_pads,
                       .format = {.bits = bits_of(c.bits), .parity = parity_of(c.parity),
                                  .two_stop = c.stop >= 2},
                       .tx = true};
    bool sync = false, host = false;
    switch (c.mode) {
        case link::Mode::sync_host: {
            const auto reg = sercom_sync_baud_reg(ref_hz, c.rate);
            if (!reg) return false;
            u.synchronous = true;
            u.baud = *reg;
            u.invert_clock = (c.flags & link::flag_invert_xck) != 0u;
            sync = host = true;
            break;
        }
        case link::Mode::sync_client:
            u.synchronous = true;
            u.external_clock = true;
            u.invert_clock = (c.flags & link::flag_invert_xck) != 0u;
            sync = true;
            break;
        case link::Mode::ircom: {
            // The coder transmits 3/16 of a bit and nothing else: a fixed
            // pulse is the AVR's TXPLCTRL, which this SERCOM has not got.
            if (c.txpl != 0u) return false;
            const auto reg = sercom_baud_reg(ref_hz, c.rate);
            if (!reg) return false;
            u.baud = *reg;
            u.irda = true;
            u.rx_pulse = c.rxpl;
            break;
        }
        default: {
            const auto b = sercom_closest_baud(ref_hz, c.rate,
                                               (c.flags & link::flag_clk2x) != 0u ? 8 : 16);
            if (!b) return false;
            u.baud = b->reg;
            u.sampling = b->sampling;
            break;
        }
    }
    if (!sercom_uart_config_valid(u) || !link_apply(u)) {
        (void)command_mode();
        return false;
    }
    link_pads_up();
    xck_pad(sync, host);
    Link::flush_rx();
    return true;
}

// ---------------------------------------------------------------------------
// Talking
// ---------------------------------------------------------------------------

constexpr uint32_t dre_spins = 400'000u;

/// One frame out: wait for room in DATA (bounded), then write it.
[[gnu::always_inline]] inline bool put_frame(uint16_t v) {
    for (uint32_t n = dre_spins; !Link::dre_flag(); --n) {
        if (n == 0u) return false;
    }
    Link::data(v);
    return true;
}

/// The wire falls idle: TXC up (the last stop bit gone), bounded. Only
/// after a frame was written: TXC is clear out of configure() and rises
/// only at a frame's departure (31.8.8), so with nothing sent it would
/// spin its whole bound.
void wait_tx_done() {
    for (uint32_t n = dre_spins; !Link::txc_flag() && n != 0u; --n) {
    }
}

/// Four bit times at the BAUD in force before an answer: RXC rises at the
/// middle of the stop bit, half a bit before the sender's own TXC (the
/// AVR peer's measured turnaround), and the DUT listens again only after
/// it. 35 us at command rate.
void turnaround_guard() { wait_us(35u); }

/// The pad to the SERCOM for an answer: the transmitter is enabled and
/// idling high already, so the hand-over is no edge.
void line_talk() {
    turnaround_guard();
    if (!TxdPin::has_function()) {
        TxdPin::function(PinFunction::c);
    }
}

void line_listen() {
    wait_tx_done();
    if (!proven) {
        txd_dark();
    }
}

void put_link(uint8_t b) { (void)put_frame(b); }

void answer(Op op, const uint8_t* p, uint8_t len) {
    line_talk();
    link::write_frame(put_link, op, p, len);
    line_listen();
}

void ack(Op op, uint8_t sum, bool good) {
    const uint8_t p[2] = {link::byte_of(op), sum};
    answer(good ? Op::ack : Op::nak, p, 2);
}

// ---------------------------------------------------------------------------
// Accounting
// ---------------------------------------------------------------------------

/// STATUS's error bits stand until written (31.8.9): cleared here, after
/// the character they belong to, so the next one is judged on its own.
[[gnu::always_inline]] inline void account(link::Report& r, uint16_t st, uint16_t d) {
    const uint16_t err = static_cast<uint16_t>(st & SercomStatus::receive_errors);
    if (err != 0u) [[unlikely]] {
        Link::clear_status(err);
        if ((err & SercomStatus::frame_error) != 0u && r.ferr < 255u) ++r.ferr;
        if ((err & SercomStatus::parity_error) != 0u && r.perr < 255u) ++r.perr;
        if ((err & SercomStatus::overflow) != 0u && r.ovf < 255u) ++r.ovf;
    }
    r.sum = static_cast<uint16_t>(r.sum + d);
    ++r.count;
}

// ---------------------------------------------------------------------------
// The actions
// ---------------------------------------------------------------------------

/// THE HOT PATH: a character polled, stamped, accounted and - for an
/// echo - written back, with no call. COUNTED IN THE RELEASE LISTING: a
/// clean character's sink path is some 34 instructions and five
/// peripheral accesses (INTFLAG, CC0, SWEVT, STATUS, DATA), the count, sum
/// and stamp held in registers and on the stack and the Params' fields
/// loaded once ahead of the loop; the echo adds twelve and two (INTFLAG
/// for DRE, DATA) - well under two microseconds a character at 48 MHz,
/// where the fastest frame this link carries (3 Mbaud at 8x) is 3.3. The
/// stamp comes FIRST: the
/// character is complete now, and what the loop does with it is not the
/// DUT's transmitter. The stamp of character k is read back at character
/// k + 1 (the read that acknowledges it), the last one after the run.
[[gnu::noinline]] link::Report run_echo(const link::Params& a, bool sink) {
    link::Report r{};
    settle();
    Link::flush_rx();
    span_arm();
    uint32_t first = 0;
    const uint16_t want = a.count;
    const uint32_t t0 = Ticker::millis();
    while (r.count < want) {
        if (Link::rxc_flag()) {
            if (r.count != 0u) {
                const uint32_t stamp = Span::cc32(0);
                if (r.count == 1u) first = stamp;
            }
            Evsys::trigger(ev_snap);
            const uint16_t st = Link::status();
            const uint16_t d = Link::data();
            account(r, st, d);
            if (!sink) {
                (void)put_frame(d);
            }
            continue;
        }
        if (Ticker::millis() - t0 >= a.ms) {
            r.flags |= link::report_timed_out;
            break;
        }
    }
    if (r.count >= 2u) {
        wait_us(2u);
        r.span = Span::cc32(0) - first;
    }
    if (!sink && r.count != 0u) {
        wait_tx_done();
    }
    return r;
}

/// The DUT's gapless frames timed on THIS crystal: a sink's span, and the
/// DUT's error against it in ppm (usart_link.hpp's calibrate). A crystal
/// peer adopts nothing: it reports. Without its crystal it reports too -
/// generator 2 on OSC48M / 2 is then the clock, and ident.xtal says 0.
link::Report run_calibrate(const link::Params& a) {
    link::Report r = run_echo(a, true);
    const uint32_t nominal =
        link::nominal_span(a.cfg.rate, link::frame_half_bits(a.cfg.bits, a.cfg.parity, a.cfg.stop),
                           r.count);
    r.aux16 = static_cast<uint16_t>(link::calibrate_ppm(nominal, r.span));
    return r;
}

link::Report run_set_link(const link::Params& a) {
    link::Report r{};
    settle();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        if (Link::rxc_flag()) {
            const uint16_t st = Link::status();
            account(r, st, Link::data());
        }
    }
    r.flags |= link::report_timed_out;
    return r;
}

link::Report run_send(const link::Params& a, bool blast) {
    link::Report r{};
    settle();
    // `send` reads aux16 as EXTRA lead-in milliseconds (usart_link.hpp).
    if (!blast && a.aux16 != 0u) {
        wait_ms(a.aux16);
    }
    const uint16_t mask = data_mask(a.cfg.bits);
    // A send leaves two bit times after each write; a blast none.
    const uint32_t gap = blast ? 0u : (2u * 1'000'000u) / (a.cfg.rate ? a.cfg.rate : 1u);
    const uint16_t repeats = blast && a.aux16 > 1u ? a.aux16 : 1u;
    const uint32_t repeat_gap_us = static_cast<uint32_t>(a.aux8) * 100u;
    link::Stream stream(a.pattern, a.value);
    const uint32_t t0 = Ticker::millis();
    for (uint16_t k = 0; k < repeats; ++k) {
        if (k != 0u) {
            if (a.ms != 0u && Ticker::millis() - t0 >= a.ms) {
                r.flags |= link::report_timed_out;
                break;
            }
            if (repeat_gap_us != 0u) wait_us(repeat_gap_us);
        }
        bool sent = true;
        for (uint16_t i = 0; i < a.count; ++i) {
            const uint16_t v = static_cast<uint16_t>(stream.next()) & mask;
            if (!put_frame(v)) {
                sent = false;
                break;
            }
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++r.count;
            if (gap != 0u) wait_us(gap);
        }
        if (!sent) break;
    }
    if (r.count != 0u) {
        wait_tx_done();
    }
    return r;
}

/// Address frame (ninth bit set) and its data frames, per group.
link::Report run_send_mpcm(const link::Params& a, const uint8_t* pairs) {
    link::Report r{};
    settle();
    uint8_t seq = a.value;
    for (uint8_t g = 0; g < a.groups; ++g) {
        const uint16_t addr = static_cast<uint16_t>(0x100u | pairs[2 * g]);
        if (!put_frame(addr)) break;
        r.sum = static_cast<uint16_t>(r.sum + addr);
        ++r.count;
        for (uint8_t k = 0; k < pairs[2 * g + 1]; ++k) {
            const uint16_t v = seq++;
            if (!put_frame(v)) break;
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++r.count;
        }
    }
    if (r.count != 0u) {
        wait_tx_done();
    }
    return r;
}

/// TxD's pad from the SERCOM to PORT, idle high and driven.
void txd_to_port() {
    TxdPin::set();
    TxdPin::output();
    TxdPin::release();
}

/// ...and back, the transmitter idling high under it already.
void txd_to_sercom() {
    TxdPin::function(PinFunction::c);
    LinkPort::dir_clear(TxdPin::mask);
}

/// TxD held low for `count` bit times by the engine, the transmitter's pad
/// under PORT: two bits of idle, the low, two bits of idle.
link::Report run_break(const link::Params& a) {
    link::Report r{};
    settle();
    const uint32_t q = bit_q4(a.cfg.rate);
    if (!bits_fit(q, a.count + 4u)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    edges_clear();
    edge_level(0, false);
    edge_level(q4_ticks(q * a.count), true);
    txd_to_port();
    const bool ok = play<TxdPin>(Drive::out, q4_ticks(2u * q));
    wait_us(q4_ticks(2u * q) / 24u + 1u);
    txd_to_sercom();
    if (!ok) r.flags |= link::report_mismatch;
    r.count = a.count;
    r.aux16 = Link::baud_reg();
    return r;
}

/// The foreign sender: four bits of idle, a break of aux16 bit times
/// (16 by default), two bits of idle, then the sync character and aux8
/// payload frames from the SERCOM at the commanded rate - every one of
/// them timed by this board's CRYSTAL.
link::Report run_autobaud_tx(const link::Params& a) {
    link::Report r{};
    settle();
    const uint32_t q = bit_q4(a.cfg.rate);
    const uint32_t n = a.aux16 != 0u ? a.aux16 : 16u;
    if (!bits_fit(q, n + 6u)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    edges_clear();
    edge_level(0, false);
    edge_level(q4_ticks(q * n), true);
    txd_to_port();
    const bool ok = play<TxdPin>(Drive::out, q4_ticks(4u * q));
    wait_us(q4_ticks(2u * q) / 24u + 1u);
    txd_to_sercom();
    (void)put_frame(a.value);                 // the sync field
    r.sum = a.value;
    ++r.count;
    for (uint8_t k = 0; k < a.aux8; ++k) {
        const uint8_t v = static_cast<uint8_t>(0xA0 + k);
        if (!put_frame(v)) break;
        r.sum = static_cast<uint16_t>(r.sum + v);
        ++r.count;
    }
    wait_tx_done();
    if (!ok) r.flags |= link::report_mismatch;
    r.aux16 = Link::baud_reg();
    return r;
}

/// One 8N1 frame's ten cells into the edge list from `at` (sixteenths of
/// a tick), cell `cell` stretched by `pct`; returns the position after the
/// stop bit.
uint32_t frame_edges(uint32_t at, uint32_t q, uint8_t value, int8_t cell, int8_t pct) {
    for (int8_t i = 0; i < 10; ++i) {
        const bool high = i == 0 ? false : (i == 9 ? true : ((value >> (i - 1)) & 1u) != 0u);
        edge_level(q4_ticks(at), high);
        int32_t w = static_cast<int32_t>(q);
        if (cell == i && pct != 0) {
            w += (static_cast<int32_t>(q) / 100) * pct;
        }
        if (w < 16 * 24) w = 16 * 24;
        at += static_cast<uint32_t>(w);
    }
    return at;
}

/// `count` frames of `value` with a clean four-bit idle between them, and
/// aux16 bit times of break ahead (0 = none) - the whole run precomputed,
/// then played by the engine on `P`. The Report's aux16 is a bit's
/// nominal length in 24 MHz ticks.
template <typename P>
link::Report run_bitbang(const link::Params& a) {
    link::Report r{};
    const uint32_t q = bit_q4(a.cfg.rate);
    if (!bits_fit(q, static_cast<uint32_t>(a.aux16) + 16u * a.count + 2u)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    edges_clear();
    uint32_t at = 0;
    if (a.aux16 != 0u) {
        edge_level(0, false);
        at = q * a.aux16;
        edge_level(q4_ticks(at), true);
        at += 2u * q;
    }
    for (uint16_t i = 0; i < a.count; ++i) {
        at = frame_edges(at, q, a.value, a.cell, a.pct);
        at += 4u * q;
        r.sum = static_cast<uint16_t>(r.sum + a.value);
        ++r.count;
    }
    P::set();
    P::output();
    P::release();
    settle();
    if (!play<P>(Drive::out, q4_ticks(2u * q))) {
        r.flags |= link::report_mismatch;
    }
    r.aux16 = static_cast<uint16_t>(q4_ticks(q) > 0xFFFFu ? 0xFFFFu : q4_ticks(q));
    return r;
}

/// `count` low pulses of aux16 ticks, aux8 microseconds apart, on TxD.
link::Report run_glitch(const link::Params& a) {
    link::Report r{};
    const uint32_t w = a.aux16 != 0u ? a.aux16 : 1u;
    const uint32_t gap = static_cast<uint32_t>(a.aux8 ? a.aux8 : 1u) * 24u;
    edges_clear();
    uint32_t at = 0;
    for (uint16_t i = 0; i < a.count; ++i) {
        edge_level(at, false);
        edge_level(at + w, true);
        at += w + gap;
    }
    txd_to_port();
    settle();
    if (!play<TxdPin>(Drive::out)) {
        r.flags |= link::report_mismatch;
    }
    txd_to_sercom();
    r.count = a.count;
    r.aux16 = a.aux16;
    return r;
}

/// THE DESK'S SHARED WIRE is the DUT's TxD (PA16 here). LISTENING, the
/// SERCOM receives on it with its transmitter on PAD[2], whose pad is not
/// muxed - TXEN = 0 would leave a pad driven (the header) - and every
/// reply this board puts on the wire comes from the engine, OPEN DRAIN:
/// DIR toggles with OUT at 0, PA16 off the SERCOM for the length of it.
bool wire_listen(const SercomBaud& b) {
    txd_dark();
    if (!link_apply({.pads = link_pads, .baud = b.reg, .sampling = b.sampling})) {
        return false;
    }
    RxdPin::function(PinFunction::c, {.input_enable = true});
    Link::flush_rx();
    return true;
}

std::optional<SercomBaud> wire_baud(uint32_t rate) {
    return sercom_closest_baud(ref_hz, rate ? rate : link::command_baud);
}

link::Report run_onewire(const link::Params& a) {
    link::Report r{};
    const uint32_t rate = a.cfg.rate ? a.cfg.rate : link::command_baud;
    const auto b = wire_baud(rate);
    if (!b || !wire_listen(*b)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    const uint32_t q = bit_q4(rate);
    settle();
    const uint32_t t0 = Ticker::millis();
    while (r.count < a.count) {
        if (Link::rxc_flag()) {
            const uint16_t st = Link::status();
            const uint16_t d = Link::data();
            account(r, st, d);
            // THE REPLY IS OPEN DRAIN, from the engine: the frame's edges
            // timed on the crystal and taken as DIR toggles with OUT at 0,
            // PA16 off the SERCOM for the length of it - the wired AND the
            // DUT's own open-drain transmitter shares, so no level is ever
            // driven against it. The lead past the DUT's stop bit is the
            // guard: the DUT turns its line round only after its own TXC.
            edges_clear();
            (void)frame_edges(0, q, static_cast<uint8_t>(d ^ 0xFFu), -1, 0);
            RxdPin::clear();
            RxdPin::input();
            RxdPin::release();
            if (!play<RxdPin>(Drive::dir, q4_ticks(4u * q))) {
                r.flags |= link::report_mismatch;
            }
            // Back to the receiver inside the reply's stop bit: the DUT's
            // next frame may follow its own RXC at once. The receiver ran
            // blind on an unmuxed pad meanwhile - RXEN off and on throws
            // away whatever it took for a frame (31.8.2).
            RxdPin::function(PinFunction::c, {.input_enable = true});
            (void)Link::enable_rx(false);
            (void)Link::enable_rx(true);
            continue;
        }
        if (Ticker::millis() - t0 >= a.ms) {
            r.flags |= link::report_timed_out;
            break;
        }
    }
    return r;
}

/// Wait for one trigger frame on the shared wire, then put `value` into
/// the DUT's NEXT frame aux16 microseconds later - OPEN DRAIN, the engine
/// timing the edges and the CPU taking each as a DIR toggle with OUT at 0,
/// so this board only ever pulls the wire low.
link::Report run_collide(const link::Params& a) {
    link::Report r{};
    const auto b = wire_baud(a.cfg.rate);
    if (!b || !wire_listen(*b)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    const uint32_t q = bit_q4(a.cfg.rate ? a.cfg.rate : link::command_baud);
    edges_clear();
    (void)frame_edges(0, q, a.value, -1, 0);   // ends on its stop bit: high
    settle();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        if (Link::rxc_flag()) {
            const uint16_t st = Link::status();
            account(r, st, Link::data());
            RxdPin::clear();
            RxdPin::input();
            RxdPin::release();
            if (!play<RxdPin>(Drive::dir, static_cast<uint32_t>(a.aux16) * 24u)) {
                r.flags |= link::report_mismatch;
            }
            RxdPin::function(PinFunction::c, {.input_enable = true});
            r.aux16 = a.value;
            return r;
        }
    }
    r.flags |= link::report_timed_out;
    return r;
}

// ---------------------------------------------------------------------------
// The command handler
// ---------------------------------------------------------------------------

void handle(const link::Frame& f) {
    ++commands;
    engaged_until = Ticker::millis() + link::engage_ms;
    const Op op = f.op;
    if (trace) {
        print(console, "  [cmd op=", hex(link::byte_of(op)), " len=", f.len,
              " standby=", standby_until != 0 ? 1 : 0, "]", crlf);
    }

    // STAND-OFF: mute to every command but the one that ends it.
    if (op == Op::standby) {
        const uint16_t ms = f.len >= link::params_size ? link::get_params(f.data).ms : 0;
        standby_until = ms ? Ticker::millis() + ms : 0;
        ack(op, f.sum, true);
        return;
    }
    if (standby_until != 0) {
        if (Ticker::millis() < standby_until) return;
        standby_until = 0;
    }

    if (op == Op::ping) {
        ack(op, f.sum, true);
        return;
    }
    if (op == Op::ident) {
        ack(op, f.sum, true);
        link::Ident d{};
        for (uint8_t i = 0; i < 8; ++i) d.label[i] = label8[i];
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

    // The waveforms and the shared wire set the link themselves; every
    // other op runs at the Cfg it carries.
    const bool own_config = op == Op::bitbang || op == Op::glitch || op == Op::drive_rxd ||
                            op == Op::onewire || op == Op::collide;
    const bool engined = op == Op::bitbang || op == Op::glitch || op == Op::drive_rxd ||
                         op == Op::brk || op == Op::autobaud_tx || op == Op::collide;
    bool ok = !engined || engine_ok;
    if (ok && !own_config) ok = apply_cfg(a.cfg);

    if (!ok) {
        r.flags |= link::report_cfg_failed;
    } else {
        switch (op) {
            case Op::set_link: r = run_set_link(a); break;
            case Op::echo: r = run_echo(a, false); break;
            case Op::sink: r = run_echo(a, true); break;
            case Op::send: r = run_send(a, false); break;
            case Op::blast: r = run_send(a, true); break;
            case Op::send_mpcm: r = run_send_mpcm(a, f.data + link::params_size); break;
            case Op::brk: r = run_break(a); break;
            case Op::autobaud_tx: r = run_autobaud_tx(a); break;
            case Op::bitbang: r = run_bitbang<TxdPin>(a); break;
            case Op::glitch: r = run_glitch(a); break;
            // The DUT's TxD pad is reached through this board's RxD pin on
            // the crossed pair: the bit-banger drives THE OTHER BOARD's TXD.
            case Op::drive_rxd: r = run_bitbang<RxdPin>(a); break;
            case Op::onewire: r = run_onewire(a); break;
            case Op::collide: r = run_collide(a); break;
            case Op::calibrate: r = run_calibrate(a); break;
            default: r.flags |= link::report_cfg_failed; break;
        }
        if ((r.flags & link::report_cfg_failed) == 0u) r.flags |= link::report_ran;
    }

    const uint32_t took = Ticker::millis() - started;
    r.ms = took > 255u ? 255u : static_cast<uint8_t>(took);
    r.op = link::byte_of(op);
    last_report = r;
    engaged_until = Ticker::millis() + link::engage_ms;
    (void)command_mode();
}

// ---------------------------------------------------------------------------
// The console
// ---------------------------------------------------------------------------

void help() {
    print(console, "usart_peer: ? help | i status | 0 command mode | 2 wiring probe | 3 trace "
                   "| 4 engine self-test", crlf);
}

void status() {
    print(console, "  link SERCOM1 fn C, crossed pair: ", proven ? "PROVEN" : "searching (dark)",
          ", BAUD=", Link::baud_reg(), " TxD pad ", TxdPin::has_function() ? "muxed" : "released",
          " rx=", Link::rx_enabled() ? 1 : 0, crlf);
    print(console, "  crystal=", crystal_ok ? "up" : "DOWN", " engine=", engine_ok ? "up" : "DOWN",
          " span=", span_ok ? "up" : "DOWN", "; edges read back ", edges_verified, ", wrong ",
          edges_wrong, crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          standby_until != 0 && Ticker::millis() < standby_until ? "  (STAND-OFF)" : "",
          engaged() ? "  (ENGAGED)" : "", crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " sum=", hex(last_report.sum), " ferr=", last_report.ferr,
          " perr=", last_report.perr, " ovf=", last_report.ovf,
          " flags=", hex(last_report.flags), " aux=", last_report.aux16,
          " ms=", last_report.ms, " span=", last_report.span, crlf);
}

/// THE WIRING PROBE, the DUT's letter v's partner. Each link pad is driven
/// at the rate of the AVR pin it meets in the crossed-pair convention -
/// PA18 (to the DUT's PE1, which the AVR peer's PE0 would reach) 2 Hz,
/// PA16 (to PE0) 4 Hz, PA19 (to PE2) 8 Hz, PA17 (to PE3) 16 Hz - for six
/// seconds, then listened to for six more under PORT pull-ups, the edge
/// count naming which of the DUT's pins each pad is tied to. The DUT's v
/// runs the two phases in the opposite order.
constexpr uint16_t probe_ms = 6000;
constexpr uint16_t probe_half_ms[4] = {250, 125, 62, 31};   // the DUT's PE0..PE3
constexpr uint32_t probe_mask[4] = {TxdPin::mask, RxdPin::mask, XckPin::mask, Pad1Pin::mask};
/// The pad each of the DUT's PE0..PE3 meets on this desk.
constexpr uint32_t meets[4] = {RxdPin::mask, TxdPin::mask, XckPin::mask, Pad1Pin::mask};

void probe_release() {
    RxdPin::release();
    Pad1Pin::release();
    TxdPin::release();
    XckPin::release();
}

void probe_drive() {
    const uint32_t all = probe_mask[0] | probe_mask[1] | probe_mask[2] | probe_mask[3];
    LinkPort::configure_mask(all, {.input_enable = true});
    LinkPort::out_clear(all);
    LinkPort::dir_set(all);
    for (uint16_t t = 0; t < probe_ms; ++t) {
        for (uint8_t i = 0; i < 4; ++i) {
            if (((t / probe_half_ms[i]) & 1u) != 0u) LinkPort::out_set(probe_mask[i]);
            else LinkPort::out_clear(probe_mask[i]);
        }
        wait_us(1000u);
    }
    LinkPort::dir_clear(all);
}

void probe_listen(uint16_t* edges4) {
    const uint32_t all = probe_mask[0] | probe_mask[1] | probe_mask[2] | probe_mask[3];
    LinkPort::dir_clear(all);
    LinkPort::configure_mask(all, {.input_enable = true, .pull = PinPull::up});
    uint32_t prev = LinkPort::in() & all;
    for (uint16_t t = 0; t < probe_ms; ++t) {
        wait_us(1000u);
        const uint32_t now = LinkPort::in() & all;
        const uint32_t ch = now ^ prev;
        for (uint8_t i = 0; i < 4; ++i) {
            if ((ch & meets[i]) != 0u) ++edges4[i];
        }
        prev = now;
    }
    LinkPort::configure_mask(all, {.input_enable = true});
}

int8_t probe_source(uint16_t e) {
    if (e < 8u) return -1;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t want = probe_ms / probe_half_ms[i];
        if (e * 4u >= want * 3u && e * 3u <= want * 4u) return static_cast<int8_t>(i);
    }
    return -2;
}

/// Console '4': the engine on PA18 alone - a 6-tick glitch, four of 300
/// ticks and a 9600-baud 0x55 frame - with the pad read back after every
/// edge whose successor is far enough (the engine's own check). PA18 meets the far board's RXD, an input: run it
/// with that board dark or listening.
void engine_self_test() {
    (void)Link::enable(false);
    edges_clear();
    edge_level(0, false);   // the narrowest: n's 6-tick glitch, both edges preloaded
    edge_level(6, true);
    uint32_t at = 2400;
    for (uint8_t i = 0; i < 4; ++i) {
        edge_level(at, false);
        edge_level(at + 300u, true);
        at += 2400u;
    }
    (void)frame_edges(at * 16u, bit_q4(9600), 0x55, -1, 0);   // sixteenths of a tick
    const uint16_t n = edge_n;
    const uint32_t v0 = edges_verified, w0 = edges_wrong;
    txd_to_port();
    const bool ok = play<TxdPin>(Drive::out);
    txd_to_sercom();
    print(console, "  engine on PA18: ", n, " edges, ", ok ? "every one seen in time" : "BROKEN",
          ", read back after ", edges_verified - v0, " of them, ", edges_wrong - w0, " wrong",
          crlf);
    (void)command_mode();
}

void wire_probe() {
    (void)Link::enable(false);
    probe_release();
    print(console, "  driving PA18/PA16/PA19/PA17 at 2/4/8/16 Hz for 6 s, then listening 6 s",
          crlf);
    probe_drive();
    uint16_t e[4] = {0, 0, 0, 0};
    probe_listen(e);
    const char* pad_of[4] = {"PA16", "PA18", "PA19", "PA17"};   // what meets PE0..PE3
    for (uint8_t i = 0; i < 4; ++i) {
        const int8_t src = probe_source(e[i]);
        print(console, "  here ", pad_of[i], ": ", e[i], " edges -> ");
        if (src >= 0) print(console, "the DUT's PE", static_cast<uint8_t>(src), crlf);
        else if (src == -1) print(console, "nothing", crlf);
        else print(console, "several or noise", crlf);
    }
    (void)command_mode();
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Console::isr(); }

/// The peer arms no SERCOM1 or TC interrupt (everything here is polled),
/// but an unbound vector is a silent spin - so a stray arm is disarmed.
extern "C" void SERCOM1_Handler() { Link::enable_interrupt(SercomFlag::all, false); }

int main() {
    SysClock::init();
    Console::init(clock, 115200);
    Ticker::init(clock);
    brio::enable_interrupts();

    const uint32_t w0 = DeviceSerial::read().word[0];
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        label8[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }

    // The crystal, on generator 2: the reference of the link and both
    // rulers. Without it, OSC48M / 2 stands in and ident.xtal says so.
    crystal_ok = Xosc::init(XoscConfig{.hz = ref_hz, .startup = 4, .on_demand = false}) &&
                 Gclk<gen_xtal>::configure(GclkConfig{.source = GclkSource::xosc});
    if (!crystal_ok) {
        (void)Gclk<gen_xtal>::configure(GclkConfig{.source = GclkSource::osc48m, .div = 2});
    }
    Evsys::bus_clock(true);
    span_ok = span_up();
    engine_ok = engine_up();

    Link::bus_clock(true);
    (void)Link::core_clock(gen_xtal);
    (void)Link::reset();

    print(console, crlf, "usart_peer - USART instrument peer (SAM C21 board ", label8,
          ", xtal=", crystal_ok ? "up" : "DOWN", ", fw ", hex(firmware_version), ")", crlf);
    print(console, "link SERCOM1 fn C, the crossed pair: RxD PA16, TxD PA18, XCK PA19; "
                   "every rate from the 24 MHz crystal; command mode 8N1 @ ",
          link::command_baud, crlf);
    print(console, "DARK until a frame checks out: TxD's pad released to PORT, no pull on any "
                   "link pad", crlf);
    if (!engine_ok || !span_ok) {
        print(console, "WARNING: the waveform engine or the span ruler did not come up", crlf);
    }
    (void)command_mode();
    last_frame_ms = Ticker::millis();
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
                    proven = false;
                    engaged_until = 0;
                    standby_until = 0;
                    (void)command_mode();
                    print(console, "  command mode, dark, wiring unproven", crlf);
                } else if (c == '2') wire_probe();
                else if (c == '4') engine_self_test();
                else if (c == '3') {
                    trace = !trace;
                    print(console, trace ? "  trace on" : "  trace off", crlf);
                } else print(console, "? for help", crlf);
                print(console, "> ");
            }
        }

        const uint32_t now = Ticker::millis();
        if (standby_until != 0 && now >= standby_until) standby_until = 0;
        // A wiring is believed only while the command channel keeps
        // proving it: rediscover_ms of quiet and the transmitter goes dark.
        if (standby_until == 0 && proven && now - last_frame_ms > link::rediscover_ms) {
            proven = false;
            (void)command_mode();
        }
        // A frame that stops half way must not eat the next one.
        if (decoder.partial() && now - last_byte_ms > 50u) {
            decoder.reset();
        }
        if (!Link::rxc_flag()) continue;
        const uint16_t st = Link::status();
        const uint16_t d = Link::data();
        last_byte_ms = Ticker::millis();
        if ((st & SercomStatus::receive_errors) != 0u) {
            Link::clear_status(st & SercomStatus::receive_errors);
            decoder.reset();
            continue;
        }
        switch (decoder.feed(static_cast<uint8_t>(d))) {
            case link::Decoder::Result::frame:
                proven = true;
                handle(decoder.frame());
                last_byte_ms = Ticker::millis();
                last_frame_ms = last_byte_ms;
                break;
            case link::Decoder::Result::bad_checksum:
                if (engaged()) {
                    ack(decoder.pending_op(), decoder.frame().sum, false);
                    ++naks;
                }
                break;
            default: break;
        }
    }
}
