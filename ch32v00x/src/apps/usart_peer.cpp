// usart_peer - the INSTRUMENT half of a two-board USART test on a
// CH32V006: the scriptable peer the far board's suite (the DUT) drives IN
// BAND over the very link under test.
//
// The wire format is usart_link.hpp, included by relative path: one file
// is the single source of truth for every board that speaks it, whatever
// its architecture. One blocking poll loop decodes a command frame,
// acknowledges it and then becomes, for a bounded moment, whatever the
// DUT needs on the other end of the wire - an echo, a silent sink, a
// generator, a flood, a break, a foreign auto-baud sender, a bit-banger a
// clean UART could never be - and restores command mode BY ITSELF when
// the action's frame count or deadline runs out.
//
// THE LINK IS USART2 ON ITS COLUMN 4 (AFIO table 7-11): TX PB0, RX PB1,
// polled through the resource - no interrupt, no ring: an instrument's
// loop owns the instance. The desk wires it crossed, full duplex, to the
// far board's USART; the peer serves full duplex alone and refuses
// onewire (no USART TX of this chip reaches its own RX pad, so it cannot
// join the DUT's TX wire with its USART). The console is USART1 on
// PD5/PD6, the probe's serial, at 115200: observability only.
//   ? help | i status and counters | 0 back to command mode, dark | 3 trace
//
// THE DARK DISCIPLINE (usart_link.hpp): while SEARCHING - no frame has
// checked out, or the channel has been quiet for rediscover_ms, or a
// stand-off is running - PB0 and PB1 are floating inputs: the transmitter
// reaches no wire and nothing is pulled, so the far board's single-board
// letters walk those wires with their own pulls. A frame that checks out
// PROVES the wiring: PB0 goes to the USART (idling high) and PB1 takes its
// pull-up. A bad checksum is nak'ed only while ENGAGED.
//
// TIME (usart_link.hpp, TIME IN THE PAYLOAD). HCLK is the module's 24 MHz
// crystal doubled by the PLL, 48 MHz (Ident::xtal = 1 when the crystal
// started; without it the HSI's PLL runs the same 48 MHz nominal, xtal =
// 0, and calibrate is refused - this peer has no adoption to offer). The
// protocol's 24 MHz tick is TWO HCLK cycles: the Report's span is read on
// the STK's cycle count (ch32v00x/ticker.hpp) and halved; the waveform
// engine's TIM1 runs at PSC 1, so its count IS the tick, or at an exact
// multiple of it where an interval would pass half the counter.
//
// THE WAVEFORM ENGINE (bitbang, drive_rxd, brk, autobaud_tx's break):
// TIM1 under its remap 0110 (table 7-8) puts CH2 on PB0 and CH3 on PB1,
// the two link pads, and keeps every other TIM1 signal off the debug port
// (code 0100 puts CH4 on PD1, SWIO). A frame's edges are ABSOLUTE compare
// values computed before the action, streamed by DMA - CH2's request on
// channel 3, CH3's on 6 (table 8-2) - into the channel's compare register
// on the channel's own match, the channel in toggle mode: no CPU between
// edges, nothing masked. The list ends with a value half the counter away,
// which the last edge's request moves into the compare register and no
// match reaches before the CPU, seeing the channel's completion, forces
// the output high: every request is answered by a transfer, so none is
// left standing on the channel's OR. A glitch is ONE PULSE (OPM, PWM mode
// 1, the counter at HCLK): exact to 21 ns. These chips have no per-pin AF
// selector (8.2.4), so a pad passes between the USART and the timer as
// GPIO high, then the other function, each step one configuration-nibble
// store with the line idle high throughout - and while the timer owns PB0
// USART2 is remapped to its column 6, whose pads (PA6, PA5) stay inputs,
// so no second alternate output reaches PB0.
//
// THE PER-OP NOTES. onewire is refused (above); collide is an emulated
// open drain on the RX pad (an output low, or the input's pull-up),
// paced on the STK, a wired-AND only with a DUT whose own pad is an
// open drain; sync_client, sync_host and every flag the chip has not got
// (clk2x: no OVER8; invert_xck: no clock pad on this column; mpcm on the
// peer's receiver) are refused; 5- and 6-bit words and 9 data bits with a
// parity bit are refused; IrDA is served (IREN, the prescaler; txpl != 0
// asks the low-power mode at PSC = txpl).
//
// THE SUPPLY at boot (the desk's rule for a board that drives a link
// pad): VDD read from VREFINT; below 3000 mV the peer stays dark and says
// so on its console, its link never opened.
//
// NOTHING PLACED IN SRAM (util/placement.hpp): the link is the raw
// resource this program drives polled, its loops this file's own and
// measured from the flash at every rate the suites ask, and the console is
// the default Uart, which has no transmit engine to place.
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "ch32v00x/adc.hpp"
#include "ch32v00x/afio.hpp"
#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"

#include "../../../avrxt/src/apps/usart_link.hpp"

using P = brio::Ch32v00xPlatform<>;

/// The crystal's 24 MHz doubled; the HSI's PLL the fallback at the same rate.
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000, 24'000'000>;
using HsiClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;
static_assert(SysClock::hz == HsiClock::hz, "the fallback must run every divisor at the same rate");
static_assert(SysClock::hz == 2u * 24'000'000u,
              "usart_link.hpp counts time in 24 MHz ticks: this peer's tick is two HCLK cycles");

namespace {

using namespace brio;
using link::Op;

using Console = Uart<1, P>;
constexpr Console console;

constexpr uint8_t link_column = 4;          ///< USART2: TX PB0, RX PB1
constexpr uint8_t parked_column = 6;        ///< USART2: TX PA6, RX PA5 - pads left inputs
using L = Usart<2>;
using TxPin = Pin<'B', 0>;
using RxPin = Pin<'B', 1>;
static_assert(afio_usart2_pads(link_column).tx == TxPin::pad && afio_usart2_pads(link_column).rx == RxPin::pad,
              "table 7-11: column 4 is TX PB0, RX PB1");

using T1 = Tim<1>;
constexpr uint8_t tim1_column = 6;          ///< 0110: CH2 PB0, CH3 PB1, nothing on PD1
constexpr uint8_t tx_ch = 1;                ///< CH2, 0-based
constexpr uint8_t rx_ch = 2;                ///< CH3, 0-based
static_assert(afio_tim1_pads(tim1_column).ch2 == TxPin::pad && afio_tim1_pads(tim1_column).ch3 == RxPin::pad,
              "table 7-8: TIM1 code 0110 puts CH2 on PB0 and CH3 on PB1");
using TxEdges = DmaTxEngine<3, uint32_t>;   ///< table 8-2: TIM1 CH2's request
using RxEdges = DmaTxEngine<6, uint32_t>;   ///< table 8-2: TIM1 CH3's request

constexpr uint16_t firmware_version = static_cast<uint16_t>((link::peer_ch32v00x << 8) | 0x02u);

bool crystal_ok = false;
bool supply_ok = false;
uint16_t supply_mv = 0;
bool proven = false;
bool trace = false;
link::Decoder decoder;
link::Report last_report;
uint32_t commands = 0, naks = 0, actions = 0;

uint32_t ack_ms = 0;
uint32_t last_byte_ms = 0;
uint32_t last_frame_ms = 0;
uint32_t standby_until = 0;
uint32_t engaged_until = 0;

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }
bool standing_off() { return standby_until != 0 && Ticker::millis() < standby_until; }

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(link::settle_ms) + 2u) {
    }
}

/// HCLK cycles to protocol ticks: two cycles a tick.
constexpr uint32_t ticks_of(uint32_t cycles) { return cycles / 2u; }

/// Any number of microseconds: the whole milliseconds on the tick count,
/// the rest on delay_us, which serves less than one tick period and
/// refuses the rest (delay.hpp).
void delay_us_now(uint32_t us) {
    if (us >= 1000u) {
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < us / 1000u + 1u) {
        }
        us %= 1000u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

// ---- the pads ------------------------------------------------------------------

/// The two link pads as the state allows (the file header's dark
/// discipline): proven and not standing off, PB0 the USART's and PB1
/// pulled up; otherwise both floating inputs.
void hold_pads() {
    if (proven && !standing_off()) {
        TxPin::function();
        RxPin::input(PinPull::up);
    } else {
        TxPin::release();
        RxPin::release();
    }
}

/// Drain whatever the receiver holds and clear its errors (STATR, then
/// DATAR: the clear 14.8.1 orders).
void flush_rx() {
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)L::status();
        (void)L::read_word();
    }
}

// ---- the link's configuration ---------------------------------------------------

struct LinkFormat {
    UartFormat format{};
    uint16_t brr = 0;
    bool irda = false;
    IrdaConfig ir{};
};

/// What this silicon can make of a Cfg; empty when it cannot.
std::optional<LinkFormat> format_of(const link::Cfg& c) {
    if (c.mode == link::Mode::sync_client || c.mode == link::Mode::sync_host) {
        return std::nullopt;   // CTLR2 bits 11:7 reserved on this part: no synchronous mode
    }
    if ((c.flags & (link::flag_clk2x | link::flag_invert_xck | link::flag_mpcm)) != 0u) {
        return std::nullopt;
    }
    LinkFormat f{};
    switch (c.bits) {
        case 7: f.format.bits = UartBits::seven; break;
        case 8: f.format.bits = UartBits::eight; break;
        case link::bits9_low:
        case link::bits9_high: f.format.bits = UartBits::nine; break;
        default: return std::nullopt;
    }
    switch (c.parity) {
        case 0: f.format.parity = UartParity::none; break;
        case 1: f.format.parity = UartParity::even; break;
        case 2: f.format.parity = UartParity::odd; break;
        default: return std::nullopt;
    }
    switch (c.stop) {
        case 1: f.format.stop = UartStop::one; break;
        case 2: f.format.stop = UartStop::two; break;
        case link::stop_half: f.format.stop = UartStop::half; break;
        case link::stop_one_half: f.format.stop = UartStop::one_and_half; break;
        default: return std::nullopt;
    }
    if (!uart_format_valid(f.format)) {
        return std::nullopt;
    }
    const uint32_t brr = usart_divisor(SysClock::hz, c.rate);
    if (!usart_divisor_valid(brr)) {
        return std::nullopt;
    }
    f.brr = static_cast<uint16_t>(brr);
    if (c.mode == link::Mode::ircom) {
        f.irda = true;
        f.ir = c.txpl != 0u ? IrdaConfig{true, c.txpl} : IrdaConfig{false, 1};
        if (!irda_valid(f.ir) || f.format.stop != UartStop::one) {
            return std::nullopt;
        }
    }
    return f;
}

uint16_t data_mask = 0xFFu;

/// The instance through its reset line, then the format whole, the
/// transmitter and the receiver on. The pads are hold_pads()'. A proven
/// TX pad is parked as a GPIO output high across the reset and the
/// reconfiguration: a USART in reset drives its alternate output LOW, and
/// that edge is a start bit to the far receiver - a garbage frame with a
/// framing error behind every action (measured: the far transport's ring
/// skipped the whole echoed run for it).
bool open_link(const LinkFormat& f) {
    if (proven && !standing_off()) {
        TxPin::output(true);
    }
    L::bus_clock(true);
    L::reset();
    L::remap(link_column);
    if (!L::configure(f.format, f.brr)) {
        return false;
    }
    if (f.irda && !L::irda(f.ir)) {
        return false;
    }
    L::enable(true);
    L::transmitter(true);
    L::receiver(true);
    data_mask = uart_data_mask(f.format);
    // THE HAND-OVER WAITS. A TX pad handed to the function the moment TE
    // rises shows the far receiver a LOW PULSE: measured on this die, a
    // stray 0xE0 frame at 1 Mbaud - some five microseconds low - behind
    // every action, gone with the pad handed over 18 us later. Two bit
    // times at the new rate and 20 us more pass first.
    delay_us_now(2u * (f.brr / 48u) + 20u);
    hold_pads();
    flush_rx();
    return true;
}

/// Command mode: 8N1 at link::command_baud.
bool command_mode() {
    link::Cfg c{};
    const auto f = format_of(c);
    decoder.reset();
    return f && open_link(*f);
}

bool apply_cfg(const link::Cfg& c) {
    if (c.apply == 0u) {
        return true;
    }
    const auto f = format_of(c);
    return f && open_link(*f);
}

uint32_t cycles_per_bit(uint32_t rate) { return rate == 0u ? 5000u : SysClock::hz / rate; }

void put_link(uint8_t b) {
    while (!L::tx_empty()) {
    }
    L::write_data(b);
}

void wait_line_idle() {
    const uint32_t t0 = Ticker::millis();
    while (!L::tx_complete() && Ticker::millis() - t0 < 50u) {
    }
}

void answer(Op op, const uint8_t* p, uint8_t len) {
    hold_pads();
    link::write_frame(put_link, op, p, len);
    wait_line_idle();
}

void ack(Op op, uint8_t sum, bool good) {
    const uint8_t p[2] = {link::byte_of(op), sum};
    answer(good ? Op::ack : Op::nak, p, 2);
}

// ---- the waveform engine ---------------------------------------------------------

/// A list's edges as ticks from the counter's start, built before the
/// action: play() turns them into counts once, outside the timed part.
struct Edges {
    static constexpr uint8_t capacity = 32;
    uint32_t t[capacity] = {};
    uint8_t n = 0;
    uint32_t now = 0;        ///< the running position, in ticks
    bool level = true;       ///< the line's level at `now`
    bool overflow = false;

    void cell(bool high, uint32_t ticks) {
        if (high != level) {
            if (n + 1u >= capacity) {   // one slot kept for the end marker
                overflow = true;
            } else {
                t[n++] = now;
            }
            level = high;
        }
        now += ticks;
    }
};

/// The counter's start lead: the first edge this many ticks after CEN.
constexpr uint32_t engine_lead = 64;
/// The longest list a counter division serves: under half the counter, so
/// that the end marker half the counter beyond the last edge is never
/// matched while the list's tail plays out.
constexpr uint32_t engine_span = 30'000;

/// The engine's tenure of a pad: GPIO high, the channel forced high and
/// enabled, then the timer's function - and for PB0 USART2 parked on its
/// column 6 first, so that no second alternate output reaches the pad.
///
/// THE FORCED LEVEL NEEDS A RUNNING COUNTER: "the current output pin state
/// is maintained when ... the counter is disabled (CEN=0)" (11.4.1, the
/// note under OE_MODE), so on a timer that has not run since its reset the
/// channel's output holds the reset level, LOW, whatever its mode says -
/// measured: the first list after every boot lost its frame to a line
/// held low until the first play started the counter. The counter runs
/// here, free, before the pad is handed over; play() stops it only with
/// the line forced high, a level the stop then holds.
template <typename Pad>
void tenure_begin(uint8_t ch) {
    Pad::output(true);
    if (ch == tx_ch) {
        L::remap(parked_column);
    }
    (void)T1::configure(TimConfig{.prescaler = 1, .period = 0xFFFF});
    (void)T1::output_channel(ch, TimChannelConfig{.mode = TimOutputMode::force_active,
                                                  .preload = false, .enable = true});
    (void)T1::main_output(true);
    T1::enable(true);
    delay_us_now(2);
    Pad::function();
}

/// And back: GPIO high, the channel disabled, the USART's column again,
/// the pads as the state allows.
template <typename Pad>
void tenure_end(uint8_t ch) {
    Pad::output(true);
    (void)T1::output_channel(ch, TimChannelConfig{.mode = TimOutputMode::force_active,
                                                  .preload = false, .enable = false});
    if (ch == tx_ch) {
        L::remap(link_column);
    }
    hold_pads();
    flush_rx();
}

/// Play one edge list on channel `ch` (its DMA engine `E`) and return
/// when the line has reached `e.now`, the end of the list's last cell.
/// The division: `div` ticks a count (PSC = 2 div - 1), the smallest that
/// keeps the whole list under engine_span counts. The counter is stopped
/// while it is programmed and started at zero; the first edge goes into
/// the compare register by store, the rest by the channel's requests,
/// then the end marker.
/// The engine's own counts, console observability: lists played, lists
/// whose channel never completed within the list's length.
uint32_t plays = 0, play_timeouts = 0, play_refused = 0;

template <typename E>
bool play(uint8_t ch, Edges& e) {
    using Channel = DmaChannel<E::channel>;
    ++plays;
    if (e.overflow || e.n == 0u) {
        ++play_refused;
        return false;
    }
    const uint32_t div = e.now / engine_span + 1u;
    if (div > 32'768u) {
        return false;
    }
    for (uint8_t i = 0; i < e.n; ++i) {
        e.t[i] = (e.t[i] + div / 2u) / div;
    }
    e.t[e.n] = e.t[e.n - 1u] + 0x8000u;   // the end marker
    const uint32_t end = e.now / div;
    T1::enable(false);
    if (!T1::configure(TimConfig{.prescaler = static_cast<uint16_t>(2u * div - 1u), .period = 0xFFFF})) {
        return false;
    }
    if (!E::start(std::span<const uint32_t>(&e.t[1], e.n))) {
        ++play_refused;
        return false;
    }
    // THE FIRST EDGE INTO THE COMPARE REGISTER BEFORE THE MODE: output_
    // channel() writes the mode, then the register, and a register still
    // holding what the stopped counter reads (zero on a fresh timer) would
    // be a match the moment toggle mode lands, a request the enable below
    // would serve with the list a step ahead. Written first, it cannot
    // be; the flag is cleared after both, before the request is enabled.
    (void)T1::set_compare(ch, static_cast<uint16_t>(e.t[0]));
    (void)T1::output_channel(ch, TimChannelConfig{.mode = TimOutputMode::toggle,
                                                  .compare = static_cast<uint16_t>(e.t[0]),
                                                  .preload = false, .enable = true});
    T1::clear_flags(T1::compare_flag(ch));
    T1::interrupts(T1::compare_dma(ch), true);
    T1::enable(true);
    // The last edge's request has moved the end marker: the line is at its
    // final level. Bounded by the list's own length on the counter.
    const uint32_t budget = 2u * end * div * 2u + 48'000u;
    const uint32_t c0 = Ticker::cycles();
    bool done = false;
    while (!(done = Channel::flag(DmaFlag::complete))) {
        if (Ticker::cycles() - c0 > budget) {
            break;
        }
    }
    (void)T1::output_channel(ch, TimChannelConfig{.mode = TimOutputMode::force_active,
                                                  .preload = false, .enable = true});
    T1::interrupts(T1::compare_dma(ch), false);
    if (done) {
        (void)E::complete();
    } else {
        (void)E::abandon();
        ++play_timeouts;
    }
    // The tail: the list's last cell played out at the line's final level.
    while (T1::count() < end) {
    }
    T1::enable(false);
    return done;
}

/// The width of bit cell `idx` (0 = start, 1..8 = data, 9 = stop) in
/// ticks, stretched or shrunk by `pct` when it is the chosen one.
uint32_t cell_ticks(uint32_t nominal, int8_t cell, int8_t pct, int8_t idx) {
    int32_t v = static_cast<int32_t>(nominal);
    if (cell == idx && pct != 0) {
        v += (static_cast<int32_t>(nominal) * pct) / 100;
    }
    if (v < 12) v = 12;   // under about twelve HCLK cycles a request cannot reload in time
    return static_cast<uint32_t>(v);
}

/// One 8N1 frame, LSB first.
void frame_edges(Edges& e, uint32_t nominal, uint8_t value, int8_t cell, int8_t pct) {
    e.cell(false, cell_ticks(nominal, cell, pct, 0));
    for (int8_t i = 0; i < 8; ++i) {
        e.cell(((value >> i) & 1u) != 0u, cell_ticks(nominal, cell, pct, static_cast<int8_t>(i + 1)));
    }
    e.cell(true, cell_ticks(nominal, cell, pct, 9));
}

/// A low run of `bits` bit times, then two of idle, on the TX pad: brk
/// and autobaud_tx's break, and bitbang's break ahead.
template <typename E>
bool play_break(uint8_t ch, uint32_t bit_ticks, uint32_t bits) {
    Edges e{};
    e.now = engine_lead;
    e.cell(false, bits * bit_ticks);
    e.cell(true, 2u * bit_ticks);
    return play<E>(ch, e);
}

// ---- accounting -------------------------------------------------------------------

[[gnu::always_inline]] inline void account(link::Report& r, uint16_t st, uint16_t d, uint16_t mask) {
    if ((st & (usart_fe | usart_pe | usart_ore)) != 0u) {
        if ((st & usart_fe) != 0u && r.ferr < 255u) ++r.ferr;
        if ((st & usart_pe) != 0u && r.perr < 255u) ++r.perr;
        if ((st & usart_ore) != 0u && r.ovf < 255u) ++r.ovf;
    }
    r.sum = static_cast<uint16_t>(r.sum + (d & mask));
    ++r.count;
}

// ---- the actions --------------------------------------------------------------------

/// echo, sink and calibrate's receiving half: the frames counted and
/// summed, the FIRST and the LAST of the commanded count stamped on the
/// STK's cycle count as their RXNE is seen - two calls a run, none a
/// character: the loop between is a status load, a data load, the tally
/// and, for echo, the store. A run the deadline ends short reports no
/// span (usart_link.hpp: 0 = not measured).
/// The last receive's timeline in milliseconds after its ack: the window
/// opening, the first frame, the end - console observability.
uint32_t last_window_ms = 0, last_first_ms = 0, last_end_ms = 0;
link::Report run_receive(const link::Params& a, bool echo) {
    link::Report r{};
    settle();
    flush_rx();
    last_window_ms = Ticker::millis() - ack_ms;
    const uint16_t count = a.count;
    const uint16_t mask = data_mask;
    uint32_t first = 0, last = 0;
    const uint32_t t0 = Ticker::millis();
    while (r.count < count) {
        const uint16_t st = L::status();
        if ((st & usart_rxne) != 0u) {
            // DATAR first, the stamp after: a frame that completes between
            // the status read and the data read is LOST, its overrun
            // cleared unseen by that very pair (14.8.1's clear sequence),
            // and a stamp between them is two calls long. Each stamp is a
            // fixed data read late, which the span cancels.
            const uint16_t d = L::read_word();
            if (r.count == 0u) {
                first = Ticker::cycles();
                last_first_ms = Ticker::millis() - ack_ms;
            } else if (r.count + 1u == count) {
                last = Ticker::cycles();
            }
            account(r, st, d, mask);
            if (echo) {
                while (!L::tx_empty()) {
                }
                L::write_word(d);
            }
            continue;
        }
        if (Ticker::millis() - t0 >= a.ms) {
            r.flags |= link::report_timed_out;
            break;
        }
    }
    last_end_ms = Ticker::millis() - ack_ms;
    if (r.count >= 2u && r.count == count) {
        r.span = ticks_of(last - first);
    }
    wait_line_idle();
    return r;
}

link::Report run_calibrate(const link::Params& a) {
    link::Report r = run_receive(a, false);
    const uint16_t half_bits = link::frame_half_bits(a.cfg.bits, a.cfg.parity, a.cfg.stop);
    const uint32_t nominal = link::nominal_span(a.cfg.rate, half_bits, r.count);
    r.aux16 = static_cast<uint16_t>(link::calibrate_ppm(nominal, r.span));
    return r;
}

link::Report run_send(const link::Params& a, bool blast) {
    link::Report r{};
    settle();
    if (!blast && a.aux16 != 0u) {
        delay_us_now(static_cast<uint32_t>(a.aux16) * 1000u);
    }
    const uint32_t gap_us = blast ? 0u : (2u * 1'000'000u) / (a.cfg.rate != 0u ? a.cfg.rate : 1u);
    const uint16_t repeats = blast && a.aux16 > 1u ? a.aux16 : 1u;
    const uint32_t repeat_gap_us = static_cast<uint32_t>(a.aux8) * 100u;
    link::Stream stream(a.pattern, a.value);
    const uint16_t mask = data_mask;
    const uint32_t t0 = Ticker::millis();
    for (uint16_t k = 0; k < repeats; ++k) {
        if (k != 0u) {
            if (a.ms != 0u && Ticker::millis() - t0 >= a.ms) {
                r.flags |= link::report_timed_out;
                break;
            }
            if (repeat_gap_us != 0u) {
                delay_us_now(repeat_gap_us);
            }
        }
        for (uint16_t i = 0; i < a.count; ++i) {
            const uint16_t v = static_cast<uint16_t>(stream.next() & mask);
            while (!L::tx_empty()) {
            }
            L::write_word(v);
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++r.count;
            if (gap_us != 0u) {
                delay_us_now(gap_us);
            }
        }
    }
    wait_line_idle();
    return r;
}

link::Report run_send_mpcm(const link::Params& a, const uint8_t* pairs) {
    link::Report r{};
    settle();
    uint8_t seq = a.value;
    for (uint8_t g = 0; g < a.groups; ++g) {
        const uint16_t addr = static_cast<uint16_t>(0x100u | pairs[2u * g]);
        while (!L::tx_empty()) {
        }
        L::write_word(addr);
        r.sum = static_cast<uint16_t>(r.sum + addr);
        ++r.count;
        for (uint8_t k = 0; k < pairs[2u * g + 1u]; ++k) {
            const uint16_t v = seq++;
            while (!L::tx_empty()) {
            }
            L::write_word(v);
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++r.count;
        }
    }
    wait_line_idle();
    return r;
}

link::Report run_break(const link::Params& a) {
    link::Report r{};
    settle();
    tenure_begin<TxPin>(tx_ch);
    const bool ok = play_break<TxEdges>(tx_ch, cycles_per_bit(a.cfg.rate) / 2u, a.count);
    tenure_end<TxPin>(tx_ch);
    r.count = a.count;
    r.aux16 = L::brr();
    if (!ok) r.flags |= link::report_mismatch;
    return r;
}

link::Report run_autobaud_tx(const link::Params& a) {
    link::Report r{};
    settle();
    tenure_begin<TxPin>(tx_ch);
    const bool ok = play_break<TxEdges>(tx_ch, cycles_per_bit(a.cfg.rate) / 2u, a.aux16 != 0u ? a.aux16 : 16u);
    tenure_end<TxPin>(tx_ch);
    put_link(a.value);
    r.sum = a.value;
    ++r.count;
    for (uint8_t k = 0; k < a.aux8; ++k) {
        const uint8_t v = static_cast<uint8_t>(0xA0u + k);
        put_link(v);
        r.sum = static_cast<uint16_t>(r.sum + v);
        ++r.count;
    }
    wait_line_idle();
    r.aux16 = L::brr();
    if (!ok) r.flags |= link::report_mismatch;
    return r;
}

/// bitbang on the TX pad, drive_rxd on the RX pad: `count` frames of
/// `value`, one cell distorted, a break ahead when aux16 asks, four bit
/// times of idle between frames.
template <typename Pad, typename E>
link::Report run_bitbang(const link::Params& a, uint8_t ch) {
    link::Report r{};
    const uint32_t nominal = cycles_per_bit(a.cfg.rate) / 2u;
    tenure_begin<Pad>(ch);
    settle();
    bool ok = true;
    if (a.aux16 != 0u) {
        ok = play_break<E>(ch, nominal, a.aux16) && ok;
    }
    for (uint16_t i = 0; i < a.count; ++i) {
        Edges e{};
        e.now = engine_lead;
        frame_edges(e, nominal, a.value, a.cell, a.pct);
        e.cell(true, 4u * nominal);   // a clean idle between frames
        ok = play<E>(ch, e) && ok;
        r.sum = static_cast<uint16_t>(r.sum + a.value);
        ++r.count;
    }
    tenure_end<Pad>(ch);
    r.aux16 = static_cast<uint16_t>(nominal > 0xFFFFu ? 0xFFFFu : nominal);
    if (!ok) r.flags |= link::report_mismatch;
    return r;
}

/// `count` low pulses of `aux16` ticks on the idle TX line, `aux8`
/// microseconds apart: ONE PULSE each - PWM mode 1 high while the
/// counter is under the compare, low from it to the reload, the counter
/// stopping itself at the update (OPM) back at zero, high.
link::Report run_glitch(const link::Params& a) {
    link::Report r{};
    tenure_begin<TxPin>(tx_ch);
    settle();
    const uint32_t width = a.aux16 != 0u ? a.aux16 : 1u;
    const uint16_t psc = 2u * width + 2u <= 0xFFFFu ? 0u : 1u;   // HCLK, or the tick
    const uint32_t counts = psc == 0u ? 2u * width : width;
    constexpr uint16_t delay_counts = 2;
    bool ok = T1::configure(TimConfig{.prescaler = psc,
                                      .period = static_cast<uint16_t>(delay_counts + counts - 1u),
                                      .one_pulse = true});
    ok = ok && T1::output_channel(tx_ch, TimChannelConfig{.mode = TimOutputMode::pwm1, .compare = delay_counts,
                                                          .preload = false, .enable = true});
    for (uint16_t i = 0; ok && i < a.count; ++i) {
        T1::enable(true);
        while (T1::enabled()) {
        }
        delay_us_now(a.aux8 != 0u ? a.aux8 : 1u);
        ++r.count;
    }
    (void)T1::output_channel(tx_ch, TimChannelConfig{.mode = TimOutputMode::force_active,
                                                     .preload = false, .enable = true});
    tenure_end<TxPin>(tx_ch);
    r.aux16 = static_cast<uint16_t>(width);
    if (!ok) r.flags |= link::report_cfg_failed;
    return r;
}

/// The RX pad as an emulated open drain: low is an output low (OUTDR
/// cleared first, so the input's pull is down for the one store between),
/// high is the input pulled up (the configuration first, then OUTDR).
void od_low() {
    RxPin::clear();
    RxPin::output();
}
void od_release() {
    RxPin::input(PinPull::down);
    RxPin::set();
}

/// Wait for one frame of the DUT, then `aux16` microseconds, then send
/// `value` as an 8N1 frame on the RX pad through the emulated open drain,
/// each bit's edge placed on the STK from the frame's start.
link::Report run_collide(const link::Params& a) {
    link::Report r{};
    settle();
    flush_rx();
    const uint32_t bit = cycles_per_bit(a.cfg.rate);
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        const uint16_t st = L::status();
        if ((st & usart_rxne) == 0u) {
            continue;
        }
        account(r, st, L::read_word(), data_mask);
        delay_us_now(a.aux16);
        const uint16_t frame = static_cast<uint16_t>((static_cast<uint16_t>(a.value) << 1) | 0x200u);
        const uint32_t c0 = Ticker::cycles();
        for (uint8_t i = 0; i < 10u; ++i) {
            if (((frame >> i) & 1u) != 0u) od_release(); else od_low();
            while (Ticker::cycles() - c0 < (i + 1u) * bit) {
            }
        }
        od_release();
        hold_pads();
        flush_rx();
        r.aux16 = a.value;
        return r;
    }
    r.flags |= link::report_timed_out;
    return r;
}

// ---- the command handler -------------------------------------------------------------

void ident_label(char out[8]) {
    const char* name = device::part_name + 4;   // the part number after its CH32 prefix
    for (uint8_t i = 0; i < 8u; ++i) {
        out[i] = *name != '\0' ? *name++ : '\0';
    }
}

void handle(const link::Frame& f) {
    ++commands;
    engaged_until = Ticker::millis() + link::engage_ms;
    const Op op = f.op;
    if (trace) {
        print(console, "  [cmd op=", hex(link::byte_of(op)), " len=", f.len, "]", crlf);
    }
    if (op == Op::standby) {
        // The ack first, while the line is still this end's; the stand-off
        // after it: answer() holds the pads as the state allows, and a
        // stand-off already running would float the TX pad under its own
        // ack, which the DUT would never hear.
        const uint16_t ms = f.len >= link::params_size ? link::get_params(f.data).ms : 0u;
        ack(op, f.sum, true);
        standby_until = ms != 0u ? Ticker::millis() + ms : 0u;
        hold_pads();   // a stand-off lets go of the line: dark
        return;
    }
    if (standing_off()) {
        return;   // deaf AND mute but for the standby that ends it
    }
    standby_until = 0;

    if (op == Op::ping) {
        ack(op, f.sum, true);
        return;
    }
    if (op == Op::ident) {
        ack(op, f.sum, true);
        link::Ident d{};
        ident_label(d.label);
        d.xtal = crystal_ok ? 1u : 0u;
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

    // What this peer refuses whatever the Cfg says.
    const bool refused = op == Op::onewire || (op == Op::calibrate && !crystal_ok);
    const bool ok = !refused && apply_cfg(a.cfg);
    if (!ok) {
        r.flags |= link::report_cfg_failed;
    } else {
        switch (op) {
            case Op::set_link: {
                settle();
                const uint32_t t0 = Ticker::millis();
                while (Ticker::millis() - t0 < a.ms) {
                    const uint16_t st = L::status();
                    if ((st & usart_rxne) != 0u) {
                        account(r, st, L::read_word(), data_mask);
                    }
                }
                r.flags |= link::report_timed_out;
                break;
            }
            case Op::echo: r = run_receive(a, true); break;
            case Op::sink: r = run_receive(a, false); break;
            case Op::calibrate: r = run_calibrate(a); break;
            case Op::send: r = run_send(a, false); break;
            case Op::blast: r = run_send(a, true); break;
            case Op::send_mpcm: r = run_send_mpcm(a, f.data + link::params_size); break;
            case Op::brk: r = run_break(a); break;
            case Op::autobaud_tx: r = run_autobaud_tx(a); break;
            case Op::bitbang: r = run_bitbang<TxPin, TxEdges>(a, tx_ch); break;
            case Op::glitch: r = run_glitch(a); break;
            // The DUT's TX wire is this board's RX pad.
            case Op::drive_rxd: r = run_bitbang<RxPin, RxEdges>(a, rx_ch); break;
            case Op::collide: r = run_collide(a); break;
            default: r.flags |= link::report_cfg_failed; break;
        }
        if ((r.flags & link::report_cfg_failed) == 0u) {
            r.flags |= link::report_ran;
        }
    }
    const uint32_t took = Ticker::millis() - started;
    r.ms = took > 255u ? 255u : static_cast<uint8_t>(took);
    r.op = link::byte_of(op);
    last_report = r;
    engaged_until = Ticker::millis() + link::engage_ms;
    (void)command_mode();
}

// ---- the console ------------------------------------------------------------------------

void help() {
    print(console, "usart_peer: ? help | i status | 0 command mode, dark | 3 trace", crlf);
}

void status() {
    print(console, "  link USART2 column 4 (TX PB0, RX PB1) ", proven ? "PROVEN" : "searching (dark)",
          standing_off() ? ", STAND-OFF" : "", crlf);
    print(console, "  BRR=", L::brr(), " commands=", commands, " actions=", actions, " naks=", naks, crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count, " sum=",
          hex(last_report.sum), " ferr=", last_report.ferr, " perr=", last_report.perr, " ovf=",
          last_report.ovf, " flags=", hex(last_report.flags), " aux=", last_report.aux16, " ms=",
          last_report.ms, " span=", last_report.span, crlf);
    print(console, "  the engine: ", plays, " lists played, ", play_timeouts, " never completed, ", play_refused,
          " refused", crlf);
    print(console, "  clk ", crystal_ok ? "HSE" : "HSI", " VDD ", supply_mv, " mV; the last receive: window at ",
          last_window_ms, " ms after the ack, first frame at ", last_first_ms, ", end at ", last_end_ms, crlf);
}

/// VDD from VREFINT (adc.hpp's supply_mv), the converter released after.
uint16_t read_supply_mv() {
    if (!Adc::init(clock)) {
        return 0;
    }
    Adc::sample_time_all(adc_sample_longest);
    Adc::select(AdcInput::vrefint);
    const uint16_t mv = Adc::supply_mv(Adc::read_settled(4));
    Adc::release();
    return mv;
}

constexpr uint16_t supply_floor_mv = 3000;

}  // namespace

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_VECTOR(usart1_handler) { (void)Console::isr(); }

int main() {
    crystal_ok = SysClock::init();
    const bool clock_ok = crystal_ok || HsiClock::init();
    (void)Console::init(clock, 115200);
    (void)Ticker::init(clock);
    enable_interrupts();

    supply_mv = read_supply_mv();
    supply_ok = supply_mv >= supply_floor_mv;

    print(console, crlf, "usart_peer - USART instrument peer on ", device::part_name, ", clk=",
          crystal_ok ? "HSE 24 MHz x2" : (clock_ok ? "HSI x2 (NO CRYSTAL)" : "FAILED"), ", fw ",
          hex(firmware_version), ", VDD ", supply_mv, " mV", crlf);
    if (!supply_ok) {
        print(console, "SUPPLY LOW (under ", supply_floor_mv, " mV): the link stays dark", crlf);
    }
    print(console, "link USART2 column 4: TX PB0, RX PB1; command mode 8N1 @ ", link::command_baud, crlf);

    T1::init();
    (void)T1::remap(tim1_column);
    TxEdges::arm(T1::ccr_address(tx_ch), 0u, DmaPriority::very_high);
    RxEdges::arm(T1::ccr_address(rx_ch), 0u, DmaPriority::very_high);
    if (supply_ok) {
        (void)command_mode();
    }
    last_frame_ms = Ticker::millis();
    help();
    print(console, "> ");

    for (;;) {
        uint8_t c = 0;
        if (Console::read_byte(c)) {
            if (c != '\r' && c != '\n') {
                print(console, static_cast<char>(c), crlf);
                if (c == '?') {
                    help();
                } else if (c == 'i') {
                    status();
                } else if (c == '0') {
                    proven = false;
                    engaged_until = 0;
                    standby_until = 0;
                    if (supply_ok) {
                        (void)command_mode();
                    }
                    print(console, "  command mode, dark until a frame checks out", crlf);
                } else if (c == '3') {
                    trace = !trace;
                    print(console, trace ? "  trace on" : "  trace off", crlf);
                } else {
                    print(console, "? for help", crlf);
                }
                print(console, "> ");
            }
        }
        if (!supply_ok) {
            continue;
        }
        const uint32_t now = Ticker::millis();
        if (standby_until != 0u && now >= standby_until) {
            standby_until = 0;
            hold_pads();
        }
        if (proven && standby_until == 0u && now - last_frame_ms > link::rediscover_ms) {
            proven = false;
            hold_pads();   // the wiring is unknown again: dark
        }
        if (decoder.partial() && now - last_byte_ms > 50u) {
            decoder.reset();
        }
        const uint16_t st = L::status();
        if ((st & usart_rxne) == 0u) {
            continue;
        }
        const uint16_t d = L::read_word();
        last_byte_ms = Ticker::millis();
        if ((st & (usart_fe | usart_pe | usart_ne | usart_ore)) != 0u) {
            decoder.reset();
            continue;
        }
        switch (decoder.feed(static_cast<uint8_t>(d))) {
            case link::Decoder::Result::frame:
                if (!proven) {
                    proven = true;
                    hold_pads();
                }
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
            default:
                break;
        }
    }
}
