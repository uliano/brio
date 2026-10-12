// usart_peer - the INSTRUMENT half of a two-board USART test on a
// CH32X035: the scriptable peer the far board's suite (the DUT) drives IN
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
// THE LINK IS USART4 ON ITS DEFAULT COLUMN (8.3.2.1, code 000): TX PB0,
// RX PB1, polled through the resource - no interrupt, no ring: an
// instrument's loop owns the instance (RM ch. 14). The desk wires it
// crossed, full duplex, to the CH32V006's USART2 on its column 4; the peer
// serves full duplex alone and refuses onewire (no USART TX of this
// package reaches its own RX pad). The console is USART2 on PA2/PA3, the
// probe's serial, at 115200: observability only.
//   ? help | i status | 0 back to command mode, dark | 3 trace | c the clock
//
// THE DARK DISCIPLINE (usart_link.hpp): while SEARCHING - no frame has
// checked out, or the channel has been quiet for rediscover_ms, or a
// stand-off is running - PB0 and PB1 are floating inputs: the transmitter
// reaches no wire and nothing is pulled, so the far board's single-board
// letters walk those wires with their own pulls. A frame that checks out
// PROVES the wiring: PB0 goes to the USART (idling high) and PB1 takes its
// pull-up. A bad checksum is nak'ed only while ENGAGED.
//
// TIME (usart_link.hpp, TIME IN THE PAYLOAD). This series has ONE clock
// root, the 48 MHz HSI, +-1.7 % over 0..70 C (the datasheet's table 3-9)
// and no crystal oscillator at all, so this peer starts as an RC peer
// (Ident::xtal = 0) and becomes as good as its DUT through CALIBRATE: the
// DUT, on a crystal, sends `count` gapless frames; the peer stamps the
// first and the last on its STK (HCLK cycles), and since the DUT's span is
// nominal_span() exactly, HCLK's real rate is the cycles over it. That rate
// is ADOPTED - kept in `hclk` and used for every divisor, every tick a
// Report carries and every edge the waveform engine places - and the
// HSITRIM field (RM 3.4.1, five bits about 116 kHz a step on the bench die,
// docs/ch32x035/clock.md) is moved to the step NEAREST 48 MHz first: a
// calibrate that finds the nearest step another one MOVES the trim, keeps
// the rate it predicts and answers xtal = 0 still, and the DUT calibrates
// again; one that finds the trim nearest ADOPTS the measured rate and
// answers xtal = 2 from then on. Every rate a calibrate measured, trim by
// trim, is kept, so a step the HSI's uneven ladder makes too big is not
// walked back and forth.
//
// THE HSI WANDERS: the same die read a few hundred ppm apart in runs a
// fraction of a second apart (docs/ch32x035/clock.md) - an adopted rate is
// good to that much and no better, and a DUT that judges a figure finer
// than that calibrates again right before it (test_ch32_serial's
// two-board key does, letter by letter). Everything this peer serves - a
// rate tolerance of 2 to 8 per cent, a stretched bit cell, a span at 500
// ppm - is coarser.
//
// THE WAVEFORM ENGINE (bitbang, drive_rxd, brk, autobaud_tx's break): TIM1
// under its remap 001 (8.3.2.1) puts its COMPLEMENTARY outputs C2N on PB0
// and C3N on PB1 - the only timer outputs that reach the link pads on this
// package - and every other TIM1 signal on a pad this image leaves a GPIO
// input. A channel's complementary output alone (CCxNE with CCxE clear)
// carries OCxREF itself, no dead time, MOE set (ch32x035/tim.hpp). A
// frame's edges are ABSOLUTE compare values computed before the action, in
// HCLK cycles of the adopted rate, streamed by DMA - CH2's request on
// channel 3, CH3's on 6 (RM 9.2.3) - into the channel's compare register
// on the channel's own match, the channel in toggle mode: no CPU between
// edges, nothing masked; the channel keeps up with edges 10 HCLK cycles
// apart (measured, docs/ch32x035/tim.md). The list ends with a value half
// the counter away, which the last edge's request moves into the compare
// register and no match reaches before the CPU, seeing the channel's
// completion, forces the output high. A glitch is ONE PULSE (OPM, PWM mode
// 1): exact to one HCLK cycle. The pad passes between the USART and the
// timer as GPIO high, then the other function, each step one
// configuration-nibble store with the line idle high throughout - these
// chips have no per-pin AF selector (8.2.4) - and while the timer owns PB0
// USART4 is moved to its column 1, whose pads stay GPIO inputs, so no
// second alternate output reaches PB0.
//
// THE OUTPUTS HOLD WHILE THE COUNTER IS STOPPED and a timer that never ran
// holds them LOW (measured, docs/ch32x035/tim.md): the engine's tenure
// starts the counter BEFORE the pad is handed to the timer, and stops it
// only with the line forced high, a level the stop then holds.
//
// THE PER-OP NOTES. onewire is refused (above); collide is an emulated
// open drain on the RX pad (an output low, or the input's pull-up),
// paced on the STK, a wired-AND only with a DUT whose own pad is an open
// drain - the CH32V006's HDSEL pad is one - and this peer's receiver reads
// the wire while it pulls: the Report's aux16 is the frame it heard, the
// AND of the two; sync_client, sync_host and every flag this column has
// not got (clk2x: no OVER8 on this series; invert_xck: no clock pad on
// this column; mpcm on the peer's receiver) are refused; 5- and 6-bit
// words and 9 data bits with a parity bit are refused; IrDA is served
// (IREN, the prescaler; txpl != 0 asks the low-power mode at PSC = txpl);
// hold_pad drives PB0 or PB1 at a level (no clock pad: pad_xck refused);
// stage is refused (a train drawn whole wants an edge table this engine
// plays one frame at a time).
//
// THE SUPPLY at boot (the desk's rule for an image that drives a link
// pad): the voltage detector at PLS 10, 3.0 V (pwr.hpp); below it the peer
// stays dark and says so on its console, its link never opened.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/tim.hpp"
#include "ch32x035/usart.hpp"
#include "util/print.hpp"

#include "../../../avrdx/src/apps/usart_link.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;
using link::Op;

using Console = Uart<2, P>;
constexpr Console console;

constexpr uint8_t link_column = 0;      ///< USART4: TX PB0, RX PB1
constexpr uint8_t parked_column = 1;    ///< USART4: TX PA5, RX PA9 - pads this image leaves inputs
using L = Usart<4>;
using TxPin = Pin<'B', 0>;
using RxPin = Pin<'B', 1>;
static_assert(afio_usart_pads(4, link_column).tx == Pad{'B', 0} && afio_usart_pads(4, link_column).rx == Pad{'B', 1},
              "8.3.2.1: USART4's code 000 is TX PB0, RX PB1");

using T1 = Tim<1>;
constexpr uint8_t tim1_column = 1;      ///< 001: C2N PB0, C3N PB1
constexpr uint8_t tx_ch = 1;            ///< CH2, 0-based: C2N on the TX pad
constexpr uint8_t rx_ch = 2;            ///< CH3: C3N on the RX pad
static_assert(afio_tim1_pads(tim1_column).ch2n == TxPin::pad && afio_tim1_pads(tim1_column).ch3n == RxPin::pad,
              "8.3.2.1: TIM1 code 001 puts C2N on PB0 and C3N on PB1");
using TxEdges = DmaTxEngine<T1::compare_dma_channel(tx_ch), uint32_t>;   ///< 9.2.3: TIM1 CH2's request, 3
using RxEdges = DmaTxEngine<T1::compare_dma_channel(rx_ch), uint32_t>;   ///< TIM1 CH3's, 6
static_assert(TxEdges::channel == 3 && RxEdges::channel == 6);

constexpr uint16_t firmware_version = static_cast<uint16_t>((link::peer_ch32x035 << 8) | 0x01u);

/// usart_link.hpp's tick: 24 MHz.
constexpr uint32_t tick_hz = 24'000'000;

bool supply_ok = false;
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

// ---- the time base ------------------------------------------------------------

/// HCLK as this peer believes it: the nominal 48 MHz until a calibrate
/// measures it, then the adopted rate (the file header's TIME).
uint32_t hclk = SysClock::hz;
/// Ident::xtal: 0 an RC peer, 2 once a calibrate adopted the DUT's clock.
uint8_t xtal_state = 0;
/// What every calibrate measured, trim by trim (0 = never measured): the
/// rate in hertz.
uint32_t trim_hz[32] = {};
/// The HSI's step in hertz the move is predicted with (RM 3.4.1's "about
/// 110 kHz"; the bench die's mean, docs/ch32x035/clock.md).
constexpr int32_t trim_step_hz = 116'000;

/// HCLK cycles to protocol ticks at the believed rate.
uint32_t ticks_of(uint32_t cycles) {
    return static_cast<uint32_t>((static_cast<uint64_t>(cycles) * tick_hz + hclk / 2u) / hclk);
}
/// Protocol ticks to HCLK cycles.
uint32_t cycles_of_ticks(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * hclk + tick_hz / 2u) / tick_hz);
}
/// One bit at `rate`, in HCLK cycles of the believed rate.
uint32_t cycles_per_bit(uint32_t rate) { return rate == 0u ? 5000u : (hclk + rate / 2u) / rate; }

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(link::settle_ms) + 2u) {
    }
}

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
        (void)RxPin::input(PinPull::up);
    } else {
        TxPin::release();
        RxPin::release();
    }
}

/// Drain whatever the receiver holds and clear its errors (STATR, then
/// DATAR: the F1 sequence 14.10.1 prescribes).
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

/// What this silicon can make of a Cfg at the believed HCLK; empty when it
/// cannot.
std::optional<LinkFormat> format_of(const link::Cfg& c) {
    if (c.mode == link::Mode::sync_client || c.mode == link::Mode::sync_host) {
        return std::nullopt;   // USART4's CK on this column is PB2, which the QFN20 does not bond
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
    const uint32_t brr = usart_divisor(hclk, c.rate);
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
/// transmitter and the receiver on. The pads are hold_pads()'. A proven TX
/// pad is parked as a GPIO output high across the reset and the
/// reconfiguration: a USART in reset drives its alternate output LOW (the
/// CH32V006's peer measured that start bit of garbage on the same block),
/// and the hand-over waits two bit times and 20 us after TE for the same
/// reason (the low pulse a TX pad shows when it is handed over as TE rises).
bool open_link(const LinkFormat& f) {
    if (proven && !standing_off()) {
        TxPin::output(true);
    }
    L::bus_clock(true);
    L::reset();
    if (!L::remap(link_column) || !L::configure(f.format, f.brr)) {
        return false;
    }
    if (f.irda && !L::irda(f.ir)) {
        return false;
    }
    L::enable(true);
    L::transmitter(true);
    L::receiver(true);
    data_mask = uart_data_mask(f.format);
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

/// A list's edges as HCLK cycles from the counter's start at the believed
/// rate, built before the action: play() turns them into counts once,
/// outside the timed part.
struct Edges {
    static constexpr uint8_t capacity = 32;
    uint32_t t[capacity] = {};
    uint8_t n = 0;
    uint32_t now = 0;        ///< the running position, in cycles
    bool level = true;       ///< the line's level at `now`
    bool overflow = false;

    void cell(bool high, uint32_t cycles) {
        if (high != level) {
            if (n + 1u >= capacity) {   // one slot kept for the end marker
                overflow = true;
            } else {
                t[n++] = now;
            }
            level = high;
        }
        now += cycles;
    }
};

/// The counter's start lead: the first edge this many cycles after CEN.
constexpr uint32_t engine_lead = 128;
/// The longest list a counter division serves: under half the counter, so
/// that the end marker half the counter beyond the last edge is never
/// matched while the list's tail plays out.
constexpr uint32_t engine_span = 30'000;
/// The channel keeps up with edges this many HCLK cycles apart and not
/// closer (measured, docs/ch32x035/tim.md): a cell narrower is widened.
constexpr uint32_t engine_min_cycles = 12;

/// The channel's output configuration for the engine: the complementary
/// output alone, no preload, in `mode`.
TimChannelConfig engine_channel(TimOutputMode mode) {
    return TimChannelConfig{.mode = mode, .preload = false, .enable = false, .complementary_enable = true};
}

/// The engine's tenure of a pad: GPIO high, the channel forced high on a
/// RUNNING counter (the outputs hold while it is stopped, and a timer that
/// never ran holds them low - the file header), then the timer's function
/// - and for PB0 USART4 moved to its column 1 first, so that no second
/// alternate output reaches the pad.
template <typename Pad>
void tenure_begin(uint8_t ch) {
    Pad::output(true);
    if (ch == tx_ch) {
        (void)L::remap(parked_column);
    }
    (void)T1::configure(TimConfig{.prescaler = 0, .period = 0xFFFF});
    (void)T1::output_channel(ch, engine_channel(TimOutputMode::force_active));
    T1::main_output(true);
    T1::enable(true);
    delay_us_now(2);
    Pad::function();
}

/// And back: GPIO high, the channel disabled, the USART's column again,
/// the pads as the state allows.
template <typename Pad>
void tenure_end(uint8_t ch) {
    Pad::output(true);
    (void)T1::output_channel(ch, TimChannelConfig{.mode = TimOutputMode::force_active, .preload = false,
                                                  .enable = false, .complementary_enable = false});
    T1::enable(false);
    if (ch == tx_ch) {
        (void)L::remap(link_column);
    }
    hold_pads();
    flush_rx();
}

/// The engine's own counts, console observability: lists played, lists
/// whose channel never completed within the list's length, lists refused.
uint32_t plays = 0, play_timeouts = 0, play_refused = 0;

/// Play one edge list on channel `ch` (its DMA engine `E`) and return
/// when the line has reached `e.now`, the end of the list's last cell.
/// The division: `div` cycles a count (PSC = div - 1), the smallest that
/// keeps the whole list under engine_span counts. The counter is stopped
/// while it is programmed - the line forced high, which it holds - and
/// started at zero; the first edge goes into the compare register by
/// store, the rest by the channel's requests, then the end marker.
template <typename E>
bool play(uint8_t ch, Edges& e) {
    using Channel = DmaChannel<E::channel>;
    ++plays;
    if (e.overflow || e.n == 0u) {
        ++play_refused;
        return false;
    }
    const uint32_t div = e.now / engine_span + 1u;
    if (div > 65'536u) {
        ++play_refused;
        return false;
    }
    for (uint8_t i = 0; i < e.n; ++i) {
        e.t[i] = (e.t[i] + div / 2u) / div;
    }
    e.t[e.n] = e.t[e.n - 1u] + 0x8000u;   // the end marker
    const uint32_t end = e.now / div;
    T1::enable(false);
    if (!T1::configure(TimConfig{.prescaler = static_cast<uint16_t>(div - 1u), .period = 0xFFFF})) {
        return false;
    }
    if (!E::start(std::span<const uint32_t>(&e.t[1], e.n))) {
        ++play_refused;
        return false;
    }
    // THE FIRST EDGE INTO THE COMPARE REGISTER BEFORE THE MODE: a register
    // still holding what the stopped counter reads would be a match the
    // moment toggle mode lands, a request the enable below would serve
    // with the list a step ahead. The flag is cleared after both, before
    // the request is enabled.
    (void)T1::set_compare(ch, static_cast<uint16_t>(e.t[0]));
    (void)T1::output_mode(ch, TimOutputMode::toggle);
    T1::clear_flags(T1::compare_flag(ch));
    T1::interrupts(T1::compare_dma(ch), true);
    T1::enable(true);
    // The last edge's request has moved the end marker: the line is at its
    // final level. Bounded by the list's own length on the counter.
    const uint32_t budget = 2u * end * div + 48'000u;
    const uint32_t c0 = Ticker::cycles();
    bool done = false;
    while (!(done = Channel::flag(DmaFlag::complete))) {
        if (Ticker::cycles() - c0 > budget) {
            break;
        }
    }
    (void)T1::output_mode(ch, TimOutputMode::force_active);
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
    return done;
}

/// The width of bit cell `idx` (0 = start, 1..8 = data, 9 = stop) in
/// cycles, stretched or shrunk by `pct` when it is the chosen one.
uint32_t cell_cycles(uint32_t nominal, int8_t cell, int8_t pct, int8_t idx) {
    int32_t v = static_cast<int32_t>(nominal);
    if (cell == idx && pct != 0) {
        v += (static_cast<int32_t>(nominal) * pct) / 100;
    }
    if (v < static_cast<int32_t>(engine_min_cycles)) {
        v = static_cast<int32_t>(engine_min_cycles);
    }
    return static_cast<uint32_t>(v);
}

/// One 8N1 frame, LSB first.
void frame_edges(Edges& e, uint32_t nominal, uint8_t value, int8_t cell, int8_t pct) {
    e.cell(false, cell_cycles(nominal, cell, pct, 0));
    for (int8_t i = 0; i < 8; ++i) {
        e.cell(((value >> i) & 1u) != 0u, cell_cycles(nominal, cell, pct, static_cast<int8_t>(i + 1)));
    }
    e.cell(true, cell_cycles(nominal, cell, pct, 9));
}

/// A low run of `bits` bit times, then two of idle: brk, autobaud_tx's
/// break, and bitbang's break ahead.
template <typename E>
bool play_break(uint8_t ch, uint32_t bit_cycles, uint32_t bits) {
    Edges e{};
    e.now = engine_lead;
    e.cell(false, bits * bit_cycles);
    e.cell(true, 2u * bit_cycles);
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

/// The last receive's timeline in milliseconds after its ack, and its
/// first-to-last span in cycles: console observability.
uint32_t last_window_ms = 0, last_first_ms = 0, last_end_ms = 0, last_span_cycles = 0;
uint16_t last_first_status = 0;

/// echo, sink and calibrate's receiving half: the frames counted and
/// summed, the FIRST and the LAST of the commanded count stamped on the
/// STK's cycle count as their RXNE is seen (each stamp a fixed data read
/// late, which the span cancels) - two calls a run, none a character: the loop between is a status load, a data load, the tally
/// and, for echo, the store. A run the deadline ends short reports no
/// span (usart_link.hpp: 0 = not measured). `span_cycles` is the span in
/// HCLK cycles, for calibrate.
link::Report run_receive(const link::Params& a, bool echo, uint32_t& span_cycles) {
    link::Report r{};
    settle();
    flush_rx();
    last_window_ms = Ticker::millis() - ack_ms;
    const uint16_t count = a.count;
    const uint16_t mask = data_mask;
    uint32_t first = 0, last = 0;
    const uint32_t t0 = Ticker::millis();
    span_cycles = 0;
    while (r.count < count) {
        const uint16_t st = L::status();
        if ((st & usart_rxne) != 0u) {
            // DATAR first, the stamp after: a stamp is Ticker::cycles()'s
            // few dozen cycles, and a frame that completes between the
            // status read and the data read is LOST with its overrun
            // cleared unseen (the F1 clear sequence, STATR then DATAR) -
            // measured at 3 Mbaud with the stamp taken first.
            const uint16_t d = L::read_word();
            if (r.count == 0u) {
                first = Ticker::cycles();
                last_first_ms = Ticker::millis() - ack_ms;
                last_first_status = st;
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
        span_cycles = last - first;
        r.span = ticks_of(span_cycles);
    }
    last_span_cycles = span_cycles;
    wait_line_idle();
    return r;
}

/// The last calibrate's figures, console observability.
uint8_t last_trim_before = 0, last_trim_after = 0;
uint32_t last_measured_hz = 0;

/// THE ADOPTION (the file header's TIME): HCLK's real rate from the span,
/// kept per trim; the trim moved to the step predicted nearest 48 MHz
/// unless that step was measured already and found no nearer; adopted
/// (xtal 2) when it stays.
void adopt(uint32_t span_cycles, uint32_t nominal_ticks) {
    const uint32_t measured = static_cast<uint32_t>(
        (static_cast<uint64_t>(span_cycles) * tick_hz + nominal_ticks / 2u) / nominal_ticks);
    const uint8_t trim = Rcc::hsi_trim();
    trim_hz[trim] = measured;
    last_trim_before = trim;
    last_measured_hz = measured;
    const int32_t off = static_cast<int32_t>(measured) - static_cast<int32_t>(SysClock::hz);
    int32_t move = (off >= 0 ? off + trim_step_hz / 2 : off - trim_step_hz / 2) / trim_step_hz;
    int32_t target = static_cast<int32_t>(trim) - move;
    if (target < 0) target = 0;
    if (target > 31) target = 31;
    const auto distance = [](uint32_t hz) {
        return hz > SysClock::hz ? hz - SysClock::hz : SysClock::hz - hz;
    };
    const uint8_t next = static_cast<uint8_t>(target);
    if (next == trim || (trim_hz[next] != 0u && distance(trim_hz[next]) >= distance(measured))) {
        hclk = measured;
        xtal_state = 2;
    } else {
        Rcc::hsi_trim(next);
        hclk = trim_hz[next] != 0u
                   ? trim_hz[next]
                   : static_cast<uint32_t>(static_cast<int32_t>(measured) +
                                           (static_cast<int32_t>(next) - static_cast<int32_t>(trim)) * trim_step_hz);
        xtal_state = 0;
    }
    last_trim_after = Rcc::hsi_trim();
}

link::Report run_calibrate(const link::Params& a) {
    uint32_t span_cycles = 0;
    link::Report r = run_receive(a, false, span_cycles);
    const uint16_t half_bits = link::frame_half_bits(a.cfg.bits, a.cfg.parity, a.cfg.stop);
    const uint32_t nominal = link::nominal_span(a.cfg.rate, half_bits, r.count);
    r.aux16 = static_cast<uint16_t>(link::calibrate_ppm(nominal, r.span));
    if (span_cycles != 0u && nominal != 0u) {
        adopt(span_cycles, nominal);
    }
    return r;
}

/// send and blast's Report::aux16: the transmitter's real rate - the
/// adopted HCLK over the divisor - against the rate asked, stated only once
/// a calibrate has adopted a crystal (an RC peer cannot tell).
void state_rate(link::Report& r, uint32_t rate) {
    if (xtal_state == 2u) {
        r.aux16 = static_cast<uint16_t>(link::rate_error_ppm(hclk, L::brr(), rate));
        r.flags |= link::report_rate_stated;
    }
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
    state_rate(r, a.cfg.rate);
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
    const bool ok = play_break<TxEdges>(tx_ch, cycles_per_bit(a.cfg.rate), a.count);
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
    const bool ok = play_break<TxEdges>(tx_ch, cycles_per_bit(a.cfg.rate), a.aux16 != 0u ? a.aux16 : 16u);
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
/// times of idle between frames - every cell at the rate asked on the
/// believed HCLK, so an adopted peer's banged rate is its DUT's crystal's.
template <typename Pad, typename E>
link::Report run_bitbang(const link::Params& a, uint8_t ch) {
    link::Report r{};
    const uint32_t nominal = cycles_per_bit(a.cfg.rate);
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
    const uint32_t t = ticks_of(nominal);
    r.aux16 = static_cast<uint16_t>(t > 0xFFFFu ? 0xFFFFu : t);
    if (!ok) r.flags |= link::report_mismatch;
    return r;
}

/// `count` low pulses of `aux16` ticks on the idle TX line, `aux8`
/// microseconds apart: ONE PULSE each - PWM mode 1 high while the counter
/// is under the compare, low from it to the reload, the counter stopping
/// itself at the update (OPM) back at zero, high, a level it holds.
link::Report run_glitch(const link::Params& a) {
    link::Report r{};
    tenure_begin<TxPin>(tx_ch);
    settle();
    const uint32_t ticks = a.aux16 != 0u ? a.aux16 : 1u;
    const uint32_t width = cycles_of_ticks(ticks) != 0u ? cycles_of_ticks(ticks) : 1u;
    constexpr uint32_t delay_counts = 2;
    const uint32_t div = (width + delay_counts) / 0xFFF0u + 1u;
    const uint32_t counts = (width + div / 2u) / div != 0u ? (width + div / 2u) / div : 1u;
    T1::enable(false);
    bool ok = T1::configure(TimConfig{.prescaler = static_cast<uint16_t>(div - 1u),
                                      .period = static_cast<uint16_t>(delay_counts + counts - 1u),
                                      .one_pulse = true});
    (void)T1::set_compare(tx_ch, delay_counts);
    ok = ok && T1::output_mode(tx_ch, TimOutputMode::pwm1);
    for (uint16_t i = 0; ok && i < a.count; ++i) {
        T1::enable(true);
        while (T1::enabled()) {
        }
        delay_us_now(a.aux8 != 0u ? a.aux8 : 1u);
        ++r.count;
    }
    tenure_end<TxPin>(tx_ch);
    r.aux16 = static_cast<uint16_t>(ticks);
    if (!ok) r.flags |= link::report_cfg_failed;
    return r;
}

/// The RX pad as an emulated open drain: low is an output low (OUTDR
/// cleared first - PB1 has no pull-down, so the pulled input lets go for
/// the one store between), high is the input released and then pulled up
/// (never an output high for a store, which would fight a far open drain).
void od_low() {
    RxPin::clear();
    RxPin::output();
}
void od_release() {
    RxPin::release();
    (void)RxPin::input(PinPull::up);
}

/// Wait for one frame of the DUT, then `aux16` microseconds, then send
/// `value` as an 8N1 frame on the RX pad through the emulated open drain,
/// each bit's edge placed on the STK from the frame's start - and the
/// receiver, which hears the wire whoever drives it, read for the frame
/// it heard meanwhile: the wired AND of the DUT's frame and this one.
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
        // The frame the wire carried while this end pulled: the receiver
        // completes it within a bit of the release.
        const uint32_t c1 = Ticker::cycles();
        r.aux16 = 0xFFFFu;
        while (Ticker::cycles() - c1 < 4u * bit) {
            const uint16_t s2 = L::status();
            if ((s2 & usart_rxne) != 0u) {
                const uint16_t w = L::read_word();
                account(r, s2, w, data_mask);
                r.aux16 = static_cast<uint16_t>(w & data_mask);
                break;
            }
        }
        hold_pads();
        flush_rx();
        return r;
    }
    r.flags |= link::report_timed_out;
    return r;
}

/// One named link pad at a level for `ms`, the USART let go: PB0 (pad_txd)
/// or PB1 (pad_rxd) push-pull, its readback at the end of the hold in
/// aux16, the pad back to the dark discipline's state after.
link::Report run_hold_pad(const link::Params& a) {
    link::Report r{};
    settle();
    const bool level = a.aux8 != 0u;
    if (a.value == link::pad_txd) {
        TxPin::output(level);
    } else {
        RxPin::output(level);
    }
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
    }
    const bool back = a.value == link::pad_txd ? TxPin::read() : RxPin::read();
    hold_pads();
    flush_rx();
    r.count = 1;
    r.aux16 = back ? 1u : 0u;
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
        // The ack while the line is still this end's, the stand-off after
        // it: answer() holds the pads as the state allows, and a stand-off
        // already running would let the TX pad go under its own ack.
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
        d.xtal = xtal_state;
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
    const bool refused = op == Op::onewire || op == Op::stage ||
                         (op == Op::hold_pad && a.value != link::pad_txd && a.value != link::pad_rxd);
    const bool ok = !refused && (op == Op::hold_pad || apply_cfg(a.cfg));
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
            case Op::echo: {
                uint32_t span = 0;
                r = run_receive(a, true, span);
                break;
            }
            case Op::sink: {
                uint32_t span = 0;
                r = run_receive(a, false, span);
                break;
            }
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
            case Op::hold_pad: r = run_hold_pad(a); break;
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

// ---- the supply and the console ----------------------------------------------------

/// The desk's supply rule for an image that drives a link pad: the voltage
/// detector at PLS 10, 3.0 V falling (pwr.hpp), given 200 us before PVD0 is
/// trusted - the manual states no settling time.
bool supply_above_floor() {
    Pwr::pvd(PvdLevel::v3_0);
    (void)delay_us(clock, 200);
    return !Pwr::supply_low();
}

void help() {
    print(console, "usart_peer: ? help | i status | 0 command mode, dark | 3 trace | c the clock", crlf);
}

void clock_status() {
    print(console, "  HCLK ", hclk, " Hz believed (", xtal_state == 2u ? "ADOPTED from a crystal DUT" : "the RC's",
          "), trim ", Rcc::hsi_trim(), ", HSICAL ", hex(Rcc::hsi_calibration()), crlf);
    print(console, "  the last calibrate: ", last_measured_hz, " Hz measured at trim ", last_trim_before,
          ", trim now ", last_trim_after, "; measured per trim:");
    for (uint8_t t = 0; t < 32u; ++t) {
        if (trim_hz[t] != 0u) {
            print(console, " ", t, "=", trim_hz[t]);
        }
    }
    print(console, crlf);
}

void status() {
    print(console, "  link USART4 column 0 (TX PB0, RX PB1) ", proven ? "PROVEN" : "searching (dark)",
          standing_off() ? ", STAND-OFF" : "", crlf);
    print(console, "  BRR=", L::brr(), " commands=", commands, " actions=", actions, " naks=", naks, crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count, " sum=",
          hex(last_report.sum), " ferr=", last_report.ferr, " perr=", last_report.perr, " ovf=",
          last_report.ovf, " flags=", hex(last_report.flags), " aux=", last_report.aux16, " ms=",
          last_report.ms, " span=", last_report.span, crlf);
    print(console, "  the engine: ", plays, " lists played, ", play_timeouts, " never completed, ", play_refused,
          " refused", crlf);
    print(console, "  supply ", supply_ok ? "above 3.0 V" : "LOW", "; the last receive: window at ", last_window_ms,
          " ms after the ack, first frame at ", last_first_ms, " (STATR ", hex(last_first_status), "), end at ",
          last_end_ms, ", span ", last_span_cycles, " cycles", crlf);
    clock_status();
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Console::isr(); }
/// The engines are bound with no interrupt; a stray line is cleared.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { (void)TxEdges::service(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)RxEdges::service(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = Ticker::init(clock);
    enable_interrupts();

    // THE SUPPLY before any link pad is driven (the desk's rule).
    supply_ok = supply_above_floor();

    print(console, crlf, "usart_peer - USART instrument peer on ", device::part_name, ", clk=",
          clock_ok ? "HSI48 (an RC peer until a crystal DUT calibrates it)" : "FAILED", ", fw ",
          hex(firmware_version), ", tick=", tick_ok ? "STK" : "FAILED", ", console=", console_ok ? "USART2" : "FAILED",
          crlf);
    if (!supply_ok) {
        print(console, "SUPPLY UNDER 3.0 V (the PVD at PLS 10): the link stays dark", crlf);
    }
    print(console, "link USART4 column 0: TX PB0, RX PB1; command mode 8N1 @ ", link::command_baud, crlf);

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
                } else if (c == 'c') {
                    clock_status();
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
