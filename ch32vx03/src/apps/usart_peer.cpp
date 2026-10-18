// usart_peer - the INSTRUMENT half of a two-board USART test on a CH32V203
// or a CH32V303: the scriptable peer the far board's suite (the DUT)
// drives IN BAND over the very link under test.
//
// The wire format is usart_link.hpp, included by relative path: one file
// is the single source of truth for every board that speaks it, whatever
// its architecture. One blocking poll loop decodes a command frame,
// acknowledges it and then becomes, for a bounded moment, whatever the DUT
// needs on the other end of the wire - an echo, a silent sink, a timer of
// its frames, a generator, a flood, a break, a foreign auto-baud sender,
// a frame drawn edge by edge that no UART could send - and restores
// command mode BY ITSELF when the action's frame count or deadline runs
// out.
//
// ONE PEER FOR THE STRATUM, ON THE PORT AND THE TIMER THE PART LINKS.
// `PeerLink<...>` names the port, its column, the timer whose two channels
// share the port's pads, and the two DMA channels table 11-5 wires to
// those channels; the part picks it:
//  - the CH32V203C8: USART3 on its default column (TX PB10, RX PB11,
//    table 10-25) and TIM2 on its code 10 (CH3 PB10, CH4 PB11, table
//    10-16), the channels' requests on DMA1's channels 1 and 7;
//  - the CH32V303RC and VC: UART4 on its SECOND column (TX PB0, RX PB1,
//    table 10-26 - its default is PC10/PC11, the SPI link's pads on the
//    desk) and TIM3 on its code 00 (CH3 PB0, CH4 PB1, table 10-17), the
//    requests on DMA1's channels 2 and 3.
// The port is polled through the resource - no interrupt, no ring: an
// instrument's loop owns the instance (RM 18.10.1's flags read and cleared
// in the loop). The desk wires it crossed, full duplex, to the far chip of
// the same stratum; the peer serves full duplex alone and refuses onewire:
// this block has no swap, so no transmitter of the peer reaches the DUT's
// transmit wire but through the peer's RECEIVE pad, which drive_rxd and
// collide drive instead. The console is USART1 on PA9/PA10, the probe's
// serial, at 115200: observability only.
//   ? help | i status | 0 back to command mode, dark | 3 trace
//   f m s the link's transmit pad class (a diagnostic; slow at boot)
// THE TRANSMIT PAD AT THE SLOWEST CLASS, the gentlest edge that holds the
// link's top rate: on the SPI pair's wires a client's fast edge reached its
// own inputs (docs/ch32vx03/spi.md), and on the serial pair every class -
// fast, medium and slow - carried the two-board ladder to 4.5 Mbaud exact
// (docs/ch32vx03/usart.md).
//
// THE DARK DISCIPLINE (usart_link.hpp): while SEARCHING - no frame has
// checked out, or the channel has been quiet for rediscover_ms, or a
// stand-off is running - both link pads are floating inputs: the
// transmitter reaches no wire and nothing is pulled, so the far board's
// single-board letters walk those wires with their own pulls. A frame
// that checks out PROVES the wiring: the TX pad goes to the port (idling
// high) and the RX pad takes its pull-up. A bad checksum is nak'ed only
// while ENGAGED.
//
// TIME (usart_link.hpp, TIME IN THE PAYLOAD). The clock is the board's 8
// MHz crystal x18, 144 MHz (vx03_peer.hpp's serial time base): Ident::xtal
// = 1, and every figure this peer states is the crystal's - the STK counts
// HCLK, six cycles a 24 MHz tick, so a Report's span is the cycles over
// six; the port's PCLK1 is 72 MHz, whose divisor of 16 is the 4.5 Mbaud of
// the link's top; and a PB1 timer counts 144 MHz, so an edge is placed to
// one core cycle. A CRYSTAL PEER ADOPTS NOTHING: calibrate stamps the DUT's
// gapless run and reports the DUT's rate against this crystal; with the
// crystal absent (the HSI's PLL at the same rate, xtal 0) calibrate is
// refused - this peer has no trim to offer, and its rates are the RC's.
// send and blast STATE their rate: PCLK1 over BRR against the rate asked
// (report_rate_stated), exact on a crystal.
//
// THE WAVEFORM ENGINE (bitbang, drive_rxd, brk, autobaud_tx's break,
// stage), written from chapters 15 and 11 and from what the scratch probe
// of this peer's bring-up measured (docs/ch32vx03/usart.md, the desk's
// instrument): the timer's channel on the pad in TOGGLE mode (15.4.7, OCxM 011,
// no preload), its compare request (CCxDE) streaming the next edge's
// compare value by DMA into CHxCVR on each match - no CPU between edges,
// nothing masked. The counter RUNS FREE over its sixteen bits and wraps:
// an edge is an absolute time modulo 2^16 counts, so a list is bounded by
// its table and not by the counter, and the one thing a wrap asks is that
// no interval between two edges exceeds the counter - the list's
// prescaler is the smallest that keeps the longest interval under 0xF000
// counts, one core cycle a count for every frame a UART sends (a 1200-baud
// stop bit is 120000 cycles, prescaler 2). The channel keeps up with edges
// 14 cycles apart and misses one 12 apart (measured, the DMA's floor), so
// the floor is 16 cycles: a cell narrower is widened (bitbang) or the
// train refused (stage). The list ends with a value half a counter beyond
// the last edge, which the last edge's request moves into the compare
// register; the CPU, seeing the channel's completion, forces the output
// high (the line's level already) long before that match.
//
// THE PAD IS SHARED, AND AN ENABLED TRANSMITTER OWNS IT. Measured with the
// pad in alternate-function push-pull: while the port has UE and TE set,
// the timer's forced low does not reach the pad; with TE clear the
// timer's enabled channel drives it (both chips) - and with neither
// driving, the CH32V303's pad reads low. So a tenure parks the pad as a
// GPIO output high, starts the counter, forces the channel high and
// enables it, clears TE, and only then hands the pad back to the
// alternate function, now the timer's; the way back is the same steps
// reversed, TE set again (the idle frame it sends is high) before the
// channel lets go. No column moves: USART3 has no other column the LQFP48
// bonds. A FORCED LEVEL IS NOT EVALUATED WHILE THE COUNTER IS STOPPED, and
// a counter that never ran holds its output low (measured on both chips),
// which is why the counter runs before the pad is handed over and stops
// only with the line high, a level the stopped counter holds.
//
// A GLITCH IS ONE PULSE (15.3.10, OPM with PWM mode 1): high while the
// counter is under the compare, low from it to the reload, the update
// stopping the counter at zero - high again, by itself (measured): exact
// to one count.
//
// THE PER-OP NOTES. onewire is refused (above); collide is an emulated
// open drain on the RX pad - an output low, or the input's pull-up, which
// on this family is the output data bit (docs/ch32vx03/pin.md) - paced on
// the STK, a wired AND with a DUT whose own pad is an open drain (the
// far chip's HDSEL pad), and this peer's receiver reads the wire while it
// pulls: the Report's aux16 is the frame it heard, the AND of the two;
// sync_client and sync_host are refused (neither port's CK reaches the
// link: UART4 has none, USART3's PB12 is the SPI link's select), as are
// clk2x (this block has no OVER8, 18.3), invert_xck and mpcm (this
// receiver wakes on ONE address, 18.10.4's ADD, where the protocol's MPCM
// hands every address frame up); 5- and 6-bit words (M_EXT is the
// CH32V303's lot's, absent on this desk's die, docs/ch32vx03/usart.md) and
// nine data bits with a parity bit are refused; the four stop codes are
// served as CTLR2.STOP takes them - and a frame this transmitter SENDS at
// the 0.5 and 1.5 codes is one and two stop bits long (measured,
// usart.md: an ordinary transmitter rounds the halves up); IrDA is served
// (IREN, txpl != 0 asks the low-power mode at PSC = txpl); hold_pad drives
// the TX or the RX pad (no clock pad on the link: pad_xck refused); stage
// draws its train whole in one edge list, refused where a pulse is under
// the floor or the train overflows the table.
//
// THE V4F'S FLOAT TAX DOES NOT REACH THE LINK: on the CH32V303 the image
// is built under ilp32f, and every path here is the main loop's; the two
// vectors the image binds - the console and the tick - are LEAF.
//
// NOTHING PLACED IN SRAM (util/placement.hpp; docs/ch32vx03/platform.md:
// the image runs from the zero-wait window, which SRAM fetch does not
// beat).
//
// The ident label is the part number after its CH32 prefix ("V203C8",
// "V303VC", vx03_peer.hpp), the die on the banner; the version's high byte
// is usart_link.hpp's peer_ch32vx03.
//
// build: boards = v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <span>

#include "ch32vx03/afio.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "vx03_peer.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrxt/src/apps/usart_link.hpp"

namespace {

using namespace brio;
using link::Op;

using P = Ch32vx03Platform<>;
using SysClock = vx03peer::UsartPeerClock;
constexpr SysClock clock;

using Console = Uart<1, P, 64, 128>;
constexpr Console console;

/// The link's port and column, the timer whose channels 3 and 4 share its
/// two pads in the timer's column, and everything that hangs on them (the
/// header's ONE PEER FOR THE STRATUM).
template <uint8_t n, uint8_t code, uint8_t t, uint8_t tcode, DmaRequest tx_req, DmaRequest rx_req>
struct PeerLink {
    static constexpr uint8_t instance = n;
    static constexpr uint8_t column = code;
    static constexpr uint8_t timer = t;
    static constexpr uint8_t timer_column = tcode;
    using Port = Usart<n>;
    using T = Tim<t>;
    static constexpr UsartPads pads = usart_pads_for(n, code);
    static_assert(usart_remap_valid(n, code), "usart_peer: the link's column must be one this part bonds");
    using TxPin = Pin<pads.tx.port, pads.tx.pin>;
    using RxPin = Pin<pads.rx.port, pads.rx.pin>;
    static constexpr uint8_t tx_ch = 2;   ///< CH3, 0-based: on the TX pad
    static constexpr uint8_t rx_ch = 3;   ///< CH4: on the RX pad
    static_assert(tim_channel_pad(t, tcode, tx_ch) == pads.tx && tim_channel_pad(t, tcode, rx_ch) == pads.rx,
                  "usart_peer: the timer's channels 3 and 4 must sit on the port's TX and RX pads");
    static constexpr DmaSlot tx_slot = DmaRequestOf<tx_req>::slot;
    static constexpr DmaSlot rx_slot = DmaRequestOf<rx_req>::slot;
    using TxEdges = DmaTxEngine<tx_slot.controller, tx_slot.channel, uint16_t>;
    using RxEdges = DmaTxEngine<rx_slot.controller, rx_slot.channel, uint16_t>;
};

/// The part's link (the header): UART4's second column where the part is
/// a CH32V303, USART3's default one on the CH32V203.
constexpr bool v303 = device::device_class == DeviceClass::v30x_d8;
using V303Link = PeerLink<4, 1, 3, 0, DmaRequest::tim3_ch3, DmaRequest::tim3_ch4>;
using V203Link = PeerLink<3, 0, 2, 2, DmaRequest::tim2_ch3, DmaRequest::tim2_ch4>;
template <bool b>
struct LinkOf {
    using type = V203Link;
};
template <>
struct LinkOf<true> {
    using type = V303Link;
};
using L = LinkOf<v303>::type;
using U = L::Port;
using T = L::T;
using TxPin = L::TxPin;
using RxPin = L::RxPin;
using TxEdges = L::TxEdges;
using RxEdges = L::RxEdges;
constexpr uint8_t tx_ch = L::tx_ch;
constexpr uint8_t rx_ch = L::rx_ch;
static_assert(L::tx_slot.controller == 1u && L::rx_slot.controller == 1u,
              "table 11-5: the two timers' channel requests are DMA1's");

constexpr uint16_t firmware_version = static_cast<uint16_t>((link::peer_ch32vx03 << 8) | 0x01u);

constexpr uint32_t hclk = SysClock::hz;
constexpr uint32_t pclk = SysClock::pclk1_hz;   ///< USART3's and UART4's bus
constexpr uint32_t timclk = SysClock::timclk1_hz;
static_assert(timclk == hclk, "the engine places edges in core cycles (vx03_peer.hpp)");

bool crystal_ok = false;
bool proven = false;
bool trace = false;
link::Decoder decoder;
link::Report last_report;
uint32_t commands = 0, naks = 0, actions = 0;
PinSpeed tx_speed = PinSpeed::slow;

uint32_t ack_ms = 0;
uint32_t last_byte_ms = 0;
uint32_t last_frame_ms = 0;
uint32_t standby_until = 0;
uint32_t engaged_until = 0;

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }
bool standing_off() { return standby_until != 0 && Ticker::millis() < standby_until; }

// ---- time ------------------------------------------------------------------

/// Core cycles to protocol ticks: six cycles a tick, exactly.
constexpr uint32_t ticks_of(uint32_t cycles) {
    return (cycles + vx03peer::usart_cycles_per_tick / 2u) / vx03peer::usart_cycles_per_tick;
}
constexpr uint32_t cycles_of_ticks(uint32_t ticks) { return ticks * vx03peer::usart_cycles_per_tick; }
/// One bit at `rate`, in sixteenths of a core cycle.
constexpr uint32_t bit_q4(uint32_t rate) {
    return rate == 0u ? 16u * 5000u : static_cast<uint32_t>((16ULL * hclk + rate / 2u) / rate);
}

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(link::settle_ms) + 2u) {
    }
}

void wait_cycles(uint32_t n) {
    const uint32_t c0 = Ticker::cycles();
    while (Ticker::cycles() - c0 < n) {
    }
}

/// Any number of microseconds: the whole milliseconds on the tick count,
/// the rest on the cycle count.
void wait_us(uint32_t us) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < us / 1000u) {
    }
    wait_cycles((us % 1000u) * (hclk / 1'000'000u));
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

// ---- the pads --------------------------------------------------------------

/// The two link pads as the state allows (the header's dark discipline).
void hold_pads() {
    if (proven && !standing_off()) {
        TxPin::function(PinDrive::push_pull, tx_speed);
        RxPin::input(PinPull::up);
    } else {
        TxPin::release();
        RxPin::release();
    }
}

/// Drain whatever the receiver holds and clear its errors: STATR, then
/// DATAR (18.10.1's clear).
void flush_rx() {
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)U::status();
        (void)U::read_word();
    }
}

// ---- the link's configuration ------------------------------------------------

struct LinkFormat {
    UartFormat format{};
    uint16_t brr = 0;
    bool irda = false;
    IrdaConfig ir{};
};

/// What this silicon can make of a Cfg; empty when it cannot (the
/// header's per-op notes).
std::optional<LinkFormat> format_of(const link::Cfg& c) {
    if (c.mode == link::Mode::sync_client || c.mode == link::Mode::sync_host) {
        return std::nullopt;
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
    const uint32_t brr = usart_divisor(pclk, c.rate);
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

/// The port through its reset line, then the format whole, the
/// transmitter and the receiver on. A proven TX pad is parked as a GPIO
/// output high across the reset and the reconfiguration: a port in reset,
/// or with TE clear, does not drive its pad, and the CH32V303's pad then
/// reads low (the header). No wait after TE: no low pulse shows as it
/// rises (measured).
bool open_link(const LinkFormat& f) {
    if (proven && !standing_off()) {
        TxPin::output(true);
    }
    U::bus_clock(true);
    U::reset();
    if (!U::remap(L::column) || !U::configure(f.format, f.brr)) {
        return false;
    }
    if (f.irda && !U::irda(f.ir)) {
        return false;
    }
    U::enable(true);
    U::transmitter(true);
    U::receiver(true);
    data_mask = uart_data_mask(f.format);
    hold_pads();
    flush_rx();
    return true;
}

/// Command mode: 8N1 at link::command_baud.
bool command_mode() {
    const auto f = format_of(link::Cfg{});
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
    while (!U::tx_empty()) {
    }
    U::write_data(b);
}

void wait_line_idle() {
    const uint32_t t0 = Ticker::millis();
    while (!U::tx_complete() && Ticker::millis() - t0 < 50u) {
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

// ---- the waveform engine -------------------------------------------------------

/// The engine's floor: two edges closer than this many core cycles are
/// more than the channel's DMA keeps up with (the header: 14 kept, 12
/// missed).
constexpr uint32_t engine_floor = 16;
/// The first edge this many cycles after the counter starts.
constexpr uint32_t engine_lead = 256;
/// The longest interval a list's prescaler leaves in one count range.
constexpr uint32_t engine_span = 0xF000;

/// A list of edges as absolute core cycles from the counter's start,
/// built before the action in sixteenths of a cycle (a bit at 921600 is
/// 156.25 cycles) and rounded once an edge.
struct Edges {
    static constexpr uint16_t capacity = 640;
    uint32_t at[capacity];
    uint16_t n = 0;
    uint64_t now_q4 = 0;      ///< the running position, sixteenths of a cycle
    bool level = true;        ///< the line's level at now_q4
    bool overflow = false;

    void start() {
        n = 0;
        now_q4 = static_cast<uint64_t>(engine_lead) << 4;
        level = true;
        overflow = false;
    }
    /// The line at `high` from absolute `t_q4` on.
    void set(bool high, uint64_t t_q4) {
        if (high == level) {
            return;
        }
        if (n >= capacity) {
            overflow = true;
        } else {
            at[n++] = static_cast<uint32_t>((t_q4 + 8u) >> 4);
        }
        level = high;
    }
    /// A cell of `q4` sixteenths at `high`, from the running position.
    void cell(bool high, uint64_t q4) {
        set(high, now_q4);
        now_q4 += q4;
    }
    uint32_t end() const { return static_cast<uint32_t>((now_q4 + 8u) >> 4); }
};

Edges edges;
uint16_t table[Edges::capacity + 1];

/// The engine's own counts, console observability: lists played, lists
/// whose channel never completed within the list's length, lists refused.
uint32_t plays = 0, play_timeouts = 0, play_refused = 0;
uint32_t last_div = 0;

/// The shortest interval of a list, in cycles, its first from the
/// counter's start; and the longest.
void intervals(const Edges& e, uint32_t& shortest, uint32_t& longest) {
    shortest = 0xFFFF'FFFFu;
    longest = 0;
    uint32_t prev = 0;
    for (uint16_t i = 0; i < e.n; ++i) {
        const uint32_t d = e.at[i] - prev;
        shortest = d < shortest ? d : shortest;
        longest = d > longest ? d : longest;
        prev = e.at[i];
    }
}

/// Whether the engine can play `e`: edges at all, inside the table, none
/// closer than the floor, the line high at its end.
bool playable(const Edges& e) {
    if (e.overflow || e.n == 0u || !e.level) {
        return false;
    }
    uint32_t shortest = 0, longest = 0;
    intervals(e, shortest, longest);
    return shortest >= engine_floor && longest / engine_span < 65'535u;
}

/// THE PLAY (the header's engine): the list's prescaler, the compare
/// values modulo the counter, the channel programmed with the counter
/// stopped - the line forced high, which a stopped counter holds - the
/// first edge into the compare register by store and the rest by the
/// channel's requests, then the end marker. Returns when the line has
/// played the list's last cell.
template <typename E>
bool play(uint8_t ch, Edges& e) {
    using Channel = DmaChannel<E::controller, E::channel>;
    ++plays;
    if (!playable(e)) {
        ++play_refused;
        return false;
    }
    uint32_t shortest = 0, longest = 0;
    intervals(e, shortest, longest);
    const uint32_t div = longest / engine_span + 1u;
    last_div = div;
    for (uint16_t i = 0; i < e.n; ++i) {
        table[i] = static_cast<uint16_t>((e.at[i] + div / 2u) / div);
    }
    table[e.n] = static_cast<uint16_t>(table[e.n - 1u] + 0x8000u);   // the end marker
    T::enable(false);
    if (!T::configure(TimConfig{.prescaler = static_cast<uint16_t>(div - 1u), .period = 0xFFFF})) {
        ++play_refused;
        return false;
    }
    if (!E::start(std::span<const uint16_t>(&table[1], e.n))) {
        ++play_refused;
        return false;
    }
    // THE FIRST EDGE INTO THE COMPARE REGISTER BEFORE THE MODE, the flag
    // cleared after both and before the request is enabled: nothing is
    // evaluated while the counter is stopped (the header), so the first
    // match is the first edge's.
    (void)T::set_compare(ch, table[0]);
    (void)T::output_mode(ch, TimOutputMode::toggle);
    T::clear_flags(T::compare_flag(ch));
    T::interrupts(T::compare_dma(ch), true);
    const uint32_t c0 = Ticker::cycles();
    T::enable(true);
    // The last edge's request has moved the end marker: the line is at its
    // final level. Bounded by the list's own length.
    const uint32_t budget = 2u * e.end() + hclk / 1000u;
    bool done = false;
    while (!(done = Channel::flag(DmaFlag::complete))) {
        if (Ticker::cycles() - c0 > budget) {
            break;
        }
    }
    (void)T::output_mode(ch, TimOutputMode::force_active);
    T::interrupts(T::compare_dma(ch), false);
    if (done) {
        (void)E::complete();
    } else {
        (void)E::abandon();
        ++play_timeouts;
    }
    // The tail: the list's last cell played out at the line's final level.
    while (Ticker::cycles() - c0 < e.end()) {
    }
    return done;
}

/// The engine's tenure of a pad (the header's PAD IS SHARED): parked high,
/// the counter running and the channel forced high, TE cleared for the TX
/// pad, then the alternate function - the timer's.
template <typename Pad>
void tenure_begin(uint8_t ch) {
    Pad::output(true, PinDrive::push_pull, tx_speed);
    T::enable(false);
    (void)T::configure(TimConfig{.prescaler = 0, .period = 0xFFFF});
    T::enable(true);
    (void)T::output_channel(ch, TimChannelConfig{.mode = TimOutputMode::force_active, .preload = false,
                                                 .enable = true});
    if (ch == tx_ch) {
        U::transmitter(false);
    }
    wait_cycles(64);
    Pad::function(PinDrive::push_pull, tx_speed);
}

/// And back: parked high, TE set again (its idle frame is high), the
/// channel let go, the counter stopped, the pads as the state allows.
template <typename Pad>
void tenure_end(uint8_t ch) {
    Pad::output(true, PinDrive::push_pull, tx_speed);
    if (ch == tx_ch) {
        U::transmitter(true);
    }
    (void)T::output_channel(ch, TimChannelConfig{.mode = TimOutputMode::force_active, .preload = false,
                                                 .enable = false});
    T::enable(false);
    hold_pads();
    flush_rx();
}

/// The width of bit cell `idx` (0 = start, 1..8 = data, 9 = stop) in
/// sixteenths, stretched or shrunk by `pct` when it is the chosen one,
/// never under the floor.
uint32_t cell_q4(uint32_t nominal, int8_t cell, int8_t pct, int8_t idx) {
    int64_t v = static_cast<int64_t>(nominal);
    if (cell == idx && pct != 0) {
        v += (static_cast<int64_t>(nominal) * pct) / 100;
    }
    if (v < static_cast<int64_t>(engine_floor) * 16) {
        v = static_cast<int64_t>(engine_floor) * 16;
    }
    return static_cast<uint32_t>(v);
}

/// One 8N1 frame of `value`, LSB first, one cell distorted.
void frame_edges(Edges& e, uint32_t nominal, uint8_t value, int8_t cell, int8_t pct) {
    e.cell(false, cell_q4(nominal, cell, pct, 0));
    for (int8_t i = 0; i < 8; ++i) {
        e.cell(((value >> i) & 1u) != 0u, cell_q4(nominal, cell, pct, static_cast<int8_t>(i + 1)));
    }
    e.cell(true, cell_q4(nominal, cell, pct, 9));
}

// ---- accounting ------------------------------------------------------------------

[[gnu::always_inline]] inline void account(link::Report& r, uint16_t st, uint16_t d, uint16_t mask) {
    if ((st & (usart_fe | usart_pe | usart_ore)) != 0u) {
        if ((st & usart_fe) != 0u && r.ferr < 255u) ++r.ferr;
        if ((st & usart_pe) != 0u && r.perr < 255u) ++r.perr;
        if ((st & usart_ore) != 0u && r.ovf < 255u) ++r.ovf;
    }
    r.sum = static_cast<uint16_t>(r.sum + (d & mask));
    ++r.count;
}

// ---- the actions ---------------------------------------------------------------------

/// The last receive's timeline in milliseconds after its ack and its
/// first-to-last span in cycles: console observability.
uint32_t last_window_ms = 0, last_first_ms = 0, last_end_ms = 0, last_span_cycles = 0;
uint16_t last_first_status = 0;

/// echo, sink and calibrate's receiving half: the frames counted and
/// summed, the FIRST and the LAST of the commanded count stamped on the
/// STK's cycle count as their RXNE is seen - DATAR read FIRST and the
/// stamp after: a frame completing between a status read and the data
/// read is lost with its overrun cleared unseen (18.10.1's clear pair,
/// the CH32X035 peer's finding on the same block), and a stamp is a call.
/// Each stamp is a fixed data read late, which the span cancels; two
/// calls a run, none a character. A run the deadline ends short reports
/// no span (usart_link.hpp: 0 = not measured).
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
        const uint16_t st = U::status();
        if ((st & usart_rxne) != 0u) {
            const uint16_t d = U::read_word();
            if (r.count == 0u) {
                first = Ticker::cycles();
                last_first_ms = Ticker::millis() - ack_ms;
                last_first_status = st;
            } else if (r.count + 1u == count) {
                last = Ticker::cycles();
            }
            account(r, st, d, mask);
            if (echo) {
                while (!U::tx_empty()) {
                }
                U::write_word(d);
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

/// calibrate as a CRYSTAL peer: the DUT's gapless run stamped, its rate
/// against this crystal reported; nothing adopted (the header's TIME).
link::Report run_calibrate(const link::Params& a) {
    uint32_t span_cycles = 0;
    link::Report r = run_receive(a, false, span_cycles);
    const uint16_t half_bits = link::frame_half_bits(a.cfg.bits, a.cfg.parity, a.cfg.stop);
    const uint32_t nominal = link::nominal_span(a.cfg.rate, half_bits, r.count);
    r.aux16 = static_cast<uint16_t>(link::calibrate_ppm(nominal, r.span));
    return r;
}

/// send and blast's Report::aux16: the transmitter's real rate - PCLK1
/// over the divisor - against the rate asked; stated on the crystal only.
void state_rate(link::Report& r, uint32_t rate) {
    if (crystal_ok) {
        r.aux16 = static_cast<uint16_t>(link::rate_error_ppm(pclk, U::brr(), rate));
        r.flags |= link::report_rate_stated;
    }
}

/// FLATTENED: link::Stream::next() is the pattern's one source and -Os
/// keeps it out of line, a call a frame in the loop that feeds DATAR at
/// 4.5 Mbaud (320 core cycles a frame) - inlined here, the loop is the
/// stream's few instructions, the TXE poll and the store.
[[gnu::flatten]] link::Report run_send(const link::Params& a, bool blast) {
    link::Report r{};
    settle();
    if (!blast && a.aux16 != 0u) {
        wait_ms(a.aux16);
    }
    // send: two bit times of idle between frames; blast: none.
    const uint32_t gap_cycles = blast ? 0u : (bit_q4(a.cfg.rate) >> 3);
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
                wait_line_idle();
                wait_us(repeat_gap_us);
            }
        }
        for (uint16_t i = 0; i < a.count; ++i) {
            const uint16_t v = static_cast<uint16_t>(stream.next() & mask);
            while (!U::tx_empty()) {
            }
            U::write_word(v);
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++r.count;
            if (gap_cycles != 0u) {
                wait_line_idle();
                wait_cycles(gap_cycles);
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
        while (!U::tx_empty()) {
        }
        U::write_word(addr);
        r.sum = static_cast<uint16_t>(r.sum + addr);
        ++r.count;
        for (uint8_t k = 0; k < pairs[2u * g + 1u]; ++k) {
            const uint16_t v = seq++;
            while (!U::tx_empty()) {
            }
            U::write_word(v);
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++r.count;
        }
    }
    wait_line_idle();
    return r;
}

/// A low run of `bits` bit times, then two of idle, on the TX pad.
bool play_break(uint32_t bit, uint32_t bits) {
    edges.start();
    edges.cell(false, static_cast<uint64_t>(bits) * bit);
    edges.cell(true, 2u * static_cast<uint64_t>(bit));
    return play<TxEdges>(tx_ch, edges);
}

link::Report run_break(const link::Params& a) {
    link::Report r{};
    settle();
    wait_line_idle();
    tenure_begin<TxPin>(tx_ch);
    const bool ok = play_break(bit_q4(a.cfg.rate), a.count);
    tenure_end<TxPin>(tx_ch);
    r.count = a.count;
    r.aux16 = U::brr();
    if (!ok) r.flags |= link::report_mismatch;
    return r;
}

link::Report run_autobaud_tx(const link::Params& a) {
    link::Report r{};
    settle();
    wait_line_idle();
    tenure_begin<TxPin>(tx_ch);
    const bool ok = play_break(bit_q4(a.cfg.rate), a.aux16 != 0u ? a.aux16 : 16u);
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
    r.aux16 = U::brr();
    if (!ok) r.flags |= link::report_mismatch;
    return r;
}

/// bitbang on the TX pad, drive_rxd on the RX pad: `count` frames of
/// `value`, one cell distorted, a break ahead when aux16 asks, four bit
/// times of idle between frames - every cell at the rate asked on the
/// crystal, the whole train ONE list.
template <typename Pad, typename E>
link::Report run_bitbang(const link::Params& a, uint8_t ch) {
    link::Report r{};
    const uint32_t nominal = bit_q4(a.cfg.rate);
    edges.start();
    if (a.aux16 != 0u) {
        edges.cell(false, static_cast<uint64_t>(a.aux16) * nominal);
        edges.cell(true, 2u * static_cast<uint64_t>(nominal));
    }
    for (uint16_t i = 0; i < a.count; ++i) {
        frame_edges(edges, nominal, a.value, a.cell, a.pct);
        edges.cell(true, 4u * static_cast<uint64_t>(nominal));   // a clean idle between frames
        r.sum = static_cast<uint16_t>(r.sum + a.value);
        ++r.count;
    }
    settle();
    if (ch == tx_ch) {
        wait_line_idle();
    }
    tenure_begin<Pad>(ch);
    const bool ok = play<E>(ch, edges);
    tenure_end<Pad>(ch);
    const uint32_t t = ticks_of((nominal + 8u) >> 4);
    r.aux16 = static_cast<uint16_t>(t > 0xFFFFu ? 0xFFFFu : t);
    if (!ok) r.flags |= link::report_mismatch;
    return r;
}

/// `count` low pulses of `aux16` ticks on the idle TX line, `aux8`
/// microseconds apart: ONE PULSE each (the header).
link::Report run_glitch(const link::Params& a) {
    link::Report r{};
    settle();
    wait_line_idle();
    tenure_begin<TxPin>(tx_ch);
    const uint32_t ticks = a.aux16 != 0u ? a.aux16 : 1u;
    const uint32_t width = cycles_of_ticks(ticks);
    constexpr uint32_t delay_counts = 2;
    const uint32_t div = (width + delay_counts) / 0xFFF0u + 1u;
    const uint32_t counts = (width + div / 2u) / div != 0u ? (width + div / 2u) / div : 1u;
    T::enable(false);
    bool ok = T::configure(TimConfig{.prescaler = static_cast<uint16_t>(div - 1u),
                                     .period = static_cast<uint16_t>(delay_counts + counts - 1u),
                                     .one_pulse = true});
    (void)T::set_compare(tx_ch, delay_counts);
    ok = ok && T::output_mode(tx_ch, TimOutputMode::pwm1);
    for (uint16_t i = 0; ok && i < a.count; ++i) {
        T::enable(true);
        while (T::enabled()) {
        }
        wait_us(a.aux8 != 0u ? a.aux8 : 1u);
        ++r.count;
    }
    // Back to the free-running counter for the tenure's way out.
    (void)T::configure(TimConfig{.prescaler = 0, .period = 0xFFFF});
    T::enable(true);
    (void)T::output_mode(tx_ch, TimOutputMode::force_active);
    wait_cycles(64);
    tenure_end<TxPin>(tx_ch);
    r.aux16 = static_cast<uint16_t>(ticks);
    if (!ok) r.flags |= link::report_cfg_failed;
    return r;
}

/// stage (usart_link.hpp): a frame train drawn whole in one edge list -
/// every cell of every frame from usart_link's own cell table, the PULSE
/// inside each frame (the line at the level opposite its cell's for aux8
/// per cent of a bit, from pct per cent into cell `cell`), the idle
/// between frames in sixteenths of a bit - at the crystal's cycles, so the
/// rate is the time base's and not a divisor's. Each frame is its run of
/// boundaries in time order (its cell starts, the pulse's two ends), the
/// level at each read off the frame; the list holds only real changes.
/// Refused where the engine cannot draw it.
link::Report run_stage(const link::Params& a) {
    link::Report r{};
    const uint8_t data = a.cfg.bits >= link::bits9_low ? 9u : a.cfg.bits;
    const uint8_t parity = a.cfg.parity;
    if (data < 5u || data > 9u || parity > 2u || (a.cfg.stop != 1u && a.cfg.stop != 2u) || a.cfg.rate == 0u) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    const uint8_t cells = static_cast<uint8_t>(1u + data + (parity != 0u ? 1u : 0u) + a.cfg.stop);
    const uint64_t q = bit_q4(a.cfg.rate);
    const uint64_t gap = (q * a.aux16) / 16u;
    const uint64_t slot = q * cells + gap;
    const bool pulse = a.cell >= 0 && a.cell < static_cast<int8_t>(cells) && a.aux8 != 0u && a.pct >= 0 &&
                       a.pct < 100;
    const uint64_t p0 = pulse ? q * static_cast<uint8_t>(a.cell) + (q * static_cast<uint8_t>(a.pct)) / 100u : 0u;
    const uint64_t p1 = pulse ? p0 + (q * a.aux8) / 100u : 0u;
    if (pulse && (((p1 - p0) >> 4) < engine_floor || p1 > slot)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    edges.start();
    const uint64_t base = edges.now_q4;
    link::Stream st(a.pattern, a.value);
    const uint16_t dmask = static_cast<uint16_t>((1u << data) - 1u);
    for (uint16_t i = 0; i < a.count; ++i) {
        const uint16_t v = static_cast<uint16_t>(st.next() & dmask);
        const uint64_t at = base + slot * i;
        // The boundaries in time order: the cell starts (the last one the
        // idle's), the pulse's two ends merged in where they fall.
        uint64_t when[link::stage_cells_most + 3];
        uint8_t nb = 0;
        for (uint8_t k = 0; k <= cells; ++k) {
            when[nb++] = q * k;
        }
        if (pulse) {
            for (const uint64_t t : {p0, p1}) {
                uint8_t j = nb++;
                while (j != 0u && when[j - 1u] > t) {
                    when[j] = when[j - 1u];
                    --j;
                }
                when[j] = t;
            }
        }
        // The pulse's level is the opposite of ITS cell's in THIS frame.
        const bool pulse_level = pulse && !link::stage_cell_high(v, data, parity, static_cast<uint8_t>(a.cell));
        for (uint8_t j = 0; j < nb; ++j) {
            const uint64_t t = when[j];
            const bool level = (pulse && t >= p0 && t < p1) ? pulse_level
                               : t >= q * cells             ? true
                                                            : link::stage_cell_high(v, data, parity,
                                                                                    static_cast<uint8_t>(t / q));
            edges.set(level, at + t);
        }
        r.sum = static_cast<uint16_t>(r.sum + v);
        ++r.count;
    }
    // Two bits of idle after the last slot: the tail played out high.
    edges.now_q4 = base + slot * a.count + 2u * q;
    if (!playable(edges)) {
        r = link::Report{};
        r.flags |= link::report_cfg_failed;
        return r;
    }
    settle();
    wait_line_idle();
    tenure_begin<TxPin>(tx_ch);
    const bool ok = play<TxEdges>(tx_ch, edges);
    tenure_end<TxPin>(tx_ch);
    const uint32_t t = ticks_of(static_cast<uint32_t>((q + 8u) >> 4));
    r.aux16 = static_cast<uint16_t>(t > 0xFFFFu ? 0xFFFFu : t);
    if (!ok) r.flags |= link::report_mismatch;
    return r;
}

/// The RX pad as an emulated open drain: low is an output low (the output
/// data bit cleared first), high is the input with its pull-up - the
/// output data bit set - never an output high, which would fight a far
/// open drain.
void od_low() {
    RxPin::clear();
    RxPin::output(PinDrive::push_pull, tx_speed);
}
void od_release() { RxPin::input(PinPull::up); }

/// Wait for one frame of the DUT, then `aux16` microseconds, then send
/// `value` as an 8N1 frame on the RX pad through the emulated open drain,
/// each bit's edge placed on the STK from the frame's start - and the
/// receiver, which hears the wire whoever drives it, read for the frame it
/// heard meanwhile: the wired AND of the DUT's frame and this one.
link::Report run_collide(const link::Params& a) {
    link::Report r{};
    settle();
    flush_rx();
    const uint32_t bit = (bit_q4(a.cfg.rate) + 8u) >> 4;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        const uint16_t st = U::status();
        if ((st & usart_rxne) == 0u) {
            continue;
        }
        account(r, st, U::read_word(), data_mask);
        wait_us(a.aux16);
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
            const uint16_t s2 = U::status();
            if ((s2 & usart_rxne) != 0u) {
                const uint16_t w = U::read_word();
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

/// One named link pad at a level for `ms`, the port let go: the TX pad
/// (pad_txd) or the RX pad (pad_rxd) push-pull, its readback at the end of
/// the hold in aux16, the pad back to the dark discipline's state after.
link::Report run_hold_pad(const link::Params& a) {
    link::Report r{};
    settle();
    const bool level = a.aux8 != 0u;
    if (a.value == link::pad_txd) {
        TxPin::output(level, PinDrive::push_pull, tx_speed);
    } else {
        RxPin::output(level, PinDrive::push_pull, tx_speed);
    }
    wait_ms(a.ms);
    const bool back = a.value == link::pad_txd ? TxPin::read() : RxPin::read();
    hold_pads();
    flush_rx();
    r.count = 1;
    r.aux16 = back ? 1u : 0u;
    return r;
}

// ---- the command handler ---------------------------------------------------------------

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
        vx03peer::label(d.label);
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

    // What this peer refuses whatever the Cfg says (the header).
    const bool refused = op == Op::onewire || (op == Op::calibrate && !crystal_ok) ||
                         (op == Op::hold_pad && a.value != link::pad_txd && a.value != link::pad_rxd);
    // The drawn train and the held pad do not ask the port for a format.
    const bool own_shape = op == Op::stage || op == Op::hold_pad;
    const bool ok = !refused && (own_shape || apply_cfg(a.cfg));
    if (!ok) {
        r.flags |= link::report_cfg_failed;
    } else {
        switch (op) {
            case Op::set_link: {
                settle();
                const uint32_t t0 = Ticker::millis();
                while (Ticker::millis() - t0 < a.ms) {
                    const uint16_t st = U::status();
                    if ((st & usart_rxne) != 0u) {
                        account(r, st, U::read_word(), data_mask);
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
            case Op::stage: r = run_stage(a); break;
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

const char* speed_name(PinSpeed s) {
    return s == PinSpeed::fast ? "fast (50 MHz)" : (s == PinSpeed::medium ? "medium (10 MHz)" : "slow (2 MHz)");
}

void help() {
    print(console, "usart_peer: ? help | i status | 0 command mode, dark | 3 trace | f m s the TX pad class", crlf);
}

void status() {
    print(console, "  link ", v303 ? "UART" : "USART", L::instance, " column ", L::column, " (TX P", L::pads.tx.port,
          L::pads.tx.pin, ", RX P", L::pads.rx.port, L::pads.rx.pin, ") ", proven ? "PROVEN" : "searching (dark)",
          standing_off() ? ", STAND-OFF" : "", crlf);
    print(console, "  BRR=", U::brr(), " commands=", commands, " actions=", actions, " naks=", naks,
          " TX pad class ", speed_name(tx_speed), crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count, " sum=",
          hex(last_report.sum), " ferr=", last_report.ferr, " perr=", last_report.perr, " ovf=",
          last_report.ovf, " flags=", hex(last_report.flags), " aux=", last_report.aux16, " ms=",
          last_report.ms, " span=", last_report.span, crlf);
    print(console, "  the engine: ", plays, " lists played, ", play_timeouts, " never completed, ", play_refused,
          " refused; the last list's prescaler ", last_div, crlf);
    print(console, "  the last receive: window at ", last_window_ms, " ms after the ack, first frame at ",
          last_first_ms, " (STATR ", hex(last_first_status), "), end at ", last_end_ms, ", span ", last_span_cycles,
          " cycles", crlf);
}

}  // namespace

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_LEAF_VECTOR(usart1_handler) { (void)Console::isr(); }

int main() {
    const bool clock_ok = vx03peer::start_usart_clock(crystal_ok);
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = Ticker::init(clock);
    enable_interrupts();

    char label[9] = {};
    vx03peer::label(label);
    print(console, crlf, "usart_peer - USART instrument peer (", device::part_name, ", ");
    vx03peer::print_die(console);
    print(console, ", label '", label, "', fw ", hex(firmware_version), ")", crlf);
    print(console, "link ", v303 ? "UART" : "USART", L::instance, " column ", L::column, ": TX P", L::pads.tx.port,
          L::pads.tx.pin, ", RX P", L::pads.rx.port, L::pads.rx.pin, "; the engine TIM", L::timer, " column ",
          L::timer_column, " CH3/CH4 on DMA1 channels ", L::tx_slot.channel, " and ", L::rx_slot.channel,
          "; command mode 8N1 @ ", link::command_baud, crlf);
    print(console, "boot: clk=",
          crystal_ok ? "HSE 8 MHz x18 = 144 MHz" : (clock_ok ? "HSI x18 (NO CRYSTAL, calibrate refused)" : "FAILED"),
          " tick=", tick_ok ? "STK" : "FAILED", " console=", console_ok ? "USART1" : "FAILED", crlf);

    // The engines, bound once with no interrupt: the binding names the two
    // compare registers, nothing a timer's configure() touches. At
    // very_high: a starved edge is a missed toggle and a whole counter
    // late (docs/design/dma.md - the peripheral's loss on starvation).
    Dma<1>::open();
    T::init();
    (void)T::remap(L::timer_column);
    (void)TxEdges::arm(T::chcvr_address(tx_ch), uint8_t{0}, DmaPriority::very_high);
    (void)RxEdges::arm(T::chcvr_address(rx_ch), uint8_t{0}, DmaPriority::very_high);
    (void)command_mode();
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
                    (void)command_mode();
                    print(console, "  command mode, dark until a frame checks out", crlf);
                } else if (c == '3') {
                    trace = !trace;
                    print(console, trace ? "  trace on" : "  trace off", crlf);
                } else if (c == 'f' || c == 'm' || c == 's') {
                    tx_speed = c == 'f' ? PinSpeed::fast : (c == 'm' ? PinSpeed::medium : PinSpeed::slow);
                    hold_pads();
                    print(console, "  TX pad class: ", speed_name(tx_speed), crlf);
                } else {
                    print(console, "? for help", crlf);
                }
                print(console, "> ");
            }
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
        const uint16_t st = U::status();
        if ((st & usart_rxne) == 0u) {
            continue;
        }
        const uint16_t d = U::read_word();
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
