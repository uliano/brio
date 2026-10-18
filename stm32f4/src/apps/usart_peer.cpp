// usart_peer - the INSTRUMENT half of a two-board USART test on the
// STM32F4: the scriptable peer a far board's serial suite (the DUT) drives
// IN BAND over the very link under test, speaking usart_link.hpp - the one
// wire format every architecture on this link compiles.
//
// Not a kernel app: one blocking loop that polls USART1, decodes a command
// frame, acknowledges it and becomes for a bounded moment what the DUT
// needs on the other end - an echo, a silent sink, a generator, a flood, a
// break, a foreign auto-baud sender, a bit-banger no clean UART could be.
// Every mode-changing command carries a frame count and a millisecond
// deadline, after which the peer restores command mode BY ITSELF (the
// protocol's recovery guarantee).
//
// LINK: USART1 on the desk's crossed pair (DS10693 table 11) -
//   PA9  USART1_TX (AF7)  to the far board's RX;  TIM1_CH2 (AF1)
//   PA10 USART1_RX (AF7)  from the far board's TX; TIM1_CH3 (AF1)
// USART1 sits on APB2 (90 MHz here): 5.625 Mbaud at OVER16, 11.25 at OVER8
// (DS10693 3.25). NO TOPOLOGY IS GUESSED: the desk fixes the crossed pair,
// so the peer is full duplex always and goes onto a shared wire (PA9 = the
// far board's RX wire) only inside onewire and collide.
//
// WHAT THIS SILICON MAKES OF THE REPERTOIRE (RM0390 ch. 25; the chapter's
// offer, item by item, is docs/stm32f4/usart.md's):
//  - ONE DATA REGISTER AND THE SHIFTER, no FIFO: the echo and the sink are
//    polled a character at a time, which the core affords - a character
//    at 7.5 Mbaud is 240 cycles of 180 MHz (the hot path's count below).
//    THE ECHO READS SR, THEN DR, THEN WRITES DR: ES0298 2.13.2 (a PE flag
//    cleared by a DR write that lands between the SR and the DR read, in
//    full duplex) is avoided by that order, the erratum's own workaround.
//    TRANSMIT DMA DECLINED for the same budget: a blast's loop is a TXE
//    poll and a store, gapless at the link's top rung.
//  - EVERY RATE FROM THE CRYSTAL: the core runs the ST-LINK's 8 MHz in
//    bypass through the PLL (Ident::xtal = 1 - the time base is
//    crystal-referenced, usart_link.hpp's meaning; 0 if the HSE did not
//    come up). The oversampling CLOSER to the rate (25.4.4): OVER16 and
//    OVER8 computed, the smaller error kept (OVER16 on a tie), OVER8 alone
//    under flag_clk2x; a rate past 11.25 Mbaud is REFUSED.
//  - FRAMES: M counts the parity bit (25.4.3), so the word is eight or nine
//    bits: 7+P, 8, 8+P and 9 are served; 5, 6 and 7 data bits without
//    parity, 6+P and 9+P have no word length and are REFUSED. Every code
//    of CR2.STOP is served on this FULL instance - one, two, 0.5 and 1.5
//    (25.6.5; 25.4.3 RECOMMENDS the halves for smartcard mode and forbids
//    them nowhere else) - except in IrDA, which wants one.
//  - THE SYNCHRONOUS ROLES ARE REFUSED. This USART has no slave (25.4.9:
//    the master's CK is an output), and its CK is PA8, which reaches the
//    far board only through a strap the desk has not got (the pair's
//    design, D2). flag_mpcm is refused: the multiprocessor wake is a
//    receiver's, the DUT's to test.
//  - IrDA SIR (25.4.12): IREN, the rate capped at 115200 as the chapter
//    caps it; normal mode at GTPR.PSC = 1, the one value 25.6.7 allows
//    there, so a receive filter (rxpl) is REFUSED in it; txpl != 0 asks
//    for the low-power encoder, IRLP with PSC = txpl / 3 APB2 periods
//    (its receiver then rejects pulses under two of them). No line
//    inversion on this USART: a far ST SIR ENDEC supplies the one each
//    way two of them need (25.4.12: "the transmit encoder output has the
//    opposite polarity to the decoder input").
//  - THE WAVEFORM ENGINE for brk, autobaud_tx's break, bitbang, glitch,
//    drive_rxd and collide: TIM1 (180 MHz, the APB2 timer clock), whose
//    CH2 is on PA9 and CH3 on PA10 (AF1, DS10693 table 11) - the link's
//    own pads. The pad is re-muxed from the USART to the timer with the
//    channel FORCED HIGH first (both idle high: no edge), the channel
//    turned to TOGGLE mode with CCR preload off, its first compare
//    written by the CPU and every later one by DMA2 stream 6 channel 0 on
//    the channel's own compare request (RM0390 table 29: TIM1_CH1/2/3 on
//    one stream, one CCxDE at a time) from a table of ABSOLUTE compare
//    values, the stream in direct mode on half-words - no CPU between
//    edges, no interrupt masked, no accumulation. The prescaler is chosen
//    per action so the longest interval fits half the 16-bit counter. The
//    cycle-counted loop is DECLINED: the image runs behind the ART, whose
//    hits and misses a loop's timing would be a function of.
//  - HDSEL for onewire and collide (25.4.10): the single wire on PA9, the
//    far board's RX wire (the G0's SWAP puts its single wire there), open
//    drain on the pad's pull-up for the length of the action.
//  - NO DRIVER ENABLE on this USART (25.3, table 148: no such row), and
//    PB13 - the desk's SPI SCK wire, which carries the far G0's
//    USART1_RTS_DE_CK - is never touched: an input as reset leaves it,
//    so the far board's RS-485 letters drive it alone.
//  - THE REPORT'S SPAN on the DWT's cycle counter (180 MHz), read once a
//    character as the loop sees it complete.
//
// TIME (usart_link.hpp, TIME IN THE PAYLOAD): every tick this peer reports
// or obeys is a 24 MHz tick: a DWT cycle is 2/15 of one, a TIM1 count at
// prescaler 0 the same, both from the crystal.
//
// THE DARK DISCIPLINE (usart_link.hpp): while it searches, PA9 is an
// input with no pull (the peer states the pad an input rather than trust
// a disabled transmitter) and PA10 the USART's receive pad with NO pull:
// the far board's single-board letters walk both wires with their own
// pulls. A frame that checks out proves the wiring: PA9 goes to the USART
// (push-pull, idling high) and PA10 takes its pull-up; rediscover_ms of
// quiet takes both back. A bad checksum is nak'ed only while ENGAGED.
//
// Console: USART2 PA2/PA3 at 115200, observability only.
//   ? help | i status and counters | 0 back to command mode, wiring
//   unproven | 2 wiring probe | 3 trace the commands | 4 the engine's
//   self-test on PA9 (the far board dark or listening)
//
// The ident label is the 96-bit unique device ID's first word in hex.
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrxt/src/apps/usart_link.hpp"

using SysClock =
    brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using link::Op;

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Console = Uart<2, console_pins>;
constexpr Console console;

constexpr uint16_t firmware_version = 0x0401;   ///< see usart_link.hpp's Ident

// ---------------------------------------------------------------------------
// The link's pads and instances
// ---------------------------------------------------------------------------

using Link = Usart<1>;
using TxPad = Pin<'A', 9>;    // USART1_TX AF7, TIM1_CH2 AF1
using RxPad = Pin<'A', 10>;   // USART1_RX AF7, TIM1_CH3 AF1
constexpr PinFunction usart_af = PinFunction::af7;
constexpr PinFunction timer_af = PinFunction::af1;
constexpr uint8_t tx_channel = 1;   // TIM1_CH2, 0-based as every channel verb
constexpr uint8_t rx_channel = 2;   // TIM1_CH3

using Engine = Tim<1>;
using EdgeDma = DmaStream<2, 6>;

/// The USART's kernel and the core, both from the crystal.
constexpr uint32_t usart_hz = SysClock::pclk2_hz;   // 90 MHz
constexpr uint32_t core_hz = SysClock::hz;           // 180 MHz: the DWT and TIM1
static_assert(usart_hz == 90'000'000u && core_hz == 180'000'000u,
              "the conversions below are written for HCLK 180 MHz, APB2 90 MHz");
static_assert(Engine::clock_hz(clock) == core_hz,
              "TIM1 counts the APB2 timer clock, twice PCLK2 (RM0390 6.2)");

// ---------------------------------------------------------------------------
// Time: 24 MHz ticks against the crystal's 180 MHz
// ---------------------------------------------------------------------------

/// 24 MHz ticks of `counts` core cycles (x 2/15), and back (x 15/2).
constexpr uint32_t ticks_of(uint32_t counts) {
    return static_cast<uint32_t>((static_cast<uint64_t>(counts) * 2u + 7u) / 15u);
}
constexpr uint32_t counts_of(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 15u + 1u) / 2u);
}
static_assert(ticks_of(180'000'000u) == 24'000'000u && counts_of(24'000'000u) == 180'000'000u);

[[gnu::always_inline]] inline uint32_t now_counts() { return CycleCounter::now(); }

void wait_counts(uint32_t n) {
    const uint32_t t0 = now_counts();
    while (now_counts() - t0 < n) {
    }
}

void wait_us(uint32_t us) { wait_counts(us * (core_hz / 1'000'000u)); }

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

// ---------------------------------------------------------------------------
// The link's configuration: a Cfg turned into registers, refused where the
// silicon has no answer
// ---------------------------------------------------------------------------

struct LinkSetup {
    UartFormat format{};
    uint16_t brr = 0;
    bool over8 = false;
    bool irda = false;
    IrdaConfig irda_cfg{};
    uint16_t mask = 0xFF;
};

uint16_t data_mask(uint8_t bits) {
    const uint8_t n = bits >= link::bits9_low ? 9 : bits;
    return static_cast<uint16_t>((1u << n) - 1u);
}

std::optional<LinkSetup> rate_setup(uint32_t rate, bool force8) {
    if (rate == 0u) {
        return std::nullopt;
    }
    LinkSetup s{};
    const std::optional<uint16_t> b16 = force8 ? std::nullopt : usart_brr(usart_hz, rate);
    const std::optional<uint16_t> b8 = usart_brr_over8(usart_hz, rate);
    auto err = [rate](uint32_t actual) {
        return actual > rate ? actual - rate : rate - actual;
    };
    if (b16 && (!b8 || err(usart_actual_baud(usart_hz, *b16)) <=
                           err(usart_actual_baud_over8(usart_hz, *b8)))) {
        s.brr = *b16;
        return s;
    }
    if (b8) {
        s.brr = *b8;
        s.over8 = true;
        return s;
    }
    return std::nullopt;
}

/// The Cfg's frame on this USART, or nothing where M has no word for it.
std::optional<UartFormat> format_of(const link::Cfg& c) {
    if (c.parity > 2u) {
        return std::nullopt;
    }
    UartFormat f{};
    const bool parity = c.parity != 0u;
    f.parity = c.parity == 1u ? UartParity::even : c.parity == 2u ? UartParity::odd : UartParity::none;
    switch (c.bits) {
        case 7: if (!parity) return std::nullopt; f.bits = UartBits::seven; break;
        case 8: f.bits = UartBits::eight; break;
        case link::bits9_low:
        case link::bits9_high: if (parity) return std::nullopt; f.bits = UartBits::nine; break;
        default: return std::nullopt;
    }
    if (c.stop == 1u) {
        f.stop = UartStop::one;
    } else if (c.stop == 2u) {
        f.stop = UartStop::two;
    } else if (c.stop == link::stop_half) {
        f.stop = UartStop::half;
    } else if (c.stop == link::stop_one_half) {
        f.stop = UartStop::one_and_half;
    } else {
        return std::nullopt;
    }
    return f;
}

std::optional<LinkSetup> setup_of(const link::Cfg& c) {
    if ((c.flags & link::flag_mpcm) != 0u) {
        return std::nullopt;
    }
    if (c.mode == link::Mode::sync_client || c.mode == link::Mode::sync_host) {
        return std::nullopt;   // no slave on this USART, no CK wire on this desk
    }
    const std::optional<UartFormat> f = format_of(c);
    if (!f) {
        return std::nullopt;
    }
    const bool ir = c.mode == link::Mode::ircom;
    if (ir && (c.rate > 115'200u || c.stop != 1u || (c.flags & link::flag_clk2x) != 0u)) {
        return std::nullopt;   // 25.4.12: SIR to 115.2 kbaud, one stop bit
    }
    std::optional<LinkSetup> s;
    if (ir) {
        const std::optional<uint16_t> b = usart_brr(usart_hz, c.rate);
        if (!b) {
            return std::nullopt;
        }
        s = LinkSetup{};
        s->brr = *b;
    } else {
        s = rate_setup(c.rate, (c.flags & link::flag_clk2x) != 0u);
    }
    if (!s) {
        return std::nullopt;
    }
    s->format = *f;
    s->mask = data_mask(c.bits);
    if (ir) {
        s->irda = true;
        IrdaConfig ic{};
        // PSC divides PCLK2 (25.6.7): the AVR's pulse lengths are 24 MHz
        // cycles, so a 24 MHz tick is 3.75 PSC counts.
        if (c.txpl != 0u) {
            const uint32_t psc = (static_cast<uint32_t>(c.txpl) * 15u / 4u + 1u) / 3u;
            ic.low_power = true;
            ic.prescaler = static_cast<uint8_t>(psc == 0u ? 1u : (psc > 255u ? 255u : psc));
        } else if (c.rxpl != 0u) {
            // 25.6.7: "In normal IrDA mode: PSC must be set to 00000001" -
            // this ENDEC's normal-mode receiver has no filter to program.
            return std::nullopt;
        }
        s->irda_cfg = ic;
    }
    return s;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

bool crystal_ok = false;
bool engine_ok = false;
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

constexpr uint16_t command_brr = usart_brr(usart_hz, link::command_baud).value();
static_assert(command_brr == 781u, "115200 at 90 MHz, OVER16: 781.25 rounded");

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(link::settle_ms) + 2u) {
    }
}

// ---------------------------------------------------------------------------
// The pads, by the dark discipline
// ---------------------------------------------------------------------------

void tx_dark() { TxPad::input(PinPull::none); }
void tx_live() { TxPad::function(usart_af, {.speed = PinSpeed::very_high}); }
void rx_pad() { RxPad::function(usart_af, {.pull = proven ? PinPull::up : PinPull::none}); }

/// The receive side cleared: SR then DR (25.6.1's sequence) until RXNE is
/// down, which drops a stale character and every error flag with it.
void flush_rx() {
    for (uint8_t k = 0; k < 4u; ++k) {
        Link::clear_by_read();
    }
}

/// Program USART1 from a setup: the block reset first, so nothing a
/// previous mode left in CR2/CR3/GTPR rides along, then the frame, the
/// options and the enables.
bool link_apply(const LinkSetup& s, bool hdsel = false) {
    Link::enable(false);
    Link::reset();
    if (!Link::configure(s.format, s.brr)) {
        return false;
    }
    if (s.over8 && !Link::oversampling8(true)) {
        return false;
    }
    if (hdsel && !Link::half_duplex(true)) {
        return false;
    }
    if (s.irda && !Link::irda(s.irda_cfg)) {
        return false;
    }
    Link::transmitter(true);
    Link::receiver(true);
    Link::enable(true);
    flush_rx();
    return true;
}

LinkSetup command_setup() {
    LinkSetup s{};
    s.brr = command_brr;
    return s;
}

bool command_mode() {
    const bool ok = link_apply(command_setup());
    if (proven) {
        tx_live();
    } else {
        tx_dark();
    }
    rx_pad();
    flush_rx();
    decoder.reset();
    return ok;
}

bool apply_cfg(const link::Cfg& c, uint16_t& mask) {
    if (!c.apply) {
        return true;
    }
    const std::optional<LinkSetup> s = setup_of(c);
    if (!s || !link_apply(*s)) {
        (void)command_mode();
        return false;
    }
    mask = s->mask;
    tx_live();
    rx_pad();
    flush_rx();
    return true;
}

// ---------------------------------------------------------------------------
// Talking
// ---------------------------------------------------------------------------

constexpr uint32_t spin_bound = 2'000'000u;

[[gnu::always_inline]] inline bool put_frame(uint16_t v) {
    USART_TypeDef& u = Link::regs();
    for (uint32_t n = spin_bound; (u.SR & USART_SR_TXE) == 0u; --n) {
        if (n == 0u) return false;
    }
    u.DR = v;
    return true;
}

void wait_tx_done() {
    for (uint32_t n = spin_bound; !Link::tx_complete() && n != 0u; --n) {
    }
}

void turnaround_guard() { wait_us(35u); }

void put_link(uint8_t b) { (void)put_frame(b); }

void answer(Op op, const uint8_t* p, uint8_t len) {
    turnaround_guard();
    if (!proven) {
        tx_live();
    }
    link::write_frame(put_link, op, p, len);
    wait_tx_done();
    if (!proven) {
        tx_dark();
    }
}

void ack(Op op, uint8_t sum, bool good) {
    const uint8_t p[2] = {link::byte_of(op), sum};
    answer(good ? Op::ack : Op::nak, p, 2);
}

// ---------------------------------------------------------------------------
// The waveform engine: TIM1 toggling on its compares, DMA2 stream 6
// writing each next ABSOLUTE compare
// ---------------------------------------------------------------------------

constexpr uint16_t edge_most = 512;
uint32_t edges[edge_most];
uint16_t table[edge_most];
uint16_t edge_n = 0;
bool edge_overflow = false;

void edges_clear() {
    edge_n = 0;
    edge_overflow = false;
}

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

/// THE ENGINE'S FLOOR, in timer counts: the shortest interval between two
/// consecutive edges the stream can follow - the compare matches, the
/// request reaches DMA2, the stream reads the next table entry and writes
/// CCR across the APB2 bridge, and only then may the counter reach the new
/// value. Measured with the console's '4' and the floor lowered for the
/// run: an interval of 8 counts (44 ns) played, of 6 the compare was
/// written too late (the edge a counter wrap later); 12 keeps a margin
/// (docs/stm32f4/usart.md).
constexpr uint32_t engine_floor_counts = 12;
/// The run's lead at prescaler 0: from EGR.UG to the first compare, the
/// stream programmed and the channel turned to toggle in between.
constexpr uint32_t lead_floor_counts = 1800;   // 10 us

constexpr DmaStreamConfig edge_cfg{.channel = 0,
                                   .direction = DmaDirection::memory_to_peripheral,
                                   .memory_increment = true,
                                   .peripheral_width = DmaWidth::half,
                                   .memory_width = DmaWidth::half,
                                   .priority = DmaPriority::very_high};

bool engine_up() {
    Engine::init();
    if (!Engine::configure({.prescaler = 0, .period = 0xFFFF})) {
        return false;
    }
    for (const uint8_t ch : {tx_channel, rx_channel}) {
        if (!Engine::output_channel(ch, {.mode = TimOutputMode::force_active, .preload = false})) {
            return false;
        }
    }
    (void)Engine::main_output(true);
    Engine::enable(true);
    Dma<2>::bus_clock(true);
    EdgeDma::stop();
    return true;
}

uint32_t engine_runs = 0, engine_broken = 0;
uint32_t edges_seen_total = 0;

/// Play the list on channel `ch` (its pad already the timer's), `lead`
/// counts at prescaler 0 from now - the same contract as the STM32G0
/// peer's engine: the run whole when every interval clears the floor, the
/// stream completed, the last edge passed in time and the pad high at the
/// end, the CPU SAMPLING THE PAD meanwhile and counting its transitions.
template <typename Pad>
bool play(uint8_t ch, uint32_t lead) {
    if (edge_overflow || edge_n < 2u || (edge_n & 1u) != 0u) {
        return false;
    }
    if (lead < lead_floor_counts) {
        lead = lead_floor_counts;
    }
    uint32_t longest = edges[0] + lead, shortest = 0xFFFFFFFFu;
    for (uint16_t i = 1; i < edge_n; ++i) {
        if (edges[i] <= edges[i - 1]) {
            return false;
        }
        const uint32_t d = edges[i] - edges[i - 1];
        if (d > longest) longest = d;
        if (d < shortest) shortest = d;
    }
    const uint32_t psc = longest / 32'000u;
    if (psc > 0xFFFFu || shortest / (psc + 1u) < engine_floor_counts) {
        return false;
    }
    const uint32_t div = psc + 1u;
    for (uint16_t i = 0; i < edge_n; ++i) {
        table[i] = static_cast<uint16_t>((lead + edges[i] + div / 2u) / div);
    }
    // Judged on the pad where every interval outlasts the sampling loop
    // and a SysTick entry inside it several times over.
    const bool judge_count = shortest >= 720u;   // 4 us

    TIM_TypeDef& t = Engine::regs();
    const uint32_t mask = Pad::mask;
    volatile uint32_t& idr = GPIOA->IDR;
    // Everything the run's bound needs is computed BEFORE the counter
    // restarts: a 64-bit division inside the lead would eat it.
    const uint16_t last = table[edge_n - 1u];
    const uint32_t bound_ms = static_cast<uint32_t>(
        (static_cast<uint64_t>(lead + edges[edge_n - 1u]) * 1000u) / core_hz + 3u);
    Engine::set_prescaler(static_cast<uint16_t>(psc));
    t.EGR = TIM_EGR_UG;
    const uint32_t started = CycleCounter::now();
    Engine::clear_flags(0xFFFFFFFFu);
    (void)Engine::output_mode(ch, TimOutputMode::toggle);
    (void)Engine::set_compare(ch, table[0]);
    const bool loaded = EdgeDma::prepare({.peripheral = Engine::ccr_address(ch),
                                          .memory = &table[1],
                                          .count = static_cast<uint16_t>(edge_n - 1u),
                                          .config = edge_cfg});
    if (!loaded) {
        (void)Engine::output_mode(ch, TimOutputMode::force_active);
        return false;
    }
    (void)EdgeDma::enable();
    Engine::interrupts(Engine::compare_dma(ch), true);

    const uint32_t t0 = Ticker::millis();
    // The line is high at the start: the sampler's first level is known,
    // not read - the first edge may come before the first look.
    uint32_t prev = mask;
    uint32_t seen = 0;
    bool ok = true;
    while (!EdgeDma::flag(DmaFlag::complete)) {
        const uint32_t now = idr & mask;
        if (now != prev) {
            ++seen;
            prev = now;
        }
        if (Ticker::millis() - t0 > bound_ms) {
            ok = false;
            break;
        }
    }
    while (ok && static_cast<int16_t>(static_cast<uint16_t>(t.CNT) - last) < 0) {
        const uint32_t now = idr & mask;
        if (now != prev) {
            ++seen;
            prev = now;
        }
        if (Ticker::millis() - t0 > bound_ms) {
            ok = false;
        }
    }
    // A compare the DMA wrote too late fires a whole counter wrap later:
    // the run is then longer than its list by about 65536 counts, which is
    // the one failure the end level cannot show (the toggles still pair).
    const uint32_t elapsed = CycleCounter::now() - started;
    if (elapsed > lead + edges[edge_n - 1u] + 32'768u * div) {
        ok = false;
    }
    (void)Engine::output_mode(ch, TimOutputMode::force_active);
    Engine::interrupts(Engine::compare_dma(ch), false);
    EdgeDma::stop();
    for (uint8_t k = 0; k < 4u; ++k) {
        const uint32_t now = idr & mask;
        if (now != prev) {
            ++seen;
            prev = now;
        }
    }
    if ((idr & mask) == 0u) {
        ok = false;
    }
    if (judge_count && seen != edge_n) {
        ok = false;
    }
    edges_seen_total += seen;
    ++engine_runs;
    if (!ok) {
        ++engine_broken;
    }
    return ok;
}

template <typename Pad>
void pad_to_timer(bool open_drain = false) {
    Pad::function(timer_af, {.pull = open_drain ? PinPull::up : PinPull::none,
                             .open_drain = open_drain, .speed = PinSpeed::very_high});
}

/// Bit times in core counts, in sixteenths.
uint32_t bit_q4(uint32_t rate) {
    if (rate == 0u) {
        rate = 9600u;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(core_hz) * 16u) / rate);
}
uint32_t q4_counts(uint32_t q4) { return (q4 + 8u) >> 4; }
bool bits_fit(uint32_t q, uint32_t n) { return n == 0u || q <= 0x7FFFFFFFu / n; }

uint32_t frame_edges(uint32_t at, uint32_t q, uint8_t value, int8_t cell, int8_t pct) {
    for (int8_t i = 0; i < 10; ++i) {
        const bool high = i == 0 ? false : (i == 9 ? true : ((value >> (i - 1)) & 1u) != 0u);
        edge_level(q4_counts(at), high);
        int32_t w = static_cast<int32_t>(q);
        if (cell == i && pct != 0) {
            w += (static_cast<int32_t>(q) / 100) * pct;
        }
        if (w < 16 * static_cast<int32_t>(engine_floor_counts)) w = 16 * static_cast<int32_t>(engine_floor_counts);
        at += static_cast<uint32_t>(w);
    }
    return at;
}

// ---------------------------------------------------------------------------
// Accounting
// ---------------------------------------------------------------------------

[[gnu::always_inline]] inline void count_errors(link::Report& r, uint32_t st) {
    if ((st & UsartFlag::fe) != 0u && r.ferr < 255u) ++r.ferr;
    if ((st & UsartFlag::pe) != 0u && r.perr < 255u) ++r.perr;
    if ((st & UsartFlag::ore) != 0u && r.ovf < 255u) ++r.ovf;
}

// ---------------------------------------------------------------------------
// The actions
// ---------------------------------------------------------------------------

/// THE HOT PATH: a character polled, stamped, accounted and - for an echo
/// - written back, with no call. SR is read once a turn and decides
/// everything: RXNE, the character's errors (their clear is the DR read
/// that follows, 25.6.1), TXE for the echo - written only after the DR
/// read, ES0298 2.13.2's order. COUNTED IN THE RELEASE LISTING
/// (docs/stm32f4/usart.md, "The peer's hot path").
[[gnu::noinline]] link::Report run_echo(const link::Params& a, uint16_t mask, bool sink) {
    link::Report r{};
    settle();
    flush_rx();
    USART_TypeDef& u = Link::regs();
    volatile uint32_t& cyc = DWT->CYCCNT;
    uint32_t first = 0, last = 0;
    uint16_t count = 0, sum = 0;
    const uint16_t want = a.count;
    const uint32_t ms = a.ms;
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        const uint32_t st = u.SR;
        if ((st & USART_SR_RXNE) != 0u) {
            const uint32_t stamp = cyc;
            const uint16_t d = static_cast<uint16_t>(u.DR);
            if ((st & UsartFlag::receive_errors) != 0u) [[unlikely]] {
                count_errors(r, st);
            }
            if (!sink) {
                if ((st & USART_SR_TXE) == 0u) [[unlikely]] {
                    (void)put_frame(d);
                } else {
                    u.DR = d;
                }
            }
            if (count == 0u) first = stamp;
            last = stamp;
            sum = static_cast<uint16_t>(sum + (d & mask));
            if (++count >= want) {
                break;
            }
            continue;
        }
        if (Ticker::millis() - t0 >= ms) {
            r.flags |= link::report_timed_out;
            break;
        }
    }
    r.count = count;
    r.sum = sum;
    if (count >= 2u) {
        r.span = ticks_of(last - first);
    }
    if (!sink && count != 0u) {
        wait_tx_done();
    }
    return r;
}

/// The DUT's gapless frames timed on THIS crystal: a sink's span and the
/// DUT's error against it in ppm. A crystal-referenced peer adopts
/// nothing (usart_link.hpp's calibrate).
link::Report run_calibrate(const link::Params& a, uint16_t mask) {
    link::Report r = run_echo(a, mask, true);
    const uint32_t nominal =
        link::nominal_span(a.cfg.rate, link::frame_half_bits(a.cfg.bits, a.cfg.parity, a.cfg.stop),
                           r.count);
    r.aux16 = static_cast<uint16_t>(link::calibrate_ppm(nominal, r.span));
    return r;
}

link::Report run_set_link(const link::Params& a, uint16_t mask) {
    link::Report r{};
    settle();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        const uint32_t st = Link::status();
        if ((st & USART_SR_RXNE) != 0u) {
            const uint16_t d = Link::read_word();
            count_errors(r, st);
            r.sum = static_cast<uint16_t>(r.sum + (d & mask));
            ++r.count;
        }
    }
    r.flags |= link::report_timed_out;
    return r;
}

/// The rate this transmitter really runs at against `asked`, in ppm - the
/// Report's aux16 for send and blast (usart_link.hpp): the kernel is APB2
/// from the crystal (taken as exact), the divisor what BRR says - OVER16
/// divides by BRR itself (sixteenths of USARTDIV), OVER8 by
/// 8 x mantissa + fraction (25.4.4). Nothing with the HSE down: the
/// Report then leaves report_rate_stated clear.
std::optional<uint16_t> rate_report(uint32_t asked) {
    if (!crystal_ok) {
        return std::nullopt;
    }
    const uint32_t brr = Link::brr();
    const uint32_t div = Link::oversampling8() ? (((brr >> 4) << 3) | (brr & 0x7u)) : brr;
    return static_cast<uint16_t>(link::rate_error_ppm(usart_hz, div, asked));
}

link::Report run_send(const link::Params& a, uint16_t mask, bool blast) {
    link::Report r{};
    settle();
    if (!blast && a.aux16 != 0u) {
        wait_ms(a.aux16);
    }
    const uint32_t gap_counts = blast ? 0u : 2u * (core_hz / (a.cfg.rate ? a.cfg.rate : 1u));
    const uint16_t repeats = blast && a.aux16 > 1u ? a.aux16 : 1u;
    const uint32_t repeat_gap_us = static_cast<uint32_t>(a.aux8) * 100u;
    link::Stream stream(a.pattern, a.value);
    USART_TypeDef& u = Link::regs();
    const uint16_t n = a.count;
    const uint32_t t0 = Ticker::millis();
    uint16_t count = 0, sum = 0;
    for (uint16_t k = 0; k < repeats; ++k) {
        if (k != 0u) {
            if (a.ms != 0u && Ticker::millis() - t0 >= a.ms) {
                r.flags |= link::report_timed_out;
                break;
            }
            if (repeat_gap_us != 0u) {
                wait_tx_done();
                wait_us(repeat_gap_us);
            }
        }
        bool sent = true;
        for (uint16_t i = 0; i < n; ++i) {
            const uint16_t v = static_cast<uint16_t>(stream.next() & mask);
            uint32_t spins = spin_bound;
            while ((u.SR & USART_SR_TXE) == 0u) {
                if (--spins == 0u) {
                    sent = false;
                    break;
                }
            }
            if (!sent) {
                break;
            }
            u.DR = v;
            sum = static_cast<uint16_t>(sum + v);
            ++count;
            if (gap_counts != 0u) {
                wait_tx_done();
                wait_counts(gap_counts);
            }
        }
        if (!sent) break;
    }
    r.count = count;
    r.sum = sum;
    if (count != 0u) {
        wait_tx_done();
    }
    if (const std::optional<uint16_t> stated = rate_report(a.cfg.rate)) {
        r.aux16 = *stated;
        r.flags |= link::report_rate_stated;
    }
    return r;
}

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
    edge_level(q4_counts(q * a.count), true);
    wait_tx_done();
    pad_to_timer<TxPad>();
    const bool ok = play<TxPad>(tx_channel, q4_counts(2u * q));
    wait_counts(q4_counts(2u * q));
    tx_live();
    if (!ok) r.flags |= link::report_mismatch;
    r.count = a.count;
    r.aux16 = Link::brr();
    return r;
}

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
    edge_level(q4_counts(q * n), true);
    wait_tx_done();
    pad_to_timer<TxPad>();
    const bool ok = play<TxPad>(tx_channel, q4_counts(4u * q));
    wait_counts(q4_counts(2u * q));
    tx_live();
    (void)put_frame(a.value);
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
    r.aux16 = Link::brr();
    return r;
}

template <typename Pad>
link::Report run_bitbang(const link::Params& a, uint8_t ch) {
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
        edge_level(q4_counts(at), true);
        at += 2u * q;
    }
    for (uint16_t i = 0; i < a.count; ++i) {
        at = frame_edges(at, q, a.value, a.cell, a.pct);
        at += 4u * q;
        r.sum = static_cast<uint16_t>(r.sum + a.value);
        ++r.count;
    }
    if (edge_overflow) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    settle();
    pad_to_timer<Pad>();
    if (!play<Pad>(ch, q4_counts(2u * q))) {
        r.flags |= link::report_mismatch;
    }
    r.aux16 = static_cast<uint16_t>(ticks_of(q4_counts(q)) > 0xFFFFu ? 0xFFFFu
                                                                     : ticks_of(q4_counts(q)));
    return r;
}

link::Report run_glitch(const link::Params& a) {
    link::Report r{};
    const uint32_t w = counts_of(a.aux16 != 0u ? a.aux16 : 1u);
    const uint32_t gap = static_cast<uint32_t>(a.aux8 ? a.aux8 : 1u) * (core_hz / 1'000'000u);
    edges_clear();
    uint32_t at = 0;
    for (uint16_t i = 0; i < a.count; ++i) {
        edge_level(at, false);
        edge_level(at + w, true);
        at += w + gap;
    }
    if (edge_overflow || w < engine_floor_counts) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    settle();
    pad_to_timer<TxPad>();
    if (!play<TxPad>(tx_channel, lead_floor_counts)) {
        r.flags |= link::report_mismatch;
    }
    tx_live();
    r.count = a.count;
    r.aux16 = a.aux16;
    return r;
}

/// stage (usart_link.hpp): a frame train drawn whole by the engine on PA9
/// (TIM1_CH2) - every cell of every frame from usart_link's own cell
/// table, the PULSE inside each frame (the line driven to the level
/// opposite its cell's for aux8 per cent of a bit, from pct per cent into
/// cell `cell`), the idle between frames in sixteenths of a bit - in the
/// crystal's core counts, so the rate is the crystal's and not a
/// divisor's. Each frame is a run of boundaries in time order (its cell
/// starts, the pulse's two ends, the next slot's start), the level at
/// each one read off the frame: the edge list holds only real changes
/// (edge_level drops the rest). Refused where the engine cannot draw it:
/// a pulse narrower than the floor, or more edges than the table holds.
link::Report run_stage(const link::Params& a) {
    link::Report r{};
    const uint8_t data = a.cfg.bits >= link::bits9_low ? 9u : a.cfg.bits;
    const uint8_t parity = a.cfg.parity;
    if (data < 5u || data > 9u || parity > 2u || (a.cfg.stop != 1u && a.cfg.stop != 2u)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    const uint8_t cells = static_cast<uint8_t>(1u + data + (parity != 0u ? 1u : 0u) + a.cfg.stop);
    const uint32_t q = bit_q4(a.cfg.rate);
    const uint32_t gap = static_cast<uint32_t>((static_cast<uint64_t>(q) * a.aux16) / 16u);
    const uint32_t slot = q * cells + gap;
    const bool pulse = a.cell >= 0 && a.cell < static_cast<int8_t>(cells) && a.aux8 != 0u && a.pct >= 0 &&
                       a.pct < 100;
    const uint32_t p0 = pulse ? q * static_cast<uint8_t>(a.cell) +
                                    static_cast<uint32_t>((static_cast<uint64_t>(q) * static_cast<uint8_t>(a.pct)) / 100u)
                              : 0u;
    const uint32_t p1 = pulse ? p0 + static_cast<uint32_t>((static_cast<uint64_t>(q) * a.aux8) / 100u) : 0u;
    if (!bits_fit(slot, a.count + 1u) ||
        (pulse && (q4_counts(p1 - p0) < engine_floor_counts || p1 > slot))) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    edges_clear();
    link::Stream st(a.pattern, a.value);
    uint32_t at = 0;
    for (uint16_t i = 0; i < a.count; ++i) {
        const uint16_t v = static_cast<uint16_t>(st.next() & data_mask(data));
        // The boundaries in time order: the cell starts (the last one the
        // idle's), the pulse's two ends merged in where they fall.
        uint32_t when[link::stage_cells_most + 3];
        uint8_t n = 0;
        for (uint8_t k = 0; k <= cells; ++k) {
            when[n++] = q * k;
        }
        if (pulse) {
            for (const uint32_t t : {p0, p1}) {
                uint8_t j = n++;
                while (j != 0u && when[j - 1u] > t) {
                    when[j] = when[j - 1u];
                    --j;
                }
                when[j] = t;
            }
        }
        // The pulse's level is the opposite of ITS cell's in THIS frame.
        const bool pulse_level =
            pulse && !link::stage_cell_high(v, data, parity, static_cast<uint8_t>(a.cell));
        for (uint8_t j = 0; j < n; ++j) {
            const uint32_t t = when[j];
            const bool level = (pulse && t >= p0 && t < p1) ? pulse_level
                               : t >= q * cells           ? true
                                                          : link::stage_cell_high(v, data, parity,
                                                                                  static_cast<uint8_t>(t / q));
            edge_level(q4_counts(at + t), level);
        }
        at += slot;
        r.sum = static_cast<uint16_t>(r.sum + v);
        ++r.count;
    }
    if (edge_overflow) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    for (uint16_t i = 1; i < edge_n; ++i) {
        if (edges[i] - edges[i - 1] < engine_floor_counts) {
            r.flags |= link::report_cfg_failed;   // two edges closer than the engine follows
            return r;
        }
    }
    settle();
    pad_to_timer<TxPad>();
    if (!play<TxPad>(tx_channel, q4_counts(2u * q))) {
        r.flags |= link::report_mismatch;
    }
    tx_live();
    r.aux16 = static_cast<uint16_t>(ticks_of(q4_counts(q)) > 0xFFFFu ? 0xFFFFu : ticks_of(q4_counts(q)));
    return r;
}

/// THE DESK'S SHARED WIRE is this board's TX wire (PA9 here, the far G0's
/// PA10, where its SWAP puts its single wire). HDSEL joins the receiver to
/// it (25.4.10): open drain on the pad's pull-up, the transmitter released
/// between frames - so the peer only ever pulls the wire low, and only to
/// reply. PA10 is left an input with no pull meanwhile.
bool wire_join(const LinkSetup& s) {
    if (!link_apply(s, true)) {
        return false;
    }
    TxPad::function(usart_af, {.pull = PinPull::up, .open_drain = true, .speed = PinSpeed::high});
    RxPad::input(PinPull::none);
    flush_rx();
    return true;
}

std::optional<LinkSetup> wire_setup(uint32_t rate) {
    return rate_setup(rate ? rate : link::command_baud, false);
}

link::Report run_onewire(const link::Params& a) {
    link::Report r{};
    const std::optional<LinkSetup> s = wire_setup(a.cfg.rate);
    if (!s || !wire_join(*s)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    const uint32_t guard = 4u * (core_hz / (a.cfg.rate ? a.cfg.rate : link::command_baud));
    const uint32_t bit2 = guard / 2u;
    settle();
    const uint32_t t0 = Ticker::millis();
    while (r.count < a.count) {
        const uint32_t st = Link::status();
        if ((st & USART_SR_RXNE) != 0u) {
            const uint16_t d = Link::read_word();
            count_errors(r, st);
            r.sum = static_cast<uint16_t>(r.sum + (d & 0xFFu));
            ++r.count;
            wait_counts(guard);
            (void)put_frame(static_cast<uint16_t>((d ^ 0xFFu) & 0xFFu));
            wait_tx_done();
            // THE REPLY'S OWN ECHO COMES AFTER TC: the receiver samples the
            // stop bit it shares with the transmitter and raises RXNE a
            // fraction of a bit after TC (measured: a flush at TC left the
            // echo standing, the loop answered it inside the DUT's next
            // frame). So the echo is waited for, bounded by two bit times,
            // and only then flushed.
            for (uint32_t c0 = now_counts(); (Link::status() & USART_SR_RXNE) == 0u && now_counts() - c0 < bit2;) {
            }
            flush_rx();
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
/// the DUT's NEXT frame aux16 microseconds later - OPEN DRAIN: PA9 handed
/// to TIM1_CH2 as an open-drain output on its pull-up, so the engine's
/// high is a release and its low a pull.
link::Report run_collide(const link::Params& a) {
    link::Report r{};
    const std::optional<LinkSetup> s = wire_setup(a.cfg.rate);
    if (!s || !wire_join(*s)) {
        r.flags |= link::report_cfg_failed;
        return r;
    }
    const uint32_t q = bit_q4(a.cfg.rate ? a.cfg.rate : link::command_baud);
    edges_clear();
    (void)frame_edges(0, q, a.value, -1, 0);
    settle();
    const uint32_t lead = static_cast<uint32_t>(a.aux16) * (core_hz / 1'000'000u);
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        const uint32_t st = Link::status();
        if ((st & USART_SR_RXNE) != 0u) {
            (void)Link::read_word();
            count_errors(r, st);
            ++r.count;
            pad_to_timer<TxPad>(true);
            if (!play<TxPad>(tx_channel, lead)) {
                r.flags |= link::report_mismatch;
            }
            TxPad::function(usart_af, {.pull = PinPull::up, .open_drain = true,
                                       .speed = PinSpeed::high});
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

    if (op == Op::standby) {
        const uint16_t ms = f.len >= link::params_size ? link::get_params(f.data).ms : 0;
        standby_until = ms ? Ticker::millis() + ms : 0;
        ack(op, f.sum, true);
        if (standby_until != 0) {
            tx_dark();
        }
        return;
    }
    if (standby_until != 0) {
        if (Ticker::millis() < standby_until) return;
        standby_until = 0;
        tx_live();
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

    const bool own_config = op == Op::bitbang || op == Op::glitch || op == Op::drive_rxd ||
                            op == Op::onewire || op == Op::collide || op == Op::stage;
    const bool engined = op == Op::bitbang || op == Op::glitch || op == Op::drive_rxd ||
                         op == Op::brk || op == Op::autobaud_tx || op == Op::collide || op == Op::stage;
    uint16_t mask = data_mask(a.cfg.bits);
    bool ok = !engined || engine_ok;
    if (ok && !own_config) ok = apply_cfg(a.cfg, mask);

    if (!ok) {
        r.flags |= link::report_cfg_failed;
    } else {
        switch (op) {
            case Op::set_link: r = run_set_link(a, mask); break;
            case Op::echo: r = run_echo(a, mask, false); break;
            case Op::sink: r = run_echo(a, mask, true); break;
            case Op::send: r = run_send(a, mask, false); break;
            case Op::blast: r = run_send(a, mask, true); break;
            case Op::send_mpcm: r = run_send_mpcm(a, f.data + link::params_size); break;
            case Op::brk: r = run_break(a); break;
            case Op::autobaud_tx: r = run_autobaud_tx(a); break;
            case Op::bitbang: r = run_bitbang<TxPad>(a, tx_channel); break;
            case Op::glitch: r = run_glitch(a); break;
            case Op::drive_rxd: r = run_bitbang<RxPad>(a, rx_channel); break;
            case Op::onewire: r = run_onewire(a); break;
            case Op::collide: r = run_collide(a); break;
            case Op::calibrate: r = run_calibrate(a, mask); break;
            case Op::stage: r = run_stage(a); break;
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
    print(console, "  link USART1 PA9/PA10, the crossed pair: ", proven ? "PROVEN" : "searching (dark)",
          ", BRR=", Link::brr(), " PA9 ", proven ? "driven" : "released", crlf);
    print(console, "  time base ", crystal_ok ? "the ST-LINK's 8 MHz crystal clock" : "NOMINAL (no HSE)",
          "; engine ", engine_ok ? "up" : "DOWN", ", runs ", engine_runs, " broken ", engine_broken,
          crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          standby_until != 0 && Ticker::millis() < standby_until ? "  (STAND-OFF)" : "",
          engaged() ? "  (ENGAGED)" : "", crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " sum=", hex(last_report.sum), " ferr=", last_report.ferr,
          " perr=", last_report.perr, " ovf=", last_report.ovf,
          " flags=", hex(last_report.flags), " aux=", last_report.aux16,
          " ms=", last_report.ms, " span=", last_report.span, crlf);
}

/// THE WIRING PROBE. PA9 driven at 4 Hz and PA10 at 2 Hz for six seconds,
/// then both listened to for six more under pull-ups, the edge count naming
/// which far pad each meets. Console-only, for a far board dark or
/// listening.
constexpr uint16_t probe_ms = 6000;

void wire_probe() {
    Link::enable(false);
    print(console, "  driving PA9 at 4 Hz and PA10 at 2 Hz for 6 s, then listening 6 s", crlf);
    TxPad::output(false);
    RxPad::output(false);
    for (uint16_t t = 0; t < probe_ms; ++t) {
        if (((t / 125u) & 1u) != 0u) TxPad::set(); else TxPad::clear();
        if (((t / 250u) & 1u) != 0u) RxPad::set(); else RxPad::clear();
        wait_us(1000u);
    }
    TxPad::input(PinPull::up);
    RxPad::input(PinPull::up);
    uint16_t e9 = 0, e10 = 0;
    bool p9 = TxPad::read(), p10 = RxPad::read();
    for (uint16_t t = 0; t < probe_ms; ++t) {
        wait_us(1000u);
        const bool n9 = TxPad::read(), n10 = RxPad::read();
        if (n9 != p9) ++e9;
        if (n10 != p10) ++e10;
        p9 = n9;
        p10 = n10;
    }
    print(console, "  heard on PA9: ", e9, " edges, on PA10: ", e10, " edges", crlf);
    (void)command_mode();
}

/// Console '4': the engine on PA9 alone - glitches of 12, 16, 24 and 48
/// counts (the floor and above), four pulses of 300 ticks and a 9600-baud
/// 0x55 frame, each run's verdict printed. PA9 meets the far board's RX,
/// an input: run it with that board dark or listening.
void engine_self_test() {
    wait_tx_done();
    pad_to_timer<TxPad>();
    for (const uint32_t w : {12u, 16u, 24u, 48u}) {
        edges_clear();
        edge_level(0, false);
        edge_level(w, true);
        edge_level(w + 6000u, false);
        edge_level(w + 6000u + w, true);
        const bool ok = play<TxPad>(tx_channel, lead_floor_counts);
        print(console, "  two glitches of ", w, " counts: ", ok ? "played" : "BROKEN", crlf);
    }
    edges_clear();
    uint32_t at = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        edge_level(at, false);
        edge_level(at + counts_of(300), true);
        at += counts_of(2400);
    }
    (void)frame_edges(at * 16u, bit_q4(9600), 0x55, -1, 0);
    const uint16_t n = edge_n;
    const uint32_t seen0 = edges_seen_total;
    const bool ok = play<TxPad>(tx_channel, lead_floor_counts);
    print(console, "  four pulses of 300 ticks and a 9600-baud 0x55: ", n, " edges, ",
          edges_seen_total - seen0, " seen on the pad, ", ok ? "whole" : "BROKEN", crlf);
    tx_live();
    (void)command_mode();
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void USART2_IRQHandler() { (void)Console::isr(); }

int main() {
    crystal_ok = SysClock::init();
    (void)Console::init(clock, 115200);
    (void)Ticker::init(clock);
    (void)CycleCounter::init();
    brio::enable_interrupts();

    const uint32_t w0 = DeviceUid::read().word[0];
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        label8[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }

    tx_dark();
    engine_ok = engine_up();
    Link::bus_clock(true);

    print(console, crlf, "usart_peer - USART instrument peer (STM32F4 board ", label8,
          ", fw ", hex(firmware_version), ", ", crystal_ok ? "the HSE in bypass" : "NO HSE", ")", crlf);
    print(console, "link USART1 PA9 TX / PA10 RX, the crossed pair, APB2 90 MHz; command mode 8N1 @ ",
          link::command_baud, " (BRR ", command_brr, ")", crlf);
    print(console, "DARK until a frame checks out: PA9 an input, no pull on either link pad", crlf);
    if (!engine_ok) {
        print(console, "WARNING: the waveform engine did not come up", crlf);
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
        if (standby_until != 0 && now >= standby_until) {
            standby_until = 0;
            if (proven) tx_live();
        }
        if (standby_until == 0 && proven && now - last_frame_ms > link::rediscover_ms) {
            proven = false;
            (void)command_mode();
        }
        if (decoder.partial() && now - last_byte_ms > 50u) {
            decoder.reset();
        }
        const uint32_t st = Link::status();
        if ((st & USART_SR_RXNE) == 0u) {
            if ((st & UsartFlag::ore) != 0u) {
                Link::clear_by_read();
            }
            continue;
        }
        const uint16_t d = Link::read_word();
        last_byte_ms = Ticker::millis();
        if ((st & UsartFlag::receive_errors) != 0u) {
            decoder.reset();
            continue;
        }
        switch (decoder.feed(static_cast<uint8_t>(d))) {
            case link::Decoder::Result::frame: {
                const bool was = proven;
                proven = true;
                if (!was) {
                    tx_live();
                    rx_pad();
                }
                handle(decoder.frame());
                last_byte_ms = Ticker::millis();
                last_frame_ms = last_byte_ms;
                break;
            }
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
