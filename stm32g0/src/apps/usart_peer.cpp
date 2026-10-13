// usart_peer - the INSTRUMENT half of a two-board USART test on the
// STM32G0: the scriptable peer a far board's serial suite (the DUT) drives
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
// LINK: USART1 on the desk's crossed pair (DS13560 table 13) -
//   PA9  USART1_TX (AF1)  to the far board's RX;  TIM1_CH2 (AF2)
//   PA10 USART1_RX (AF1)  from the far board's TX; TIM1_CH3 (AF2)
// NO TOPOLOGY IS GUESSED: the desk fixes the crossed pair, so the peer is
// full duplex always and goes onto the far board's TX wire (PA10) only
// inside onewire and collide (usart_link.hpp's Topology).
//
// WHAT THIS SILICON MAKES OF THE REPERTOIRE (RM0444 ch. 33; the chapter's
// offer, item by item, is docs/stm32g0/usart.md's):
//  - THE FIFOS ARE USED: the link runs with FIFOEN (33.5.4), eight deep
//    each way. The echo and the sink drain the receive FIFO polled, the
//    ISR word read BEFORE each RDR so each entry's PE/FE/NE are its own
//    (the per-entry flags of 33.5.4), and cleared BEFORE the RDR read that
//    pops their entry - after it, the clear takes the next entry's flag
//    too (measured: test_stm32_serial letter 4, the driver's task fixed
//    with it); a blast keeps the transmit FIFO topped up, gapless
//    at the link's 7.5 Mbaud with no DMA. TRANSMIT DMA DECLINED: the FIFO
//    is eight frames of slack under a loop that costs a fraction of a
//    frame (the hot path's count below). Not in IrDA, where 33.8.1
//    forbids FIFOEN.
//  - EVERY RATE FROM THE TRACKED KERNEL. The core runs the HSI16 through
//    the PLL (no HSE on this board), some 0.3 % off nominal and WANDERING
//    - a few hundred ppm over seconds, measured - so it is measured
//    against the LSE crystal ALL THE TIME, in hardware - the stratum's
//    LseTracker: TIM16's TI1 is the LSE (TISEL), captured every eight
//    periods, DMA1 channel 2 moving each capture into a ring of 4096
//    (a second). Before every action the core's
//    rate over the last 250 ms becomes every divisor, tick and edge the
//    action computes, and a span is converted at the rate over its own
//    window (Ident::xtal = 1). Without the crystal it falls back to the
//    nominal 64 MHz and says xtal = 0. PCLK is the kernel (5.4.21's mux
//    left on it); HSI16 as the kernel DECLINED - a lower ceiling (1 Mbaud
//    at OVER16) and the same oscillator.
//  - THE OVERSAMPLING CLOSER TO THE RATE (33.5.7): OVER16 and OVER8 both
//    computed from the tracked kernel, the smaller error kept (OVER16 on
//    a tie, for its three-sample noise vote), OVER8 alone under
//    flag_clk2x. OVER8 BUYS RANGE, NOT RESOLUTION: BRR[2:0] = USARTDIV[3:0]
//    >> 1 drops USARTDIV's last bit (33.5.7's own examples), so both
//    oversamplings divide the kernel by a whole number n, OVER8 reaching
//    n = 8..15; 8 Mbaud is the ceiling and a rate past it is REFUSED.
//  - FRAMES: CR1.M counts the parity bit (33.5.5), so the protocol's 6+P,
//    7, 7+P, 8, 8+P and 9 are served (seven-, eight- and nine-bit words);
//    5 and 6 data bits without parity and 9 with parity have no word
//    length and are REFUSED. Every code of CR2.STOP is served - one, two,
//    0.5 and 1.5 (33.8.3; 33.5.5 RECOMMENDS the halves for smartcard mode
//    and forbids them nowhere else) - except in IrDA, which wants one.
//  - THE SYNCHRONOUS ROLES ARE REFUSED ON THIS DESK. USART1's CK is PB3
//    or PA12, both carrying another wire (SPI SCK, the I2C bus), and the
//    far board's synchronous clock reaches PB3 only through a strap the
//    desk has not got (the pair's design, D2). flag_mpcm is refused too:
//    the multiprocessor wake is a receiver's, the DUT's to test.
//  - IrDA SIR (33.5.18): IREN, the rate capped at 115200 as the chapter
//    caps it, GTPR.PSC from rxpl (the glitch filter: pulses under two PSC
//    periods rejected); txpl != 0 asks for the low-power encoder, IRLP
//    with PSC = txpl / 3 kernel periods (a pulse of three PSC periods).
//    TXINV and RXINV are set, because two ST SIR ENDECs face each other
//    with opposite polarities ("the transmit encoder output has the
//    opposite polarity to the decoder input", 33.5.18) and both act on
//    the SIR side of the ENDEC - measured: TXINV turns the encoder's
//    high pulses on a low idle into low pulses on a high idle, RXINV lets
//    the decoder take the far encoder's frames (test_stm32_serial letter
//    r). BOTH, never one: an ENDEC is half duplex, and an encoder idling
//    low into the far decoder is a standing pulse its receiver chews,
//    which keeps the far transmitter from encoding.
//  - THE WAVEFORM ENGINE for brk, autobaud_tx's break, bitbang, glitch,
//    drive_rxd and collide: TIM1, whose CH2 is on PA9 and CH3 on PA10 (AF2,
//    DS13560 table 13) - the link's own pads. The pad is re-muxed from
//    the USART to the timer with the channel FORCED HIGH first (both idle
//    high: no edge), the channel turned to TOGGLE mode with CCR preload
//    off, its first compare written by the CPU and every later one by DMA1
//    channel 1 on the channel's own compare request (DMAMUX, table 55's
//    TIM1_CC2/CC3) from a table of ABSOLUTE compare values - no CPU
//    between edges, no interrupt masked, no accumulation. The prescaler
//    is chosen per action so the longest interval fits half the 16-bit
//    counter; the shortest interval the DMA can follow is the floor below.
//    The cycle-counted loop is DECLINED: the image runs from flash behind
//    two wait states and the prefetch, and a loop's timing would be a
//    function of its alignment.
//  - SWAP + HDSEL for onewire and collide (33.8.2, 33.5.15): the single
//    wire on PA10, the far board's TX wire, open drain on the pad's
//    pull-up for the length of the action.
//  - THE REPORT'S SPAN on TIM2, free-running 32 bits at the core's
//    frequency, read once a character as the loop sees it complete.
//
// TIME (usart_link.hpp, TIME IN THE PAYLOAD): every tick this peer reports
// or obeys is a 24 MHz tick, converted from the core's counts through the
// TRACKED frequency: ticks = counts x 24 MHz / core_hz, at 64 MHz three
// eighths.
//
// THE DARK DISCIPLINE (usart_link.hpp): while it searches, PA9 is an
// input with no pull (33.5.2: a disabled transmitter returns its pad to
// the port's configuration - the peer states the pad an input) and PA10
// the USART's receive pad with NO pull: the far board's single-board
// letters walk both wires with their own pulls. A frame that checks out
// proves the wiring: PA9 goes to the USART (push-pull, idling high) and
// PA10 takes its pull-up; rediscover_ms of quiet takes both back. A bad
// checksum is nak'ed only while ENGAGED.
//
// Console: USART2 PA2/PA3 at 115200, observability only.
//   ? help | i status and counters | 0 back to command mode, wiring
//   unproven | 2 wiring probe | 3 trace the commands | 4 the engine's
//   self-test on PA9 (the far board dark or listening) | c the core's rate
//   now, into use | t the tracker's rate over three windows
//
// Placement (docs/stm32g0/platform.md, "Code in SRAM"): nothing in SRAM,
// the console InFlash. The link is USART1 polled by the peer's own loop,
// not a Uart's interrupt path, and the console is observability only.
//
// The ident label is the 96-bit unique ID's first word in hex (RM0444
// 41.1; this family has no user label).
//
// build: boards = g0b1re,g071rb
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "stm32g0/clock.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/lse_tracker.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/usart_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using link::Op;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Console = Uart<2, console_pins>;
constexpr Console console;

constexpr uint16_t firmware_version = 0x0301;   ///< see usart_link.hpp's Ident

// ---------------------------------------------------------------------------
// The link's pads and instances
// ---------------------------------------------------------------------------

using Link = Usart<1>;
using TxPad = Pin<'A', 9>;    // USART1_TX AF1, TIM1_CH2 AF2
using RxPad = Pin<'A', 10>;   // USART1_RX AF1, TIM1_CH3 AF2
constexpr PinFunction usart_af = PinFunction::af1;
constexpr PinFunction timer_af = PinFunction::af2;
constexpr uint8_t tx_channel = 1;   // TIM1_CH2, 0-based as every channel verb
constexpr uint8_t rx_channel = 2;   // TIM1_CH3

using Engine = Tim<1>;
using EdgeDma = DmaChannel<1, 1>;
using Span = Tim<2>;

// ---------------------------------------------------------------------------
// The time base: the core measured against the LSE
// ---------------------------------------------------------------------------

/// The core's frequency, TRACKED against the LSE (the nominal one when the
/// crystal is absent): what every divisor and tick conversion uses.
uint32_t core_hz = SysClock::hz;
/// Ident::xtal: 1 when the LSE runs and the tracker converged, 2 when a
/// DUT was adopted through calibrate, 0 otherwise.
uint8_t time_base = 0;
int32_t core_ppm = 0;   ///< the core against nominal, as last tracked

/// THE TRACKER. The HSI16 under the core WANDERS - measured on this
/// board: some 400 ppm between a second after boot and a minute later,
/// and 150 ppm between gapless runs a second apart (docs/stm32g0/usart.md,
/// "The peer's time base") - so a measurement once at boot is not a time
/// base, and the core is measured against the LSE ALL THE TIME, by
/// hardware: the stratum's LseTracker (stm32g0/lse_tracker.hpp, TIM16
/// capturing the crystal every eight periods, DMA1 channel 2 filling a
/// ring of 4096 - a second - with no CPU), read as the core's rate over
/// any stretch of the last second; before every action the last 250 ms.
/// A second of the crystal: a DUT's span is up to nine tenths of one, and
/// is converted at the rate over its own window.
using Tracker = LseTracker<1, 2, 4096>;
bool tracking = false;

/// The crystal started (the RTC domain's, behind its write gate), then
/// the tracker on it.
bool tracker_up() {
    Pwr::bus_clock(true);
    Pwr::rtc_domain_unlock(true);
    if (!RtcDomain::lse_ready()) {
        RtcDomain::lse_enable(true);
        if (!RtcDomain::lse_wait_ready(4'000'000UL)) {
            Pwr::rtc_domain_unlock(false);
            return false;
        }
    }
    Pwr::rtc_domain_unlock(false);
    return Tracker::start(clock);
}

/// The core's rate over the last `k` capture intervals, or nothing.
std::optional<uint32_t> tracked_hz(uint16_t k) {
    if (!tracking) {
        return std::nullopt;
    }
    return Tracker::hz(k);
}

/// The rate over the last 250 ms, into core_hz: before every action, so
/// the divisors and the engine's edges are the core's as it is now.
void track_now() {
    if (time_base != 1u) {
        return;
    }
    const std::optional<uint32_t> hz = tracked_hz(1024u);
    if (hz) {
        core_hz = *hz;
        core_ppm = static_cast<int32_t>((static_cast<int64_t>(core_hz) - static_cast<int64_t>(SysClock::hz)) *
                                        1'000'000 / static_cast<int64_t>(SysClock::hz));
    }
}

/// 24 MHz ticks of `counts` core counts, and back - the measured rate in
/// both, 64-bit arithmetic outside every loop that matters.
uint32_t ticks_of(uint32_t counts) {
    return static_cast<uint32_t>((static_cast<uint64_t>(counts) * 24'000'000ull +
                                  core_hz / 2u) / core_hz);
}
uint32_t counts_of(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * core_hz + 12'000'000ull) /
                                 24'000'000ull);
}

/// A span of `counts` core counts that ENDED NOW, in 24 MHz ticks, on the
/// core's rate over that very stretch (the tracker's intervals covering
/// it, sixteen at least): the HSI's wander outside the span is not in it.
uint32_t span_ticks(uint32_t counts) {
    if (time_base == 1u && tracking) {
        const std::optional<uint32_t> hz = Tracker::hz_over(counts);
        if (hz) {
            return static_cast<uint32_t>((static_cast<uint64_t>(counts) * 24'000'000ull + *hz / 2u) / *hz);
        }
    }
    return ticks_of(counts);
}

/// The span ruler and the waits: TIM2 free-running, 32 bits at the core.
void span_up() {
    Span::init();
    (void)Span::configure({.prescaler = 0, .period = 0xFFFFFFFFu});
    Span::enable(true);
}

[[gnu::always_inline]] inline uint32_t now_counts() { return TIM2->CNT; }

void wait_counts(uint32_t n) {
    const uint32_t t0 = now_counts();
    while (now_counts() - t0 < n) {
    }
}

void wait_us(uint32_t us) { wait_counts(static_cast<uint32_t>((static_cast<uint64_t>(us) * core_hz) / 1'000'000u)); }

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

// ---------------------------------------------------------------------------
// The link's configuration: a Cfg turned into registers, refused where the
// silicon has no answer
// ---------------------------------------------------------------------------

/// What one Cfg becomes, computed once before the action runs.
struct LinkSetup {
    UartFormat format{};
    /// CR2.STOP when it is a code UartFormat cannot name (0.5, 1.5).
    UartStop half_stop = UartStop::one;
    uint16_t brr = 0;
    bool over8 = false;
    bool fifo = true;
    bool irda = false;
    IrdaConfig irda_cfg{};
    uint16_t mask = 0xFF;    ///< the data bits a character carries
};

uint16_t data_mask(uint8_t bits) {
    const uint8_t n = bits >= link::bits9_low ? 9 : bits;
    return static_cast<uint16_t>((1u << n) - 1u);
}

/// The divisor closer to `rate` of the two oversamplings, from the
/// measured kernel; OVER8 alone when `force8`.
std::optional<LinkSetup> rate_setup(uint32_t rate, bool force8) {
    if (rate == 0u) {
        return std::nullopt;
    }
    LinkSetup s{};
    const std::optional<uint16_t> b16 = force8 ? std::nullopt : usart_brr(core_hz, rate);
    const std::optional<uint16_t> b8 = usart_brr_over8(core_hz, rate);
    auto err = [rate](uint32_t actual) {
        return actual > rate ? actual - rate : rate - actual;
    };
    if (b16 && (!b8 || err(usart_actual_baud(core_hz, *b16)) <=
                           err(usart_actual_baud_over8(core_hz, *b8)))) {
        s.brr = *b16;
        s.over8 = false;
        return s;
    }
    if (b8) {
        s.brr = *b8;
        s.over8 = true;
        return s;
    }
    return std::nullopt;
}

/// The Cfg's frame on this USART, or nothing where it has no word length.
/// A half-bit stop code is the setup's own field (below): UartFormat
/// speaks one and two.
std::optional<UartFormat> format_of(const link::Cfg& c) {
    UartFormat f{};
    const bool parity = c.parity != 0u;
    f.parity = c.parity == 1u ? UartParity::even : c.parity == 2u ? UartParity::odd : UartParity::none;
    if (c.parity > 2u) {
        return std::nullopt;
    }
    // CR1.M counts the parity bit (33.5.5).
    switch (c.bits) {
        case 6: if (!parity) return std::nullopt; f.bits = UartBits::seven; break;
        case 7: f.bits = parity ? UartBits::eight : UartBits::seven; break;
        case 8: f.bits = parity ? UartBits::nine : UartBits::eight; break;
        case link::bits9_low:
        case link::bits9_high: if (parity) return std::nullopt; f.bits = UartBits::nine; break;
        default: return std::nullopt;
    }
    if (c.stop == 2u) {
        f.stop_bits = 2;
    } else if (c.stop == 1u || c.stop == link::stop_half || c.stop == link::stop_one_half) {
        f.stop_bits = 1;
    } else {
        return std::nullopt;
    }
    return f;
}

/// The whole Cfg, or nothing: what this peer refuses.
std::optional<LinkSetup> setup_of(const link::Cfg& c) {
    if ((c.flags & link::flag_mpcm) != 0u) {
        return std::nullopt;
    }
    if (c.mode == link::Mode::sync_client || c.mode == link::Mode::sync_host) {
        return std::nullopt;   // no CK wire on this desk (the header)
    }
    const std::optional<UartFormat> f = format_of(c);
    if (!f) {
        return std::nullopt;
    }
    const bool ir = c.mode == link::Mode::ircom;
    if (ir && (c.rate > 115'200u || c.stop != 1u)) {
        return std::nullopt;   // 33.5.18: SIR to 115.2 kbaud, one stop bit
    }
    if (ir && (c.flags & link::flag_clk2x) != 0u) {
        return std::nullopt;   // IrDA takes the OVER8 = 0 formula alone (33.5.7)
    }
    std::optional<LinkSetup> s = ir ? std::optional<LinkSetup>{LinkSetup{}}
                                    : rate_setup(c.rate, (c.flags & link::flag_clk2x) != 0u);
    if (ir) {
        const std::optional<uint16_t> b = usart_brr(core_hz, c.rate);
        if (!b) {
            return std::nullopt;
        }
        s->brr = *b;
    }
    if (!s) {
        return std::nullopt;
    }
    s->format = *f;
    s->half_stop = c.stop == link::stop_half       ? UartStop::half
                   : c.stop == link::stop_one_half ? UartStop::one_and_half
                                                   : UartStop::one;
    s->mask = data_mask(c.bits);
    if (ir) {
        s->irda = true;
        s->fifo = false;   // 33.8.1: FIFOEN must not be set in IrDA mode
        IrdaConfig ic{};
        if (c.txpl != 0u) {
            // The low-power pulse is three PSC periods (33.5.18): txpl is
            // the AVR's pulse length in 24 MHz cycles.
            const uint32_t psc = (counts_of(c.txpl) + 1u) / 3u;
            ic.low_power = true;
            ic.prescaler = static_cast<uint8_t>(psc == 0u ? 1u : (psc > 255u ? 255u : psc));
        } else {
            // Normal mode: PSC is the glitch filter alone - pulses under
            // two PSC periods are rejected; rxpl asks for that width.
            const uint32_t psc = counts_of(c.rxpl) / 2u;
            ic.prescaler = static_cast<uint8_t>(psc == 0u ? 1u : (psc > 255u ? 255u : psc));
        }
        s->irda_cfg = ic;
    }
    return s;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

bool engine_ok = false;
/// A frame has checked out recently: the wiring is proven, PA9 drives and
/// PA10 is pulled up. Cleared after rediscover_ms of quiet.
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
uint16_t command_brr = 0;

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(link::settle_ms) + 2u) {
    }
}

// ---------------------------------------------------------------------------
// The pads, by the dark discipline
// ---------------------------------------------------------------------------

/// PA9 under the port: an input with no pull, no driver on the wire.
void tx_dark() { TxPad::input(PinPull::none); }
/// PA9 to the USART: push-pull at very high speed (a 7.5 Mbaud cell is
/// 133 ns), the transmitter enabled and idling high under it already.
void tx_live() { TxPad::function(usart_af, {.speed = PinSpeed::very_high}); }
/// PA10 to the USART's receiver, pulled up only once proven.
void rx_pad() { RxPad::function(usart_af, {.pull = proven ? PinPull::up : PinPull::none}); }

/// Clear the receive side: the FIFO flushed (RQR.RXFRQ) and the receive
/// flags down - TC LEFT ALONE: it stands from the enable's idle frame on
/// (ISR's reset value has it set), and wait_tx_done() before an engine run
/// reads it; cleared here, it would rise only after a frame and the wait
/// spin its whole bound.
void flush_rx() {
    Link::request(UsartRequest::flush_receive);
    Link::clear_flags(UsartClear::receive_errors | UsartClear::idle | UsartClear::rtof |
                      UsartClear::cmf);
}

/// Program USART1 from a setup, disabled first (33.8.x: the frame fields
/// take a write only with UE = 0). False when a verb refused.
bool link_apply(const LinkSetup& s, bool swap_hdsel = false) {
    Link::enable(false);
    if (!Link::configure(s.format, s.brr)) {
        return false;
    }
    if (s.over8 && !Link::oversampling(true)) {
        return false;
    }
    if (s.half_stop != UartStop::one && !Link::stop_bits(s.half_stop)) {
        return false;
    }
    if (s.fifo && !Link::fifo(true)) {
        return false;
    }
    if (swap_hdsel && (!Link::swap(true) || !Link::half_duplex(true))) {
        return false;
    }
    if (s.irda) {
        if (!Link::invert(true, true, false) || !Link::irda(s.irda_cfg)) {
            return false;
        }
    }
    Link::enable(true);
    flush_rx();
    return true;
}

LinkSetup command_setup() {
    LinkSetup s{};
    s.brr = command_brr;
    return s;
}

/// Command mode: async 8N1 at link::command_baud on the crossed pair, the
/// pads by the dark discipline.
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

/// A commanded configuration. False when this silicon cannot express it -
/// the caller reports it as a refusal and is back in command mode.
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

constexpr uint32_t spin_bound = 400'000u;

/// One frame out: wait for room in the transmit FIFO (bounded), then
/// write it.
[[gnu::always_inline]] inline bool put_frame(uint16_t v) {
    USART_TypeDef& u = Link::regs();
    for (uint32_t n = spin_bound; (u.ISR & UsartFlag::txfnf) == 0u; --n) {
        if (n == 0u) return false;
    }
    u.TDR = v;
    return true;
}

/// The wire falls idle: TC up (the last stop bit gone and nothing
/// queued), bounded.
void wait_tx_done() {
    for (uint32_t n = spin_bound; !Link::flag(UsartFlag::tc) && n != 0u; --n) {
    }
}

/// Four bit times at the command rate before an answer: the DUT listens
/// again only after its own stop bit (35 us).
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
// The waveform engine: TIM1 toggling on its compares, DMA1 channel 1
// writing each next ABSOLUTE compare
// ---------------------------------------------------------------------------

/// The edge list: times in core counts from the run's start, strictly
/// increasing, alternating FALL (even index) and RISE (odd) from an
/// idle-high line, ending on a rise.
constexpr uint16_t edge_most = 512;
uint32_t edges[edge_most];
uint16_t table[edge_most];   ///< the compares the DMA writes, mod 2^16
uint16_t edge_n = 0;
bool edge_overflow = false;

void edges_clear() {
    edge_n = 0;
    edge_overflow = false;
}

/// Append a level change at `t`; a level the line already has is ignored.
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
/// consecutive edges the DMA can follow - the compare matches, the request
/// crosses the DMAMUX, the channel reads the next table entry from SRAM
/// and writes CCR across the APB, and only then may the counter reach the
/// new value. Measured with the console's '4' and the floor lowered for
/// the run: an interval of 10 counts played, of 8 and below the compare
/// was written too late (the edge a counter wrap later); 12 keeps two
/// counts of margin (docs/stm32g0/usart.md).
constexpr uint32_t engine_floor_counts = 12;
/// The run's lead, in counts at the run's prescaler: from the counter's
/// reset (EGR.UG) to the first compare, the channel turned to toggle and
/// the DMA started in between.
constexpr uint32_t lead_floor_counts = 640;   // 10 us at 64 MHz

/// The DMA word for the table: memory to TIM1's CCR, half-words, the
/// highest level (nothing else runs during a waveform).
constexpr uint32_t edge_ccr = dma_ccr_config({.direction = DmaDirection::memory_to_peripheral,
                                              .memory_increment = true,
                                              .peripheral_width = DmaWidth::half,
                                              .memory_width = DmaWidth::half,
                                              .priority = DmaPriority::very_high});

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
    Dma<1>::bus_clock(true);
    EdgeDma::stop();
    return true;
}

uint32_t engine_runs = 0, engine_broken = 0;
uint32_t edges_seen_total = 0;


/// Play the list on channel `ch` (its pad already the timer's), `lead`
/// counts at prescaler 0 from now. Returns whether the run played whole:
/// every interval at or above the floor, the DMA done, the last edge
/// passed in time, the pad high at the end. While the DMA plays, the CPU
/// SAMPLES THE PAD (IDR, single-cycle on this core's IOPORT) and counts
/// its transitions - the engine checked on every run, not assumed: an
/// edge the DMA missed toggles the line a counter wrap later and the
/// count comes out wrong. A list with an interval under the sampling
/// loop's own period is judged on its timing and end level alone.
template <typename Pad>
bool play(uint8_t ch, uint32_t lead) {
    if (edge_overflow || edge_n < 2u || (edge_n & 1u) != 0u) {
        return false;
    }
    if (lead < lead_floor_counts) {
        lead = lead_floor_counts;
    }
    // The prescaler that fits the longest interval in HALF the counter
    // (the end test below is a signed 16-bit difference).
    uint32_t longest = edges[0] + lead, shortest = 0xFFFFFFFFu;
    for (uint16_t i = 1; i < edge_n; ++i) {
        const uint32_t d = edges[i] - edges[i - 1];
        if (edges[i] <= edges[i - 1]) {
            return false;
        }
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
    // The pad samples are worth judging where every interval outlasts the
    // sampling loop and a SysTick entry inside it (some fifty cycles)
    // several times over.
    const bool judge_count = shortest >= 256u;

    TIM_TypeDef& t = Engine::regs();
    const uint32_t mask = Pad::mask;
    volatile uint32_t& idr = GPIOA->IDR;
    // Everything the run's bound needs is computed BEFORE the counter
    // restarts: a 64-bit division inside the lead would eat it.
    const uint16_t last = table[edge_n - 1u];
    const uint32_t bound_ms = static_cast<uint32_t>(
        (static_cast<uint64_t>(lead + edges[edge_n - 1u]) * 1000u) / core_hz + 3u);
    Engine::set_prescaler(static_cast<uint16_t>(psc));
    t.EGR = TIM_EGR_UG;                         // CNT = 0, PSC in force
    const uint32_t started = TIM2->CNT;
    Engine::clear_flags(0xFFFFFFFFu);
    (void)Engine::output_channel(ch, {.mode = TimOutputMode::toggle, .compare = table[0],
                                      .preload = false});
    EdgeDma::set_peripheral(Engine::ccr_address(ch));
    (void)DmaMux::request(EdgeDma::mux_channel, Engine::dma_compare_request(ch));
    EdgeDma::restart(edge_ccr, &table[1], static_cast<uint16_t>(edge_n - 1u));
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
    const uint32_t elapsed = TIM2->CNT - started;
    if (elapsed > lead + edges[edge_n - 1u] + 32'768u * div) {
        ok = false;
    }
    // The line is high after its last rise: forced there, it stays.
    (void)Engine::output_channel(ch, {.mode = TimOutputMode::force_active, .preload = false});
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

/// A pad from its USART function to the timer's, the channel forced high
/// already (no edge: the USART idles high too), and back.
template <typename Pad>
void pad_to_timer(bool open_drain = false) {
    Pad::function(timer_af, {.pull = open_drain ? PinPull::up : PinPull::none,
                             .open_drain = open_drain, .speed = PinSpeed::very_high});
}

/// Bit times in counts, in sixteenths: the unit every precomputed edge is
/// built from (sixteenths keep a bit's fraction from drifting over a run).
uint32_t bit_q4(uint32_t rate) {
    if (rate == 0u) {
        rate = 9600u;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(core_hz) * 16u) / rate);
}
uint32_t q4_counts(uint32_t q4) { return (q4 + 8u) >> 4; }
bool bits_fit(uint32_t q, uint32_t n) { return n == 0u || q <= 0x7FFFFFFFu / n; }

/// One 8N1 frame's ten cells from `at` (sixteenths of a count), cell
/// `cell` stretched by `pct`; returns the position after the stop bit.
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

/// A character's errors from the ISR word read for it.
[[gnu::always_inline]] inline void count_errors(link::Report& r, uint32_t st) {
    if ((st & UsartFlag::fe) != 0u && r.ferr < 255u) ++r.ferr;
    if ((st & UsartFlag::pe) != 0u && r.perr < 255u) ++r.perr;
    if ((st & UsartFlag::ore) != 0u && r.ovf < 255u) ++r.ovf;
}

// ---------------------------------------------------------------------------
// The actions
// ---------------------------------------------------------------------------

/// THE HOT PATH: a character polled, stamped, accounted and - for an echo
/// - written back, with no call. The ISR word is read once a turn and
/// decides everything: RXFNE, the entry's errors, TXFNF for the echo. The
/// stamp comes first (the character is complete now); the count, the sum,
/// the stamps, the mask and the register bases live in registers, every
/// Params field read before the loop. COUNTED IN THE RELEASE LISTING
/// (docs/stm32g0/usart.md, "The peer's hot path").
[[gnu::noinline]] link::Report run_echo(const link::Params& a, uint16_t mask, bool sink) {
    link::Report r{};
    settle();
    flush_rx();
    USART_TypeDef& u = Link::regs();
    volatile uint32_t& cnt = TIM2->CNT;
    uint32_t first = 0, last = 0;
    uint16_t count = 0, sum = 0;
    const uint16_t want = a.count;
    const uint32_t ms = a.ms;
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        const uint32_t st = u.ISR;
        if ((st & UsartFlag::rxfne) != 0u) {
            const uint32_t stamp = cnt;
            if ((st & UsartFlag::receive_errors) != 0u) [[unlikely]] {
                u.ICR = st & UsartClear::receive_errors;   // before the pop: the next entry's are its own
                count_errors(r, st);
            }
            const uint16_t d = static_cast<uint16_t>(u.RDR);
            if (!sink) {
                if ((st & UsartFlag::txfnf) == 0u) [[unlikely]] {
                    (void)put_frame(d);
                } else {
                    u.TDR = d;
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
        r.span = span_ticks(last - first);
    }
    if (!sink && count != 0u) {
        wait_tx_done();
    }
    return r;
}

/// The DUT's gapless frames timed on THIS time base: a sink's span, and
/// the DUT's error against it in ppm. A crystal-referenced peer adopts
/// nothing; one on the nominal oscillator ADOPTS the DUT (the measured
/// core frequency moved by the DUT's error, xtal = 2), as usart_link.hpp
/// says.
link::Report run_calibrate(const link::Params& a, uint16_t mask) {
    link::Report r = run_echo(a, mask, true);
    const uint32_t nominal =
        link::nominal_span(a.cfg.rate, link::frame_half_bits(a.cfg.bits, a.cfg.parity, a.cfg.stop),
                           r.count);
    const int16_t ppm = link::calibrate_ppm(nominal, r.span);
    r.aux16 = static_cast<uint16_t>(ppm);
    if (time_base == 0u && r.count >= 2u && r.span != 0u && (r.flags & link::report_timed_out) == 0u) {
        // A span SHORTER than nominal on this clock (ppm > 0) is, with the
        // DUT taken as exact, this core counting fewer cycles than it
        // believes it runs: its true rate is the assumed one times
        // nominal / measured = 1 + ppm.
        core_hz = static_cast<uint32_t>((static_cast<int64_t>(core_hz) * (1'000'000 + ppm)) / 1'000'000);
        time_base = 2;
        command_brr = usart_brr(core_hz, link::command_baud).value_or(command_brr);
    }
    return r;
}

link::Report run_set_link(const link::Params& a, uint16_t mask) {
    link::Report r{};
    settle();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        const uint32_t st = Link::status();
        if ((st & UsartFlag::rxfne) != 0u) {
            if ((st & UsartFlag::receive_errors) != 0u) {
                Link::clear_flags(st & UsartClear::receive_errors);   // before the pop
                count_errors(r, st);
            }
            const uint16_t d = Link::read_word();
            r.sum = static_cast<uint16_t>(r.sum + (d & mask));
            ++r.count;
        }
    }
    r.flags |= link::report_timed_out;
    return r;
}

/// The rate this transmitter really runs at against `asked`, in ppm - the
/// Report's aux16 for send and blast (usart_link.hpp): the kernel is the
/// tracked core (or the adopted one), the divisor what BRR says - OVER16
/// divides by USARTDIV = BRR, OVER8 by USARTDIV / 2 with USARTDIV[3:0] =
/// BRR[2:0] << 1 (33.5.7: the last bit dropped). Nothing on the nominal
/// oscillator: the Report then leaves report_rate_stated clear.
/// `took` is the run's length in core counts: the kernel's rate is taken
/// over that very window from the tracker (the HSI moves between the
/// action's start and its end), the adopted or nominal one otherwise.
std::optional<uint16_t> rate_report(uint32_t asked, uint32_t took) {
    if (time_base == 0u) {
        return std::nullopt;
    }
    uint32_t hz = core_hz;
    if (time_base == 1u && tracking) {
        hz = Tracker::hz_over(took).value_or(core_hz);
    }
    const uint32_t brr = Link::brr();
    const bool over8 = Link::oversampling();
    const uint32_t div = over8 ? ((brr & 0xFFF0u) | ((brr & 0x7u) << 1)) : brr;
    return static_cast<uint16_t>(link::rate_error_ppm(static_cast<uint64_t>(hz) * (over8 ? 2u : 1u), div, asked));
}

/// `send` leaves two bit times of idle after each frame ON THE WIRE (it
/// waits for TC, the FIFO would otherwise close the gap); `blast` none, the
/// transmit FIFO kept topped up. A blast's repeats run the pattern on.
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
    const uint32_t c0 = now_counts();
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
            while ((u.ISR & UsartFlag::txfnf) == 0u) {
                if (--spins == 0u) {
                    sent = false;
                    break;
                }
            }
            if (!sent) {
                break;
            }
            u.TDR = v;
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
    if (const std::optional<uint16_t> stated = rate_report(a.cfg.rate, now_counts() - c0)) {
        r.aux16 = *stated;
        r.flags |= link::report_rate_stated;
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

/// PA9 held low for `count` bit times by the engine: two bits of idle,
/// the low, back to the USART.
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

/// The foreign sender: a break of aux16 bit times (16 by default), two
/// bits of idle, then the sync character and aux8 payload frames from the
/// USART at the commanded rate - every one timed by the measured kernel.
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
    r.aux16 = Link::brr();
    return r;
}

/// `count` frames of `value` with a clean four-bit idle between them, and
/// aux16 bit times of break ahead (0 = none) - the whole run precomputed,
/// then played by the engine on `Pad`. The Report's aux16 is a bit's
/// nominal length in 24 MHz ticks.
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

/// `count` low pulses of aux16 ticks, aux8 microseconds apart, on PA9.
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
        // Narrower than the DMA can follow, or more edges than the table:
        // a capability this engine has not got.
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
/// - every cell of every frame from usart_link's own cell table, the
/// PULSE inside each frame (the line driven to the level opposite its
/// cell's for aux8 per cent of a bit, from pct per cent into cell `cell`),
/// the idle between frames in sixteenths of a bit - at the TRACKED core's
/// counts, so the rate is the crystal's and not a divisor's. Each frame
/// is a run of boundaries in time order (its cell starts, the pulse's two
/// ends, the next slot's start), the level at each one read off the
/// frame: the edge list holds only real changes (edge_level drops the
/// rest). Refused where the engine cannot draw it: a pulse narrower than
/// the floor, or more edges than the table holds.
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

/// THE DESK'S SHARED WIRE is the far board's TX (PA10 here). SWAP puts
/// the USART's TX on PA10 and HDSEL joins the receiver to it (33.5.15):
/// open drain on the pad's pull-up, the transmitter released between
/// frames - so the peer only ever pulls the wire low, and only to reply.
/// PA9 is dark meanwhile. The reply's own echo is flushed.
std::optional<LinkSetup> wire_setup(uint32_t rate) {
    std::optional<LinkSetup> s = rate_setup(rate ? rate : link::command_baud, false);
    if (s) s->fifo = true;
    return s;
}

bool wire_join(const LinkSetup& s) {
    tx_dark();
    if (!link_apply(s, true)) {
        return false;
    }
    RxPad::function(usart_af, {.pull = PinPull::up, .open_drain = true, .speed = PinSpeed::high});
    flush_rx();
    return true;
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
        if ((st & UsartFlag::rxfne) != 0u) {
            if ((st & UsartFlag::receive_errors) != 0u) {
                Link::clear_flags(st & UsartClear::receive_errors);   // before the pop
                count_errors(r, st);
            }
            const uint16_t d = Link::read_word();
            r.sum = static_cast<uint16_t>(r.sum + (d & 0xFFu));
            ++r.count;
            // The guard at the wire's own rate: the DUT turns its line
            // round only after its stop bit is out.
            wait_counts(guard);
            (void)put_frame(static_cast<uint16_t>((d ^ 0xFFu) & 0xFFu));
            wait_tx_done();
            // The reply's own echo may land after TC (the stm32f4 peer
            // measured it a fraction of a bit late): waited for, two bit
            // times at most, and only then flushed.
            for (uint32_t c0 = now_counts(); (Link::status() & UsartFlag::rxfne) == 0u && now_counts() - c0 < bit2;) {
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
/// the DUT's NEXT frame aux16 microseconds later - OPEN DRAIN: PA10 handed
/// to TIM1_CH3 as an open-drain output on its pull-up, so the engine's
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
    (void)frame_edges(0, q, a.value, -1, 0);   // ends on its stop bit: high
    settle();
    const uint32_t lead = static_cast<uint32_t>(a.aux16) * (core_hz / 1'000'000u);
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        const uint32_t st = Link::status();
        if ((st & UsartFlag::rxfne) != 0u) {
            if ((st & UsartFlag::receive_errors) != 0u) {
                Link::clear_flags(st & UsartClear::receive_errors);   // before the pop
                count_errors(r, st);
            }
            (void)Link::read_word();
            ++r.count;
            pad_to_timer<RxPad>(true);
            if (!play<RxPad>(rx_channel, lead)) {
                r.flags |= link::report_mismatch;
            }
            RxPad::function(usart_af, {.pull = PinPull::up, .open_drain = true,
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

    // STAND-OFF: mute to every command but the one that ends it; the
    // transmitter released as well.
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
        d.xtal = time_base;
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
                            op == Op::onewire || op == Op::collide || op == Op::stage;
    const bool engined = op == Op::bitbang || op == Op::glitch || op == Op::drive_rxd ||
                         op == Op::brk || op == Op::autobaud_tx || op == Op::collide || op == Op::stage;
    uint16_t mask = data_mask(a.cfg.bits);
    track_now();
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
            // The DUT's TX pad is reached through this board's RX pad on
            // the crossed pair: the bit-banger drives THE OTHER BOARD's TX
            // wire.
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
                   "| 4 engine self-test | c the core now | t the tracker", crlf);
}

void print_ppm(int32_t ppm) {
    print(console, ppm < 0 ? "-" : "+", static_cast<uint32_t>(ppm < 0 ? -ppm : ppm), " ppm");
}

void status() {
    print(console, "  link USART1 PA9/PA10, the crossed pair: ", proven ? "PROVEN" : "searching (dark)",
          ", BRR=", Link::brr(), " PA9 ", proven ? "driven" : "released", crlf);
    print(console, "  core ", core_hz, " Hz (");
    print_ppm(core_ppm);
    print(console, " off 64 MHz), time base ",
          time_base == 1u ? "the LSE crystal" : time_base == 2u ? "a DUT adopted" : "NOMINAL",
          "; engine ", engine_ok ? "up" : "DOWN", ", runs ", engine_runs, " broken ",
          engine_broken, crlf);
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

/// Console '4': the engine on PA9 alone - glitches of 12, 16, 24 and 64
/// counts (the floor and above), four of 300 ticks and a 9600-baud
/// 0x55 frame, each run's verdict printed. PA9 meets the far board's RX,
/// an input: run it with that board dark or listening.
void engine_self_test() {
    wait_tx_done();
    pad_to_timer<TxPad>();
    for (const uint32_t w : {12u, 16u, 24u, 64u}) {
        edges_clear();
        edge_level(0, false);
        edge_level(w, true);
        edge_level(w + 2000u, false);
        edge_level(w + 2000u + w, true);
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

void measure() {
    if (!tracking) {
        tracking = tracker_up();
        wait_ms(300);   // the ring filled once
    }
    time_base = tracking && tracked_hz(Tracker::most).has_value() ? 1u : 0u;
    if (time_base == 1u) {
        track_now();
    } else {
        core_hz = SysClock::hz;
        core_ppm = 0;
    }
    command_brr = usart_brr(core_hz, link::command_baud).value_or(556);
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void BRIO_STM32G0_USART2_HANDLER() { (void)Console::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    (void)Console::init(clock, 115200);
    (void)Ticker::init(clock);
    brio::enable_interrupts();

    const uint32_t w0 = DeviceUid::read().word[0];
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        label8[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }

    // The pads dark FIRST: out of reset they are analog (no pull, no
    // driver), and they stay undriven through the second of measurement.
    tx_dark();
    span_up();
    measure();
    engine_ok = engine_up();

    Link::bus_clock(true);
    Link::reset();
    (void)Link::kernel_clock(UsartClock::pclk);

    print(console, crlf, "usart_peer - USART instrument peer (STM32G0 board ", label8,
          ", fw ", hex(firmware_version), ", clock ", clock_ok ? "PLL" : "FAILED", ")", crlf);
    print(console, "the core measured against the LSE: ", core_hz, " Hz (");
    print_ppm(core_ppm);
    print(console, "), ", time_base == 1u ? "every rate from it" : "NO CRYSTAL: nominal rates", crlf);
    print(console, "link USART1 PA9 TX / PA10 RX, the crossed pair; command mode 8N1 @ ",
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
                else if (c == 'c') {
                    measure();
                    (void)command_mode();
                    print(console, "  core ", core_hz, " Hz (");
                    print_ppm(core_ppm);
                    print(console, ")", crlf);
                } else if (c == 't') {
                    print(console, "  the core against the LSE over 250 ms: ", tracked_hz(1024u).value_or(0),
                          " Hz, 16 ms: ", tracked_hz(64).value_or(0), ", 4 ms: ", tracked_hz(16).value_or(0),
                          "; in use ", core_hz, crlf);
                } else if (c == '3') {
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
        // A wiring is believed only while the command channel keeps
        // proving it: rediscover_ms of quiet and the pads go dark.
        if (standby_until == 0 && proven && now - last_frame_ms > link::rediscover_ms) {
            proven = false;
            (void)command_mode();
        }
        // A frame that stops half way must not eat the next one.
        if (decoder.partial() && now - last_byte_ms > 50u) {
            decoder.reset();
        }
        const uint32_t st = Link::status();
        if ((st & UsartFlag::rxfne) == 0u) {
            if ((st & UsartFlag::ore) != 0u) {
                Link::clear_flags(UsartClear::ore);
            }
            continue;
        }
        if ((st & UsartFlag::receive_errors) != 0u) {
            Link::clear_flags(st & UsartClear::receive_errors);   // before the pop
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
