// spi_peer - a scriptable SPI CLIENT for the other end of a two-board
// bus test: the far board is the bus host and the DUT (test_stm32_spi)
// and drives this one IN BAND over the very bus both are testing, every
// command travelling as a checksummed frame in the same traffic.
//
// The wire format is spi_link.hpp, included by relative path: one file
// is the single source of truth for every board that speaks it,
// whatever its architecture. What this silicon makes of it:
//
//  - EVERY WINDOW IS ONE CALL OF THE DRIVER'S RUN (SpiClient::answer,
//    exchange - stm32g0/spi.hpp), and here it is the FIFO that shapes
//    it: 35.5.8 wants "the data register of the slave ... [to] already
//    contain data to be sent before starting communication with the
//    master", so the run fills the TXFIFO with the select still high, and
//    from then on writes on TXE, two 8-bit frames to a half-word access
//    (35.5.9's data packing) - the answers three frames ahead of the
//    shifter, where the RXNE reload this peer ran kept one.
//
//  - THE EXCHANGE IS SERVED BY THE DMA ENGINES BY DEFAULT (below), and
//    `spilink::spare_polled_pump` asks for the driver's polled run
//    instead, so a suite measures both boundaries; `spare_answer_only`
//    asks for the answer run alone (the report then counts the answers
//    written and judges nothing). Only the COMMAND listener is a
//    per-frame loop: a command character is tens of microseconds at the
//    host's command rate and the receive FIFO is four frames deep.
//
//  - THE EXTENDED EXCHANGE IS SERVED WHOLE (spi_link.hpp), and every
//    exchange's report raises report_extended:
//    `spare_wide` - 16-bit frames: CR2.DS at sixteen bits, the engines
//    in half-word beats (one element type, uint16_t, serves both widths:
//    the span's element is the beat), the runs over uint16_t;
//    `spare_crc` - the CRC unit armed with the command's polynomial and
//    CRCL at the frame's width (35.5.14). On the ENGINES the chapter's
//    own "CRC transfer managed by DMA" does the whole of it: the transmit
//    channel counts the data frames alone and the block appends its
//    TXCRC after the last of them, CRCNEXT never written; the receive
//    channel counts the data frames AND the host's CRC frame ("the
//    counter of the reception DMA channel can be set to the number of
//    data frames to receive including the CRC"), and the block compares
//    that frame with its RXCRC by itself. On the POLLED serve
//    (`spare_polled_pump`) a CRC run of this app's, over the resource:
//    the TXFIFO filled with the select high, CRCNEXT written "as soon as
//    the last data is written in the SPIx_DR register" (35.9.1), one
//    frame received a turn under the frame's own threshold, the host's
//    CRC frame the last one taken. Either way CRCERR is read once the
//    select has risen;
//    `spare_receive_only` - MISO NOT DRIVEN for the window (the pad left
//    in analog): the receive channel alone, or the driver's receive()
//    run on the polled serve.
//
//  - A RUN SAMPLES NOTHING INSIDE THE BURST, so the exchange's report
//    fields are this peer's own (spi_link.hpp's Report leaves them to the
//    peer's header): aux0 = SR after the burst (CRCERR bit 4, OVR bit 6),
//    aux1 = the milliseconds from the run's start to its END (255 =
//    nothing came), aux2 bit 0 = the select read low once after the run,
//    bit 1 = the host closed it, bit 2 = the DMA engines served it; aux3
//    = RXCRCR's low byte under spare_crc, else 0.
//
//  - THE BUFFERING REGIMES COLLAPSE. There is no BUFEN/BUFWR pair in
//    ch. 35: this client is always FIFO-buffered and the two preloads
//    play BUFWR's role. spilink::Cfg::regime is therefore MAPPED, not
//    translated: regime_buffer_wait (the only regime this link ever
//    commands) starts the run with the select high and delivers the
//    exact rx[i] = P_B(i) alignment that regime promises; the other two
//    start it only once the select has fallen, which on this silicon
//    means the shifter's leftover leads - stated here, exercised nowhere.
//
//  - THE PROTOCOL CARRIES OPTIONS THIS SILICON HAS NOT GOT. flag_wrcol
//    and flag_feed_tx name conditions chapter 35 does not have and are
//    ignored; Op::mspi asks for a USART in host-SPI mode and is answered
//    report_cfg_failed - this stratum's USART has no such mode.
//    Everything else of the repertoire is served: ping/ident/report,
//    exchange (extended too), sink_slow, ss_pulse, host_burst (8-bit
//    frames: the extended bits are an exchange's).
//
//  - THE ANSWER LINE'S EDGE RATE IS THIS APP'S, NOT THE DRIVER'S, AND IT
//    DEPENDS ON THE MODE. SpiClient's default class for MISO is
//    VERY-HIGH (its Config::miso_speed), and on this jumper-wire bus that
//    edge is an
//    AGGRESSOR on the clock beside it. Measured against the STM32F446RE
//    host it is paired with (test_stm32f4_spi letter x, ten bursts a mode
//    at the command rate, the host's SCK at medium): at very-high 11 of 40
//    bursts slip and at high 8 and 11 of 40 - every one in mode 1 or mode
//    2, THE TWO MODES THAT SAMPLE ON THE FALLING EDGE while the data lines
//    change on the rising one, so this edge lands on an SCK just settled
//    HIGH and a coupled dip through VIL is one sampling edge too many,
//    BOTH ENDS disagreeing from the same frame on. At medium and at low 0
//    of 40. But the ladder's top rung asks the OTHER way, for the fast
//    edge: at 22.5 MHz in mode 0 (twenty windows of 32 frames from the
//    F446's polled host and from its streams, a diagnostic run once per
//    class; that suite's letter u judges the rung) the answer line at
//    very-high is exact in 8-bit and 16-bit frames alike, at high 16 and
//    20 of 20 16-bit windows read wrong (bits 2 and 15 of the frame, the
//    data valid time against the far host's setup) while 8-bit ones hold,
//    at medium every window is wrong in both widths. So the pad goes out at
//    VERY-HIGH in modes 0 and 3 - where the edge lands on a LOW clock and
//    the slip never shows - and at MEDIUM in modes 1 and 2 (console 's'
//    cycles the first class, 'f' the second). A HOST's data and clock pads
//    are the same kind of aggressor (docs/stm32g0/spi.md; the stm32f4
//    suite hosts with its SCK and its MOSI at medium).
//
// THE DARK LISTENER discipline: the command-mode client never drives
// MISO (drive_output = false, which leaves the pad in analog), the
// answer line wakes only for one answer window after a frame that
// CHECKED OUT, an unknown op is dropped in silence, and a bad checksum
// is nak'ed only while ENGAGED. The select wire is held up by THIS
// board's internal pull-up at all times - a pad under an INPUT
// alternate function still follows its pull (measured) - so the pull is
// re-stated after every re-init.
//
// Link: SPI1 at AF0, the SAME PIN NAMES the DUT hosts on (SPI's own
// MOSI/MISO naming carries the direction, so nothing is crossed) -
//   PB3  SCK   (input here)
//   PB4  MISO  (this end's output, driven only to answer)
//   PB5  MOSI  (input here)
//   PA15 NSS   (the hardware select input; this board's pull-up holds it
//               high, and for Op::host_burst it becomes a GPIO chip
//               select this end drives)
// The same four pads carry this board as the bus HOST for
// Op::host_burst - SpiHost never claims the NSS pad (35.5.5: with SSM
// the pin is free), which is what makes that role swap a re-init and not
// a re-jumpering.
//
// Console: USART2 PA2/PA3 at 115200 on the ST-LINK's virtual COM port,
// observability only.
//   ? help | i status and counters | 0 back to the dark client | 3 trace |
//   s / f cycle the answer line's class in modes 0 and 3 / 1 and 2
//
// Placement (docs/stm32g0/platform.md, "Code in SRAM"): nothing in SRAM,
// the SpiHost of Op::host_burst left InFlash. The client's runs are
// polled and inline in their caller, paced by the far host's SCK and kept
// ahead by the FIFO, so the wire waits for nothing the fetch could
// shorten; the host's burst is the shape the DUT's own host is measured
// against, and the G031K8's 8 KB keeps the RAM for the runs' buffers.
//
// The ident label is the 96-bit unique device ID's first word in hex
// (RM0444 41.1 through brio::DeviceUid) - this family's identity is
// factory-programmed and there is no USERROW to label; ident.xtal is
// ALWAYS 0, because a Nucleo-64 fits no HSE crystal and this stratum has
// no HSE root at all.
//
// build: boards = g071rb,g0b1re,g031k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>
#include <type_traits>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/spi.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same
// file.
#include "../../../avrxt/src/apps/spi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using spilink::Op;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Console = Uart<2, console_pins>;
constexpr Console console;

// ---------------------------------------------------------------------------
// The link's pads, both roles (the DUT's own layout, pin name for pin name)
// ---------------------------------------------------------------------------

constexpr SpiPins link_pins{
    .sck = {'B', 3, PinFunction::af0},
    .miso = {'B', 4, PinFunction::af0},
    .mosi = {'B', 5, PinFunction::af0},
    .nss = {'A', 15, PinFunction::af0},
};

using Client = SpiClient<1, link_pins>;
using Host = SpiHost<1, link_pins>;
using Raw = Spi<1>;

// The two pads this app names outside the driver: the answer line (the
// console's "is MISO driven" reading) and the select, which is a GPIO
// this end drives in two of the actions.
using MisoPin = Pin<link_pins.miso.port, link_pins.miso.pin>;
using NssPin = Pin<link_pins.nss.port, link_pins.nss.pin>;

constexpr uint16_t firmware_version = 0x0302;   ///< see spi_link.hpp's Ident

/// THE ANSWER LINE'S EDGE RATE (see the header): VERY-HIGH in the two
/// modes that sample on the rising edge, MEDIUM in the two that sample on
/// the falling one - where this edge lands on an SCK just settled high.
/// Every init that drives the output names it in the client's Config
/// (miso_class()); the console's 's' and 'f' cycle the two for the bench.
PinSpeed miso_rising = PinSpeed::very_high;
PinSpeed miso_falling = PinSpeed::medium;
/// The class the last init asked for.
PinSpeed miso_applied = PinSpeed::very_high;

PinSpeed next_class(PinSpeed s) {
    switch (s) {
        case PinSpeed::very_high: return PinSpeed::high;
        case PinSpeed::high: return PinSpeed::medium;
        case PinSpeed::medium: return PinSpeed::low;
        default: return PinSpeed::very_high;
    }
}

const char* speed_name(PinSpeed s) {
    switch (s) {
        case PinSpeed::low: return "low";
        case PinSpeed::medium: return "medium";
        case PinSpeed::high: return "high";
        default: return "very_high";
    }
}

/// The answer line's class for a window in this SPI mode (0..3).
PinSpeed miso_class(uint8_t mode) {
    const bool falling_edge_sampler = mode == 1u || mode == 2u;
    miso_applied = falling_edge_sampler ? miso_falling : miso_rising;
    return miso_applied;
}

/// The exchange's DMA engines, on DMA1 channels 1 and 2 - the two
/// vectors this app binds. Armed ONCE, in main(): the binding is SPI1's
/// DR and its two DMAMUX requests, nothing an SPI re-init touches. Their
/// element is uint16_t, the widest frame they serve: a run of uint8_t
/// moves a byte a beat and a run of uint16_t a half-word (35.5.9's two
/// access widths), the span's element choosing.
using PeerTx = DmaTxEngine<1, 1, uint16_t>;
using PeerRx = DmaRxEngine<1, 2, uint16_t>;
bool dma_ok = false;

bool trace = false;

spilink::Decoder decoder;
spilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, serves = 0;
uint32_t last_byte_ms = 0;
uint32_t engaged_until = 0;

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }

// ---- waits ----------------------------------------------------------------------

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// A microsecond hold of arbitrary length: delay_us is capped below one
/// SysTick period BY DESIGN (a tick or more is TimeEvent territory), and
/// this instrument's holds are exactly the blocking waits that cap keeps
/// out of KERNEL programs - this is not one, so the hold is spent in
/// chunks the cap admits.
void hold_us(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

// ---- the runs --------------------------------------------------------------------
//
// Every window is ONE CALL of the driver's polled run, in the flash behind
// its prefetch (docs/stm32g0/spi.md counts the turn). The answers are
// precomputed, the received frames only STORED, and every judgement
// happens after the burst.

using Run = Client::Run;

/// A run's spin budget for `ms` milliseconds: one spin is four polls of SR
/// and a look at the select pad, some twenty-four core cycles at 64 MHz
/// (the listing), so this is a bound of the right order, not a clock.
constexpr uint32_t spins_for_ms(uint32_t ms) { return ms * (SysClock::hz / 24'000u); }

// ---- the client's standing configurations ---------------------------------------

void wait_nss_high(uint16_t ms);

/// The select wire is shared; while this end is (or is becoming) a
/// client, its own pull-up is what keeps the line from floating low and
/// selecting it at random. A pad under an INPUT alternate function still
/// follows its pull, and every re-init rewrites PUPDR, so this is
/// re-stated after each one.
void hold_nss_up() { NssPin::pull(PinPull::up); }

/// DARK: mode 0, MSb first, MISO NOT driven (the pad stays in analog).
/// Everything returns here, and SPE goes up so the shifter runs and the
/// command listener can poll RXNE.
bool go_dark() {
    // A CLIENT RECONFIGURES BETWEEN WINDOWS, NEVER INSIDE ONE. The host
    // releases the select about thirty microseconds after its last clock
    // edge, and a peer that re-inits the instant the last frame lands is
    // still inside that window - an RCC reset and a fresh SPE with NSS
    // low. Nothing here is measured to have been broken by it; the rule
    // is the cheap one to keep, and it costs those thirty microseconds
    // of a five-millisecond settle.
    wait_nss_high(50);
    // NO DISABLE PROCEDURE HERE, and it is a measurement and not an
    // omission: init() begins with an RCC reset of the instance, which
    // takes the FIFOs, SPE and BSY down at once, while 35.5.9's
    // procedure WAITS for a transmit FIFO the host has stopped clocking
    // - up to two bounded spins, ~31 ms on this core, spent between two
    // windows the protocol gives 5 ms - so a client that spends it
    // answers every frame longer than an ack too late.
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .bits = SpiDataSize::bits8,
                                         .lsb_first = false,
                                         .nss = SpiNss::hardware_input,
                                         .drive_output = false});
    hold_nss_up();
    Client::enable(0x00, 0x00);
    decoder.reset();
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

/// The answer window's configuration: MISO driven, everything else the
/// command channel's.
bool arm_answer() {
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .bits = SpiDataSize::bits8,
                                         .lsb_first = false,
                                         .nss = SpiNss::hardware_input,
                                         .drive_output = true,
                                         .miso_speed = miso_class(0)});
    hold_nss_up();
    return ok;
}

/// What a command asked this client to become. See the header: only
/// regime_buffer_wait has an exact counterpart on this silicon, and it
/// is the one the G0 suite commands.
///
/// Everything here frames on the NSS pad: the alternative was TRIED AND
/// MEASURED (see run_exchange).
///
/// The extended exchange's three: `wide` sets 16-bit frames; `crc` arms
/// the CRC unit on `poly` with CRCL at the frame's width (spi_link.hpp: a
/// CRC of the frame's width) - an even polynomial, zero included, is
/// refused by the driver (35.5.14's note), which the report says as
/// report_cfg_failed.
bool apply_cfg(const spilink::Cfg& c, bool drive_output = true, bool wide = false,
               bool crc = false, uint16_t poly = 0x0007u) {
    wait_nss_high(50);   // see go_dark(): a client reconfigures BETWEEN windows
    const bool ok = Client::init(clock, {.mode = mode_of(c.mode),
                                         .bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8,
                                         .lsb_first = c.dord != 0,
                                         .nss = SpiNss::hardware_input,
                                         .crc = crc,
                                         .crc_length = wide ? SpiCrcLength::crc16
                                                            : SpiCrcLength::crc8,
                                         .crc_polynomial = poly,
                                         .drive_output = drive_output,
                                         .miso_speed = miso_class(c.mode & 0x03u)});
    hold_nss_up();
    return ok;
}

// ---- answering ------------------------------------------------------------------

void wait_nss_high(uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Client::selected() && Ticker::millis() - t0 < ms) {
    }
}

/// Serve ONE answer window: the frame padded with zeros to the most the
/// host clocks (spilink::answer_bytes), handed to the answer run - the
/// TXFIFO filled with the select still high, every later pair on TXE. The
/// window ENDS WHEN THE HOST CLOSES IT, the run reading the select pad; a
/// host that clocks fewer than answer_bytes and raises the select ends it
/// as surely as one that clocks them all.
void serve(const uint8_t* buf, uint8_t n) {
    wait_nss_high(50);                    // the command window must close first
    if (!arm_answer()) {
        (void)go_dark();
        return;
    }
    uint8_t padded[spilink::answer_bytes] = {};
    for (uint8_t i = 0; i < n && i < spilink::answer_bytes; ++i) {
        padded[i] = buf[i];
    }
    Client::enable();
    (void)Client::answer(std::span<const uint8_t>(padded, spilink::answer_bytes),
                         spins_for_ms(spilink::serve_ms));
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

// ---- the actions ----------------------------------------------------------------

/// Frames an exchange carries at most; the buffers are half-words so that a
/// 16-bit run and its engines take them whole, and an 8-bit one reads the
/// same storage as bytes. One more frame in for the host's CRC.
constexpr uint16_t exchange_cap = 256;
alignas(4) uint16_t x_out[exchange_cap];
alignas(4) uint16_t x_in[exchange_cap + 1u];

uint8_t* out_bytes() { return reinterpret_cast<uint8_t*>(x_out); }
uint8_t* in_bytes() { return reinterpret_cast<uint8_t*>(x_in); }

/// The host burst's per-frame buffers, TWO bytes although the burst is
/// 8-bit: SpiHost's frame store is width-general and writes a half-word
/// for a frame above eight bits, which the compiler's bounds check sees
/// before it sees the width.
uint8_t burst_tx[2];
uint8_t burst_rx[2];

/// Judge `frames` received frames against the host's stream - their BYTES,
/// in memory order, spi_link.hpp's extended accounting (count the frames;
/// sum, mism, idx, got and exp over the bytes).
void judge(spilink::Report& r, const spilink::Params& a, uint16_t frames, bool wide) {
    const bool reversed = (a.flags & spilink::flag_expect_reversed) != 0;
    const uint8_t* const got = in_bytes();
    const uint16_t bytes = static_cast<uint16_t>(wide ? frames * 2u : frames);
    for (uint16_t i = 0; i < bytes; ++i) {
        const uint8_t want = spilink::wide_byte(a.pattern, a.seed_a, i, wide, reversed);
        if (got[i] != want) {
            if (r.mism == 0) {
                r.idx = static_cast<uint8_t>(i < 255u ? i : 255u);
                r.got = got[i];
                r.exp = want;
            }
            ++r.mism;
        }
        r.sum = static_cast<uint16_t>(r.sum + got[i]);
    }
    r.count = frames;
}

/// DR at a frame's own access width (35.5.9, figure 363): a byte for an
/// 8-bit frame, a half-word for a 16-bit one.
template <typename T>
[[gnu::always_inline]] inline void put_frame(T v) {
    if constexpr (std::is_same_v<T, uint8_t>) {
        Raw::data_byte(v);
    } else {
        Raw::data_halfword(v);
    }
}
template <typename T>
[[gnu::always_inline]] inline T get_frame() {
    if constexpr (std::is_same_v<T, uint8_t>) {
        return Raw::data_byte();
    } else {
        return Raw::data_halfword();
    }
}

/// THE CRC RUN, over the resource - the driver's runs carry no CRC phase
/// (docs/stm32g0/spi.md's gap list). 35.5.14 "CRC transfer managed by
/// CPU": the answers into the TXFIFO while TXE admits them (three byte
/// frames or two half-words, the select still high, 35.5.8), CRCNEXT "as
/// soon as the last data is written in the SPIx_DR register" (35.9.1),
/// and then the frames received one a turn - RXNE at the frame's own
/// threshold, which configure() set - the host's CRC frame the last. A
/// turn polls SR once and serves both flags, so a frame never waits for
/// the other direction. `in` holds n + 1 frames.
template <typename T>
Run crc_run(const T* out, T* in, uint16_t n, uint32_t spins) {
    const SPI_TypeDef& regs = Raw::regs();
    uint16_t w = 0;
    uint16_t r = 0;
    const uint16_t want = static_cast<uint16_t>(n + 1u);
    while (w < n && (regs.SR & SPI_SR_TXE) != 0u) {
        put_frame<T>(out[w++]);
    }
    if (w == n) {
        Raw::crc_next();
    }
    while (!Client::selected()) {
        if (--spins == 0u) {
            return {0, Client::RunEnd::stalled};
        }
    }
    Client::RunEnd how = Client::RunEnd::complete;
    uint8_t look = 0;
    while (r < want) {
        const uint32_t sr = regs.SR;
        if (w < n && (sr & SPI_SR_TXE) != 0u) {
            put_frame<T>(out[w++]);
            if (w == n) {
                Raw::crc_next();
            }
        }
        if ((sr & SPI_SR_RXNE) != 0u) {
            in[r++] = get_frame<T>();
            continue;
        }
        if (++look == 4u) {
            look = 0;
            if (!Client::selected()) {
                how = Client::RunEnd::closed;
                break;
            }
            if (--spins == 0u) {
                how = Client::RunEnd::stalled;
                break;
            }
        }
    }
    // A frame that landed beside the close is the run's.
    while (r < want && (regs.SR & SPI_SR_RXNE) != 0u) {
        in[r++] = get_frame<T>();
    }
    return {r, how};
}

/// THE DMA SERVE, in 35.5.x's order ("Communication using DMA"): RXDMAEN,
/// the channels, TXDMAEN, SPE last - so the transmit channel's first
/// beats fill the TXFIFO before the select falls (a request is a level
/// this controller serves on enable): the preload. A receive-only serve
/// leaves the transmit side out entirely. Under `crc` the chapter's own
/// "CRC transfer managed by DMA": the transmit channel counts the data
/// frames alone and the block appends its CRC after the last of them; the
/// receive channel counts the host's CRC frame too, so the window's last
/// frame lands in `x_in[n]`. The receive channel's count is the
/// transaction's progress.
template <typename T>
Run dma_serve(uint16_t n, bool transmit, bool crc, uint32_t ms) {
    const uint16_t in_n = static_cast<uint16_t>(crc ? n + 1u : n);
    (void)Client::disable();
    (void)Raw::dma_receive(true);
    (void)PeerRx::start(std::span<T>(reinterpret_cast<T*>(x_in), in_n));
    if (transmit) {
        (void)PeerTx::start(std::span<const T>(reinterpret_cast<const T*>(x_out), n));
        (void)Raw::dma_transmit(true);
    }
    Raw::enable();
    const uint32_t t0 = Ticker::millis();
    while (!PeerRx::full() && Ticker::millis() - t0 < ms) {
        (void)PeerRx::take();
    }
    (void)PeerRx::take();
    const bool all_in = PeerRx::full();
    const uint16_t got = PeerRx::taken();
    PeerTx::stop();
    PeerRx::stop();
    return {got, all_in ? Client::RunEnd::complete : Client::RunEnd::stalled};
}

/// The polled serves, one run each, at the frame's width.
template <typename T>
Run polled_serve(uint16_t n, bool answer_only, bool receive_only, bool crc, uint32_t spins) {
    const T* const out = reinterpret_cast<const T*>(x_out);
    T* const in = reinterpret_cast<T*>(x_in);
    Client::enable();
    if (crc) {
        return crc_run<T>(out, in, n, spins);
    }
    if (receive_only) {
        return Client::receive(std::span<T>(in, n), spins);
    }
    if (answer_only) {
        return Client::answer(std::span<const T>(out, n), spins);
    }
    return Client::exchange(std::span<const T>(out, n), std::span<T>(in, n), spins);
}

/// The workhorse: the host sends P_A(i), this client answers P_B(i).
///
/// TWO SERVES, so both boundaries stay measurable. THE DEFAULT IS THE
/// DMA ENGINES; `spilink::spare_polled_pump` asks for the driver's polled
/// run instead (the exchange, the answers alone with `spare_answer_only`,
/// the frames in alone with `spare_receive_only`, or this app's CRC run
/// with `spare_crc`). The extended exchange's bits as the header says.
spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    r.flags |= spilink::report_extended;
    const bool wide = (a.spare & spilink::spare_wide) != 0;
    const bool crc = (a.spare & spilink::spare_crc) != 0;
    const bool receive_only = (a.spare & spilink::spare_receive_only) != 0;
    const bool answer_only = !receive_only && (a.spare & spilink::spare_answer_only) != 0;
    const bool preload = a.cfg.regime == spilink::regime_buffer_wait;
    const uint16_t n = a.count < exchange_cap ? a.count : exchange_cap;
    // THE NSS PAD FRAMES THE BURST, and the alternative was MEASURED and
    // is worse: with SSM/SSI (the frame boundary on the clock alone)
    // this client counts the host's own CPOL settling edge - the one a
    // host makes when it primes CPOL = 1 with the select still high -
    // and every frame is then one bit early, five runs of five. A client
    // that frames on the pad ignores it, which is what the pad is for.
    // What the pad does NOT cure is the falling-edge slip in the header.
    // A receive-only client that sends nothing has no CRC frame to send:
    // refused beside spare_crc (spi_link.hpp).
    if (n == 0u || (crc && receive_only) ||
        !apply_cfg(a.cfg, !receive_only, wide, crc, a.aux16)) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    {
        spilink::Stream out(a.pattern, a.seed_b);
        uint8_t* const o = out_bytes();
        const uint16_t bytes = static_cast<uint16_t>(wide ? 2u * n : n);
        for (uint16_t i = 0; i < bytes; ++i) {
            o[i] = out.next();
        }
        uint8_t* const in = in_bytes();
        for (uint16_t i = 0; i < sizeof x_in; ++i) {
            in[i] = 0;
        }
    }
    const bool engines = dma_ok && (preload || receive_only) && !answer_only &&
                         (a.spare & spilink::spare_polled_pump) == 0u;

    const uint32_t t0 = Ticker::millis();
    Run run{};
    if (engines) {
        run = wide ? dma_serve<uint16_t>(n, !receive_only, crc, a.ms)
                   : dma_serve<uint8_t>(n, !receive_only, crc, a.ms);
        r.aux2 |= 0x04;                   // marks the DMA serve in the report
    } else {
        if (!preload && !receive_only) {
            // NO PRELOAD (a regime this suite never commands): the run
            // starts once the select has fallen, so the shifter's leftover
            // leads, which is stated in the header.
            while (!Client::selected() && Ticker::millis() - t0 < a.ms) {
            }
        }
        const uint32_t spins = spins_for_ms(a.ms);
        run = wide ? polled_serve<uint16_t>(n, answer_only, receive_only, crc, spins)
                   : polled_serve<uint8_t>(n, answer_only, receive_only, crc, spins);
    }
    const uint32_t dt = Ticker::millis() - t0;
    // The select's close, THEN the status: CRCERR is decided on the host's
    // CRC frame, the last of the window.
    wait_nss_high(20);
    const uint32_t sr = Client::status();
    r.aux0 = static_cast<uint8_t>(sr);
    r.aux1 = run.count == 0u ? 255u : (dt > 254u ? 254u : static_cast<uint8_t>(dt));
    if (Client::selected()) r.aux2 |= 0x01;
    if (run.end == Client::RunEnd::closed) r.aux2 |= 0x02;
    if ((sr & SpiFlag::overrun) != 0u) r.flags |= spilink::report_bufovf;
    if (crc) {
        r.aux3 = static_cast<uint8_t>(Raw::rx_crc());
        if ((sr & SpiFlag::crc_error) != 0u) r.flags |= spilink::report_crcerr;
    }

    if (answer_only) {
        r.count = run.count;   // answers written; nothing was received
    } else {
        // The CRC frame is not counted: `count` data frames, then the host's.
        judge(r, a, run.count > n ? n : run.count, wide);
    }
    if (r.count < a.count) r.flags |= spilink::report_timed_out;
    (void)go_dark();
    return r;
}

/// The loss semantics: DR is never read for the whole burst, so the
/// receive FIFO (four 8-bit frames on this part) fills and OVR stands.
/// The status is OR'ed DURING the burst - a flag that comes and goes is
/// invisible afterwards - and what the FIFO retained is drained at the
/// end. The burst's end is the DEADLINE the command carries.
spilink::Report run_sink_slow(const spilink::Params& a) {
    spilink::Report r{};
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    Client::enable(0x00, 0x00);          // something harmless to shift out
    uint8_t seen = 0;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
        seen = static_cast<uint8_t>(seen | static_cast<uint8_t>(Client::status()));
        if (Client::overrun()) r.flags |= spilink::report_bufovf;
    }
    r.flags |= spilink::report_timed_out;   // by construction: the deadline IS the exit
    r.sum = seen;
    r.got = static_cast<uint8_t>(Client::status());
    uint8_t vals[4] = {};
    uint8_t retained = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        const auto v = Client::poll();
        if (!v) break;
        if (retained < 4) vals[retained] = static_cast<uint8_t>(*v);
        ++retained;
    }
    r.exp = static_cast<uint8_t>(Client::status());
    r.count = retained;
    r.aux0 = vals[0];
    r.aux1 = vals[1];
    r.aux2 = vals[2];
    r.aux3 = vals[3];
    (void)go_dark();
    return r;
}

/// The second driver on the shared select wire. The SPI is handed back
/// first, so nothing of this board is on the other three wires.
spilink::Report run_ss_pulse(const spilink::Params& a) {
    spilink::Report r{};
    Client::release();
    if (a.aux8) wait_ms(a.aux8);
    NssPin::output(false);
    hold_us(a.aux16 ? a.aux16 : 1000u);
    NssPin::input(PinPull::up);
    r.aux0 = 1;
    r.count = 1;
    (void)go_dark();
    return r;
}

/// THE ROLES INVERT: this end becomes the bus HOST for a bounded burst
/// (the DUT's own CLIENT half needs a clock, and there is nobody else on
/// this wire). The choreography: the ack is already served, both boards
/// count the same lead-in milliseconds, and at their
/// end the DUT is a client with its first answers preloaded. PA15 is
/// driven LOW from GPIO for the WHOLE burst - one transaction, the thing
/// hardware NSS cannot frame (35.5.5 frames the peripheral's lifetime,
/// not a transaction) - and SpiHost never claims that pad, which is what
/// makes the swap a re-init.
///
/// aux16 is "the instrument's SCK division of its own clock", and this
/// instrument's clock is 64 MHz: the division is turned into the BR code
/// that produces AT MOST PCLK/aux16, so a commanded 32 is 2 MHz here.
spilink::Report run_host_burst(const spilink::Params& a) {
    spilink::Report r{};

    Client::release();
    NssPin::output(true);                 // deselected before anything moves

    const uint32_t lead = a.aux8 ? a.aux8 : 20u;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < lead) {
    }

    uint16_t div = a.aux16 ? a.aux16 : spilink::command_division;
    if (div < 2u) div = 2u;
    const uint32_t pclk = SysClock::pclk_hz;
    const auto code = spi_rate_for(pclk, pclk / div);
    if (!Host::init(clock) || !code) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    const SpiClock rate = *code;
    r.aux1 = static_cast<uint8_t>(spi_division(rate) >> 2);   // /4 to fit a byte
    (void)Host::bit_order(a.cfg.dord != 0);
    // The mode is primed BEFORE the select falls: a CPOL flip inside an
    // open window is one extra edge and the selected client counts it
    // (SpiHost::prime()'s own comment).
    Host::prime(mode_of(a.cfg.mode), rate);
    hold_us(200);

    spilink::Stream out(a.pattern, a.seed_a);      // what THIS end sends
    spilink::Stream want(a.pattern, a.seed_b);     // what the client owes

    NssPin::clear();                               // select, for the whole burst
    hold_us(spilink::gap_us);
    while (r.count < a.count && Ticker::millis() - t0 < lead + a.ms) {
        burst_tx[0] = out.next();
        burst_rx[0] = 0;
        Host::Request req{};
        req.cs = {};                               // the burst owns the select
        req.tx = lend<Lease::reply>(static_cast<const uint8_t*>(burst_tx));
        req.rx = lend<Lease::reply>(burst_rx);
        req.len = 1;
        req.clock = rate;
        req.mode = mode_of(a.cfg.mode);
        req.bits = SpiDataSize::bits8;
        req.polled = true;
        if (!Host::start(req)) {
            break;                                 // a polled request completes here
        }
        account(r, burst_rx[0], want.next());
        hold_us(spilink::gap_us);
    }
    NssPin::set();
    r.aux0 = static_cast<uint8_t>(Raw::status());
    if (r.count < a.count) r.flags |= spilink::report_timed_out;

    // Whatever happened, back to the dark client; the pads change hands
    // under the GPIO-held high select.
    Host::release();
    NssPin::input(PinPull::up);
    (void)go_dark();
    return r;
}

// ---- the command handler --------------------------------------------------------

char label8[9] = {};   ///< the unique ID's word 0 in hex: this board's name

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
        d.xtal = 0;                       // no HSE on a Nucleo-64, and no HSE root
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
        // Op::mspi asks for a USART in host-SPI mode and this
        // stratum's USART has no such mode; cfg_failed says "not on this
        // peer" rather than pretending.
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
    print(console, "spi_peer: ? help | i status | 0 back to the dark client | 3 trace | "
                   "s / f cycle the MISO pad speed in modes 0/3 / 1/2",
          crlf);
}

void status() {
    print(console, "  client SPI1 AF0, enabled=", Raw::enabled(),
          " role=", Raw::role() == SpiRole::client ? "client" : "HOST",
          " mode=", static_cast<uint8_t>(Raw::mode()), " SR=", hex(Raw::status()), crlf);
    print(console, "  NSS pin reads ", NssPin::read() ? "high" : "LOW",
          ", selected=", Client::selected(),
          ", MISO driven=", MisoPin::has_function(),
          ", MISO pad speed=", speed_name(miso_rising), " in modes 0/3, ",
          speed_name(miso_falling), " in modes 1/2 (last applied ", speed_name(miso_applied), ")",
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

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void BRIO_STM32G0_USART2_HANDLER() { (void)Console::isr(); }

/// SPI1's line carries nothing - every window is a polled run - but
/// SpiClient::init() opens it at the NVIC, so a stray level is DISARMED
/// rather than looped on.
extern "C" void SPI1_IRQHandler() {
    Raw::rxne_interrupt(false);
    Raw::txe_interrupt(false);
    Raw::error_interrupt(false);
}

/// The engines' completions: the exchange polls the receive channel, so
/// there is nothing to dispatch - but the interrupts the claim arms must
/// land somewhere that is not the crt's silent spin, and the flags must
/// be cleared or the line stays asserted.
extern "C" void DMA1_Channel1_IRQHandler() { (void)PeerTx::service(); }
extern "C" void DMA1_Channel2_3_IRQHandler() { (void)PeerRx::service(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // The board's name: the unique device ID's first word, in hex.
    const uint32_t w0 = brio::DeviceUid::read().word[0];
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        label8[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }

    // The exchange's engines, bound once: SPI1's DR and its two DMAMUX
    // requests, nothing an SPI re-init touches.
    brio::Dma<1>::bus_clock(true);
    PeerTx::arm(Raw::data_address(), Raw::dma_tx_request());
    PeerRx::arm(Raw::data_address(), Raw::dma_rx_request());
    dma_ok = true;

    // The select pad readable (and held up) before the first go_dark():
    // its wait for a released select reads the pad, and a pad still in
    // its analog reset state has no input buffer to read.
    NssPin::input(PinPull::up);

    print(console, crlf, "spi_peer - SPI instrument client (STM32G0 board ", label8,
          ", xtal=none (no HSE fitted; the HSE root is unbuilt), fw ",
          hex(firmware_version), ")", crlf);
    print(console, "client SPI1 AF0: SCK PB3, MISO PB4 (pad speed ", speed_name(miso_rising),
          " in modes 0/3, ", speed_name(miso_falling),
          " in 1/2), MOSI PB5, NSS PA15 (pulled up); command mode "
          "= SPI mode 0, MSb first", crlf);
    print(console, "DARK by default: MISO is driven only for one answer window, after "
                   "a frame that checked out", crlf);
    print(console, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
          " tick=", tick_ok ? "SysTick" : "FAILED",
          " console=", console_ok ? "USART2" : "FAILED", crlf);
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
                } else if (c == 's') {
                    miso_rising = next_class(miso_rising);
                    print(console, "  MISO pad speed in modes 0/3 now ", speed_name(miso_rising),
                          " (applied at the next answer window; very_high is the default)",
                          crlf);
                } else if (c == 'f') {
                    miso_falling = next_class(miso_falling);
                    print(console, "  MISO pad speed in modes 1/2 now ", speed_name(miso_falling),
                          " (applied at the next exchange; medium is the default)", crlf);
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
