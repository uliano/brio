// spi_peer - a scriptable SPI CLIENT for the other end of a two-board
// bus test: the far board is the bus host and the DUT, and drives this
// one IN BAND over the very bus both are testing, every command
// travelling as a checksummed frame in the same traffic.
//
// The wire format is spi_link.hpp, included by relative path: one file
// is the single source of truth for every board that speaks it,
// whatever its architecture. What this silicon makes of it:
//
//  - EVERY WINDOW IS ONE CALL OF THE DRIVER'S RUN (SpiClient::answer,
//    exchange - stm32f4/spi.hpp), written on TXE: this block has one
//    transmit buffer in front of one shifter, the buffer moves into the
//    shifter at a frame's first bit and TXE rises then (RM0390 26.3.9,
//    ES0298 2.14.6), so the run writes the answer for frame k + 1 while
//    frame k shifts - seven bits of window, where the answer written at
//    the RXNE of the frame before (the reload this peer ran) had half a
//    bit and broke against back-to-back frames from 1 MHz. A client that
//    falls behind does not stall the bus - the host reads whatever the
//    shifter held, a wrong answer and never a missing one.
//
//  - THE EXCHANGE IS SERVED BY THE DMA ENGINES BY DEFAULT, SPI2_TX on
//    DMA1 stream 4 and SPI2_RX on stream 3, both channel 0 (RM0390 table
//    28), started before SPE in 26.3.11's order so the transmit stream's
//    first beat IS the preload; `spilink::spare_polled_pump` asks for the
//    polled run instead, so a suite measures both boundaries, and
//    `spilink::spare_answer_only` for the answer run alone (the report
//    then counts the answers written and judges nothing). Only the
//    COMMAND listener is a per-frame loop: a command character is tens of
//    microseconds at the host's command rate and the protocol leaves 20 us
//    between characters.
//
//  - THE EXTENDED EXCHANGE IS SERVED WHOLE (spi_link.hpp), and every
//    exchange's report raises report_extended:
//    `spare_wide` - 16-bit frames: CR1.DFF, the streams in half-word
//    beats (one element type, uint16_t, serves both widths: the span's
//    element is the beat), the runs over uint16_t;
//    `spare_crc` - the CRC unit armed with the command's polynomial, the
//    CRC the frame's width (26.4.2). On the ENGINES the chapter's own
//    "CRC transfer managed by DMA": both streams count the data frames,
//    the block appends its TXCRC after the transmit stream's last and
//    compares the host's CRC frame with its RXCRC by itself, CRCNEXT
//    never written - and that frame "is always loaded into" DR, read from
//    there. On the POLLED serve (`spare_polled_pump`) a CRC run of this
//    app's over the resource, in the driver's run shape: CRCNEXT written
//    "as soon as the last data is written to the SPI_DR register" (26.7.1),
//    the host's CRC frame the last one read. Either way CRCERR is read
//    once the select has risen;
//    `spare_receive_only` - MISO NOT DRIVEN for the window (the pad parked
//    in analog): the receive stream alone, or the driver's receive() run
//    on the polled serve.
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
//    chapter 26: this client always answers from its buffer, and
//    spilink::Cfg::regime is MAPPED rather than translated -
//    regime_buffer_wait starts the run with the select high and delivers
//    the exact
//    rx[i] = P_B(i) alignment that regime promises, and the other two
//    start the run only once the select has fallen, so the shifter's
//    leftover leads and the client's stream follows one place late. That
//    is what a DUT measuring the dummy byte reads back.
//
//  - THE PROTOCOL CARRIES OPTIONS THIS SILICON HAS NOT GOT. flag_wrcol
//    and flag_feed_tx name conditions chapter 28 does not have and are
//    ignored; Op::mspi asks for a USART in host-SPI mode, which this
//    stratum's USART chapter has but no board of it wires, and is
//    answered report_cfg_failed. Everything else of the repertoire is
//    served: ping/ident/report, exchange (on the engines or the run,
//    answer-only too, extended too), sink_slow, ss_pulse, host_burst
//    (8-bit frames: the extended bits are an exchange's).
//
//  - BSY IS NOT LOOKED AT ON THIS SIDE. ES0298 2.14.6: a client's BSY
//    may stay high after a transfer, so RXNE is the witness on the
//    receiving side and the NSS pad is the witness for the window - the
//    run's own end.
//
// THE DARK LISTENER discipline: the command-mode client never drives
// MISO (drive_output = false, which parks the pad in analog), the answer
// line wakes only for one answer window after a frame that CHECKED OUT,
// an unknown op is dropped in silence, and a bad checksum is nak'ed only
// while ENGAGED. The select wire is held up by THIS board's internal
// pull-up at all times, and the pull is re-stated after every re-init.
//
// Link: SPI2 at AF5, the SAME PIN NAMES the DUT hosts on (SPI's own
// MOSI/MISO naming carries the direction, so nothing is crossed) -
//   PB12 NSS   (the hardware select input; this board's pull-up holds it
//              high, and for Op::host_burst it becomes a GPIO chip
//              select this end drives)
//   PB13 SCK   (input here)
//   PB14 MISO  (this end's output, driven only to answer)
//   PB15 MOSI  (input here)
// SPI2 is the APB1 instance, so its own clock is 45 MHz at the 180 MHz
// this board runs - and it is the instance ES0298 2.14.5's pad-speed
// ceiling leaves room on (table 5: 84 MHz of APB at high or very-high,
// 75 at medium, 25 at low - for the SCK pad of a host), where the APB2
// instances at 90 MHz have none.
//
// Console: USART2 PA2/PA3 at 115200 on the ST-LINK's virtual COM port,
// observability only.
//   ? help | i status and counters | 0 back to the dark client |
//   3 trace | s cycle the MISO pad speed | k cycle the SCK pad speed
//   the burst's host runs at
//
// The ident label is the 96-bit unique device ID's first word in hex
// (through brio::DeviceUid); ident.xtal is ALWAYS 0, because this board
// fits no crystal - its HSE is the ST-LINK's 8 MHz clock, taken in
// bypass.
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>
#include <type_traits>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrxt/src/apps/spi_link.hpp"

using SysClock =
    brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using spilink::Op;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af7},
    .rx = {'A', 3, PinFunction::af7},
};
using Console = Uart<2, console_pins>;
constexpr Console console;

// ---------------------------------------------------------------------------
// The link's pads, both roles (the DUT's own layout, pin name for pin name)
// ---------------------------------------------------------------------------

constexpr SpiPins link_pins{
    .sck = {'B', 13, PinFunction::af5},
    .miso = {'B', 14, PinFunction::af5},
    .mosi = {'B', 15, PinFunction::af5},
    .nss = {'B', 12, PinFunction::af5},
};

using Client = SpiClient<2, link_pins>;
using Host = SpiHost<2, link_pins>;
using Raw = Spi<2>;

// The two pads this app names outside the driver: the answer line (the
// console's "is MISO driven" reading) and the
// select, which is a GPIO this end drives in two of the actions.
using MisoPin = Pin<link_pins.miso.port, link_pins.miso.pin>;
using NssPin = Pin<link_pins.nss.port, link_pins.nss.pin>;

constexpr uint16_t firmware_version = 0x0402;   ///< see spi_link.hpp's Ident

/// THE ANSWER LINE'S SLEW CLASS, AND IT IS A MEASUREMENT. The driver's
/// default class is very-high, which is ES0298 2.14.5 speaking about the
/// SCK pad of a HOST; on a bus of jumper wires that edge is
/// an AGGRESSOR at the CLIENT's end, and the four classes were walked
/// with the console's 's' against an STM32G0B1RE host's slip statistics
/// (test_stm32_spi letter x: ten bursts of eight frames in each mode,
/// the host's own MOSI one class below its driver's):
///
///   very-high  12 bursts of 40 slip, in modes 0, 1 and 3
///   high       7 of 40, every one in mode 1 or mode 2 - the two modes
///              that SAMPLE ON THE FALLING EDGE, where the answer line
///              changes on the rising one
///   medium     12 of 40, modes 1 and 2 again
///   low        0 of 40
///
/// The rate ladder does not find the slow pad: at low the host's ladder
/// is exact to 16 MHz on both serves (letter p), above DS10693 table 58's
/// 4 MHz for the class at 50 pF - the wire is lighter than that. Every
/// init that drives the pad therefore names LOW in the client's Config,
/// and the console's 's' cycles the four classes for the bench.
PinSpeed miso_speed = PinSpeed::low;

/// THE TWO DRIVEN PADS' SLEW CLASSES WHEN THIS END HOSTS (Op::host_burst),
/// AND THEY ARE MEASUREMENTS TOO. This end holds the select (PB12) low
/// from GPIO for the whole burst, and on these wires the edges beside it
/// couple into it: with SpiHost's default very-high on MOSI the far
/// STM32G0B1RE's select pad sees 4 to 19 edges a burst where 2 are sent,
/// whatever SCK's class from medium up (counted in a diagnostic build of
/// the G0's suite, not kept: an EXTI line counting both edges on the pad,
/// ten bursts of each client shape; its pull-up changes nothing, so the
/// line is driven, not floating). A client that takes NSS
/// from that pad is deselected mid-frame and its bit count starts again:
/// 65 to 79 bursts of 100 lose a frame or slip, both ends wrong from one
/// frame on, in every direction (letter r's exchange, letter y's single
/// line out and its receive-only run). A client under SSM, which reads
/// the pad once to see the select fall and then ignores it, is exact in
/// every burst that carries the same glitches - the select alone is hit.
/// With MOSI at MEDIUM the same diagnostic counts exactly 2 edges in 360
/// bursts of 360 and every shape is exact, SCK at medium, high or
/// very-high alike (and test_stm32_spi's letters r and y, which count no
/// edges, exact in twenty runs each). SCK at low with MOSI at very-high
/// cures it too, so which edge couples into PB12 is not isolated; but
/// SCK at low is outside ES0298 2.14.5's table 5 for this APB1 (25 MHz of APB
/// at low against 45): medium on both pads keeps the erratum's ceiling
/// (75 MHz), and DS10693 table 58's reach for medium (25 MHz at 50 pF)
/// is above the fastest burst this instance can clock (PCLK1/2 = 22.5
/// MHz; every suite asks for a division of 32 or more). The rate
/// is not the lever: a larger division keeps the same edges. The
/// console's 'k' cycles SCK's four classes for the bench; MOSI's has no
/// key.
PinSpeed burst_sck_speed = PinSpeed::medium;
constexpr PinSpeed burst_mosi_speed = PinSpeed::medium;

const char* speed_name(PinSpeed s) {
    switch (s) {
        case PinSpeed::low: return "low";
        case PinSpeed::medium: return "medium";
        case PinSpeed::high: return "high";
        default: return "very_high";
    }
}


bool trace = false;

spilink::Decoder decoder;
spilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, serves = 0;
uint32_t last_byte_ms = 0;
uint32_t engaged_until = 0;

bool engaged() { return engaged_until != 0 && Ticker::millis() < engaged_until; }

// ---- waits ----------------------------------------------------------------

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// A microsecond hold of arbitrary length: delay_us is capped below one
/// SysTick period BY DESIGN, and this instrument's holds are exactly the
/// blocking waits that cap keeps out of KERNEL programs - this is not
/// one, so the hold is spent in chunks the cap admits.
void hold_us(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

// ---- the runs ----------------------------------------------------------------
//
// Every window is ONE CALL of the driver's polled run, in the flash: the
// ART accelerator serves a loop at zero wait states, so nothing here is
// placed in SRAM (docs/stm32f4/spi.md counts the turn). The answers are
// precomputed, the received frames only STORED, and every judgement
// happens after the burst.

using Run = Client::Run;

/// A run's spin budget for `ms` milliseconds: one spin is four polls of
/// SR on APB1 and a look at the select pad, some thirty-six core cycles at
/// 180 MHz (the listing), so this is a bound of the right order, not a
/// clock.
constexpr uint32_t spins_for_ms(uint32_t ms) { return ms * (SysClock::hz / 36'000u); }

/// The exchange's DMA engines: SPI2_TX on DMA1 stream 4 and SPI2_RX on
/// stream 3, channel 0 both (RM0390 table 28 - the reserve checks the two
/// cells). Bound with no interrupt: the exchange polls the receive
/// stream's count, so there is no handler to bind. Their element is
/// uint16_t, DR's width: a run of uint8_t moves a byte a beat and a run
/// of uint16_t a half-word, the span's element choosing.
using PeerTx = DmaTxEngine<1, 4, 0, uint16_t>;
using PeerRx = DmaRxEngine<1, 3, 0, uint16_t>;

void wait_nss_high(uint16_t ms);

/// The select wire is shared; while this end is (or is becoming) a
/// client, its own pull-up is what keeps the line from floating low and
/// selecting it at random. Every re-init rewrites PUPDR, so this is
/// re-stated after each one.
void hold_nss_up() { NssPin::pull(PinPull::up); }

// ---- the client's standing configurations ---------------------------------

/// DARK: mode 0, MSb first, MISO NOT driven (the pad stays in analog).
/// Everything returns here, and SPE goes up so the shifter runs and the
/// command listener can poll RXNE.
bool go_dark() {
    // A CLIENT RECONFIGURES BETWEEN WINDOWS, NEVER INSIDE ONE: the host
    // releases the select tens of microseconds after its last clock
    // edge, and a peer that re-inits the instant the last frame lands is
    // still inside that window.
    wait_nss_high(50);
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .bits = SpiDataSize::bits8,
                                         .lsb_first = false,
                                         .nss = SpiNss::hardware_input,
                                         .drive_output = false});
    hold_nss_up();
    Client::enable(0x00);
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
                                         .miso_speed = miso_speed});
    hold_nss_up();
    return ok;
}

/// What a command asked this client to become. Everything here frames on
/// the NSS PAD: with SSM/SSI the frame boundary would be the clock
/// alone, and a host's own settling edge when it primes CPOL = 1 with
/// the select still high would then be counted into the first frame.
///
/// The extended exchange's three: `wide` sets 16-bit frames (DFF); `crc`
/// arms the CRC unit on `poly` - the CRC is the frame's width on this
/// block (26.4.2: "CRC8 or CRC16 calculation depending on the data format
/// selected through the DFF bit"), and an even polynomial, zero included,
/// is refused by the driver (26.4.2's note, ES0298 2.14.4), which the
/// report says as report_cfg_failed.
bool apply_cfg(const spilink::Cfg& c, bool drive_output = true, bool wide = false,
               bool crc = false, uint16_t poly = 0x0007u) {
    wait_nss_high(50);   // see go_dark(): a client reconfigures BETWEEN windows
    const bool ok = Client::init(clock, {.mode = mode_of(c.mode),
                                         .bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8,
                                         .lsb_first = c.dord != 0,
                                         .nss = SpiNss::hardware_input,
                                         .crc = crc,
                                         .crc_polynomial = poly,
                                         .drive_output = drive_output,
                                         .miso_speed = miso_speed});
    hold_nss_up();
    return ok;
}

// ---- answering -------------------------------------------------------------

void wait_nss_high(uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Client::selected() && Ticker::millis() - t0 < ms) {
    }
}

/// Serve ONE answer window: the frame padded with zeros to the most the
/// host clocks (spilink::answer_bytes), handed to the answer run - buf[0]
/// into the empty buffer with the select still high, every later answer
/// on its frame's TXE. The window ENDS WHEN THE HOST CLOSES IT, the run
/// reading the select pad; a host that clocks fewer than answer_bytes
/// and raises the select ends it as surely as one that clocks them all.
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

// ---- accounting ------------------------------------------------------------

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


// ---- the actions -----------------------------------------------------------

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

bool dma_ok = false;

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

/// THE CRC RUN, over the resource - the driver's runs carry no CRC phase
/// (docs/stm32f4/spi.md's gap list). 26.4.2 "CRC transfer managed by
/// CPU", on the run's own shape: the first answer into the empty buffer
/// with the select still high, every later one on its frame's TXE, and
/// CRCNEXT written "as soon as the last data is written to the SPI_DR
/// register" (26.7.1, CR1.CRCNEXT) - so the CRC follows the frame the
/// last answer rides in. Each frame read on its RXNE, the host's CRC frame
/// the last one taken ("the received CRC is stored in the Rx buffer like
/// any other data frame"). `in` holds n + 1 frames.
template <typename T>
Run crc_run(const T* out, T* in, uint16_t n, uint32_t spins) {
    SPI_TypeDef& regs = Raw::regs();
    uint16_t w = 0;
    uint16_t r = 0;
    const uint16_t want = static_cast<uint16_t>(n + 1u);
    Client::RunEnd how = Client::RunEnd::complete;
    regs.DR = out[w++];
    if (w == n) {
        Raw::crc_next();
    }
    while (!Client::selected()) {
        if (--spins == 0u) {
            return {0, Client::RunEnd::stalled};
        }
    }
    auto await = [&](uint32_t flag) {
        for (;;) {
            for (uint8_t k = 0; k < 4u; ++k) {
                if ((regs.SR & flag) != 0u) {
                    return true;
                }
            }
            if (!Client::selected()) {
                how = Client::RunEnd::closed;
                return false;
            }
            if (--spins == 0u) {
                how = Client::RunEnd::stalled;
                return false;
            }
        }
    };
    while (r < want) {
        if (w < n) {
            const T next = out[w];
            if (!await(SPI_SR_TXE)) {
                break;
            }
            regs.DR = next;
            if (++w == n) {
                Raw::crc_next();
            }
        }
        if (!await(SPI_SR_RXNE)) {
            break;
        }
        in[r++] = static_cast<T>(regs.DR);
    }
    // A frame that landed beside the close is the run's.
    if (r < want && (regs.SR & SPI_SR_RXNE) != 0u) {
        in[r++] = static_cast<T>(regs.DR);
    }
    return {r, how};
}

/// THE DMA SERVE: 26.3.11's order - RXDMAEN, the streams started, TXDMAEN,
/// SPE last - so the transmit stream's first request is the standing TXE
/// of an idle block and its first beat lands in the buffer before the
/// select falls: the preload, and from there one beat a TXE. A
/// receive-only serve leaves the transmit side out. Under `crc` the
/// chapter's "CRC transfer managed by DMA" (26.4.2): both streams count
/// the DATA frames, the block appends its CRC after the transmit stream's
/// last and checks the host's by itself - and the CRC frame it received
/// "is always loaded into" DR, so it is read from there (into x_in[n])
/// once it stands. The receive stream's count is the transaction's
/// progress; the closing order is the chapter's too (streams, then the
/// requests).
template <typename T>
Run dma_serve(uint16_t n, bool transmit, bool crc, uint32_t ms) {
    Raw::dma_receive(true);
    (void)PeerRx::start(std::span<T>(reinterpret_cast<T*>(x_in), n));
    if (transmit) {
        (void)PeerTx::start(std::span<const T>(reinterpret_cast<const T*>(x_out), n));
        Raw::dma_transmit(true);
    }
    Client::enable();
    const uint32_t t0 = Ticker::millis();
    while (!PeerRx::full() && Ticker::millis() - t0 < ms) {
        (void)PeerRx::take();
    }
    (void)PeerRx::take();
    const bool all_in = PeerRx::full();
    uint16_t got = PeerRx::taken();
    if (crc && all_in) {
        while (!Raw::rx_ready() && Client::selected() && Ticker::millis() - t0 < ms) {
        }
        if (Raw::rx_ready()) {
            reinterpret_cast<T*>(x_in)[n] = static_cast<T>(Raw::data());
            ++got;
        }
    }
    PeerTx::stop();
    PeerRx::stop();
    Raw::dma_requests(false, false);
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

/// The workhorse: the host sends P_A(i), this client answers P_B(i) -
/// on the DMA engines by default, on the driver's polled run with
/// `spare_polled_pump`, the answers alone with `spare_answer_only`, the
/// frames in alone with `spare_receive_only`; 16-bit frames with
/// `spare_wide`, the CRC with `spare_crc` (the header).
spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    r.flags |= spilink::report_extended;
    const bool wide = (a.spare & spilink::spare_wide) != 0;
    const bool crc = (a.spare & spilink::spare_crc) != 0;
    const bool receive_only = (a.spare & spilink::spare_receive_only) != 0;
    const bool answer_only = !receive_only && (a.spare & spilink::spare_answer_only) != 0;
    const bool preload = a.cfg.regime == spilink::regime_buffer_wait;
    const uint16_t n = a.count < exchange_cap ? a.count : exchange_cap;
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
                         (a.spare & spilink::spare_polled_pump) == 0;

    const uint32_t t0 = Ticker::millis();
    Run run{};
    if (engines) {
        run = wide ? dma_serve<uint16_t>(n, !receive_only, crc, a.ms)
                   : dma_serve<uint8_t>(n, !receive_only, crc, a.ms);
        r.aux2 |= 0x04;
    } else {
        if (!preload && !receive_only) {
            // NO PRELOAD: the run starts once the select has fallen, so the
            // first frame the host clocks out is whatever the shifter held
            // and this end's stream follows ONE PLACE LATE - what
            // spilink's regime_normal names on a silicon with no
            // transmit-buffer mode to choose.
            const uint32_t w0 = Ticker::millis();
            while (!Client::selected() && Ticker::millis() - w0 < a.ms) {
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
/// receive buffer holds one frame and OVR stands for every frame after
/// it. The status is OR'ed DURING the burst - a flag that comes and goes
/// is invisible afterwards - and what the buffer retained is drained at
/// the end. The burst's end is the DEADLINE the command carries.
spilink::Report run_sink_slow(const spilink::Params& a) {
    spilink::Report r{};
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    Client::enable(0x00);                // something harmless to shift out
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
/// count the same lead-in milliseconds, and at their end the DUT is a
/// client with its first answer preloaded. PB12 is driven LOW from GPIO
/// for the WHOLE burst - one transaction, the thing hardware NSS cannot
/// frame - and SpiHost never claims that pad, which is what makes the
/// swap a re-init.
///
/// aux16 is "the instrument's SCK division of its own clock", and this
/// instrument's SPI2 clock is PCLK1: the division is turned into the BR
/// code that produces AT MOST PCLK1/aux16.
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
    const uint32_t pclk = SysClock::pclk1_hz;
    const auto code = spi_rate_for(pclk, pclk / div);
    if (!Host::init(clock, {.sck_speed = burst_sck_speed, .mosi_speed = burst_mosi_speed}) ||
        !code) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    const SpiClock rate = *code;
    r.aux1 = static_cast<uint8_t>(static_cast<uint8_t>(rate));   // the BR code itself
    (void)Host::bit_order(a.cfg.dord != 0);
    // The mode is primed BEFORE the select falls: a CPOL flip inside an
    // open window is one extra edge and the selected client counts it.
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

// ---- the command handler ---------------------------------------------------

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
        d.xtal = 0;                       // no crystal fitted: the HSE is a bypass clock
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
        // Op::mspi asks for a USART in host-SPI mode with no select, and
        // no board of this stratum wires one; cfg_failed says "not on
        // this peer" rather than pretending.
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

// ---- the console -----------------------------------------------------------

void help() {
    print(console, "spi_peer: ? help | i status | 0 back to the dark client | 3 trace | "
                   "s cycle the MISO pad speed | k cycle the SCK pad speed as host",
          crlf);
}

void status() {
    print(console, "  client SPI2 AF5, enabled=", Raw::enabled(), " SR=", hex(Raw::status()),
          " DMA1 serve=", dma_ok ? "armed" : "off", crlf);
    print(console, "  NSS pin reads ", NssPin::read() ? "high" : "LOW",
          ", selected=", Client::selected(), ", MISO driven=", MisoPin::has_function(),
          ", MISO pad speed=", speed_name(miso_speed),
          ", SCK pad speed as host=", speed_name(burst_sck_speed), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          " served=", serves, engaged() ? "  (ENGAGED)" : "  (dark, mute)", crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " mism=", last_report.mism, " sum=", hex(last_report.sum),
          " flags=", hex(last_report.flags), " idx=", last_report.idx,
          " got=", hex(last_report.got), " exp=", hex(last_report.exp), crlf);
    print(console, "               aux=", hex(last_report.aux0), " ", hex(last_report.aux1),
          " ", hex(last_report.aux2), " ", hex(last_report.aux3), " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void USART2_IRQHandler() { (void)Console::isr(); }

/// SPI2's line carries nothing - every window is a polled run - but
/// SpiClient::init() opens it at the NVIC, so a stray level is DISARMED
/// rather than looped on.
extern "C" void SPI2_IRQHandler() {
    Raw::rxne_interrupt(false);
    Raw::txe_interrupt(false);
    Raw::error_interrupt(false);
}

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

    // The exchange's engines, bound once: the binding names SPI2's DR and
    // the request cell, nothing an SPI re-init touches.
    brio::Dma<1>::init();
    PeerTx::arm(Raw::data_address(), brio::DmaPriority::high, brio::DmaInterrupts::none);
    PeerRx::arm(Raw::data_address(), brio::DmaPriority::very_high, brio::DmaInterrupts::none);
    dma_ok = true;

    // The select pad readable (and held up) before the first go_dark():
    // its wait for a released select reads the pad.
    NssPin::input(PinPull::up);

    print(console, crlf, "spi_peer - SPI instrument client (STM32F446RE board ", label8,
          ", xtal=none (the HSE is the probe's clock, taken in bypass), fw ",
          hex(firmware_version), ")", crlf);
    print(console, "client SPI2 AF5: NSS PB12 (pulled up), SCK PB13, MISO PB14 (pad speed ",
          speed_name(miso_speed), "), MOSI PB15; command mode = SPI mode 0, MSb first", crlf);
    print(console, "DARK by default: MISO is driven only for one answer window, after a "
                   "frame that checked out", crlf);
    print(console, "boot: clk=", clock_ok ? "PLL 180 MHz" : "FAILED",
          " pclk1=", SysClock::pclk1_hz / 1'000'000u, " MHz tick=", tick_ok ? "SysTick" : "FAILED",
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
                if (c == '?') {
                    help();
                } else if (c == 'i') {
                    status();
                } else if (c == '0') {
                    engaged_until = 0;
                    (void)go_dark();
                    print(console, "  dark client restored, engagement dropped", crlf);
                } else if (c == '3') {
                    trace = !trace;
                    print(console, trace ? "  trace on" : "  trace off", crlf);
                } else if (c == 's') {
                    miso_speed = miso_speed == PinSpeed::very_high ? PinSpeed::high
                                 : miso_speed == PinSpeed::high    ? PinSpeed::medium
                                 : miso_speed == PinSpeed::medium  ? PinSpeed::low
                                                                   : PinSpeed::very_high;
                    print(console, "  MISO pad speed now ", speed_name(miso_speed),
                          " (applied at the next answer window)", crlf);
                } else if (c == 'k') {
                    burst_sck_speed = burst_sck_speed == PinSpeed::very_high ? PinSpeed::high
                                      : burst_sck_speed == PinSpeed::high    ? PinSpeed::medium
                                      : burst_sck_speed == PinSpeed::medium  ? PinSpeed::low
                                                                             : PinSpeed::very_high;
                    print(console, "  SCK pad speed as host now ", speed_name(burst_sck_speed),
                          " (applied at the next host_burst)", crlf);
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
