// spi_peer - a scriptable SPI CLIENT for the other end of a two-board bus
// test, on a CH32V006: the far board is the bus host and the DUT, and
// drives this one IN BAND over the very bus both are testing, every
// command travelling as a checksummed frame in the same traffic.
//
// The wire format is spi_link.hpp, included by relative path: one file is
// the single source of truth for every board that speaks it, whatever its
// architecture. What this silicon makes of it - the STM32F1's SPI under
// WCH's register names, RM ch. 16:
//
//  - THE BUS IS SPI1 ON ITS COLUMN 0 (AFIO table 7-12, code 000) WITH THE
//    HARDWARE SELECT: SCK PC5, MOSI PC6, MISO PC7, NSS PC1 - the desk's
//    SPI column, whole since the I2C moved to column 3. The client frames
//    on the NSS PAD (SSM clear): a host's settling edge with the select
//    still high is not counted into the first frame, and the runs below
//    read the same pad for the window's close.
//  - ONE TRANSMIT BUFFER IN FRONT OF ONE SHIFTER, no FIFO (figure 16-1):
//    TXE rises as a frame enters the shifter (16.2.3), so an answer is
//    written on the TXE of the frame before it - most of a frame of
//    window - and a client that falls behind does not stall the bus: the
//    host reads whatever the shifter held. Every polled window is ONE RUN
//    over the driver's per-frame verbs (writable, write, poll, selected),
//    each one register access inline.
//  - THE EXCHANGE IS SERVED BY THE DMA ENGINES BY DEFAULT, SPI1's receive
//    on channel 2 and its transmit on channel 3 (table 8-2), polled
//    through the receive channel's count with no interrupt; the transmit
//    channel's first beat lands in the empty buffer before the select
//    falls - the preload - and the rest one a TXE. THE REQUESTS ARE RAISED
//    AROUND THE CHANNELS: a request that rises while its channel is
//    disabled is LATCHED by this controller (docs/ch32v00x/spi.md), so
//    RXDMAEN goes up only once the receive channel runs and TXDMAEN once
//    the transmit one does. `spilink::spare_polled_pump` asks for the
//    polled run instead, `spilink::spare_answer_only` for the answer run
//    alone (the report then counts the answers written and judges
//    nothing).
//  - A RUN SAMPLES NOTHING INSIDE THE BURST, so the exchange's report
//    fields are this peer's own (spi_link.hpp leaves them to the peer's
//    header): aux0 = STATR after the burst, aux1 = the milliseconds from
//    the run's start to its END (255 = nothing came), aux2 bit 0 = the
//    select read low once after the run, bit 1 = the host closed it, bit 2
//    = the DMA engines served it; aux3 = 0.
//  - THE BUFFERING REGIMES COLLAPSE, as on every F1-lineage peer: this
//    client always answers from its one buffer, and spilink::Cfg::regime
//    is MAPPED - regime_buffer_wait starts the run with the select high,
//    the first answer preloaded, the exact rx[i] = P_B(i) alignment that
//    regime promises; the other two start the run once the select has
//    fallen, so the shifter's leftover leads and the stream follows one
//    place late - the samc21 and stm32g0 peers' mapping.
//  - THE PROTOCOL CARRIES OPTIONS THIS SILICON HAS NOT GOT: flag_wrcol and
//    flag_feed_tx name conditions chapter 16 does not have and are
//    ignored; Op::mspi asks for a USART in host-SPI mode, and no USART of
//    this part reaches these wires - answered report_cfg_failed.
//  - BSY IS NOT LOOKED AT ON THIS SIDE: RXNE is the receiving side's
//    witness and the NSS pad the window's.
//  - THE ANSWER WINDOWS ARE CLOCKED IN MODE 3, not spi_link's mode 0.
//    16.2.3: "if you need to send in slave mode, you need to set CPOL to
//    mode 2 or mode 3" - and measured against the CH32X035's host, this
//    client's answers in mode 0 come back one bit early in the windows
//    that follow another answer (the first sample racing the change), its
//    answers in modes 1 and 2 are wrong, and mode 3's are exact. So the
//    ack, nak, ident and report windows are served CPOL = 1, CPHA = 1 and
//    the far host finds that at its rendezvous (spi_link's alternate
//    answer mode); the command windows stay mode 0, where this client only
//    receives. An exchange's answers go in whatever mode the command
//    names, which is what the far host's matrix measures - and measured
//    there, this client is served in modes 0 and 3 both ways and in NEITHER
//    direction in modes 1 and 2 (its MISO a fixed pattern, what it hears
//    wrong), the far host's waveforms the standard's in all four. The peer
//    serves modes 1 and 2 all the same, so a DUT measures them - its
//    driver's SpiClient refuses them (spi_client_mode_served), so there it
//    configures the block BY HAND, the resource's verbs on the same pads;
//    its report says what arrived.
//  - THE EXTENDED EXCHANGE (spi_link's spare_wide, spare_crc,
//    spare_receive_only) is not served: this peer runs 8-bit frames
//    whatever the command asks and never raises report_extended, which is
//    how a host knows (the CH32X035 peer serves it).
//
// THE DARK LISTENER discipline: the command-mode client never drives MISO
// (drive_output = false, the pad a floating input), the answer line wakes
// only for one answer window after a frame that CHECKED OUT, an unknown op
// is dropped in silence, and a bad checksum is nak'ed only while ENGAGED.
// The select wire is held up by THIS board's internal pull-up (about 45
// kOhm) whenever this end is a client.
//
// THE ROLES INVERT for Op::host_burst: SPI1 becomes the bus HOST on the
// same pads, PC1 a GPIO chip select this end drives low for the whole
// burst - one transaction, which a hardware select cannot frame - and
// ss_pulse drives PC1 low from GPIO for its microseconds, the SPI released
// first: the far host's MODF from a second chip.
//
// THE SUPPLY at boot (the desk's rule for a board that drives a link pad):
// VDD read from VREFINT; below 3000 mV the peer stays off the bus and says
// so on its console.
//
// Console: USART1 on PD5/PD6 at 115200 - the probe's serial -
// observability only.
//   ? help | i status and counters | 0 back to the dark client | 3 trace
//
// The ident label is the part number after its CH32 prefix ("V006");
// ident.xtal says whether the 24 MHz crystal started (the PLL doubles it
// to 48 MHz, the HSI's PLL the fallback at the same rate).
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "ch32v00x/adc.hpp"
#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/spi.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/spi_link.hpp"

namespace {

using namespace brio;
using spilink::Op;

using P = Ch32v00xPlatform<>;
/// The crystal's 24 MHz doubled; the HSI's PLL the fallback at the same rate.
using SysClock = Clock<ClockSource::pll, 48'000'000, 24'000'000>;
using HsiClock = Clock<ClockSource::pll, 48'000'000>;
constexpr SysClock clock;
static_assert(SysClock::hz == HsiClock::hz, "the fallback must run every divisor at the same rate");

using Console = Uart<1, P>;
constexpr Console console;

/// SPI1's column 0, every signal (the header).
constexpr SpiPins link_pins = spi1_pins_for(0);
static_assert(link_pins.sck == Pad{'C', 5} && link_pins.mosi == Pad{'C', 6} && link_pins.miso == Pad{'C', 7} &&
                  link_pins.nss == Pad{'C', 1},
              "AFIO table 7-12: SPI1's code 000 is SCK PC5, MOSI PC6, MISO PC7, NSS PC1");

using Client = SpiClient<1, link_pins>;
using Host = SpiHost<1, link_pins>;
using Raw = Spi<1>;
using SckPin = Pin<'C', 5>;
using MosiPin = Pin<'C', 6>;
using MisoPin = Pin<'C', 7>;
using NssPin = Pin<'C', 1>;

constexpr uint16_t firmware_version =
    static_cast<uint16_t>((spilink::peer_ch32v00x << 8) | 0x02u);   ///< spi_link.hpp's Ident

static_assert(device::part_name[0] == 'C' && device::part_name[1] == 'H' &&
                  device::part_name[2] == '3' && device::part_name[3] == '2',
              "spi_peer: the ident label is the part number after its CH32 prefix");

/// THE ANSWER WINDOWS ARE SERVED IN MODE 3 (the header): this client's
/// answers are exact in mode 3 and not in mode 0 (measured), so every ack,
/// nak, ident and report window is configured CPOL = 1, CPHA = 1 and the
/// host clocks it so; the command windows stay mode 0, which a receiving
/// client is served in.
constexpr uint8_t answer_window_mode = 3;

/// The exchange's engines, bound once with NO interrupt: the serve polls
/// the receive channel's count.
using PeerTx = DmaTxEngine<3>;
using PeerRx = DmaRxEngine<2>;

bool crystal_ok = false;
bool supply_ok = false;
uint16_t supply_mv = 0;
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

/// A microsecond hold of any length: delay_us is bounded below one tick by
/// design, so a longer hold is spent in chunks it admits.
void hold_us(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

void wait_nss_high(uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Client::selected() && Ticker::millis() - t0 < ms) {
    }
}

// ---- the client's standing configurations ---------------------------------

/// DARK: mode 0, MSb first, MISO NOT driven. Everything returns here, and
/// SPE goes up so the shifter runs and the command listener polls RXNE.
/// A client reconfigures BETWEEN windows, never inside one: the far host
/// releases the select some tens of microseconds after its last edge.
bool go_dark() {
    wait_nss_high(50);
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .bits = SpiDataSize::bits8,
                                         .lsb_first = false,
                                         .nss = SpiNss::hardware_input,
                                         .drive_output = false});
    Pfic::disable(Raw::irq());   // polled: the vector stays dark
    Raw::enable();               // nothing loaded: MISO is not driven anyway
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

/// A client in a mode SpiClient refuses on this part (modes 1 and 2,
/// spi_client_mode_served), configured BY HAND for a DUT to measure: what
/// Client::init() does, through the resource - the gate, the reset, the
/// column, the configuration, the inputs, MISO handed over last.
bool client_by_hand(const Client::Config& c) {
    Pfic::disable(Raw::irq());
    Raw::bus_clock(true);
    Raw::reset();
    Afio::remap_spi1(link_pins.remap);
    if (!Raw::configure({.role = SpiRole::client, .mode = c.mode, .bits = c.bits, .lsb_first = c.lsb_first,
                         .nss = SpiNss::hardware_input})) {
        return false;
    }
    SckPin::input();
    MosiPin::input();
    NssPin::input(PinPull::up);
    MisoPin::function();
    Raw::flush_rx();
    return true;
}

/// What a command asked this client to become, MISO driven.
bool apply_cfg(const spilink::Cfg& c) {
    wait_nss_high(50);
    const Client::Config cc{.mode = mode_of(c.mode),
                            .bits = SpiDataSize::bits8,
                            .lsb_first = c.dord != 0,
                            .nss = SpiNss::hardware_input,
                            .drive_output = true};
    const bool ok = Client::config_served(cc) ? Client::init(clock, cc) : client_by_hand(cc);
    Pfic::disable(Raw::irq());
    return ok;
}

// ---- the runs ----------------------------------------------------------------
//
// Over the driver's per-frame verbs, each one register access inline: the
// answer for frame k + 1 written on frame k's TXE, a received frame read on
// its RXNE, the NSS pad read once for every four polls. The answers are
// precomputed, the received frames only stored, every judgement after.

enum class RunEnd : uint8_t { complete, closed, stalled };
struct Run {
    uint16_t count;
    RunEnd end;
};

/// A run's spin budget: a spin is four polls and a look at the pad, some
/// thirty cycles of this core - a bound of the right order, not a clock.
constexpr uint32_t spins_for_ms(uint32_t ms) { return ms * (SysClock::hz / 30'000u); }

[[gnu::always_inline]] inline bool await_select(uint32_t& spins) {
    while (!Client::selected()) {
        if (--spins == 0u) {
            return false;
        }
    }
    return true;
}

/// Four polls a look: true when the flag stands, false on the close or the
/// budget (`how` says which).
template <bool tx>
[[gnu::always_inline]] inline bool await(uint32_t& spins, RunEnd& how) {
    for (;;) {
        for (uint8_t k = 0; k < 4u; ++k) {
            if (tx ? Raw::txe() : Raw::rxne()) {
                return true;
            }
        }
        if (!Client::selected()) {
            how = RunEnd::closed;
            return false;
        }
        if (--spins == 0u) {
            how = RunEnd::stalled;
            return false;
        }
    }
}

/// THE ANSWER RUN: `out` on TXE alone, nothing read (the receive buffer
/// overruns from the second frame, cleared at the tail). The first answer
/// goes into the empty buffer at once, with the select high; the run ends
/// when the host raises it.
Run answer_run(const uint8_t* out, uint16_t n, uint32_t spins) {
    uint16_t i = 0;
    RunEnd how = RunEnd::closed;
    if (n != 0u) {
        Raw::data8(out[i++]);
    }
    Raw::enable();   // SPE after the preload, the DMA serve's order
    if (!await_select(spins)) {
        Raw::flush_rx();
        return {i, RunEnd::stalled};
    }
    for (; i < n; ++i) {
        const uint8_t next = out[i];
        if (!await<true>(spins, how)) {
            Raw::flush_rx();
            return {i, how};
        }
        Raw::data8(next);
    }
    while (Client::selected()) {
        if (--spins == 0u) {
            how = RunEnd::stalled;
            break;
        }
    }
    Raw::flush_rx();
    return {i, how};
}

/// THE EXCHANGE RUN: answers as answer_run()'s, and each turn reads the
/// frame the TXE belonged to on its RXNE - inside the next frame, the
/// receive buffer being one frame deep.
Run exchange_run(const uint8_t* out, uint8_t* in, uint16_t n, uint32_t spins) {
    if (n == 0u) {
        return {0, RunEnd::complete};
    }
    Raw::flush_rx();
    uint16_t w = 0;
    uint16_t r = 0;
    RunEnd how = RunEnd::complete;
    Raw::data8(out[w++]);
    Raw::enable();   // SPE after the preload, the DMA serve's order
    if (!await_select(spins)) {
        return {0, RunEnd::stalled};
    }
    for (; w < n; ++w, ++r) {
        const uint8_t next = out[w];
        if (!await<true>(spins, how)) {
            goto drain;
        }
        Raw::data8(next);
        if (!await<false>(spins, how)) {
            goto drain;
        }
        in[r] = Raw::data8();
    }
    if (!await<false>(spins, how)) {
        goto drain;
    }
    in[r++] = Raw::data8();
    return {r, RunEnd::complete};
drain:
    if (r < n && Raw::rxne()) {
        in[r++] = Raw::data8();
    }
    return {r, how};
}

// ---- answering -------------------------------------------------------------

/// Serve ONE answer window: the frame padded with zeros to the most the
/// host clocks (spilink::answer_bytes), the run ending when the host
/// closes the window.
void serve(const uint8_t* buf, uint8_t n) {
    wait_nss_high(50);                    // the command window must close first
    if (!apply_cfg(spilink::Cfg{.apply = 1, .mode = answer_window_mode})) {
        (void)go_dark();
        return;
    }
    uint8_t padded[spilink::answer_bytes] = {};
    for (uint8_t i = 0; i < n && i < spilink::answer_bytes; ++i) {
        padded[i] = buf[i];
    }
    (void)answer_run(padded, spilink::answer_bytes, spins_for_ms(spilink::serve_ms));
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

constexpr uint16_t exchange_cap = 64;
uint8_t x_out[exchange_cap];
uint8_t x_in[exchange_cap];

/// The host burst's frame buffers, TWO bytes although the burst is 8-bit:
/// SpiHost's frame store is width-general.
uint8_t burst_tx[2];
uint8_t burst_rx[2];

/// The DMA serve: the receive channel and RXDMAEN, the transmit channel
/// and TXDMAEN - its first beat the standing TXE of an idle buffer, the
/// preload - then SPE. The receive channel's count is the progress; the
/// requests come down with the channels.
Run dma_exchange(uint16_t n, uint32_t ms) {
    (void)PeerRx::start(std::span<uint8_t>(x_in, n));
    Raw::dma_requests(false, true);
    (void)PeerTx::start(std::span<const uint8_t>(x_out, n));
    Raw::dma_requests(true, true);
    Raw::enable();
    const uint32_t t0 = Ticker::millis();
    while (!PeerRx::full() && Ticker::millis() - t0 < ms) {
        (void)PeerRx::take();
    }
    (void)PeerRx::take();
    const bool all_in = PeerRx::full();
    const uint16_t got = PeerRx::taken();
    Raw::dma_requests(false, false);
    PeerTx::stop();
    PeerRx::stop();
    return {got, all_in ? RunEnd::complete : RunEnd::stalled};
}

/// The workhorse: the host sends P_A(i), this client answers P_B(i) - on
/// the DMA engines by default, the polled run with `spare_polled_pump`, the
/// answers alone with `spare_answer_only`.
spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    const bool preload = a.cfg.regime == spilink::regime_buffer_wait;
    const uint16_t n = a.count < exchange_cap ? a.count : exchange_cap;
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    {
        spilink::Stream out(a.pattern, a.seed_b);
        for (uint16_t i = 0; i < n; ++i) {
            x_out[i] = out.next();
        }
    }
    const bool answer_only = (a.spare & spilink::spare_answer_only) != 0;
    const bool engines = preload && !answer_only && (a.spare & spilink::spare_polled_pump) == 0;

    const uint32_t t0 = Ticker::millis();
    Run run{};
    if (engines) {
        run = dma_exchange(n, a.ms);
        r.aux2 |= 0x04;
    } else {
        if (!preload) {
            // NO PRELOAD: the run starts once the select has fallen, so the
            // first frame the host clocks out is the shifter's leftover and
            // this end's stream follows ONE PLACE LATE - regime_normal on a
            // silicon with no transmit-buffer mode to choose.
            const uint32_t w0 = Ticker::millis();
            while (!Client::selected() && Ticker::millis() - w0 < a.ms) {
            }
        }
        run = answer_only ? answer_run(x_out, n, spins_for_ms(a.ms))
                          : exchange_run(x_out, x_in, n, spins_for_ms(a.ms));
    }
    const uint32_t dt = Ticker::millis() - t0;
    r.aux0 = static_cast<uint8_t>(Raw::status());
    r.aux1 = run.count == 0u ? 255u : (dt > 254u ? 254u : static_cast<uint8_t>(dt));
    if (Client::selected()) r.aux2 |= 0x01;
    if (run.end == RunEnd::closed) r.aux2 |= 0x02;
    if (Client::overrun()) r.flags |= spilink::report_bufovf;

    if (answer_only) {
        r.count = run.count;   // answers written; nothing was received
    } else {
        spilink::Stream in(a.pattern, a.seed_a);
        const bool reversed = (a.flags & spilink::flag_expect_reversed) != 0;
        for (uint16_t i = 0; i < run.count; ++i) {
            const uint8_t v = in.next();
            account(r, x_in[i], reversed ? spilink::bit_reverse(v) : v);
        }
    }
    if (r.count < a.count) r.flags |= spilink::report_timed_out;
    (void)go_dark();
    return r;
}

/// The loss semantics: DATAR is never read for the whole burst, so the
/// receive buffer holds one frame and OVR stands for every frame after it.
/// The status is OR'ed DURING the burst, what the buffer retained drained
/// at the end. The burst's end is the deadline the command carries.
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
        seen = static_cast<uint8_t>(seen | static_cast<uint8_t>(Raw::status()));
        if (Client::overrun()) r.flags |= spilink::report_bufovf;
    }
    r.flags |= spilink::report_timed_out;   // by construction: the deadline IS the exit
    r.sum = seen;
    r.got = static_cast<uint8_t>(Raw::status());
    uint8_t vals[4] = {};
    uint8_t retained = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        const auto v = Client::poll();
        if (!v) break;
        if (retained < 4) vals[retained] = static_cast<uint8_t>(*v);
        ++retained;
    }
    r.exp = static_cast<uint8_t>(Raw::status());
    r.count = retained;
    r.aux0 = vals[0];
    r.aux1 = vals[1];
    r.aux2 = vals[2];
    r.aux3 = vals[3];
    (void)go_dark();
    return r;
}

/// The second driver on the shared select wire: PC1 low from GPIO for
/// aux16 microseconds (1000 by default), the SPI handed back first, so
/// nothing of this board is on the other three wires.
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

/// THE ROLES INVERT: this end becomes the bus HOST for a bounded burst.
/// The ack is already served, both boards count the same lead-in, and at
/// its end the DUT is a client with its first answer preloaded. PC1 is a
/// GPIO driven LOW for the whole burst, SpiHost never claiming that pad.
/// aux16 is "the instrument's SCK division of its own clock": the BR code
/// producing AT MOST HCLK/aux16.
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
    const auto code = spi_rate_for(SysClock::hz, SysClock::hz / div);
    if (!Host::init(clock) || !code) {
        r.flags |= spilink::report_cfg_failed;
        Host::release();
        NssPin::input(PinPull::up);
        (void)go_dark();
        return r;
    }
    Pfic::disable(Raw::irq());            // every request below is polled
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

/// The part number after its CH32 prefix, NUL padded to eight characters.
void ident_label(char out[8]) {
    const char* name = device::part_name + 4;
    uint8_t i = 0;
    for (; i < 8u && name[i] != '\0'; ++i) {
        out[i] = name[i];
    }
    for (; i < 8u; ++i) {
        out[i] = '\0';
    }
}

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
        ident_label(d.label);
        d.xtal = crystal_ok ? 1u : 0u;
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
        // Op::mspi asks for a USART in host-SPI mode with no select; no
        // USART of this part reaches these wires.
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

// ---- the supply and the console ----------------------------------------------

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

void help() {
    print(console, "spi_peer: ? help | i status | 0 back to the dark client | 3 trace", crlf);
}

void status() {
    print(console, "  client SPI1 column 0, enabled=", Raw::enabled(), " STATR=", hex(Raw::status()), crlf);
    print(console, "  NSS PC1 reads ", NssPin::read() ? "high" : "LOW", ", selected=", Client::selected(),
          ", supply ", supply_mv, " mV", crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks, " served=", serves,
          engaged() ? "  (ENGAGED)" : "  (dark, mute)", crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " mism=", last_report.mism, " sum=", hex(last_report.sum), " flags=", hex(last_report.flags),
          " idx=", last_report.idx, " got=", hex(last_report.got), " exp=", hex(last_report.exp), crlf);
    print(console, "               aux=", hex(last_report.aux0), " ", hex(last_report.aux1), " ",
          hex(last_report.aux2), " ", hex(last_report.aux3), " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Console::isr(); }

/// SPI1's line carries nothing - every window is a polled run - and both
/// tasks' init() open it at the PFIC, so a stray level is DISARMED rather
/// than looped on.
extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    Raw::rxne_interrupt(false);
    Raw::txe_interrupt(false);
    Raw::error_interrupt(false);
}
/// The engines are bound with no interrupt; a stray line is cleared.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() { (void)PeerRx::service(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { (void)PeerTx::service(); }

int main() {
    crystal_ok = SysClock::init();
    const bool clock_ok = crystal_ok || HsiClock::init();
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = Ticker::init(clock);
    enable_interrupts();

    supply_mv = read_supply_mv();
    supply_ok = supply_mv >= supply_floor_mv;

    // The engines, bound once and with no interrupt: the binding names
    // DATAR, nothing an SPI re-init touches. Receive at very_high, transmit
    // at high (docs/design/dma.md).
    PeerTx::arm(Raw::data_address(), 0u, DmaPriority::high);
    PeerRx::arm(Raw::data_address(), 0u, DmaPriority::very_high);

    char label[9] = {};
    ident_label(label);
    // The select pad readable (and held up) before the first go_dark().
    if (supply_ok) {
        NssPin::input(PinPull::up);
        (void)go_dark();
    }

    print(console, crlf, "spi_peer - SPI instrument client (", device::part_name, ", label '", label,
          "', fw ", hex(firmware_version), ", VDD ", supply_mv, " mV)", crlf);
    print(console, "client SPI1 column 0: NSS PC1 (the hardware select, pulled up), SCK PC5, MOSI PC6, "
                   "MISO PC7; command mode = SPI mode 0, MSb first", crlf);
    print(console, "DARK by default: MISO is driven only for one answer window, after a frame that "
                   "checked out", crlf);
    print(console, "boot: clk=", crystal_ok ? "HSE 24 MHz x2" : (clock_ok ? "HSI x2 (NO CRYSTAL)" : "FAILED"),
          " tick=", tick_ok ? "STK" : "FAILED", " console=", console_ok ? "USART1" : "FAILED", crlf);
    if (!supply_ok) {
        print(console, "SUPPLY LOW (under ", supply_floor_mv, " mV): the link pads stay floating", crlf);
    }
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
                    if (supply_ok) {
                        (void)go_dark();
                    }
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

        if (!supply_ok) {
            continue;
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
