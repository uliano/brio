// spi_peer - a scriptable SPI CLIENT for the other end of a two-board bus
// test, on a CH32X035: the far board is the bus host and the DUT, and
// drives this one IN BAND over the very bus both are testing, every
// command travelling as a checksummed frame in the same traffic.
//
// The wire format is spi_link.hpp, included by relative path: one file is
// the single source of truth for every board that speaks it, whatever its
// architecture. What this silicon makes of it - the STM32F1's SPI under
// WCH's register names, RM ch. 16:
//
//  - THE BUS IS SPI1 ON ITS COLUMN 0 (8.3.2.1, code 00) WITH THE HARDWARE
//    SELECT: NSS PA4, SCK PA5, MISO PA6, MOSI PA7 - the one column this
//    package bonds whole, wired to the CH32V006's column 0. The client
//    frames on the NSS PAD (SSM clear, the pad pulled up by the client's
//    init, some 110 kOhm): a host's settling edge with the select still
//    high is not counted into the first frame, and the runs read the same
//    pad for the window's close.
//  - THE CLIENT LISTENS IN EVERY MODE AND BOTH ORDERS, AND ANSWERS IN EVERY
//    MODE MSb FIRST (measured on this die, docs/ch32x035/spi.md: 16.2.3's
//    "mode 2 or mode 3" does not hold here, 16.3.1's "LSB only as host"
//    does - LSb first its MISO never moves). So the answer windows are
//    spi_link's mode 0, no alternate answer mode, and an exchange that asks
//    an ANSWERING client LSb first is refused (report_cfg_failed: the
//    driver's SpiClient refuses it, spi_client_answers()); a host clocking
//    LSb first against this MSb-first client asks flag_expect_reversed,
//    and spare_receive_only - MISO dark - listens LSb first.
//  - ONE TRANSMIT BUFFER IN FRONT OF ONE SHIFTER, no FIFO: an answer is
//    written on the TXE of the frame before it (16.2.3). The polled
//    windows are the driver's RUNS (SpiClient::answer, exchange,
//    receive), each one call.
//  - THE EXCHANGE IS SERVED BY THE DMA ENGINES BY DEFAULT, SPI1's receive
//    on channel 2 and its transmit on channel 3 (the channel IS the
//    request on this series), with no interrupt: the window is read off
//    the NSS pad and the receive channel's count once at its close; the
//    transmit channel's first beat lands in the
//    empty buffer before the select falls - the preload. The requests are
//    raised AROUND the channels: receive channel, RXDMAEN, transmit
//    channel, TXDMAEN, then SPE (the controller holds a raised request,
//    docs/ch32x035/dma.md). `spare_polled_pump` asks for the polled run
//    instead, `spare_answer_only` for the answer run alone.
//  - THE EXTENDED EXCHANGE IS SERVED WHOLE (spi_link.hpp): `spare_wide`
//    16-bit frames - DFF, the engines in half-word beats, the runs over
//    uint16_t; `spare_crc` on a polled run (the CRC unit armed with the
//    command's polynomial, CRCNEXT set the instant the last answer is in
//    the buffer, the host's CRC frame taken as one more frame and CRCERR
//    read once the window closes); `spare_receive_only` on the receive
//    engine alone (the polled `receive()` run with spare_polled_pump),
//    MISO not driven. Every such report raises report_extended. host_burst
//    honours spare_wide too.
//  - A RUN SAMPLES NOTHING INSIDE THE BURST, so the exchange's report
//    fields are this peer's own (spi_link.hpp leaves them to the peer's
//    header): aux0 = STATR's low byte after the burst (CRCERR bit 4, OVR
//    bit 6), aux1 = the milliseconds from the run's start to its END (255
//    = nothing came), aux2 bit 0 = the select read low once after the run,
//    bit 1 = the host closed it, bit 2 = the DMA engines served it; aux3 =
//    RXCRCR's low byte under spare_crc, else 0.
//  - THE BUFFERING REGIMES COLLAPSE, as on every F1-lineage peer:
//    regime_buffer_wait preloads the first answer with the select high
//    (rx[i] = P_B(i)); the other two start the run once the select has
//    fallen, the shifter's leftover leading and the stream one place late.
//  - ITS RATE IS ITS MISO PAD'S (docs/ch32x035/spi.md): exact both ways
//    to 6 MHz; above that the client loses SCK edges while PA6 stands
//    high. The console's f/l/h set what a receive-only serve does with
//    PA6 (floating, the protocol's; a GPIO low; a GPIO high), the
//    diagnostic that found it.
//  - THE PROTOCOL CARRIES OPTIONS THIS SILICON HAS NOT GOT: flag_wrcol and
//    flag_feed_tx name conditions chapter 16 does not have and are
//    ignored; Op::mspi asks for a USART in host-SPI mode, and no USART of
//    this package reaches these wires - answered report_cfg_failed.
//
// THE DARK LISTENER discipline: the command-mode client never drives MISO
// (drive_output = false, the pad a floating input), the answer line wakes
// only for one answer window after a frame that CHECKED OUT, an unknown op
// is dropped in silence, and a bad checksum is nak'ed only while ENGAGED.
//
// THE ROLES INVERT for Op::host_burst: SPI1 becomes the bus HOST on the
// same pads, PA4 a GPIO chip select this end drives low for the whole burst
// (one frame a request, spi_link's gap_us apart), and ss_pulse drives PA4
// low from GPIO for its microseconds, the SPI released first: the far
// host's MODF from a second chip. THAT HOST KEEPS THE DEFAULT PLACEMENT,
// its loops in the flash (util/placement.hpp): one polled frame a request,
// the gap between requests the protocol's, so the loop's frame is never
// the bound - and the client's runs place nothing.
//
// THE SUPPLY at boot (the desk's rule for an image that drives a link pad):
// the voltage detector at PLS 10, 3.0 V (pwr.hpp); below it the peer stays
// off the bus, every pad floating, and says so on its console. PB3, the
// I2C pull-up rail, is never touched.
//
// Console: USART2 on PA2/PA3 at 115200 - the probe's serial -
// observability only.
//   ? help | i status and counters | 0 back to the dark client | 3 trace
//   | f/l/h a receive-only serve's PA6 floating / a GPIO low / high
//
// The ident label is the part number after its CH32 prefix ("X035F8U6");
// ident.xtal is 0: this series runs on its HSI, no crystal.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/spi.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/spi_link.hpp"

namespace {

using namespace brio;
using spilink::Op;

using P = Ch32x035Platform<>;
using SysClock = Clock<ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

using Console = Uart<2, P>;
constexpr Console console;

/// SPI1's column 0, every signal (the header).
constexpr SpiPins link_pins = spi1_default_pins;
static_assert(link_pins.nss == Pad{'A', 4} && link_pins.sck == Pad{'A', 5} && link_pins.miso == Pad{'A', 6} &&
                  link_pins.mosi == Pad{'A', 7},
              "8.3.2.1: SPI1's code 00 is NSS PA4, SCK PA5, MISO PA6, MOSI PA7");

using Client = SpiClient<1, link_pins>;
using Host = SpiHost<1, link_pins>;
using Raw = Spi<1>;
using NssPin = Pin<'A', 4>;
using MisoPin = Pin<'A', 6>;

constexpr uint16_t firmware_version =
    static_cast<uint16_t>((spilink::peer_ch32x035 << 8) | 0x01u);   ///< spi_link.hpp's Ident

static_assert(device::part_name[0] == 'C' && device::part_name[1] == 'H' && device::part_name[2] == '3' &&
                  device::part_name[3] == '2',
              "spi_peer: the ident label is the part number after its CH32 prefix");

/// The exchange's engines at DATAR's width, bound once with NO interrupt:
/// the serve polls the receive channel's count. 8-bit frames go in byte
/// beats, 16-bit ones in half-word beats.
using PeerTx = DmaTxEngine<3, uint16_t>;
using PeerRx = DmaRxEngine<2, uint16_t>;

bool supply_ok = false;
bool trace = false;
/// The console's diagnostic of a receive-only serve's MISO pad: 0 floating
/// (the protocol's), 1 a GPIO held low, 2 a GPIO held high.
uint8_t listen_miso = 0;

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

SpiMode mode_of(uint8_t m) {
    switch (m & 0x03u) {
        case 1: return SpiMode::mode1;
        case 2: return SpiMode::mode2;
        case 3: return SpiMode::mode3;
        default: return SpiMode::mode0;
    }
}

/// DARK: mode 0, MSb first, 8 bits, MISO NOT driven, SPE up so the shifter
/// runs and the command listener polls RXNE. A client reconfigures BETWEEN
/// windows, never inside one.
bool go_dark() {
    wait_nss_high(50);
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .bits = SpiDataSize::bits8,
                                         .lsb_first = false,
                                         .nss = SpiNss::hardware_input,
                                         .drive_output = false});
    Pfic::disable(Raw::irq());   // polled: the vector stays dark
    Raw::enable();
    decoder.reset();
    return ok;
}

/// What an action asked this client to become: false when the driver
/// refuses it (an answering client LSb first, a zero CRC polynomial).
bool apply_cfg(const spilink::Cfg& c, bool wide, bool crc, uint16_t poly, bool drive) {
    wait_nss_high(50);
    const bool ok = Client::init(clock, {.mode = mode_of(c.mode),
                                         .bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8,
                                         .lsb_first = c.dord != 0,
                                         .nss = SpiNss::hardware_input,
                                         .crc = crc,
                                         .crc_polynomial = poly,
                                         .drive_output = drive});
    Pfic::disable(Raw::irq());
    return ok;
}

// ---- the runs ----------------------------------------------------------------

using Run = Client::Run;
using RunEnd = Client::RunEnd;

/// A run's spin budget: a spin is four polls and a look at the pad, some
/// thirty cycles of this core - a bound of the right order, not a clock.
constexpr uint32_t spins_for_ms(uint32_t ms) { return ms * (SysClock::hz / 30'000u); }

/// THE CRC RUN, over the resource: SpiClient::exchange()'s shape, with
/// CRCNEXT set the instant the LAST answer is in the buffer (16.2.5: "set
/// the CRCNEXT bit on the last word or half-word to transmit the CRC and
/// perform the receive CRC checksum") and one more frame taken - the
/// host's CRC. `in` holds n + 1 frames.
template <typename T>
Run crc_run(const T* out, T* in, uint16_t n, uint32_t spins) {
    SpiRegs& regs = Raw::regs();
    Raw::flush_rx();
    uint16_t w = 0;
    uint16_t r = 0;
    RunEnd how = RunEnd::complete;
    regs.DATAR = out[w++];
    if (w == n) {
        Raw::crc_next();
    }
    while (!Client::selected()) {
        if (--spins == 0u) {
            return {0, RunEnd::stalled};
        }
    }
    auto await = [&](uint16_t flag) {
        for (;;) {
            for (uint8_t k = 0; k < 4u; ++k) {
                if ((regs.STATR & flag) != 0u) {
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
    };
    while (r < static_cast<uint16_t>(n + 1u)) {
        if (w < n) {
            const T next = out[w];
            if (!await(spi_txe)) {
                break;
            }
            regs.DATAR = next;
            if (++w == n) {
                Raw::crc_next();
            }
        }
        if (!await(spi_rxne)) {
            break;
        }
        in[r++] = static_cast<T>(regs.DATAR);
    }
    if (how != RunEnd::complete && r < static_cast<uint16_t>(n + 1u) && (regs.STATR & spi_rxne) != 0u) {
        in[r++] = static_cast<T>(regs.DATAR);
    }
    return {r, how};
}

// ---- answering -------------------------------------------------------------

/// Serve ONE answer window in mode 0: the frame padded with zeros to the
/// most the host clocks (spilink::answer_bytes), the run ending when the
/// host closes the window.
void serve(const uint8_t* buf, uint8_t n) {
    wait_nss_high(50);   // the command window must close first
    if (!apply_cfg(spilink::Cfg{.apply = 1, .mode = 0}, false, false, 0, true)) {
        (void)go_dark();
        return;
    }
    uint8_t padded[spilink::answer_bytes] = {};
    for (uint8_t i = 0; i < n && i < spilink::answer_bytes; ++i) {
        padded[i] = buf[i];
    }
    Client::enable();
    (void)Client::answer(std::span<const uint8_t>(padded, spilink::answer_bytes), spins_for_ms(spilink::serve_ms));
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

// ---- the actions -----------------------------------------------------------

/// Frames an exchange carries at most; the buffers are half-words so that a
/// 16-bit run and its engines take them whole, and an 8-bit one reads the
/// same storage as bytes. One more frame in for the host's CRC.
constexpr uint16_t exchange_cap = 256;
alignas(4) uint16_t x_out[exchange_cap];
alignas(4) uint16_t x_in[exchange_cap + 1u];

uint8_t* out_bytes() { return reinterpret_cast<uint8_t*>(x_out); }
uint8_t* in_bytes() { return reinterpret_cast<uint8_t*>(x_in); }

/// The host burst's frame buffers, one 16-bit frame.
alignas(4) uint8_t burst_tx[2];
alignas(4) uint8_t burst_rx[2];

/// Judge `frames` received frames' bytes against the host's stream: count
/// the frames, sum, mism, idx, got and exp over the bytes (spi_link's
/// extended exchange).
void judge(spilink::Report& r, const spilink::Params& a, uint16_t frames, bool wide, const uint8_t* got) {
    const bool reversed = (a.flags & spilink::flag_expect_reversed) != 0;
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

/// The DMA serve: the receive channel and RXDMAEN, the transmit channel and
/// TXDMAEN - its first beat the standing TXE of an idle buffer, the preload -
/// then SPE. A receive-only serve leaves the transmit side alone. The
/// receive channel's count is the progress; the requests come down with the
/// channels.
template <typename T>
Run dma_serve(uint16_t n, bool transmit, uint32_t ms) {
    (void)PeerRx::start(std::span<T>(reinterpret_cast<T*>(x_in), n));
    Raw::dma_requests(false, true);
    if (transmit) {
        (void)PeerTx::start(std::span<const T>(reinterpret_cast<const T*>(x_out), n));
        Raw::dma_requests(true, true);
    }
    Raw::enable();
    // The WINDOW is the select pad's: the serve waits for it to fall and
    // then to rise again, reading GPIO and never the controller - the
    // channel's count is read once, at the end.
    const uint32_t t0 = Ticker::millis();
    while (!Client::selected() && Ticker::millis() - t0 < ms) {
    }
    while (Client::selected() && Ticker::millis() - t0 < ms) {
    }
    (void)PeerRx::take();
    const bool all_in = PeerRx::full();
    const uint16_t got = PeerRx::taken();
    Raw::dma_requests(false, false);
    PeerTx::stop();
    PeerRx::stop();
    return {got, all_in ? RunEnd::complete : RunEnd::stalled};
}

/// The polled serves, one run each, at the frame's width.
template <typename T>
Run polled_serve(uint16_t n, bool answer_only, bool receive_only, bool crc, uint32_t spins) {
    T* const out = reinterpret_cast<T*>(x_out);
    T* const in = reinterpret_cast<T*>(x_in);
    if (crc) {
        Raw::enable();
        return crc_run<T>(out, in, n, spins);
    }
    if (receive_only) {
        Raw::enable();
        Raw::flush_rx();
        return Client::receive(std::span<T>(in, n), spins);
    }
    Client::enable();
    if (answer_only) {
        return Client::answer(std::span<const T>(out, n), spins);
    }
    return Client::exchange(std::span<const T>(out, n), std::span<T>(in, n), spins);
}

/// The workhorse: the host sends P_A(i), this client answers P_B(i) - on the
/// DMA engines by default, the polled run with spare_polled_pump, the answers
/// alone with spare_answer_only, the frames in alone with spare_receive_only;
/// 16-bit frames with spare_wide, the CRC with spare_crc.
spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    r.flags |= spilink::report_extended;
    const bool wide = (a.spare & spilink::spare_wide) != 0;
    const bool crc = (a.spare & spilink::spare_crc) != 0;
    const bool receive_only = (a.spare & spilink::spare_receive_only) != 0;
    const bool answer_only = !receive_only && (a.spare & spilink::spare_answer_only) != 0;
    const bool preload = a.cfg.regime == spilink::regime_buffer_wait;
    const uint16_t n = a.count < exchange_cap ? a.count : exchange_cap;
    if (n == 0u || (crc && receive_only) || !apply_cfg(a.cfg, wide, crc, a.aux16, !receive_only)) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    if (receive_only && listen_miso != 0u) {
        MisoPin::output(listen_miso == 2u);   // the console's diagnostic
    }
    {
        spilink::Stream out(a.pattern, a.seed_b);
        uint8_t* const o = out_bytes();
        for (uint16_t i = 0; i < static_cast<uint16_t>(wide ? 2u * n : n); ++i) {
            o[i] = out.next();
        }
        uint8_t* const in = in_bytes();
        for (uint16_t i = 0; i < sizeof x_in; ++i) {
            in[i] = 0;
        }
    }
    const bool engines = (preload || receive_only) && !answer_only && !crc &&
                         (a.spare & spilink::spare_polled_pump) == 0;

    const uint32_t t0 = Ticker::millis();
    Run run{};
    if (engines) {
        run = wide ? dma_serve<uint16_t>(n, !receive_only, a.ms) : dma_serve<uint8_t>(n, !receive_only, a.ms);
        r.aux2 |= 0x04;
    } else {
        if (!preload && !receive_only) {
            // NO PRELOAD: the run starts once the select has fallen, so the
            // first frame the host clocks out is the shifter's leftover and
            // this end's stream follows ONE PLACE LATE.
            const uint32_t w0 = Ticker::millis();
            while (!Client::selected() && Ticker::millis() - w0 < a.ms) {
            }
        }
        const uint32_t spins = spins_for_ms(a.ms);
        run = wide ? polled_serve<uint16_t>(n, answer_only, receive_only, crc, spins)
                   : polled_serve<uint8_t>(n, answer_only, receive_only, crc, spins);
    }
    const uint32_t dt = Ticker::millis() - t0;
    // The select's close, then the status: CRCERR is decided on the host's
    // CRC frame, the last of the window.
    wait_nss_high(20);
    const uint16_t st = Raw::status();
    r.aux0 = static_cast<uint8_t>(st);
    r.aux1 = run.count == 0u ? 255u : (dt > 254u ? 254u : static_cast<uint8_t>(dt));
    if (Client::selected()) r.aux2 |= 0x01;
    if (run.end == RunEnd::closed) r.aux2 |= 0x02;
    if ((st & spi_ovr) != 0u) r.flags |= spilink::report_bufovf;
    if (crc) {
        r.aux3 = static_cast<uint8_t>(Raw::rx_crc());
        if ((st & spi_crcerr) != 0u) r.flags |= spilink::report_crcerr;
    }

    if (answer_only) {
        r.count = run.count;   // answers written; nothing was received
    } else {
        // The CRC frame is not counted: `count` data frames, then the host's.
        const uint16_t frames = run.count > n ? n : run.count;
        judge(r, a, frames, wide, in_bytes());
    }
    if (r.count < a.count) r.flags |= spilink::report_timed_out;
    (void)go_dark();
    return r;
}

/// The loss semantics: DATAR is never read for the whole burst, so the
/// receive buffer holds one frame and OVR stands for every frame after it.
/// The status is OR'ed DURING the burst, what the buffer retained drained at
/// the end. The burst's end is the deadline the command carries.
spilink::Report run_sink_slow(const spilink::Params& a) {
    spilink::Report r{};
    if (!apply_cfg(a.cfg, false, false, 0, true)) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    Client::enable(0x00);   // something harmless to shift out
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

/// The second driver on the shared select wire: PA4 low from GPIO for aux16
/// microseconds (1000 by default), the SPI handed back first, so nothing of
/// this board is on the other three wires.
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

/// THE ROLES INVERT: this end becomes the bus HOST for a bounded burst. The
/// ack is already served, both boards count the same lead-in, and at its end
/// the DUT is a client with its first answer preloaded. PA4 is a GPIO driven
/// LOW for the whole burst, SpiHost never claiming that pad; one frame a
/// polled request, gap_us apart. aux16 is "the instrument's SCK division of
/// its own clock": the BR code producing AT MOST HCLK/aux16. spare_wide
/// clocks 16-bit frames, the streams taken two bytes a frame.
spilink::Report run_host_burst(const spilink::Params& a) {
    spilink::Report r{};
    r.flags |= spilink::report_extended;
    const bool wide = (a.spare & spilink::spare_wide) != 0;
    Client::release();
    NssPin::output(true);   // deselected before anything moves

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
    Pfic::disable(Raw::irq());   // every request below is polled
    const SpiClock rate = *code;
    const SpiDataSize bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    r.aux1 = static_cast<uint8_t>(static_cast<uint8_t>(rate));   // the BR code itself
    (void)Host::bit_order(a.cfg.dord != 0);
    // The mode is primed BEFORE the select falls: a CPOL flip inside an open
    // window is one extra edge and the selected client counts it.
    Host::prime(mode_of(a.cfg.mode), rate, bits);
    hold_us(200);

    spilink::Stream out(a.pattern, a.seed_a);   // what THIS end sends
    uint8_t got[2 * exchange_cap];
    uint16_t frames = 0;
    const uint16_t cap = a.count < exchange_cap ? a.count : exchange_cap;

    NssPin::clear();   // select, for the whole burst
    hold_us(spilink::gap_us);
    while (frames < cap && Ticker::millis() - t0 < lead + a.ms) {
        burst_tx[0] = out.next();
        burst_tx[1] = wide ? out.next() : 0u;
        burst_rx[0] = 0;
        burst_rx[1] = 0;
        Host::Request req{};
        req.cs = {};   // the burst owns the select
        req.tx = lend<Lease::reply>(static_cast<const uint8_t*>(burst_tx));
        req.rx = lend<Lease::reply>(static_cast<uint8_t*>(burst_rx));
        req.len = 1;
        req.clock = rate;
        req.mode = mode_of(a.cfg.mode);
        req.bits = bits;
        req.polled = true;
        if (!Host::start(req)) {
            break;   // a polled request completes here
        }
        got[wide ? 2u * frames : frames] = burst_rx[0];
        if (wide) {
            got[2u * frames + 1u] = burst_rx[1];
        }
        ++frames;
        hold_us(spilink::gap_us);
    }
    NssPin::set();
    // What came back is the DUT's seed_b stream: judged as a client's
    // report judges the host's (judge() reads seed_a, so swap the seeds).
    spilink::Params back = a;
    back.seed_a = a.seed_b;
    back.flags = 0;
    judge(r, back, frames, wide, got);
    r.aux0 = static_cast<uint8_t>(Raw::status());
    if (r.count < a.count) r.flags |= spilink::report_timed_out;

    // Whatever happened, back to the dark client; the pads change hands under
    // the GPIO-held high select.
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
        d.xtal = 0;
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
        // Op::mspi asks for a USART in host-SPI mode with no select; no USART
        // of this package reaches these wires.
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

/// The desk's supply rule for an image that drives a link pad: the voltage
/// detector at PLS 10, 3.0 V falling (pwr.hpp), given 200 us before PVD0 is
/// trusted - the manual states no settling time.
bool supply_above_floor() {
    Pwr::pvd(PvdLevel::v3_0);
    (void)delay_us(clock, 200);
    return !Pwr::supply_low();
}

void help() {
    print(console, "spi_peer: ? help | i status | 0 back to the dark client | 3 trace | f/l/h a receive-only "
                   "serve's MISO floating (the protocol's) / a GPIO low / high",
          crlf);
}

void status() {
    print(console, "  client SPI1 column 0, enabled=", Raw::enabled(), " STATR=", hex(Raw::status()), crlf);
    print(console, "  NSS PA4 reads ", NssPin::read() ? "high" : "LOW", ", selected=", Client::selected(),
          ", supply ", supply_ok ? "above 3.0 V" : "LOW", crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks, " served=", serves,
          engaged() ? "  (ENGAGED)" : "  (dark, mute)", crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count, " mism=", last_report.mism,
          " sum=", hex(last_report.sum), " flags=", hex(last_report.flags), " idx=", last_report.idx,
          " got=", hex(last_report.got), " exp=", hex(last_report.exp), crlf);
    print(console, "               aux=", hex(last_report.aux0), " ", hex(last_report.aux1), " ",
          hex(last_report.aux2), " ", hex(last_report.aux3), " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Console::isr(); }

/// SPI1's line carries nothing - every window is a polled run or the
/// engines - and both tasks' init() open it at the PFIC, so a stray level
/// is DISARMED rather than looped on.
extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    Raw::rxne_interrupt(false);
    Raw::txe_interrupt(false);
    Raw::error_interrupt(false);
}
/// The engines are bound with no interrupt; a stray line is cleared.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() { (void)PeerRx::service(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { (void)PeerTx::service(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = Ticker::init(clock);
    enable_interrupts();

    // THE SUPPLY before any link pad is driven (the desk's rule).
    supply_ok = supply_above_floor();

    // The engines, bound once and with no interrupt: the binding names DATAR,
    // nothing an SPI re-init touches. Receive at very_high, transmit at high
    // (docs/design/dma.md).
    PeerTx::arm(Raw::data_address(), 0u, DmaPriority::high);
    PeerRx::arm(Raw::data_address(), 0u, DmaPriority::very_high);

    char label[9] = {};
    ident_label(label);
    if (supply_ok) {
        (void)go_dark();
    }

    print(console, crlf, "spi_peer - SPI instrument client (", device::part_name, ", label '", label, "', fw ",
          hex(firmware_version), ")", crlf);
    print(console, "client SPI1 column 0: NSS PA4 (the hardware select, pulled up), SCK PA5, MISO PA6, MOSI PA7; "
                   "command mode = SPI mode 0, MSb first; answers in mode 0",
          crlf);
    print(console, "DARK by default: MISO is driven only for one answer window, after a frame that checked out", crlf);
    print(console, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=", tick_ok ? "STK" : "FAILED",
          " console=", console_ok ? "USART2" : "FAILED", " supply=", supply_ok ? "above 3.0 V" : "LOW", crlf);
    if (!supply_ok) {
        print(console, "SUPPLY UNDER 3.0 V (the PVD at PLS 10): the link pads stay floating", crlf);
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
                } else if (c == 'f' || c == 'l' || c == 'h') {
                    listen_miso = c == 'f' ? 0u : (c == 'l' ? 1u : 2u);
                    print(console, "  a receive-only serve's MISO: ", c == 'f' ? "floating" : (c == 'l' ? "GPIO low" : "GPIO high"), crlf);
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
