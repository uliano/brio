// spi_peer - a scriptable SPI CLIENT for the other end of a two-board
// bus test: the far board is the bus host and the DUT (test_stm32_spi)
// and drives this one IN BAND over the very bus both are testing, every
// command travelling as a checksummed frame in the same traffic.
//
// The wire format is spi_link.hpp, included by relative path: one file
// is the single source of truth for every board that speaks it,
// whatever its architecture. What this silicon makes of it:
//
//  - THE PUMP RUNS ONE AHEAD, and here it is the FIFO that says so
//    rather than a documented latency: a client answering on RXNE
//    writes its next frame AFTER the frame in flight has taken whatever
//    the TXFIFO held, so an answer written in the gap is one character
//    late. 35.5.8 is explicit that "the data register of the slave must
//    already contain data to be sent before starting communication with
//    the master", so the opening is TWO preloads before the host's clock
//    can arrive (SpiClient::enable(a, b)) and every received frame loads
//    the next-PLUS-ONE - the same shape test_stm32_spi's own
//    `peer_service()` runs.
//
//  - THE PUMP RUNS ON THE SPI1 INTERRUPT, not in a polled loop. Only the
//    COMMAND listener is polled (a command character is 32 us at
//    PCLK/256 and the receive FIFO is four frames deep, so the loop has
//    slack); an answer window and the software-pump exchange arm RXNE.
//    So on this peer `spilink::spare_polled_pump` means "the SOFTWARE
//    reload on the RXNE interrupt" as against the DMA engines - the bit
//    names the software path, and that is what it selects here.
//
//  - THE BUFFERING REGIMES COLLAPSE. There is no BUFEN/BUFWR pair in
//    ch. 35: this client is always FIFO-buffered and the two preloads
//    play BUFWR's role. spilink::Cfg::regime is therefore MAPPED, not
//    translated: regime_buffer_wait (the only regime this link ever
//    commands) runs the preloaded one-ahead pump and delivers the exact
//    rx[i] = P_B(i) alignment that regime promises; the other two run
//    WITHOUT the preloads, which on this silicon means the shifter's
//    leftover leads - stated here, exercised nowhere.
//
//  - THE PROTOCOL CARRIES OPTIONS THIS SILICON HAS NOT GOT. flag_wrcol
//    and flag_feed_tx name conditions chapter 35 does not have and are
//    ignored; Op::mspi asks for a USART in host-SPI mode and is answered
//    report_cfg_failed - this stratum's USART has no such mode.
//    Everything else of the repertoire is served: ping/ident/report,
//    exchange, sink_slow, ss_pulse, host_burst.
//
//  - THE ANSWER LINE'S EDGE RATE IS THIS APP'S, NOT THE DRIVER'S.
//    SpiClient hands MISO to the peripheral at VERY-HIGH speed, and on
//    this jumper-wire bus that edge is an AGGRESSOR: with MISO at
//    very-high the DUT's four-mode matrix slips in 20 bursts of 160 -
//    byte-exact for its first frames, then BOTH ENDS DISAGREE FROM THE
//    SAME FRAME ON, the rest of the stream one bit out of place - and
//    every one of the twenty is mode 1 or mode 2, THE TWO MODES THAT
//    SAMPLE ON THE FALLING EDGE, where the data lines change on the
//    RISING edge: this client's MISO edge lands, its own propagation
//    delay after the clock edge, on an SCK that has just settled HIGH,
//    and a fast falling edge coupled into the clock wire beside it dips
//    it through VIL - a spurious falling edge, i.e. one sampling edge
//    too many. In modes 0 and 3 the same edge lands on a LOW clock and
//    nothing happens (0 of 80). One notch down, at HIGH, the slip is
//    gone - 0 of 120 - and the ladders are UNCHANGED (the DMA serve
//    exact to PCLK/2 = 32 MHz, the software pump to 16 MHz); medium
//    costs the top rung (16 MHz) and low two (8 MHz), on both serves. So
//    this app re-states the pad at HIGH after every init that drives it
//    (console 's' cycles the four speeds). The DUT-side pads keep the
//    driver's very-high: only the slave's edge lags the clock.
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
//   ? help | i status and counters | 0 back to the dark client | 3 trace
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
#include "../../../avrdx/src/apps/spi_link.hpp"

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

constexpr uint16_t firmware_version = 0x0300;   ///< see spi_link.hpp's Ident

/// THE ANSWER LINE'S EDGE RATE (see the header): HIGH, one notch
/// below the driver's very-high, which is where the falling-edge slip
/// stops with nothing of the ladder lost. init() rewrites the pad, so the
/// choice is re-stated after every init that drives the output; the
/// console's 's' cycles the four speeds for the bench.
PinSpeed miso_speed = PinSpeed::high;

const char* speed_name(PinSpeed s) {
    switch (s) {
        case PinSpeed::low: return "low";
        case PinSpeed::medium: return "medium";
        case PinSpeed::high: return "high";
        default: return "very_high";
    }
}

void restate_miso_speed() {
    if (MisoPin::has_function()) {
        MisoPin::function(link_pins.miso.function, {.speed = miso_speed});
    }
}

/// The exchange's DMA engines, on DMA1 channels 1 and 2 - the two
/// vectors this app binds. Armed at the start of every engined exchange
/// (the SPI re-inits under them freely: a claim binds a DATA address and
/// a DMAMUX request, not a configuration).
using PeerTx = DmaTxEngine<1, 1>;
using PeerRx = DmaRxEngine<1, 2>;
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

// ---- the one-ahead pump, on SPI1's own interrupt --------------------------------
//
// One pump serves both the answer windows and the software-pump
// exchange: the answers are precomputed, the received frames are only
// STORED, and every judgement happens after the burst - a reload path
// of a few register accesses is what a burst at rate needs.

constexpr uint16_t pump_cap = 64;

uint16_t pump_out[pump_cap];
volatile uint8_t pump_in[pump_cap];
volatile uint16_t pump_out_n = 0;   ///< how many of pump_out are meaningful
volatile uint16_t pump_tx_n = 0;
volatile uint16_t pump_rx_n = 0;
uint16_t pump_filler = 0x00;
volatile uint32_t pump_err = 0;
volatile bool pump_live = false;

uint16_t pump_next_answer() {
    const uint16_t i = pump_tx_n;
    pump_tx_n = static_cast<uint16_t>(i + 1u);
    return i < pump_out_n ? pump_out[i] : pump_filler;
}

void pump_service() {
    const uint32_t f = Client::isr();
    if (f == 0u) {
        return;
    }
    if ((f & SpiFlag::rxne) != 0u) {
        const uint16_t v = Client::poll().value_or(0);
        if (pump_rx_n < pump_cap) {
            pump_in[pump_rx_n] = static_cast<uint8_t>(v);
        }
        pump_rx_n = static_cast<uint16_t>(pump_rx_n + 1u);
        Client::write(pump_next_answer());
        return;
    }
    // An error, and only where ERRIE was deliberately armed: record it
    // ONCE and disarm, because every one of these flags is a LEVEL and a
    // handler that leaves one standing re-enters for ever.
    pump_err = pump_err | (f & SpiFlag::errors);
    Client::error_interrupt(false);
}

/// Load the answers and open the window: RXNE armed, SPE up, and the
/// FIRST TWO ANSWERS in the FIFO before the host's clock can arrive.
/// Call it with NSS still high.
void pump_arm(const uint16_t* answers, uint16_t n, uint16_t filler) {
    Nvic::disable(Raw::irq());
    pump_tx_n = 0;
    pump_rx_n = 0;
    pump_err = 0;
    pump_filler = filler;
    pump_out_n = n < pump_cap ? n : pump_cap;
    for (uint16_t i = 0; i < pump_out_n; ++i) {
        pump_out[i] = answers[i];
    }
    pump_live = true;
    Client::rxne_interrupt(true);
    Nvic::enable(Raw::irq());
    // THE TWO PRELOADS IN ORDER, and they are two statements on purpose:
    // the evaluation order of two calls in one argument list is
    // unspecified, and here it would decide which byte leaves first.
    const uint16_t first = pump_next_answer();
    const uint16_t second = pump_next_answer();
    Client::enable(first, second);
}

void pump_stop() {
    pump_live = false;
    Client::rxne_interrupt(false);
    Client::error_interrupt(false);
}

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
    pump_stop();
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
    pump_stop();
    const bool ok = Client::init(clock, {.mode = SpiMode::mode0,
                                         .bits = SpiDataSize::bits8,
                                         .lsb_first = false,
                                         .nss = SpiNss::hardware_input,
                                         .drive_output = true});
    hold_nss_up();
    restate_miso_speed();
    return ok;
}

/// What a command asked this client to become. See the header: only
/// regime_buffer_wait has an exact counterpart on this silicon, and it
/// is the one the G0 suite commands.
///
/// `nss` is a parameter because the alternative was TRIED AND MEASURED
/// (see run_exchange): everything here frames on the NSS pad.
bool apply_cfg(const spilink::Cfg& c, bool drive_output = true,
               SpiNss nss = SpiNss::hardware_input) {
    pump_stop();
    wait_nss_high(50);   // see go_dark(): a client reconfigures BETWEEN windows
    const bool ok = Client::init(clock, {.mode = mode_of(c.mode),
                                         .bits = SpiDataSize::bits8,
                                         .lsb_first = c.dord != 0,
                                         .nss = nss,
                                         .drive_output = drive_output});
    hold_nss_up();
    restate_miso_speed();
    return ok;
}

// ---- answering ------------------------------------------------------------------

void wait_nss_high(uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Client::selected() && Ticker::millis() - t0 < ms) {
    }
}

/// Serve ONE answer window with the one-ahead pump: buf[0] and buf[1]
/// into the FIFO before the select falls, and every received frame loads
/// the next one. The host clocks spilink::answer_bytes dummies; the tail
/// pads with zeros, which the decoder ignores.
void serve(const uint8_t* buf, uint8_t n) {
    wait_nss_high(50);                    // the command window must close first
    if (!arm_answer()) {
        (void)go_dark();
        return;
    }
    uint16_t answers[pump_cap];
    const uint16_t count = n < pump_cap ? n : pump_cap;
    for (uint16_t i = 0; i < count; ++i) {
        answers[i] = buf[i];
    }
    pump_arm(answers, count, 0x00);
    const uint32_t t0 = Ticker::millis();
    while (pump_rx_n < spilink::answer_bytes &&
           Ticker::millis() - t0 < spilink::serve_ms) {
    }
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

struct Streams {
    spilink::Stream in{};
    spilink::Stream out{};
    bool reversed = false;

    explicit Streams(const spilink::Params& a)
        : in(a.pattern, a.seed_a),
          out(a.pattern, a.seed_b),
          reversed((a.flags & spilink::flag_expect_reversed) != 0) {}

    uint8_t next_expected() {
        const uint8_t v = in.next();
        return reversed ? spilink::bit_reverse(v) : v;
    }
};

// ---- the actions ----------------------------------------------------------------

constexpr uint16_t exchange_cap = pump_cap;
uint16_t x_out[exchange_cap];
uint8_t x_in[exchange_cap];
uint8_t x_dma_out[exchange_cap];

/// The host burst's per-frame buffers, TWO bytes although the burst is
/// 8-bit: SpiHost's frame store is width-general and writes a half-word
/// for a frame above eight bits, which the compiler's bounds check sees
/// before it sees the width.
uint8_t burst_tx[2];
uint8_t burst_rx[2];

/// The workhorse: the host sends P_A(i), this client answers P_B(i).
///
/// TWO SERVES, so both boundaries stay measurable. THE DEFAULT IS THE
/// DMA ENGINES, the arrangement that measures the self-link exact to
/// PCLK/4 with both ends engined: the peripheral disabled, TXDMAEN and
/// RXDMAEN set, both channels claimed and started, and SPE raised LAST
/// so the transmit channel's own first beats ARE the preload (a request
/// is a level this controller serves on enable). The receive block's
/// completion
/// is the transaction's. `spilink::spare_polled_pump` asks for the
/// SOFTWARE pump instead - the one-ahead reload on RXNE - whose
/// boundary is an interrupt entry per frame.
spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    const bool preload = a.cfg.regime == spilink::regime_buffer_wait;
    const uint16_t n = a.count < exchange_cap ? a.count : exchange_cap;
    // THE NSS PAD FRAMES THE BURST, and the alternative was MEASURED and
    // is worse: with SSM/SSI (the frame boundary on the clock alone)
    // this client counts the host's own CPOL settling edge - the one a
    // host makes when it primes CPOL = 1 with the select still high -
    // and every frame is then one bit early, five runs of five. A client
    // that frames on the pad ignores it, which is what the pad is for.
    // What the pad does NOT cure is the falling-edge slip in the header.
    if (!apply_cfg(a.cfg)) {
        r.flags |= spilink::report_cfg_failed;
        return r;
    }
    {
        spilink::Stream out(a.pattern, a.seed_b);
        for (uint16_t i = 0; i < n; ++i) {
            const uint8_t v = out.next();
            x_out[i] = v;
            x_dma_out[i] = v;
        }
    }

    if (dma_ok && preload && (a.spare & spilink::spare_polled_pump) == 0u) {
        pump_stop();
        (void)Client::disable();
        (void)Raw::dma_transmit(true);
        (void)Raw::dma_receive(true);
        PeerRx::arm(Raw::data_address(), Raw::dma_rx_request());
        PeerTx::arm(Raw::data_address(), Raw::dma_tx_request());
        (void)PeerRx::start(x_in, n);
        (void)PeerTx::start(x_dma_out, n);
        Raw::enable();

        const uint32_t t0 = Ticker::millis();
        while (!PeerRx::full() && Ticker::millis() - t0 < a.ms) {
            (void)PeerRx::take();
        }
        (void)PeerRx::take();
        const bool all_in = PeerRx::full();
        const uint16_t got = PeerRx::taken();
        PeerTx::stop();
        PeerRx::stop();
        r.aux0 = static_cast<uint8_t>(Client::status());
        r.aux1 = all_in ? 0u : 255u;
        r.aux2 = 0x04;                    // marks the DMA serve in the report
        Streams s(a);
        for (uint16_t i = 0; i < got && i < n; ++i) {
            account(r, x_in[i], s.next_expected());
        }
        if (r.count < a.count) r.flags |= spilink::report_timed_out;
        if (Client::overrun()) r.flags |= spilink::report_bufovf;
        (void)go_dark();
        return r;
    }

    // The SOFTWARE pump: the two preloads, then one reload per received
    // frame. Without the preloads (a regime this suite never commands)
    // the shifter's leftover leads, which is stated in the header.
    pump_arm(x_out, preload ? n : 0u, preload ? 0x00u : 0xFFu);
    r.aux3 = static_cast<uint8_t>(Client::status());   // SR right after the preloads
    const uint32_t t0 = Ticker::millis();
    while (pump_rx_n < n && Ticker::millis() - t0 < a.ms) {
    }
    const uint16_t got = pump_rx_n < exchange_cap ? pump_rx_n : exchange_cap;
    r.aux0 = static_cast<uint8_t>(Client::status());
    r.aux1 = got == 0u ? 255u : 0u;
    if (Client::overrun()) r.flags |= spilink::report_bufovf;

    Streams s(a);
    for (uint16_t i = 0; i < got; ++i) {
        account(r, pump_in[i], s.next_expected());
    }
    if (r.count < a.count) r.flags |= spilink::report_timed_out;
    if (Client::selected()) r.aux2 |= 0x02;
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
    pump_stop();
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
    pump_stop();
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

    pump_stop();
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
                   "s cycle the MISO pad speed",
          crlf);
}

void status() {
    print(console, "  client SPI1 AF0, enabled=", Raw::enabled(),
          " role=", Raw::role() == SpiRole::client ? "client" : "HOST",
          " mode=", static_cast<uint8_t>(Raw::mode()), " SR=", hex(Raw::status()), crlf);
    print(console, "  NSS pin reads ", NssPin::read() ? "high" : "LOW",
          ", selected=", Client::selected(),
          ", MISO driven=", MisoPin::has_function(),
          ", MISO pad speed=", speed_name(miso_speed), crlf);
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

/// SPI1's line, and the one-ahead pump is the only thing on it. A stray
/// level with no window open is DISARMED rather than looped on.
extern "C" void SPI1_IRQHandler() {
    if (pump_live) {
        pump_service();
        return;
    }
    Raw::rxne_interrupt(false);
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

    brio::Dma<1>::bus_clock(true);
    dma_ok = true;

    // The select pad readable (and held up) before the first go_dark():
    // its wait for a released select reads the pad, and a pad still in
    // its analog reset state has no input buffer to read.
    NssPin::input(PinPull::up);

    print(console, crlf, "spi_peer - SPI instrument client (STM32G0 board ", label8,
          ", xtal=none (no HSE fitted; the HSE root is unbuilt), fw ",
          hex(firmware_version), ")", crlf);
    print(console, "client SPI1 AF0: SCK PB3, MISO PB4 (pad speed ",
          speed_name(miso_speed), "), MOSI PB5, NSS PA15 (pulled up); command mode "
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
                    miso_speed = miso_speed == PinSpeed::high     ? PinSpeed::medium
                                 : miso_speed == PinSpeed::medium ? PinSpeed::low
                                 : miso_speed == PinSpeed::low    ? PinSpeed::very_high
                                                                  : PinSpeed::high;
                    print(console, "  MISO pad speed now ", speed_name(miso_speed),
                          " (applied at the next answer window; high is the "
                          "default, very_high slips the falling-edge samplers)",
                          crlf);
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
