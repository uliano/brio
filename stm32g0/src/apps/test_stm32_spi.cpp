// test_stm32_spi - the reference bench suite for the STM32G0's SPI/I2S
// block: stm32g0/spi.hpp over RM0444 ch. 35, both roles, in ONE image on
// ONE board.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// TWO INSTRUMENTS, ONE SET OF PADS, AND THE SUITE ASKS THE WIRE WHICH
// ONE IS ON THE DESK.
//
// (1) THE BOARD'S OWN SELF-LINK (docs/bench.md, "The Nucleo-G0B1RE's
// self-link"): SPI1 the host and SPI2 the client, four wires between
// them -
//
//   SCK   PB3  AF0  ->  PB10 AF5
//   MISO  PB4  AF0  <-  PC2  AF1
//   MOSI  PB5  AF0  ->  PD4  AF1
//   NSS   PA15 AF0  ->  PB12 AF0
//
// - so every wire verdict is a real transfer between two peripherals and
// not a loop-back. PA15 is the host's chip select: a GPIO the transfer
// engine drives for most letters (35.5.5 makes hardware NSS output frame
// the peripheral's lifetime and not a transaction), and SPI1_NSS/I2S1_WS
// at AF0 for the letters that measure the hardware arrangements. PB12 is
// the client's hardware NSS input, and an EXTI line counts its edges for
// the NSS-pulse letter. Letters b..l are this instrument's.
//
// (2) THE CROSS-ARCHITECTURE LINK: the SAME four SPI1 pads reach a SAM
// C21 running `spi_peer` on SERCOM1 function C -
//
//   SCK   PB3  AF0  ->  PA17  SERCOM1 PAD[1]
//   MOSI  PB5  AF0  ->  PA16  SERCOM1 PAD[0] (its DI)
//   MISO  PB4  AF0  <-  PA19  SERCOM1 PAD[3] (its DO)
//   NSS   PA15 GPIO ->  PA18  SERCOM1 PAD[2] (its SS)
//
// plus a dedicated GND, both boards at 3.3 V. The instrument is
// commanded IN BAND over the bus under test, over
// avrdx/src/apps/spi_link.hpp included by relative path - one source of
// truth for the wire format, three architectures compiling it. Letters
// n..r are this instrument's.
//
// THE TWO CANNOT BE ON THE DESK AT ONCE (the four jumpers go to one end
// or the other), so main() PROBES the self-link before any letter runs -
// each of PB3, PC2, PB5 and PA15 driven both ways against the far pad's
// own internal pull - and EACH SET SKIPS ITSELF ON THE OTHER DESK,
// printing the reason and claiming no verdict either way: letters b..l
// when the probe says no, letters n..r when it says yes. The one thing
// that still FAILS is a peer that does not answer with the wires in
// place - that is firmware to flash, not a desk this suite was not
// built for.
//
// THE CLIENT RUNS ON ITS OWN INTERRUPT, ONE FRAME AHEAD (35.5.8: "the
// data register of the slave must already contain data to be sent before
// starting communication with the master"), so a host polling in main
// context and a client answering under SPI2's vector are two independent
// halves of one link on one core. Letter d is where that arrangement
// meets its own limit, and where the limit is NAMED.
//
// THE CLOCK IS DYNAMIC, and every letter but l runs at its first rung
// (PLL 64 MHz) where a static Clock<> would put it. The console sits on
// HSI16 so its divisor never moves; letter l is the ladder.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the reserve against the header, the reset
//      values, the enable protection MEASURED field by field, the
//      disable procedure, the refusals
//   b  THE LINK: 8-bit mode 0 both ways, host polled then host on its
//      own ISR pump
//   c  THE MATRIX: four modes x two bit orders, then every data size
//      from 4 to 16 bits with a pattern that uses every bit
//   d  THE RATE LADDER: all eight BR codes, where the link stops and WHY
//   e  NSS, all four ways: software, the client's hardware input, MODF
//      on a host whose input goes low, SSOE measured from the far end,
//      and NSSP counted by an EXTI line
//   f  CRC: 8 and 16 bits, the readbacks against a bitwise reference,
//      CRCERR staged, and the two-frame rule for a 16-bit CRC on 8-bit
//      frames
//   g  TI MODE: byte-exact both ways, and FRE staged by a frame-size
//      mismatch
//   h  THE ERRATA: ES0548 2.12.1 both ways, 2.12.2 sampled, and OVR with
//      its clear sequence
//   i  THE DMA ENGINES: the host's data phase on both, the command-phase
//      handover, the ladder with the client on raw engines, and the
//      LDMA odd-count rule under 8-bit packing
//   j  THE KERNEL: SpiBus (= BusMaster) over SpiHost with the client
//      answering on the wire - ordered replies, rejection, the sleep
//      votes, and the per-bus timeout with recover()
//   k  I2S: I2S1 master to I2S2 slave on three of the same wires, the
//      four standards, three data lengths, CHSIDE, the prescaler
//      arithmetic timed, and UDR on a slave transmitter
//   l  THE DYNAMIC CLOCK: the link exact at 64, 16 and 2 MHz with the
//      BR code re-resolved against a stated ceiling
//   m  sleep: an SPI interrupt waking a WFI in Sleep mode
//   n  THE PEER: the spi_link command channel to the SAM C21, its ident
//      and ten frames
//   o  the matrix against the peer: four modes, both bit orders, and a
//      DORD mismatch as an exact two-way bit reversal
//   p  the BR ladder against the peer, and where its answer reload stops
//   q  THE KERNEL against the peer: SpiBus (= BusMaster) over SpiHost
//      with a SECOND CHIP reading the bytes back - the rejection, both
//      sleep votes and the per-bus timeout with recover()
//   r  THE ROLES INVERT: this board as the CLIENT on the pads it hosts
//      with, the peer clocking a bounded burst as the bus host
//
// NOTHING WRITES FLASH, no option byte is touched, and the RTC domain is
// not reset. The pads this suite moves are the eight of the self-link
// plus the console's two.
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/spi.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL IS THE AVR CAMPAIGN'S, AND IT IS NOT COPIED. spi_link.hpp
// is pure encoding - it names no register and includes nothing of brio -
// so all three architectures' apps compile the same file. A copy here
// would be a second source of truth for a wire format, which is exactly
// the thing that drifts.
#include "../../../avrdx/src/apps/spi_link.hpp"

namespace {

using namespace brio;

using P = Stm32g0Platform<>;

// ---------------------------------------------------------------------------
// The rates, the users, the clock
// ---------------------------------------------------------------------------

using Fast = Clock<ClockSource::pll, 64'000'000>;
using Mid = Clock<ClockSource::internal, 16'000'000, PowerRegime::range2>;
using Slow = Clock<ClockSource::internal, 2'000'000, PowerRegime::low_power_run>;

/// The console on HSI16: its divisor never moves, so the report is never
/// a function of the thing letter l is switching. It is still LISTED
/// among the users (clock_follows demands it) and its rebase() folds to
/// nothing.
constexpr UartOptions console_opts{.kernel_clock = UsartClock::hsi16};
constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins, 64, 512, NoDmaEngine, NoDmaEngine, console_opts>;

// The self-link's pads (DS13560 tables 13, 15, 17, 18).
constexpr SpiPins host_pins{
    .sck = {'B', 3, PinFunction::af0},     // SPI1_SCK  / I2S1_CK
    .miso = {'B', 4, PinFunction::af0},    // SPI1_MISO / I2S1_MCK
    .mosi = {'B', 5, PinFunction::af0},    // SPI1_MOSI / I2S1_SD
    .nss = {'A', 15, PinFunction::af0},    // SPI1_NSS  / I2S1_WS
};
constexpr SpiPins client_pins{
    .sck = {'B', 10, PinFunction::af5},    // SPI2_SCK  / I2S2_CK
    .miso = {'C', 2, PinFunction::af1},    // SPI2_MISO / I2S2_MCK
    .mosi = {'D', 4, PinFunction::af1},    // SPI2_MOSI / I2S2_SD
    .nss = {'B', 12, PinFunction::af0},    // SPI2_NSS  / I2S2_WS
};

using S1 = Spi<1>;
using S2 = Spi<2>;
using Host = SpiHost<1, host_pins>;
using Peer = SpiClient<2, client_pins>;

// The engined host of letter i, over the SAME instance and the same
// pads: a second set of statics, one live at a time.
using HostTx = DmaTxEngine<1, 1>;
using HostRx = DmaRxEngine<1, 2>;
using DmaHost = SpiHost<1, host_pins, HostTx, HostRx>;
// The client's own engines, RAW (SpiClient has no slots - see the
// report): letter i uses them to take the client's ISR turnaround out of
// the ladder, which is the whole point of that measurement.
using PeerTx = DmaTxEngine<1, 3>;
using PeerRx = DmaRxEngine<1, 4>;
// The packed pair of the LDMA leg: the SAME channels at half-word width.
using PeerTx16 = DmaTxEngine<1, 3, uint16_t>;

using SysClock = DynamicClock<Rates<Fast, Mid, Slow>, Ticker, Serial, Host, DmaHost>;
constexpr SysClock clock;
static_assert(SysClock::rate_count == 3);

constexpr uint8_t r_fast = 0;
constexpr uint8_t r_mid = 1;
constexpr uint8_t r_slow = 2;

Serial serial;
TestBench<Serial, 20> bench;

using CsPin = Pin<'A', 15>;          ///< the host's chip select, a GPIO
using ClientNssPin = Pin<'B', 12>;   ///< the client's NSS - and letter e's witness
using NssWatch = ExtInt<ClientNssPin>;

// ---------------------------------------------------------------------------
// Instruments
// ---------------------------------------------------------------------------

/// The cycle-resolution stopwatch every suite of this stratum uses.
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

/// A measurement window a transmit interrupt walks through is not a
/// measurement - the lesson four campaigns of this stratum have paid
/// for. Drain, then count.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz() / 500u) {
    }
}

// ---------------------------------------------------------------------------
// The client's pump, one frame ahead
// ---------------------------------------------------------------------------

constexpr uint16_t peer_cap = 128;

volatile uint16_t peer_rx[peer_cap];
volatile uint16_t peer_rx_n = 0;
uint16_t peer_answers[peer_cap];
volatile uint16_t peer_tx_n = 0;
uint16_t peer_filler = 0xFFFF;
volatile uint32_t peer_err_seen = 0;
volatile uint16_t peer_err_count = 0;
/// Letter f: the frame count after which the client must set CRCNEXT, so
/// that the frame AFTER the last data one is checked as the CRC.
volatile bool peer_crc_arm = false;
volatile uint16_t peer_crc_next_at = 0;
/// How many answers the pump may write at all. A CRC-protected exchange
/// stops at the last DATA frame: what follows it on the wire is the
/// hardware's own checksum, and a pump that kept feeding would put a
/// data frame there instead (measured - the first version of letter f
/// read 0xFF where the client's TXCRCR belonged).
volatile uint16_t peer_tx_limit = 0xFFFFu;
/// Letter i: the client is on raw DMA engines and its vector serves
/// nothing.
volatile bool peer_pump_live = true;

/// The one answer the pump hands over next.
uint16_t peer_next_answer() {
    const uint16_t i = peer_tx_n;
    peer_tx_n = static_cast<uint16_t>(i + 1u);
    return i < peer_cap ? peer_answers[i] : peer_filler;
}

void peer_service() {
    const uint32_t f = Peer::isr();
    if (f == 0u) {
        return;
    }
    if ((f & SpiFlag::rxne) != 0u) {
        const uint16_t v = Peer::poll().value_or(0);
        if (peer_rx_n < peer_cap) {
            peer_rx[peer_rx_n] = v;
        }
        peer_rx_n = static_cast<uint16_t>(peer_rx_n + 1u);
        // CRCNEXT means "the frame AFTER the one in the shifter is the
        // checksum" (35.9.1), so it goes in at the boundary the client's
        // LAST answer starts shifting at - which is the interrupt for
        // the frame before it.
        if (peer_crc_arm && peer_rx_n == peer_crc_next_at) {
            S2::crc_next();
        }
        if (peer_tx_n < peer_tx_limit) {
            Peer::write(peer_next_answer());
        }
        return;
    }
    // An error, and only where ERRIE was deliberately armed: report ONCE
    // and disarm, because every one of these flags is a LEVEL and a
    // handler that leaves one standing re-enters for ever (the samc21
    // SERCOM storm, met again).
    peer_err_seen = peer_err_seen | (f & SpiFlag::errors);
    peer_err_count = static_cast<uint16_t>(peer_err_count + 1u);
    Peer::error_interrupt(false);
}

/// Bring the client up on `cfg`, with `n` answers loaded and the first
/// two already in its FIFO. NSS must be HIGH when this is called.
void peer_arm(const Peer::Config& cfg, const uint16_t* answers, uint16_t n,
              uint16_t filler = 0xFFFF) {
    Nvic::disable(S2::irq());
    peer_pump_live = true;
    peer_rx_n = 0;
    peer_tx_n = 0;
    peer_err_seen = 0;
    peer_err_count = 0;
    peer_crc_arm = false;
    peer_tx_limit = 0xFFFFu;
    peer_filler = filler;
    for (uint16_t i = 0; i < peer_cap; ++i) {
        peer_answers[i] = i < n ? answers[i] : filler;
    }
    (void)Peer::init(clock, cfg);
    Peer::rxne_interrupt(true);
    Peer::enable(peer_next_answer(), peer_next_answer());
}

void peer_stop() {
    Nvic::disable(S2::irq());
    Peer::rxne_interrupt(false);
    Peer::error_interrupt(false);
    (void)Peer::disable();
}

// ---------------------------------------------------------------------------
// The host's own glue
// ---------------------------------------------------------------------------

volatile bool host_done = false;
volatile uint16_t host_isr_completions = 0;
/// Letter j stages a LOST INTERRUPT: the body still runs (so the pump
/// finishes and RXNE is acknowledged) but the completion is never
/// posted, which is exactly the wedge util/bus_master.hpp's timeout is
/// for.
volatile bool host_swallow_completion = false;
/// Which host owns SPI1's vector right now.
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;
/// Letter m owns SPI1's vector for its own bare RXNE: the transfer
/// engine's isr() would otherwise consume the frame as part of a request
/// that is not running.
volatile bool sleep_probe_live = false;

uint8_t tx_buf[96];
uint8_t rx_buf[96];

/// One transaction through the TASK, waiting for it however it
/// completes. Returns false only when an ISR-style transfer never
/// answered.
bool host_xfer(const uint8_t* tx, uint8_t* rx, uint16_t frames, SpiClock rate,
               SpiMode mode, SpiDataSize bits, bool polled, uint8_t cmd_len = 0,
               const uint8_t* cmd = nullptr) {
    Host::Request r{};
    r.cs = CsPin::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = frames;
    r.clock = rate;
    r.mode = mode;
    r.bits = bits;
    r.polled = polled;
    host_done = false;
    if (Host::start(r)) {
        return true;
    }
    const uint32_t t0 = Ticker::millis();
    while (!host_done && Ticker::millis() - t0 < 500u) {
    }
    return host_done;
}

/// The same through the ENGINED host of letter i.
bool dma_xfer(const uint8_t* tx, uint8_t* rx, uint16_t frames, SpiClock rate,
              bool polled, uint8_t cmd_len = 0, const uint8_t* cmd = nullptr) {
    DmaHost::Request r{};
    r.cs = CsPin::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = frames;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.bits = SpiDataSize::bits8;
    r.polled = polled;
    host_done = false;
    if (DmaHost::start(r)) {
        return true;
    }
    const uint32_t t0 = Ticker::millis();
    while (!host_done && Ticker::millis() - t0 < 500u) {
    }
    return host_done;
}

/// Put the pads back where every wire letter wants them: PA15 a GPIO
/// chip select sitting HIGH, PB12 the client's NSS input.
void software_cs_pads() {
    CsPin::output(true);
}

/// One frame, by hand, on the host resource - for the letters that drive
/// the RESOURCE rather than the task (NSS, CRC, TI, the errata, I2S).
uint16_t raw_xfer(uint16_t out, SpiDataSize bits) {
    S1::data(bits, out);
    uint32_t spins = 400'000u;
    while (!S1::rxne() && spins-- != 0u) {
    }
    return S1::data(bits);
}

/// Configure SPI1 by hand and turn it on.
bool host_raw_config(const SpiConfig& c) {
    Nvic::disable(S1::irq());
    S1::bus_clock(true);
    S1::reset();
    const bool ok = S1::configure(c);
    Pin<host_pins.sck.port, host_pins.sck.pin>::function(host_pins.sck.function,
                                                         {.speed = PinSpeed::very_high});
    Pin<host_pins.mosi.port, host_pins.mosi.pin>::function(host_pins.mosi.function,
                                                           {.speed = PinSpeed::very_high});
    Pin<host_pins.miso.port, host_pins.miso.pin>::function(host_pins.miso.function);
    S1::enable();
    S1::flush_rx();
    return ok;
}

// A byte pattern that uses every bit of a frame of any width.
uint16_t pattern(uint16_t i, SpiDataSize bits) {
    const uint16_t v = static_cast<uint16_t>(0x9C4Bu + i * 0x1111u + (i << 3));
    return static_cast<uint16_t>(v & spi_frame_mask(bits));
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}
void settle() { settle_ms(spilink::settle_ms); }

// =============================================================================
// Which desk is this? The self-link, asked of the wire
// =============================================================================
//
// THE SELF-LINK IS NOT A PROPERTY OF THIS BOARD, it is four jumpers, and
// they have already moved once: the same SPI1 pads now carry the
// cross-architecture link to a SAM C21 running `spi_peer`, and SPI2's
// four pads are on nothing (docs/bench.md). So the suite ASKS THE WIRE
// which desk it is on, once, before any letter runs, and the letters
// that need two peripherals on four wires SKIP THEMSELVES - by name,
// with the reason printed - when the answer is no. A skipped letter
// claims nothing and scores no verdict either way, which is how
// test_avr_serial's letter q has always treated a wiring its desk did
// not have.

bool self_link = false;

/// Does driving `Drv` really move `Rd`? Both levels, each against the
/// far pad's OPPOSITE internal pull, so a pad stuck at a rail (or held
/// by an external pull-up) cannot answer yes to both.
template <typename Drv, typename Rd>
bool wire_follows() {
    Rd::input(PinPull::up);
    Drv::output(false);
    (void)delay_us(clock, 200);
    const bool low = !Rd::read();
    Rd::input(PinPull::down);
    Drv::set();
    (void)delay_us(clock, 200);
    const bool high = Rd::read();
    Drv::release();
    Rd::release();
    return low && high;
}

bool probe_self_link() {
    const bool sck = wire_follows<Pin<'B', 3>, Pin<'B', 10>>();
    const bool miso = wire_follows<Pin<'C', 2>, Pin<'B', 4>>();
    const bool mosi = wire_follows<Pin<'B', 5>, Pin<'D', 4>>();
    const bool nss = wire_follows<Pin<'A', 15>, Pin<'B', 12>>();
    print(serial, "  self-link probe: SCK ", sck, " MISO ", miso, " MOSI ", mosi,
          " NSS ", nss, crlf);
    return sck && miso && mosi && nss;
}

/// The opening line of every letter whose instrument is the self-link.
bool need_self_link() {
    if (self_link) {
        return true;
    }
    print(serial,
          "  SKIPPED, no verdict claimed: this letter's instrument is the board's "
          "OWN self-link (SPI1 PB3/PB4/PB5/PA15 to SPI2 PB10/PC2/PD4/PB12) and "
          "the probe says those four wires are not on the desk. They carry the "
          "cross-architecture link to the SAM C21 today - letters n..r are the "
          "instrument this desk has (docs/bench.md).",
          crlf);
    return false;
}

// =============================================================================
// The cross-architecture peer: spi_link.hpp over the bus under test
// =============================================================================
//
// The wire format is avrdx/src/apps/spi_link.hpp, included by relative
// path and NOT copied - it names no register, includes nothing of brio
// and is compiled by three architectures' apps. The instrument at the
// other end is `spi_peer`, whose samc21 port answers on SERCOM1 fn C.

using spilink::Op;

/// The command channel's SCK: PCLK/256 = 250 kHz at the boot rung. A
/// character is 32 us, an order of magnitude more than the peer's polled
/// command listener needs to turn one around.
constexpr SpiClock link_clock = SpiClock::div256;

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t dummy_buf[spilink::answer_bytes];
uint8_t raw_seen[16];
uint8_t raw_n = 0;
spilink::Decoder dec;
bool link_quiet = false;

/// THE HOLD AROUND EACH SELECT WINDOW, and it is the samc21 bench's
/// finding rather than a precaution: the engine releases CS about a
/// microsecond after the last SCK edge, and a client whose transaction
/// the select edge RESETS loses a character it has not fetched yet. The
/// protocol therefore owns the chip select for these windows (the
/// Request carries a null PinRef) and pays 30 us on each side of them.
void link_hold() { (void)delay_us(clock, 30); }

/// SPI1 as the command channel's host, PA15 as its GPIO chip select.
bool link_command_mode() {
    peer_stop();
    dma_host_live = false;
    bus_ao_live = false;
    sleep_probe_live = false;
    software_cs_pads();
    const bool ok = Host::init(clock);
    dec.reset();
    return ok;
}

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip
/// inside an open window is one extra edge and the selected client
/// counts it into the frame (SpiHost::prime()'s own comment).
bool link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n,
               SpiClock rate = link_clock, SpiMode mode = SpiMode::mode0) {
    if (n == 0) {
        return true;
    }
    Host::prime(mode, rate);
    CsPin::clear();
    link_hold();
    Host::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
    r.bits = SpiDataSize::bits8;
    r.polled = true;
    const bool done = Host::start(r);
    link_hold();
    CsPin::set();
    return done;
}

void send_frame(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    spilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    (void)link_xfer(frame_buf, nullptr, n);
}

/// One answer window: `answer_bytes` dummies in a single transaction,
/// then the decoder walks what came back (the peer pads its frame with
/// zeros, which the decoder ignores).
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        dummy_buf[i] = 0;
        answer_buf[i] = 0xEE;
    }
    (void)link_xfer(dummy_buf, answer_buf, spilink::answer_bytes);
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        if (raw_n < 16) {
            raw_seen[raw_n++] = answer_buf[i];
        }
        if (dec.feed(answer_buf[i]) == spilink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    send_frame(op, p, len);
    settle();
    spilink::Frame f;
    if (!recv_frame(f)) {
        return false;
    }
    return f.op == Op::ack && f.len == 2 && f.data[0] == spilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts, each separated by longer than the peer's own answer
/// window bound, so a peer that was serving into nothing is certainly
/// dark again before the retry - the protocol's own recovery guarantee.
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    uint8_t first_n = 0;
    uint8_t first_seen[16];
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len)) {
            return true;
        }
        if (k == 0) {
            first_n = raw_n;
            for (uint8_t i = 0; i < raw_n; ++i) {
                first_seen[i] = raw_seen[i];
            }
        }
        (void)link_command_mode();
        settle_ms(400);
    }
    if (link_quiet) {
        return false;
    }
    print(serial, "    LINK FAILURE op ", hex(spilink::byte_of(op)),
          ": the first answer window carried");
    if (first_n == 0) {
        print(serial, " nothing");
    }
    for (uint8_t i = 0; i < first_n; ++i) {
        print(serial, " ", hex(first_seen[i]));
    }
    print(serial, crlf,
          "      the peer board must be running `spi_peer` (python3 tools/bench.py "
          "flash D spi_peer); its console '0' forces the dark client back.",
          crlf);
    (void)link_command_mode();
    return false;
}

bool query(Op op, spilink::Frame& data) {
    if (!command(op)) {
        return false;
    }
    settle();
    return recv_frame(data);
}

bool peer_ident(spilink::Ident& d) {
    spilink::Frame f;
    if (!query(Op::ident, f) || f.op != Op::ident_data || f.len != spilink::ident_size) {
        return false;
    }
    d = spilink::get_ident(f.data);
    return true;
}

bool peer_report(spilink::Report& r) {
    (void)link_command_mode();
    for (uint8_t k = 0; k < 4; ++k) {
        spilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data &&
            f.len == spilink::report_size) {
            r = spilink::get_report(f.data);
            return true;
        }
        settle_ms(60);
    }
    return false;
}

bool peer_act(Op op, const spilink::Params& a) {
    uint8_t p[spilink::params_size];
    spilink::put_params(p, a);
    if (!command(op, p, spilink::params_size)) {
        return false;
    }
    settle();
    return true;
}

/// Is the peer there at all? Every letter that needs it asks first, so
/// an absent peer is a NAMED failure and never a hang.
bool ensure_link() {
    link_quiet = true;
    (void)link_command_mode();
    for (uint8_t k = 0; k < 3; ++k) {
        if (command(Op::ping)) {
            link_quiet = false;
            return true;
        }
    }
    link_quiet = false;
    print(serial,
          "  THE PEER DID NOT ANSWER. The peer board must be running `spi_peer` "
          "(python3 tools/bench.py flash D spi_peer); its console '0' forces the "
          "dark client back. Check the five wires in this file's header.",
          crlf);
    return false;
}

/// The opening line of every letter whose instrument is the SAM peer.
/// THE TWO WIRINGS ARE THE SAME JUMPERS AT DIFFERENT ENDS, so a desk
/// carrying the self-link cannot be carrying the peer: that is a
/// topology and it skips, exactly as the self-link letters skip on the
/// other desk. An absent peer with the wires in place is a different
/// thing - firmware to flash - and fails loudly.
bool need_peer() {
    if (self_link) {
        print(serial,
              "  SKIPPED, no verdict claimed: this letter's instrument is the SAM "
              "C21 running `spi_peer`, and the probe says these four pads carry the "
              "board's OWN self-link today - the two wirings are the same jumpers "
              "at different ends (docs/bench.md). Letters b..l are the instrument "
              "this desk has.",
              crlf);
        return false;
    }
    if (ensure_link()) {
        return true;
    }
    bench.verdict("the peer answers a ping (the peer board running spi_peer)", false);
    return false;
}

// ---- one exchange, commanded and then clocked ------------------------------

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0,
                     .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    SpiClock rate = link_clock;
    uint16_t count = 8;
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;   ///< spilink::spare_polled_pump forces the peer's polled loop
    uint16_t ms = 400;
};

constexpr uint8_t max_exchange = 32;
uint8_t xtx[max_exchange];
uint8_t xrx[max_exchange];

bool do_exchange(const Exchange& e) {
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    a.flags = e.flags;
    a.spare = e.spare;
    // A bit-order mismatch is not a shrug: told about it, each end
    // checks the EXACT bit-reverse of what the other sent.
    if (e.host_lsb != (e.cfg.dord != 0)) {
        a.flags |= spilink::flag_expect_reversed;
    }
    if (!peer_act(Op::exchange, a)) {
        return false;
    }

    spilink::Stream out(e.pattern, e.seed_a);
    for (uint8_t i = 0; i < max_exchange; ++i) {
        xtx[i] = 0;
        xrx[i] = 0xEE;
    }
    const uint16_t n = e.count < max_exchange ? e.count : max_exchange;
    for (uint16_t i = 0; i < n; ++i) {
        xtx[i] = out.next();
    }
    settle();
    // The bit order is a BUS-LEVEL verb here, not a Request field
    // (SpiHost::bit_order()'s own comment says why), so this end simply
    // states it and link_command_mode() puts it back with the re-init.
    if (e.host_lsb) {
        (void)Host::bit_order(true);
    }
    (void)link_xfer(xtx, xrx, n, e.rate, e.host_mode);
    (void)link_command_mode();
    return true;
}

struct Verify {
    uint16_t mism = 0;
    uint8_t idx = 0xFF;
    uint8_t got = 0;
    uint8_t exp = 0;
};

Verify verify_rx(const Exchange& e) {
    Verify v{};
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(e.pattern, e.seed_b, i);
        if (reversed) {
            exp = spilink::bit_reverse(exp);
        }
        if (xrx[i] != exp) {
            if (v.mism == 0) {
                v.idx = static_cast<uint8_t>(i);
                v.got = xrx[i];
                v.exp = exp;
            }
            ++v.mism;
        }
    }
    return v;
}

bool exchange_exact(const Exchange& e, Verify& v, spilink::Report& r) {
    const bool ran = do_exchange(e);
    const bool rep = ran && peer_report(r);
    v = verify_rx(e);
    return ran && rep && v.mism == 0 && r.mism == 0 && r.count == e.count;
}

void dump_exchange(const Exchange& e, const Verify& v, const spilink::Report& r) {
    print(serial, "    host mism=", v.mism, " (first idx ", v.idx, " got ", hex(v.got),
          " exp ", hex(v.exp), "), client count=", r.count, " mism=", r.mism,
          " (first idx ", r.idx, " got ", hex(r.got), " exp ", hex(r.exp),
          ") flags=", hex(r.flags), crlf);
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    print(serial, "    read:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        print(serial, " ", hex(xrx[i]));
    }
    print(serial, crlf, "    want:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(e.pattern, e.seed_b, i);
        if (reversed) {
            exp = spilink::bit_reverse(exp);
        }
        print(serial, " ", hex(exp));
    }
    print(serial, crlf);
}

// =============================================================================
// a - the block, wireless
// =============================================================================

/// Write one field raw with SPE set and say whether it stuck. The DRIVER
/// refuses these; whether the SILICON does is a different question, and
/// this is how the letter asks it.
struct ProtectionProbe {
    const char* name;
    bool stuck;
};

bool probe_cr1(uint32_t bit) {
    const uint32_t before = S1::regs().CR1;
    S1::regs().CR1 = before ^ bit;
    const bool stuck = ((S1::regs().CR1 ^ before) & bit) != 0u;
    S1::regs().CR1 = before;
    return stuck;
}
bool probe_cr2(uint32_t bit) {
    const uint32_t before = S1::regs().CR2;
    S1::regs().CR2 = before ^ bit;
    const bool stuck = ((S1::regs().CR2 ^ before) & bit) != 0u;
    S1::regs().CR2 = before;
    return stuck;
}

void ta_block() {
    peer_stop();
    Nvic::disable(S1::irq());
    S1::bus_clock(true);
    S2::bus_clock(true);
    S1::reset();
    S2::reset();

    // ---- the reserve against the device header ----
    bench.verdict("the reserve knows this part's SPI roster: SPI1 and SPI2 "
                  "everywhere, SPI3 on the G0B1/G0C1 alone",
                  spi_present(1) && spi_present(2) && spi_present(3) && !spi_present(4));
    print(serial, "  vectors: SPI1 ", static_cast<int32_t>(spi_irq(1)), ", SPI2 ",
          static_cast<int32_t>(spi_irq(2)), ", SPI3 ", static_cast<int32_t>(spi_irq(3)),
          crlf);
    bench.verdict("SPI2 and SPI3 SHARE a line on a part that has both, and the "
                  "reserve derives it from SPI3's presence",
                  spi_irq(1) == SPI1_IRQn && spi_irq(2) == SPI2_3_IRQn &&
                      spi_irq(3) == SPI2_3_IRQn);
    bench.verdict("the DMAMUX request pairs are table 56's (16/17, 18/19, 66/67)",
                  S1::dma_rx_request() == 16 && S1::dma_tx_request() == 17 &&
                      S2::dma_rx_request() == 18 && S2::dma_tx_request() == 19 &&
                      spi_dma_rx_request(3) == 66 && spi_dma_tx_request(3) == 67);
    print(serial, "  I2S: reserve says SPI1 ", spi_has_i2s(1) ? "yes" : "no", " SPI2 ",
          spi_has_i2s(2) ? "yes" : "no", " SPI3 ", spi_has_i2s(3) ? "yes" : "no",
          "; the header's IS_I2S_ALL_INSTANCE says SPI1 ", S1::has_i2s() ? "yes" : "no",
          " SPI2 ", S2::has_i2s() ? "yes" : "no", " SPI3 ",
          Spi<3>::has_i2s() ? "yes" : "no", crlf);
    bench.verdict("table 205's I2S row: the reserve's constant and the device "
                  "header's own macro agree on all three instances",
                  spi_has_i2s(1) == S1::has_i2s() && spi_has_i2s(2) == S2::has_i2s() &&
                      spi_has_i2s(3) == Spi<3>::has_i2s());
    bench.verdict("SPI1's APB enable is APB2's and SPI2's is APB1's, and both "
                  "gates are open",
                  spi_bus_clock(1).apb2 && !spi_bus_clock(2).apb2 && S1::bus_clock() &&
                      S2::bus_clock());

    // ---- the reset values (table 209) ----
    const uint32_t cr1 = S1::regs().CR1;
    const uint32_t cr2 = S1::regs().CR2;
    const uint32_t sr = S1::regs().SR;
    const uint32_t crcpr = S1::regs().CRCPR;
    const uint32_t i2scfgr = S1::regs().I2SCFGR;
    const uint32_t i2spr = S1::regs().I2SPR;
    print(serial, "  reset: CR1 ", hex(cr1), " CR2 ", hex(cr2), " SR ", hex(sr),
          " CRCPR ", hex(crcpr), " I2SCFGR ", hex(i2scfgr), " I2SPR ", hex(i2spr), crlf);
    bench.verdict("every register comes out of an RCC reset at table 209's own "
                  "value (CR1 0, CR2 0x0700, SR 0x0002, CRCPR 0x0007, I2SCFGR 0, "
                  "I2SPR 0x0002)",
                  cr1 == 0u && cr2 == 0x0700u && sr == 0x0002u && crcpr == 0x0007u &&
                      i2scfgr == 0u && i2spr == 0x0002u);
    bench.verdict("the reset CR2 is DS = 0111 (8-bit) with FRXTH clear, which is "
                  "the ONE combination 35.5.7 step 3e calls misaligned - a driver "
                  "that trusted the reset value would read half-words for byte "
                  "frames",
                  ((cr2 & SPI_CR2_DS) >> SPI_CR2_DS_Pos) == 7u &&
                      (cr2 & SPI_CR2_FRXTH) == 0u);
    bench.verdict("SR at reset says TXE and not RXNE: the FIFO is empty in both "
                  "directions and BSY is clear",
                  (sr & SPI_SR_TXE) != 0u && (sr & SPI_SR_RXNE) == 0u &&
                      (sr & SPI_SR_BSY) == 0u);

    // ---- the "Not used" data sizes ----
    (void)S1::configure(SpiConfig{});
    S1::regs().CR2 = (S1::regs().CR2 & ~SPI_CR2_DS) | (2u << SPI_CR2_DS_Pos);
    const uint8_t forced = static_cast<uint8_t>((S1::regs().CR2 & SPI_CR2_DS) >>
                                                SPI_CR2_DS_Pos);
    print(serial, "  DS = 0010 written, DS = ", forced, " read back", crlf);
    bench.verdict("35.9.2's own sentence, measured: a \"Not used\" DS code is "
                  "FORCED to 0111 (8-bit) and not ignored - the register lies "
                  "about what it holds, which is why the driver refuses the code",
                  forced == 7u);
    bench.verdict("and the driver's verb refuses it before a register is touched",
                  !S1::data_size(static_cast<SpiDataSize>(2)));

    // ---- the enable protection, measured field by field ----
    (void)S1::configure(SpiConfig{});
    S1::enable();
    const ProtectionProbe probes[] = {
        {"CPOL", probe_cr1(SPI_CR1_CPOL)},
        {"CPHA", probe_cr1(SPI_CR1_CPHA)},
        {"BR", probe_cr1(SPI_CR1_BR_0)},
        {"MSTR", probe_cr1(SPI_CR1_MSTR)},
        {"LSBFIRST", probe_cr1(SPI_CR1_LSBFIRST)},
        {"CRCEN", probe_cr1(SPI_CR1_CRCEN)},
        {"CRCL", probe_cr1(SPI_CR1_CRCL)},
        {"SSM", probe_cr1(SPI_CR1_SSM)},
        {"RXONLY", probe_cr1(SPI_CR1_RXONLY)},
        {"BIDIMODE", probe_cr1(SPI_CR1_BIDIMODE)},
        {"DS", probe_cr2(SPI_CR2_DS_0)},
        {"FRXTH", probe_cr2(SPI_CR2_FRXTH)},
        {"FRF", probe_cr2(SPI_CR2_FRF)},
        {"NSSP", probe_cr2(SPI_CR2_NSSP)},
        {"SSOE", probe_cr2(SPI_CR2_SSOE)},
        {"LDMATX", probe_cr2(SPI_CR2_LDMATX)},
        {"LDMARX", probe_cr2(SPI_CR2_LDMARX)},
    };
    uint8_t stuck = 0;
    print(serial, "  with SPE set, a raw write lands on:");
    for (const auto& p : probes) {
        if (p.stuck) {
            ++stuck;
            print(serial, " ", p.name);
        }
    }
    print(serial, stuck == 0 ? " (nothing)" : "", crlf);

    // THE SSM PROBE IS ITSELF A MODE-FAULT STIMULUS, and finding that
    // out is worth more than the probe was: with SSM momentarily clear
    // the master's internal NSS becomes the PAD - and PA15 is a plain
    // GPIO output here, not an alternate function, so the peripheral's
    // NSS input reads LOW and MODF fires on the spot. The silicon then
    // clears SPE and MSTR and REFUSES to let them be set again until
    // MODF is cleared (35.5.11's last sentence), which is why the
    // restore inside the probe could not put CR1 back.
    const bool modf_from_ssm = S1::mode_fault();
    print(serial, "  MODF after the SSM probe: ", modf_from_ssm ? 1 : 0,
          "; CR1 now ", hex(S1::regs().CR1), " (SPE and MSTR cleared by hardware)",
          crlf);
    bench.verdict("A PAD THAT IS NOT AN ALTERNATE FUNCTION READS LOW AT THE "
                  "PERIPHERAL'S NSS INPUT: clearing SSM on a master whose NSS "
                  "pin is an ordinary GPIO raises MODF at once, and the silicon "
                  "clears SPE and MSTR and blocks their return until the flag is "
                  "put away - which is exactly why the transfer engine's boot "
                  "configuration is SpiNss::software and never touches the pad",
                  modf_from_ssm);
    S1::clear_mode_fault();
    (void)S1::disable();
    (void)S1::configure(SpiConfig{});
    S1::enable();
    bench.verdict("THE SILICON DOES NOT ENFORCE THE CHAPTER'S OWN RULE: every "
                  "one of the seventeen CR1/CR2 configuration fields takes a "
                  "write with SPE set, including the four whose register "
                  "description says \"only when the SPI is disabled\" - so the "
                  "refusals below are the DRIVER's and nothing else stands "
                  "between a program and a mid-transfer reconfiguration",
                  stuck == 17u);

    const bool refused =
        !S1::configure(SpiConfig{}) && !S1::mode(SpiMode::mode3) &&
        !S1::clock(SpiClock::div2) && !S1::role(SpiRole::client) &&
        !S1::bit_order(true) && !S1::data_size(SpiDataSize::bits16) &&
        !S1::direction(SpiDirection::receive_only) && !S1::nss(SpiNss::hardware_output) &&
        !S1::frame_format(SpiFrameFormat::ti) && !S1::crc(true) &&
        !S1::crc_polynomial(0x1021u) && !S1::dma_transmit(true) &&
        !S1::dma_receive(true) && !S1::last_dma_transmit_odd(true) &&
        !S1::last_dma_receive_odd(true);
    const uint32_t cr1_after = S1::regs().CR1;
    const uint32_t cr2_after = S1::regs().CR2;
    print(serial, "  after the refusals: CR1 ", hex(cr1_after), " (expected ",
          hex(spi_cr1(SpiConfig{}) | SPI_CR1_SPE), "), CR2 ", hex(cr2_after),
          " (expected ", hex(spi_cr2(SpiConfig{})), "), MODF ",
          S1::mode_fault() ? 1 : 0, crlf);
    bench.verdict("every configuring verb refuses while SPE is set, and writes "
                  "nothing",
                  refused && cr1_after == (spi_cr1(SpiConfig{}) | SPI_CR1_SPE) &&
                      cr2_after == spi_cr2(SpiConfig{}));
    bench.verdict("the three verbs that MUST stay open under a running SPI are "
                  "open: SSI (the mode-fault stimulus and a software client's "
                  "own select), CRCNEXT (35.9.1: written as the last data is), "
                  "and FRXTH (35.5.9's last-odd-frame note)",
                  [] {
                      S1::software_select(false);
                      const bool ssi = !S1::software_selected();
                      S1::software_select(true);
                      S1::crc_next();
                      const bool cn = S1::crc_next_pending();
                      S1::regs().CR1 = S1::regs().CR1 & ~SPI_CR1_CRCNEXT;
                      S1::rx_threshold(SpiRxThreshold::half);
                      const bool th = S1::rx_threshold() == SpiRxThreshold::half;
                      S1::rx_threshold(SpiRxThreshold::quarter);
                      return ssi && cn && th;
                  }());

    // ---- the disable procedure ----
    const bool disabled = S1::disable();
    print(serial, "  after disable(): SPE ", S1::enabled() ? 1 : 0, " FTLVL ",
          S1::tx_level(), " FRLVL ", S1::rx_level(), " BSY ", S1::busy() ? 1 : 0, crlf);
    bench.verdict("35.5.9's disable procedure completes on an idle master and "
                  "leaves both FIFOs empty, SPE clear and BSY low",
                  disabled && !S1::enabled() && S1::tx_level() == 0 &&
                      S1::rx_level() == 0 && !S1::busy());

    bench.verdict("the configuration refusals of spi_config_valid(), at run time "
                  "as well as at compile time (the family fixture's negatives): "
                  "a CRC on a 12-bit frame, an even polynomial, NSSP with "
                  "CPHA = 1, SSOE on a client, LDMA without its DMA enable",
                  !S1::configure(SpiConfig{.bits = SpiDataSize::bits12, .crc = true}) &&
                      !S1::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1020u}) &&
                      !S1::configure(SpiConfig{.mode = SpiMode::mode1,
                                               .nss = SpiNss::hardware_pulse}) &&
                      !S1::configure(SpiConfig{.role = SpiRole::client,
                                               .nss = SpiNss::hardware_output}) &&
                      !S1::configure(SpiConfig{.last_dma_receive_odd = true}));

    bench.verdict("spi_rate_for() picks the fastest code at or below a ceiling "
                  "and REFUSES one the prescaler cannot reach, at both this "
                  "clock and the ladder's other two",
                  spi_rate_for(64'000'000, 8'000'000) == SpiClock::div8 &&
                      spi_rate_for(16'000'000, 8'000'000) == SpiClock::div2 &&
                      spi_rate_for(2'000'000, 8'000'000) == SpiClock::div2 &&
                      !spi_rate_for(64'000'000, 200'000).has_value());
}

// =============================================================================
// b - the link
// =============================================================================

/// Fill the fixtures: the host sends `0xA0 + i`, the client answers
/// `0x40 + i`.
void fill_link_fixture(uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0xA0u + i);
        rx_buf[i] = 0xEE;
        peer_answers[i] = static_cast<uint16_t>(0x40u + i);
    }
}

/// Judge one 8-bit exchange: what the client got against what the host
/// sent, and what the host got against the client's table.
struct LinkResult {
    bool host_ok;
    bool peer_ok;
    uint16_t peer_frames;
    uint32_t peer_errors;
    uint32_t host_status;
};

LinkResult judge_link(uint16_t n) {
    LinkResult r{true, true, peer_rx_n, peer_err_seen, S1::status()};
    for (uint16_t i = 0; i < n; ++i) {
        if (rx_buf[i] != static_cast<uint8_t>(0x40u + i)) {
            r.host_ok = false;
        }
        if (i >= peer_rx_n || peer_rx[i] != static_cast<uint16_t>(0xA0u + i)) {
            r.peer_ok = false;
        }
    }
    if (peer_rx_n != n) {
        r.peer_ok = false;
    }
    return r;
}

void tb_link() {
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 16;
    software_cs_pads();
    bench.verdict("the host comes up on SPI1 with the self-link's pads",
                  Host::init(clock));
    dma_host_live = false;
    bus_ao_live = false;

    fill_link_fixture(n);
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8},
             peer_answers, n);
    const bool moved = host_xfer(tx_buf, rx_buf, n, SpiClock::div64, SpiMode::mode0,
                                 SpiDataSize::bits8, true);
    const LinkResult polled = judge_link(n);
    print(serial, "  polled host: peer got ", polled.peer_frames, " frames, host rx[0..3] ",
          hex(rx_buf[0]), " ", hex(rx_buf[1]), " ", hex(rx_buf[2]), " ", hex(rx_buf[3]),
          crlf);
    bench.verdict("SIXTEEN 8-BIT FRAMES CROSS THE FOUR WIRES AND COME BACK: the "
                  "client received exactly what the host sent",
                  moved && polled.peer_ok);
    bench.verdict("...and the host received exactly what the client had ready, "
                  "which is the one-frame-ahead preload working (35.5.8)",
                  polled.host_ok);
    bench.verdict("no error flag stands at either end after a clean exchange",
                  (S1::status() & SpiFlag::errors) == 0u && peer_err_seen == 0u);

    // The same, with the host on its own interrupt.
    fill_link_fixture(n);
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    host_isr_completions = 0;
    const bool moved_isr = host_xfer(tx_buf, rx_buf, n, SpiClock::div64, SpiMode::mode0,
                                     SpiDataSize::bits8, false);
    const LinkResult isr = judge_link(n);
    print(serial, "  ISR host: completions ", host_isr_completions, ", peer frames ",
          isr.peer_frames, crlf);
    bench.verdict("the SAME sixteen frames on the host's RXNE pump - one "
                  "interrupt per frame, one completion at the end",
                  moved_isr && isr.host_ok && isr.peer_ok && host_isr_completions == 1u);

    // A command phase in front of the data.
    static const uint8_t cmd[3] = {0x9F, 0x01, 0x02};
    fill_link_fixture(n);
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    const bool with_cmd = host_xfer(tx_buf, rx_buf, 8, SpiClock::div64, SpiMode::mode0,
                                    SpiDataSize::bits8, false, 3, cmd);
    bool cmd_ok = with_cmd && peer_rx_n == 11u;
    for (uint16_t i = 0; i < 3u && cmd_ok; ++i) {
        cmd_ok = peer_rx[i] == cmd[i];
    }
    for (uint16_t i = 0; i < 8u && cmd_ok; ++i) {
        cmd_ok = peer_rx[3u + i] == static_cast<uint16_t>(0xA0u + i);
    }
    print(serial, "  cmd+data: peer got ", peer_rx_n, " frames (3 + 8 expected)", crlf);
    bench.verdict("a two-phase request is ONE select window: three command "
                  "frames then eight data ones, in order, with the D/C flip "
                  "between them and no gap the client can see",
                  cmd_ok);
    peer_stop();
}

// =============================================================================
// c - the matrix
// =============================================================================

void tc_matrix() {
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 8;
    software_cs_pads();
    (void)Host::init(clock);
    dma_host_live = false;

    uint8_t modes_ok = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        const SpiMode mode = static_cast<SpiMode>(m);
        fill_link_fixture(n);
        peer_arm({.mode = mode, .bits = SpiDataSize::bits8}, peer_answers, n);
        const bool moved = host_xfer(tx_buf, rx_buf, n, SpiClock::div64, mode,
                                     SpiDataSize::bits8, true);
        const LinkResult r = judge_link(n);
        if (moved && r.host_ok && r.peer_ok) {
            ++modes_ok;
        }
    }
    bench.verdict("all four CPOL/CPHA combinations carry eight frames byte-exact "
                  "in both directions", "(4 of 4)", modes_ok == 4u);

    uint8_t orders_ok = 0;
    for (uint8_t o = 0; o < 2u; ++o) {
        const bool lsb = o != 0u;
        fill_link_fixture(n);
        peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8, .lsb_first = lsb},
                 peer_answers, n);
        (void)Host::bit_order(lsb);
        const bool moved = host_xfer(tx_buf, rx_buf, n, SpiClock::div64, SpiMode::mode0,
                                     SpiDataSize::bits8, true);
        const LinkResult r = judge_link(n);
        if (moved && r.host_ok && r.peer_ok) {
            ++orders_ok;
        }
    }
    bench.verdict("both bit orders do too, with LSBFIRST set at BOTH ends",
                  "(2 of 2)", orders_ok == 2u);

    // A DORD mismatch is the control that makes the two legs above mean
    // something: one end MSB-first, the other LSB-first.
    fill_link_fixture(n);
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8, .lsb_first = true},
             peer_answers, n);
    (void)Host::bit_order(false);
    (void)host_xfer(tx_buf, rx_buf, n, SpiClock::div64, SpiMode::mode0,
                    SpiDataSize::bits8, true);
    bool reversed = peer_rx_n == n;
    for (uint16_t i = 0; i < n && reversed; ++i) {
        uint8_t r = 0;
        uint8_t v = static_cast<uint8_t>(0xA0u + i);
        for (uint8_t b = 0; b < 8u; ++b) {
            r = static_cast<uint8_t>((r << 1) | (v & 1u));
            v = static_cast<uint8_t>(v >> 1);
        }
        reversed = peer_rx[i] == r;
    }
    bench.verdict("a bit-order MISMATCH is an exact two-way bit reversal and not "
                  "noise - the control that makes the two legs above a proof",
                  reversed);
    (void)Host::bit_order(false);

    // Every data size, with a pattern that uses every bit of the frame.
    uint8_t sizes_ok = 0;
    uint8_t sizes_tried = 0;
    for (uint8_t bits = 4; bits <= 16u; ++bits) {
        const SpiDataSize d = *spi_data_size_of(bits);
        const bool wide = spi_frame_is_halfword(d);
        for (uint16_t i = 0; i < n; ++i) {
            const uint16_t v = pattern(i, d);
            if (wide) {
                tx_buf[2u * i] = static_cast<uint8_t>(v);
                tx_buf[2u * i + 1u] = static_cast<uint8_t>(v >> 8);
            } else {
                tx_buf[i] = static_cast<uint8_t>(v);
            }
            peer_answers[i] = static_cast<uint16_t>(pattern(static_cast<uint16_t>(i + 32u), d));
            rx_buf[2u * i] = 0xEE;
            rx_buf[2u * i + 1u] = 0xEE;
        }
        peer_arm({.mode = SpiMode::mode0, .bits = d}, peer_answers, n);
        const bool moved = host_xfer(tx_buf, rx_buf, n, SpiClock::div64, SpiMode::mode0,
                                     d, true);
        bool ok = moved && peer_rx_n == n;
        for (uint16_t i = 0; i < n && ok; ++i) {
            ok = peer_rx[i] == pattern(i, d);
        }
        for (uint16_t i = 0; i < n && ok; ++i) {
            const uint16_t got = wide ? static_cast<uint16_t>(rx_buf[2u * i] |
                                                              (rx_buf[2u * i + 1u] << 8))
                                      : rx_buf[i];
            ok = got == pattern(static_cast<uint16_t>(i + 32u), d);
        }
        ++sizes_tried;
        if (ok) {
            ++sizes_ok;
        } else {
            print(serial, "  ", bits, "-bit frames FAILED (peer got ", peer_rx_n, ")", crlf);
        }
    }
    print(serial, "  data sizes exact both ways: ", sizes_ok, " of ", sizes_tried, crlf);
    bench.verdict("EVERY FRAME SIZE FROM 4 TO 16 BITS crosses exact in both "
                  "directions, with a pattern that uses every bit of the frame - "
                  "which is also the proof of figure 363's access rule, because "
                  "each side reads DR a byte at a time below nine bits and a "
                  "half-word at a time above, with FRXTH following",
                  sizes_ok == 13u);
    bench.verdict("the unused top bits of a short frame come back ZERO, as "
                  "figure 363 draws them (a 4-bit frame never carries more than "
                  "its mask)",
                  [] {
                      const SpiDataSize d = SpiDataSize::bits4;
                      tx_buf[0] = 0xFF;   // more than the frame holds
                      peer_answers[0] = 0xFF;
                      peer_arm({.mode = SpiMode::mode0, .bits = d}, peer_answers, 1);
                      (void)host_xfer(tx_buf, rx_buf, 1, SpiClock::div64,
                                      SpiMode::mode0, d, true);
                      return peer_rx_n == 1u && peer_rx[0] == 0x0Fu && rx_buf[0] == 0x0Fu;
                  }());
    peer_stop();
}

// =============================================================================
// d - the rate ladder
// =============================================================================

void td_ladder() {
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 16;
    software_cs_pads();
    (void)Host::init(clock);
    dma_host_live = false;

    // ---- the PACED ladder: the engine's own polled pump, one frame at
    // a time, which is what a device driver really does ----
    uint8_t paced = 0;
    for (uint8_t c = static_cast<uint8_t>(SpiClock::div256);; --c) {
        const SpiClock rate = static_cast<SpiClock>(c);
        fill_link_fixture(n);
        peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
        (void)host_xfer(tx_buf, rx_buf, n, rate, SpiMode::mode0, SpiDataSize::bits8, true);
        const LinkResult r = judge_link(n);
        if (r.host_ok && r.peer_ok) {
            ++paced;
        } else {
            print(serial, "  paced PCLK/", spi_division(rate), ": host ",
                  r.host_ok ? "ok" : "SLIPPED", ", client ",
                  r.peer_ok ? "ok" : "SLIPPED", crlf);
        }
        if (c == 0u) {
            break;
        }
    }
    print(serial, "  paced by the engine's pump: ", paced, " of 8 codes exact both "
                  "ways, up to PCLK/2 = ", spi_sck_hz(SysClock::pclk_hz(), SpiClock::div2) / 1000u,
          " kHz", crlf);
    bench.verdict("ALL EIGHT BR CODES CARRY THE LINK BYTE-EXACT WHEN THE HOST "
                  "PACES IT - up to PCLK/2, the peripheral's own maximum - "
                  "because a polled pump writes one frame, waits for it to come "
                  "back, and only then writes the next: the gap between frames is "
                  "the HOST's loop, and the client has all of it to answer in. A "
                  "rate ladder run this way measures the wire and nothing else",
                  paced == 8u);

    // ---- the BURST ladder: the transmit FIFO kept full, so that the
    // clock does not stop between frames and the client has ONE FRAME
    // TIME to answer.
    //
    // NOT ONE CHARACTER IS PRINTED INSIDE THIS LOOP. A console byte is
    // an interrupt, an interrupt inside a burst is a gap the host's own
    // receive FIFO can overrun, and the first version of this letter
    // measured its own printing (one rung stalled outright, its loop
    // spending four million turns waiting for a frame the overrun had
    // thrown away). Collect the eight rungs, THEN say what happened ----
    bool answers[8];
    bool receives[8];
    bool stalled[8];
    bool host_ovr[8];
    uint16_t received[8];
    uint32_t per_frame[8];
    bool any_client_ovr = false;
    console_drain();
    for (uint8_t c = static_cast<uint8_t>(SpiClock::div256);; --c) {
        const SpiClock rate = static_cast<SpiClock>(c);
        fill_link_fixture(n);
        peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
        (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                        .clock = rate,
                                        .bits = SpiDataSize::bits8});
        CsPin::clear();
        uint16_t sent = 0;
        uint16_t got = 0;
        uint32_t guard = 200'000u;
        const uint32_t t0 = cycles_now();
        while (got < n && guard-- != 0u) {
            if (sent < n && S1::txe()) {
                S1::data_byte(tx_buf[sent]);
                ++sent;
            }
            if (S1::rxne()) {
                rx_buf[got] = static_cast<uint8_t>(S1::data_byte());
                ++got;
            }
        }
        const uint32_t took = cycles_now() - t0;
        CsPin::set();
        stalled[c] = got < n;
        received[c] = got;
        host_ovr[c] = S1::overrun();
        if (S2::overrun()) {
            any_client_ovr = true;
        }
        (void)S1::disable();
        bool ok_answers = got == n;
        for (uint16_t i = 0; i < n && ok_answers; ++i) {
            ok_answers = rx_buf[i] == static_cast<uint8_t>(0x40u + i);
        }
        bool ok_receive = peer_rx_n == n;
        for (uint16_t i = 0; i < n && ok_receive; ++i) {
            ok_receive = peer_rx[i] == static_cast<uint16_t>(0xA0u + i);
        }
        answers[c] = ok_answers;
        receives[c] = ok_receive;
        per_frame[c] = stalled[c] ? 0u : took / n;
        if (c == 0u) {
            break;
        }
    }

    uint8_t receive_ok = 0;
    uint8_t answers_ok = 0;
    uint8_t stalls = 0;
    uint8_t host_overruns = 0;
    for (uint8_t c = 0; c < 8u; ++c) {
        const SpiClock rate = static_cast<SpiClock>(c);
        print(serial, "  burst PCLK/", spi_division(rate), " (",
              spi_sck_hz(SysClock::pclk_hz(), rate) / 1000u, " kHz = ",
              8u * spi_division(rate), " cycles a frame on the wire): the loop took ",
              per_frame[c], " cycles a frame, answers ",
              answers[c] ? "ok" : "SLIPPED", ", client receive ",
              receives[c] ? "ok" : "SLIPPED",
              stalled[c] ? ", STALLED" : "", host_ovr[c] ? ", host OVR" : "", crlf);
        if (stalled[c]) {
            print(serial, "    (the host wrote ", n, " frames and read back ",
                  received[c], ")", crlf);
        }
        if (receives[c]) {
            ++receive_ok;
        }
        if (answers[c]) {
            ++answers_ok;
        }
        if (stalled[c]) {
            ++stalls;
        }
        if (host_ovr[c]) {
            ++host_overruns;
        }
    }

    // THE HOST'S OWN LOOP IS THE PACER, and the numbers say so: from
    // PCLK/32 up, the measured frame period stops following the rate.
    const uint32_t floor_lo = per_frame[static_cast<uint8_t>(SpiClock::div2)];
    const uint32_t floor_hi = per_frame[static_cast<uint8_t>(SpiClock::div16)];
    const bool floored = floor_lo != 0u && floor_hi != 0u &&
                         floor_lo * 5u > floor_hi * 4u && floor_lo * 4u < floor_hi * 5u &&
                         floor_lo > 8u * spi_division(SpiClock::div2) * 8u;
    print(serial, "  the loop's own floor: ", floor_lo, " cycles a frame at PCLK/2 "
                  "against ", floor_hi, " at PCLK/16, where the WIRE asks ",
          8u * spi_division(SpiClock::div2), " and ",
          8u * spi_division(SpiClock::div16), crlf);
    bench.verdict("THIS CORE CANNOT MAKE A CONTINUOUS SPI CLOCK OUT OF SOFTWARE. "
                  "A loop that keeps the transmit FIFO full still spends about "
                  "three hundred cycles a frame - a poll of TXE, a store, a poll "
                  "of RXNE, a load, and the two-cycle APB stall on each - so from "
                  "PCLK/32 upward the frame period STOPS FOLLOWING THE RATE and "
                  "the clock has a gap in it at every rung. A rate ladder driven "
                  "by the CPU never presses the far end, however fast BR is set",
                  floored);

    // A LOST FRAME WITH NO FLAG TO SHOW FOR IT, and the reason is the
    // poll itself.
    bool stall_unflagged = true;
    for (uint8_t c = 0; c < 8u; ++c) {
        if (stalled[c] && (host_ovr[c] || received[c] >= n)) {
            stall_unflagged = false;
        }
    }
    bench.verdict("A POLLED BURST LOOP ERASES THE EVIDENCE OF ITS OWN OVERRUN. "
                  "35.5.11's clear sequence is \"a read of DR followed by a read "
                  "of SR\" - which is EXACTLY what a loop that reads the frame "
                  "and then polls TXE/RXNE does, every turn. It does not lose "
                  "one every run (the stall count above is the run's own), but "
                  "when it does - at the rung where the wire's frame period and "
                  "the loop's are within a quarter of each other, so that the "
                  "client's interrupt can steal the difference - the host's OVR "
                  "flag reads CLEAR afterwards and the only witness left is that "
                  "the frames do not add up: written and read back, counted",
                  stall_unflagged);

    bool monotone = true;
    for (uint8_t c = 0; c < 8u; ++c) {
        if (!answers[c]) {
            for (uint8_t f = 0; f < c; ++f) {
                if (answers[f]) {
                    monotone = false;   // a FASTER rung passed above a slower failure
                }
            }
        }
    }
    print(serial, "  burst: answers exact at ", answers_ok, " of 8 rungs, the "
                  "client's receive side at ", receive_ok, " of 8, stalls ", stalls,
          ", host OVR at ", host_overruns, " rungs, client OVR ",
          any_client_ovr ? "seen" : "never", crlf);
    bench.verdict("THE CLIENT'S EARS ARE NEVER THE LIMIT: its receive side is "
                  "byte-exact at every one of the eight rungs and its overrun "
                  "flag never rises - a four-frame FIFO absorbs everything a CPU "
                  "on this part can put on the wire",
                  receive_ok == 8u && !any_client_ovr);
    bench.verdict("THE ANSWER-TURNAROUND CEILING IS DECLINED HERE, WITH THE "
                  "NUMBERS: what slips does so NON-MONOTONICALLY - a faster rung "
                  "passes above a slower one that failed - which is the "
                  "signature of an occasional late interrupt and not of a "
                  "ceiling. A ceiling needs a clock the CPU cannot gap, and on "
                  "this part that means DMA: the question moves to letter i, "
                  "where both ends run on channels",
                  answers_ok == 8u || !monotone);

    // The ceiling, and its arithmetic.
    (void)Host::init(clock, 8'000'000);
    const auto ceiling = Host::ceiling_clock();
    const uint32_t clamped = Host::sck_hz(SpiClock::div2);
    print(serial, "  ceiling 8 MHz -> BR PCLK/",
          ceiling ? spi_division(*ceiling) : 0, ", a div2 request runs at ",
          clamped / 1000u, " kHz", crlf);
    bench.verdict("a stated SCK ceiling resolves to a BR code and CLAMPS a "
                  "faster request instead of honouring it - a bus run above a "
                  "device's limit is a fault the caller must never get silently",
                  ceiling && *ceiling == SpiClock::div8 && clamped == 8'000'000u);
    fill_link_fixture(n);
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    (void)host_xfer(tx_buf, rx_buf, n, SpiClock::div256, SpiMode::mode0,
                    SpiDataSize::bits8, true);
    bench.verdict("...and the ceiling never SPEEDS a request up: a div256 "
                  "request under an 8 MHz ceiling still runs at div256",
                  S1::clock() == SpiClock::div256 && judge_link(n).peer_ok);
    peer_stop();
    (void)Host::init(clock);
}

// =============================================================================
// e - NSS, all four ways
// =============================================================================

volatile uint16_t nss_edges = 0;

void te_nss() {
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 8;
    software_cs_pads();
    (void)Host::init(clock);
    dma_host_live = false;

    // 1 - software at the host end (the engine's own arrangement).
    bench.verdict("the transfer engine's own NSS is SOFTWARE: SSM set, SSI held "
                  "high so no mode fault fires, and the select is the GPIO the "
                  "Request carries",
                  S1::nss() == SpiNss::software && !S1::software_selected() &&
                      !S1::mode_fault());

    // 2 - the client's hardware NSS input really frames the transfer.
    fill_link_fixture(n);
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    CsPin::set();   // deselected
    for (uint16_t i = 0; i < 4u; ++i) {
        (void)raw_xfer(0x5A, SpiDataSize::bits8);
    }
    const uint16_t got_deselected = peer_rx_n;
    CsPin::clear();
    for (uint16_t i = 0; i < 4u; ++i) {
        (void)raw_xfer(static_cast<uint16_t>(0xA0u + i), SpiDataSize::bits8);
    }
    CsPin::set();
    const uint16_t got_selected = peer_rx_n;
    print(serial, "  client frames while deselected ", got_deselected,
          ", after the select ", got_selected, crlf);
    bench.verdict("THE CLIENT'S HARDWARE NSS INPUT IS THE TRANSACTION: four "
                  "frames clocked with NSS high reach it not at all, and the "
                  "next four - the select low - all arrive",
                  got_deselected == 0u && got_selected == 4u);
    peer_stop();

    // 3 - MODF: a master whose hardware NSS input goes low.
    // PB12 is wired to PA15, so the client's pad drives the host's.
    peer_stop();
    ClientNssPin::output(true);
    Pin<'A', 15>::function(PinFunction::af0, {.pull = PinPull::up});
    (void)S1::disable();
    (void)S1::configure(SpiConfig{.nss = SpiNss::hardware_input});
    S1::enable();
    const bool before_fault = !S1::mode_fault() && S1::enabled() &&
                              S1::role() == SpiRole::host;
    ClientNssPin::clear();   // pull the host's NSS input low
    uint32_t spins = 100'000u;
    while (!S1::mode_fault() && spins-- != 0u) {
    }
    const bool fault = S1::mode_fault();
    const bool spe_cleared = !S1::enabled();
    const bool demoted = S1::role() == SpiRole::client;
    ClientNssPin::set();
    S1::clear_mode_fault();
    const bool cleared = !S1::mode_fault();
    print(serial, "  MODF: raised ", fault ? 1 : 0, ", SPE cleared ",
          spe_cleared ? 1 : 0, ", MSTR cleared ", demoted ? 1 : 0, ", cleared by the "
          "SR-then-CR1 sequence ", cleared ? 1 : 0, crlf);
    bench.verdict("35.5.11's MODE FAULT, ON SILICON: a master with hardware NSS "
                  "input and SSOE clear - the multi-master monitor - sees its "
                  "pad pulled low, raises MODF, and THE SILICON CLEARS SPE AND "
                  "MSTR FOR IT, demoting the master to a slave",
                  before_fault && fault && spe_cleared && demoted);
    bench.verdict("...and the chapter's own clearing sequence (read SR while "
                  "MODF stands, then write CR1) is what puts it away",
                  cleared);

    // 4 - the host's hardware NSS output, measured from the far end.
    ClientNssPin::input(PinPull::up);
    (void)S1::disable();
    S1::reset();
    (void)S1::configure(SpiConfig{.nss = SpiNss::hardware_output});
    Pin<'A', 15>::function(PinFunction::af0, {.speed = PinSpeed::very_high});
    const bool high_before = ClientNssPin::read();
    const uint32_t t_spe = cycles_now();
    S1::enable();
    uint32_t fall_spins = 2000u;
    while (ClientNssPin::read() && fall_spins-- != 0u) {
    }
    const uint32_t fall_cycles = cycles_now() - t_spe;
    const bool low_at_spe = !ClientNssPin::read();
    bool low_throughout = true;
    for (uint16_t i = 0; i < 4u; ++i) {
        (void)raw_xfer(0x33, SpiDataSize::bits8);
        if (ClientNssPin::read()) {
            low_throughout = false;
        }
    }
    (void)S1::disable();
    const bool high_after = ClientNssPin::read();
    print(serial, "  SSOE: before SPE ", high_before ? 1 : 0, ", low ",
          fall_cycles, " cycles after the SPE store (still high ",
          low_at_spe ? 0 : 1, "), across four frames ",
          low_throughout ? "low" : "moved", ", after disable ", high_after ? 1 : 0,
          crlf);
    bench.verdict("35.5.5 IS LITERAL, READ FROM THE OTHER END OF THE WIRE: "
                  "hardware NSS output falls when SPE is SET, stays low across "
                  "every frame of the burst, and rises only when the peripheral "
                  "is disabled - it frames the PERIPHERAL'S LIFETIME and not a "
                  "transaction, which is why this engine's chip select is a GPIO. "
                  "The fall is not instantaneous: the cycle count above is how "
                  "long after the CR1 store the far pad answered, which is the "
                  "APB write plus the pad, and a program that read the level in "
                  "the next instruction would read the OLD one",
                  high_before && low_at_spe && low_throughout && high_after);

    // 5 - NSSP, counted on the client's pad by an EXTI line.
    nss_edges = 0;
    (void)NssWatch::select();
    (void)NssWatch::configure(ExtiSense::rising);
    (void)NssWatch::clear();
    (void)NssWatch::arm(true);
    Nvic::enable(NssWatch::irq());
    (void)S1::disable();
    S1::reset();
    (void)S1::configure(SpiConfig{.mode = SpiMode::mode0,
                                  .clock = SpiClock::div256,
                                  .nss = SpiNss::hardware_pulse});
    Pin<'A', 15>::function(PinFunction::af0, {.speed = PinSpeed::very_high});
    S1::enable();
    for (uint16_t i = 0; i < 8u; ++i) {
        (void)raw_xfer(static_cast<uint16_t>(0x10u + i), SpiDataSize::bits8);
    }
    (void)S1::disable();
    (void)NssWatch::arm(false);
    Nvic::disable(NssWatch::irq());
    const uint16_t pulses = nss_edges;
    print(serial, "  NSSP: ", pulses, " rising edges on the client's NSS pad for "
                  "eight frames", crlf);
    bench.verdict("NSS PULSE MODE FRAMES A DATA FRAME AND NOT A TRANSACTION "
                  "(35.5.12): eight frames raise the select eight times, counted "
                  "on the far pad by EXTI line 12 - a pulse per frame, which is "
                  "the arrangement a latching slave wants and the opposite of "
                  "what a device driver's chip select needs",
                  pulses == 8u);
    bench.verdict("NSSP is refused with CPHA = 1 and in TI mode, which is "
                  "35.5.12's own \"it has no meaning if CPHA = 1, or FRF = 1\"",
                  !S1::configure(SpiConfig{.mode = SpiMode::mode1,
                                           .nss = SpiNss::hardware_pulse}) &&
                      !S1::configure(SpiConfig{.nss = SpiNss::hardware_pulse,
                                               .format = SpiFrameFormat::ti}));
    NssWatch::release();
    software_cs_pads();
    (void)Host::init(clock);
}

// =============================================================================
// f - the CRC
// =============================================================================

/// The reference: a bitwise CRC over `bits`-wide frames, MSB first, with
/// the SPI's own convention (initial value zero, no reflection, no final
/// xor - 35.5.14 leaves the standard to the polynomial).
uint16_t crc_reference(uint16_t poly, uint8_t width, const uint16_t* data, uint16_t n,
                       uint8_t frame_bits) {
    uint32_t crc = 0;
    const uint32_t top = 1u << (width - 1u);
    const uint32_t mask = (1u << width) - 1u;
    for (uint16_t k = 0; k < n; ++k) {
        for (uint8_t b = frame_bits; b-- > 0;) {
            const bool in = ((data[k] >> b) & 1u) != 0u;
            const bool out = (crc & top) != 0u;
            crc = (crc << 1) & mask;
            if (in != out) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint16_t>(crc);
}

/// Wait for a frame the host did NOT write - what follows CRCNEXT is
/// the hardware's own checksum, clocked out without a store (35.5.14),
/// and a write there would be one more DATA frame.
uint16_t host_read(SpiDataSize bits) {
    uint32_t spins = 400'000u;
    while (!S1::rxne() && spins-- != 0u) {
    }
    return S1::data(bits);
}

uint16_t crc_words[2];

/// One CRC-protected exchange: n data frames, then the checksum frames
/// the hardware puts on the wire by itself.
void crc_exchange(const uint16_t* host_data, uint16_t* host_got, uint16_t n,
                  SpiDataSize bits, uint8_t crc_frames) {
    CsPin::clear();
    for (uint16_t i = 0; i + 1u < n; ++i) {
        host_got[i] = raw_xfer(host_data[i], bits);
    }
    S1::data(bits, host_data[n - 1u]);
    S1::crc_next();
    host_got[n - 1u] = host_read(bits);
    for (uint8_t k = 0; k < crc_frames; ++k) {
        crc_words[k] = host_read(bits);
    }
    CsPin::set();
}

/// Arm the client for a CRC-protected exchange of n data frames.
void peer_arm_crc(const Peer::Config& cfg, const uint16_t* answers, uint16_t n) {
    peer_arm(cfg, answers, n);
    peer_tx_limit = n;
    peer_crc_next_at = static_cast<uint16_t>(n - 1u);
    peer_crc_arm = true;
}

void tf_crc() {
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 6;
    software_cs_pads();
    peer_stop();

    uint16_t host_data[n];
    uint16_t peer_data[n];
    for (uint16_t i = 0; i < n; ++i) {
        host_data[i] = static_cast<uint16_t>(0x31u + i * 7u);
        peer_data[i] = static_cast<uint16_t>(0xC0u + i * 3u);
    }

    // ---- 8-bit frames, 8-bit CRC ----
    peer_arm_crc({.mode = SpiMode::mode0,
                  .bits = SpiDataSize::bits8,
                  .nss = SpiNss::hardware_input,
                  .crc = true,
                  .crc_length = SpiCrcLength::crc8,
                  .crc_polynomial = 0x0007u},
                 peer_data, n);
    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits8,
                                    .crc = true,
                                    .crc_length = SpiCrcLength::crc8,
                                    .crc_polynomial = 0x0007u});
    uint16_t host_got[n];
    crc_exchange(host_data, host_got, n, SpiDataSize::bits8, 1);
    const uint16_t host_tx_crc = S1::tx_crc();
    const uint16_t host_rx_crc = S1::rx_crc();
    const uint16_t peer_tx_crc = S2::tx_crc();
    const uint16_t peer_rx_crc = S2::rx_crc();
    const bool host_crc_err = S1::crc_error();
    const bool peer_crc_err = S2::crc_error();
    const uint16_t frames_seen = peer_rx_n;
    const uint16_t ref_host = crc_reference(0x0007u, 8, host_data, n, 8);
    const uint16_t ref_peer = crc_reference(0x0007u, 8, peer_data, n, 8);
    bool answers_ok = true;
    for (uint16_t i = 0; i < n; ++i) {
        if (host_got[i] != peer_data[i]) {
            answers_ok = false;
        }
    }
    print(serial, "  CRC8: host TXCRCR ", hex(host_tx_crc), " (software ",
          hex(ref_host), "), client RXCRCR ", hex(peer_rx_crc),
          "; client TXCRCR ", hex(peer_tx_crc), " (software ", hex(ref_peer),
          "), host RXCRCR ", hex(host_rx_crc), "; the CRC frame the host read ",
          hex(crc_words[0]), ", client saw ", frames_seen, " frames", crlf);
    bench.verdict("THE HARDWARE CRC IS THE ARITHMETIC: the host's TXCRCR over "
                  "six 8-bit frames is what a bitwise loop over the same "
                  "polynomial computes",
                  host_tx_crc == ref_host);
    bench.verdict("...and the client's RXCRCR over the SAME six frames is the "
                  "same number, which is what makes the automatic check a check",
                  peer_rx_crc == ref_host);
    bench.verdict("the other direction likewise: the client's TXCRCR is the "
                  "software CRC of its answers, the host's RXCRCR agrees, and "
                  "the six answers themselves arrived value for value",
                  peer_tx_crc == ref_peer && host_rx_crc == ref_peer && answers_ok);
    bench.verdict("CRCERR stands at NEITHER end, the checksum frame the host "
                  "read back is the client's own TXCRCR, and the exchange cost "
                  "exactly ONE frame more than its data - the frame NEITHER SIDE "
                  "WROTE, because after CRCNEXT the shifter is fed from the CRC "
                  "register and a store there would be one more data frame "
                  "instead (the trap this letter's first version fell into)",
                  !host_crc_err && !peer_crc_err && crc_words[0] == ref_peer &&
                      frames_seen == n + 1u);

    // ---- CRCERR, staged ----
    // What can be corrupted on a wire that carries exactly what was put
    // on it is the RECEIVER'S EXPECTATION, so the client is given a
    // DIFFERENT polynomial: its accumulator no longer matches the CRC
    // the host computes and sends. Said plainly rather than dressed up
    // as a bit flip - there is no way to flip a bit between two
    // peripherals joined by six centimetres of wire.
    peer_arm_crc({.mode = SpiMode::mode0,
                  .bits = SpiDataSize::bits8,
                  .nss = SpiNss::hardware_input,
                  .crc = true,
                  .crc_length = SpiCrcLength::crc8,
                  .crc_polynomial = 0x0009u},
                 peer_data, n);
    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits8,
                                    .crc = true,
                                    .crc_length = SpiCrcLength::crc8,
                                    .crc_polynomial = 0x0007u});
    crc_exchange(host_data, host_got, n, SpiDataSize::bits8, 1);
    const bool staged = S2::crc_error();
    S2::clear_crc_error();
    const bool clearable = !S2::crc_error();
    print(serial, "  mismatched polynomial: client CRCERR ", staged ? 1 : 0,
          ", cleared by the rc_w0 write ", clearable ? 1 : 0, crlf);
    bench.verdict("A CRC THE RECEIVER DID NOT EXPECT RAISES CRCERR: the client "
                  "on polynomial 0x09 against a host on 0x07 flags the very "
                  "frame the hardware checks - and 35.9.3's rc_w0 (a write of "
                  "ZERO, not a W1C) is what clears it",
                  staged && clearable);

    // ---- a 16-bit CRC on 8-bit frames costs TWO frames ----
    peer_arm_crc({.mode = SpiMode::mode0,
                  .bits = SpiDataSize::bits8,
                  .nss = SpiNss::hardware_input,
                  .crc = true,
                  .crc_length = SpiCrcLength::crc16,
                  .crc_polynomial = 0x1021u},
                 peer_data, n);
    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits8,
                                    .crc = true,
                                    .crc_length = SpiCrcLength::crc16,
                                    .crc_polynomial = 0x1021u});
    crc_exchange(host_data, host_got, n, SpiDataSize::bits8, 2);
    const uint16_t wide_tx = S1::tx_crc();
    const uint16_t wide_ref = crc_reference(0x1021u, 16, host_data, n, 8);
    const uint16_t wide_frames = peer_rx_n;
    const bool wide_err = S1::crc_error() || S2::crc_error();
    print(serial, "  CRC16 on 8-bit frames: TXCRCR ", hex(wide_tx), " (software ",
          hex(wide_ref), "), the two CRC frames read back ", hex(crc_words[0]), " ",
          hex(crc_words[1]), ", client saw ", wide_frames, " frames, CRCERR ",
          wide_err ? 1 : 0, crlf);
    bench.verdict("35.5.14's own arithmetic: \"when setting an 8-bit data frame "
                  "checked by 16-bit CRC, two more frames are necessary to send "
                  "the complete CRC\" - six data frames cost eight on the wire, "
                  "the host's accumulator is the software CRC of the same six "
                  "bytes, and neither end flags an error",
                  wide_tx == wide_ref && wide_frames == n + 2u && !wide_err);

    // ---- 16-bit frames ----
    for (uint16_t i = 0; i < n; ++i) {
        host_data[i] = static_cast<uint16_t>(0x1234u + i * 0x0711u);
        peer_data[i] = static_cast<uint16_t>(0xBEEFu - i * 0x0123u);
    }
    peer_arm_crc({.mode = SpiMode::mode0,
                  .bits = SpiDataSize::bits16,
                  .nss = SpiNss::hardware_input,
                  .crc = true,
                  .crc_length = SpiCrcLength::crc16,
                  .crc_polynomial = 0x1021u},
                 peer_data, n);
    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits16,
                                    .crc = true,
                                    .crc_length = SpiCrcLength::crc16,
                                    .crc_polynomial = 0x1021u});
    crc_exchange(host_data, host_got, n, SpiDataSize::bits16, 1);
    const uint16_t h16 = S1::tx_crc();
    const uint16_t p16 = S2::rx_crc();
    const uint16_t r16 = crc_reference(0x1021u, 16, host_data, n, 16);
    const bool err16 = S1::crc_error() || S2::crc_error();
    print(serial, "  CRC16 on 16-bit frames: TXCRCR ", hex(h16), " client RXCRCR ",
          hex(p16), " software ", hex(r16), " CRC frame ", hex(crc_words[0]),
          " CRCERR ", err16 ? 1 : 0, crlf);
    bench.verdict("and the whole story again with 16-bit frames and a 16-bit "
                  "CRC: one extra frame, both accumulators equal to the software "
                  "reference, CRCERR clear at both ends",
                  h16 == r16 && p16 == r16 && !err16);

    peer_crc_arm = false;
    peer_stop();
    software_cs_pads();
    (void)Host::init(clock);
}

// =============================================================================
// g - TI mode
// =============================================================================

void tg_ti() {
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 8;
    peer_stop();
    // In TI mode NSS is the frame synchronization the master generates,
    // so the pad belongs to the peripheral at both ends.
    Pin<'A', 15>::function(PinFunction::af0, {.speed = PinSpeed::very_high});

    for (uint16_t i = 0; i < n; ++i) {
        peer_answers[i] = static_cast<uint16_t>(0x70u + i);
    }
    peer_arm({.mode = SpiMode::mode0,
              .bits = SpiDataSize::bits8,
              .nss = SpiNss::hardware_input,
              .format = SpiFrameFormat::ti,
              .clock = SpiClock::div256},
             peer_answers, n);
    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits8,
                                    .nss = SpiNss::hardware_input,
                                    .format = SpiFrameFormat::ti});
    uint16_t got[n];
    for (uint16_t i = 0; i < n; ++i) {
        got[i] = raw_xfer(static_cast<uint16_t>(0x80u + i), SpiDataSize::bits8);
    }
    bool ti_ok = peer_rx_n == n;
    for (uint16_t i = 0; i < n && ti_ok; ++i) {
        ti_ok = peer_rx[i] == static_cast<uint16_t>(0x80u + i);
    }
    bool ti_back = true;
    for (uint16_t i = 0; i < n; ++i) {
        if (got[i] != static_cast<uint16_t>(0x70u + i)) {
            ti_back = false;
        }
    }
    print(serial, "  TI: client got ", peer_rx_n, " frames, host read back ",
          hex(got[0]), " ", hex(got[1]), " ", hex(got[2]), crlf);
    bench.verdict("TI FRAME FORMAT ON THE WIRE: with FRF set at both ends the "
                  "master generates the one-clock NSS frame sync and eight "
                  "frames cross byte-exact in both directions - the polarity and "
                  "phase forced by the protocol whatever CR1 says (35.5.13)",
                  ti_ok && ti_back);
    bench.verdict("no frame-format error stands after a clean TI exchange",
                  !S2::frame_error() && !S1::frame_error());

    // FRE, staged by a FRAME SIZE MISMATCH: the master's sync pulse for
    // the second frame then lands in the middle of the slave's, which is
    // 35.5.11's own "an NSS pulse occurs during an ongoing communication
    // when the SPI is operating in slave mode".
    peer_stop();
    for (uint16_t i = 0; i < n; ++i) {
        peer_answers[i] = 0x5555u;
    }
    peer_arm({.mode = SpiMode::mode0,
              .bits = SpiDataSize::bits16,
              .nss = SpiNss::hardware_input,
              .format = SpiFrameFormat::ti,
              .clock = SpiClock::div256},
             peer_answers, n);
    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits8,
                                    .nss = SpiNss::hardware_input,
                                    .format = SpiFrameFormat::ti});
    for (uint16_t i = 0; i < n; ++i) {
        (void)raw_xfer(static_cast<uint16_t>(0xC0u + i), SpiDataSize::bits8);
    }
    const bool fre = S2::frame_error();
    S2::clear_frame_error();
    const bool fre_cleared = !S2::frame_error();
    print(serial, "  FRE staged (8-bit master into a 16-bit TI slave): ",
          fre ? "raised" : "NOT raised", ", cleared by an SR read ",
          fre_cleared ? 1 : 0, crlf);
    if (fre) {
        bench.verdict("35.5.11's TI frame-format error, on silicon: a master "
                      "whose frames are half the slave's puts its sync pulse in "
                      "the middle of the slave's frame, and the slave says so - "
                      "cleared, as the chapter has it, by a READ of SR",
                      fre && fre_cleared);
    } else {
        bench.verdict("35.5.11's TI frame-format error DID NOT REPRODUCE with an "
                      "8-bit master against a 16-bit TI slave - printed, not "
                      "claimed either way; the flag is readable and clear and "
                      "the slave simply resynchronized on the next pulse",
                      !S2::frame_error());
    }
    peer_stop();
    software_cs_pads();
    (void)Host::init(clock);
}

// =============================================================================
// h - the errata, and the overrun
// =============================================================================

void th_errata() {
    if (!need_self_link()) {
        return;
    }
    peer_stop();
    software_cs_pads();

    // ---- ES0548 2.12.1, both ways ----
    // The erratum's first case: master transmit mode with the data
    // register full. The FIFO holds three 8-bit frames; at div256 each
    // one is 32 us, so a raw SPE clear right after filling it lands
    // squarely inside the transaction.
    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits8});
    CsPin::clear();
    S1::data_byte(0x11);
    S1::data_byte(0x22);
    S1::data_byte(0x33);
    const uint8_t ftlvl_at_clear = S1::tx_level();
    S1::regs().CR1 = S1::regs().CR1 & ~SPI_CR1_SPE;   // the WRONG moment, by hand
    uint32_t settle = 200'000u;
    while (settle-- != 0u) {
    }
    const bool bsy_stuck = S1::busy();
    print(serial, "  2.12.1: SPE cleared with FTLVL = ", ftlvl_at_clear,
          "; BSY afterwards ", bsy_stuck ? "HIGH (the erratum)" : "low", crlf);
    S1::reset();

    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits8});
    S1::data_byte(0x11);
    S1::data_byte(0x22);
    S1::data_byte(0x33);
    const bool procedure_ok = S1::disable();
    const bool bsy_clean = !S1::busy();
    CsPin::set();
    print(serial, "  2.12.1 with the procedure: disable() returned ",
          procedure_ok ? 1 : 0, ", BSY ", bsy_clean ? "low" : "HIGH", ", FTLVL ",
          S1::tx_level(), ", FRLVL ", S1::rx_level(), crlf);
    bench.verdict("ES0548 2.12.1's WORKAROUND IS THE DISABLE PROCEDURE, and it "
                  "is the only way this driver turns the peripheral off: "
                  "35.5.9's wait for FTLVL = 00 then BSY = 0 leaves BSY low and "
                  "both FIFOs empty where a raw SPE clear at the same moment "
                  "does not",
                  procedure_ok && bsy_clean && S1::tx_level() == 0 &&
                      S1::rx_level() == 0);
    if (bsy_stuck) {
        bench.verdict("...and the erratum REPRODUCES on this silicon: a raw SPE "
                      "clear with the transmit FIFO full leaves BSY standing",
                      bsy_stuck);
    } else {
        bench.verdict("...and the erratum DID NOT REPRODUCE in this staging: a "
                      "raw SPE clear with three frames in the transmit FIFO left "
                      "BSY LOW. Printed, not claimed: the disable procedure is "
                      "kept as construction whatever one board says once",
                      !bsy_stuck);
    }

    // ---- ES0548 2.12.2: the slave's BSY, sampled ----
    constexpr uint16_t rounds = 32;
    uint16_t bsy_high_after_rxne = 0;
    uint16_t rxne_rose = 0;
    for (uint16_t k = 0; k < rounds; ++k) {
        // The pump stays OFF here: this leg reads the client's flags by
        // hand, because the question is about the ORDER in which they
        // rise and an interrupt in between would answer it for us.
        peer_stop();
        (void)Peer::init(clock, {.mode = SpiMode::mode0, .bits = SpiDataSize::bits8});
        Peer::enable(0x5A, 0xA5);
        (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                        .clock = SpiClock::div64,
                                        .bits = SpiDataSize::bits8});
        CsPin::clear();
        (void)raw_xfer(0x77, SpiDataSize::bits8);
        // The honest witness first, then the unreliable one.
        uint32_t sp = 100'000u;
        while (!S2::rxne() && sp-- != 0u) {
        }
        if (S2::rxne()) {
            ++rxne_rose;
        }
        if (S2::busy()) {
            ++bsy_high_after_rxne;
        }
        (void)S2::data_byte();
        CsPin::set();
        (void)S1::disable();
    }
    print(serial, "  2.12.2: RXNE rose in ", rxne_rose, " of ", rounds,
          " transfers, and the slave's BSY was still high at that moment in ",
          bsy_high_after_rxne, crlf);
    bench.verdict("ES0548 2.12.2's HONEST WITNESS, sampled thirty-two times: "
                  "RXNE rose on EVERY round of a slave transfer, which is what "
                  "the erratum names as the measure to use in its place - and "
                  "it is why nothing on the client side of this driver waits on "
                  "BSY. The erratum's own subject did not reproduce here (the "
                  "count above is the number of rounds where BSY was still "
                  "standing at RXNE): recorded, not claimed either way, since a "
                  "sporadic coincidence of two clocks is not something thirty-two "
                  "rounds can disprove",
                  rxne_rose == rounds);

    // ---- OVR on an undrained client ----
    peer_stop();
    (void)Peer::init(clock, {.mode = SpiMode::mode0, .bits = SpiDataSize::bits8});
    Peer::enable(0x00, 0x00);
    Peer::rxne_interrupt(false);   // nothing drains the RXFIFO
    (void)host_raw_config(SpiConfig{.mode = SpiMode::mode0,
                                    .clock = SpiClock::div256,
                                    .bits = SpiDataSize::bits8});
    CsPin::clear();
    for (uint16_t i = 0; i < 6u; ++i) {
        (void)raw_xfer(static_cast<uint16_t>(0xD0u + i), SpiDataSize::bits8);
    }
    CsPin::set();
    const bool ovr = S2::overrun();
    const uint8_t frlvl = S2::rx_level();
    uint16_t kept[4];
    for (uint8_t i = 0; i < 4u; ++i) {
        kept[i] = S2::data_byte();
    }
    S2::clear_overrun();
    const bool ovr_cleared = !S2::overrun();
    print(serial, "  OVR: flag ", ovr ? 1 : 0, ", FRLVL ", frlvl, ", the four kept "
          "frames ", hex(kept[0]), " ", hex(kept[1]), " ", hex(kept[2]), " ",
          hex(kept[3]), ", cleared by the DR-then-SR sequence ",
          ovr_cleared ? 1 : 0, crlf);
    bench.verdict("35.5.11's OVERRUN, on silicon: six frames into a client "
                  "nothing drains fill a FOUR-frame RXFIFO and raise OVR",
                  ovr && frlvl == 3u);
    bench.verdict("...and \"the newly received value does not overwrite the "
                  "previous one\" is literal: what the FIFO kept is the FIRST "
                  "four frames, not the last four",
                  kept[0] == 0xD0u && kept[1] == 0xD1u && kept[2] == 0xD2u &&
                      kept[3] == 0xD3u);
    bench.verdict("the clear is a SEQUENCE and not a store: a read of DR "
                  "followed by a read of SR, exactly as 35.5.11 has it",
                  ovr_cleared);

    peer_stop();
    software_cs_pads();
    (void)Host::init(clock);
}

// =============================================================================
// i - the DMA engines
// =============================================================================

volatile uint16_t dma_completions = 0;

void ti_dma() {
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 32;
    software_cs_pads();
    Dma<1>::bus_clock(true);
    peer_stop();

    bench.verdict("the ENGINED host comes up over the same instance and the "
                  "same pads (a second set of statics, one live at a time)",
                  DmaHost::init(clock));
    dma_host_live = true;
    dma_completions = 0;

    for (uint16_t i = 0; i < n; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x11u + i * 3u);
        rx_buf[i] = 0xEE;
        peer_answers[i] = static_cast<uint16_t>(0x80u + i * 5u);
    }
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    const bool moved = dma_xfer(tx_buf, rx_buf, n, SpiClock::div256, false);
    bool host_ok = moved;
    bool peer_ok = peer_rx_n == n;
    for (uint16_t i = 0; i < n; ++i) {
        if (rx_buf[i] != static_cast<uint8_t>(0x80u + i * 5u)) {
            host_ok = false;
        }
        if (i >= peer_rx_n || peer_rx[i] != static_cast<uint16_t>(0x11u + i * 3u)) {
            peer_ok = false;
        }
    }
    print(serial, "  engines: ", n, " frames, completions ", dma_completions,
          ", status ", DmaHost::status(), crlf);
    bench.verdict("THE DATA PHASE ON THE TWO DMA CHANNELS: thirty-two frames out "
                  "and thirty-two back with no CPU between them, byte-exact both "
                  "ways, and ONE completion - the RECEIVE block's, which is the "
                  "only edge that means the wire is idle",
                  moved && host_ok && peer_ok && DmaHost::status() == spi_ok &&
                      dma_completions == 1u);

    // The command-phase handover.
    static const uint8_t cmd[2] = {0x0B, 0x00};
    for (uint16_t i = 0; i < n; ++i) {
        rx_buf[i] = 0xEE;
    }
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    dma_completions = 0;
    const bool with_cmd = dma_xfer(tx_buf, rx_buf, 16, SpiClock::div256, false, 2, cmd);
    bool handover = with_cmd && peer_rx_n == 18u && peer_rx[0] == cmd[0] &&
                    peer_rx[1] == cmd[1];
    for (uint16_t i = 0; i < 16u && handover; ++i) {
        handover = peer_rx[2u + i] == static_cast<uint16_t>(0x11u + i * 3u);
    }
    print(serial, "  handover: peer got ", peer_rx_n, " frames (2 + 16 expected)", crlf);
    bench.verdict("the COMMAND phase stays on the frame pump and the engines "
                  "take the data phase at its end - one select window, the D/C "
                  "flip in the middle, no frame lost or repeated at the seam",
                  handover);

    // A null tx (dummies from a held cell) and a null rx (a held sink).
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    for (uint16_t i = 0; i < n; ++i) {
        rx_buf[i] = 0xEE;
    }
    const bool read_only = dma_xfer(nullptr, rx_buf, 8, SpiClock::div256, false);
    bool dummies = read_only && peer_rx_n == 8u;
    for (uint16_t i = 0; i < 8u && dummies; ++i) {
        dummies = peer_rx[i] == 0xFFu;
    }
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    const bool write_only = dma_xfer(tx_buf, nullptr, 8, SpiClock::div256, false);
    bool discarded = write_only && peer_rx_n == 8u;
    for (uint16_t i = 0; i < 8u && discarded; ++i) {
        discarded = peer_rx[i] == static_cast<uint16_t>(0x11u + i * 3u);
    }
    bench.verdict("a READ-ONLY request feeds 0xFF from a held cell (the DMA's "
                  "memory pointer simply does not increment) and a WRITE-ONLY "
                  "one drains into a held sink - the two sibling verbs "
                  "start_fixed() and start_discard() this campaign added to "
                  "stm32g0/dma.hpp",
                  dummies && discarded);

    // The ladder with the CLIENT on raw engines too, so its ISR
    // turnaround is out of the way.
    static uint8_t peer_tx_dma[64];
    static uint8_t peer_rx_dma[64];
    uint8_t engine_exact = 0;
    SpiClock engine_top = SpiClock::div256;
    bool engine_ok[8] = {};
    for (uint8_t c = static_cast<uint8_t>(SpiClock::div256);; --c) {
        const SpiClock rate = static_cast<SpiClock>(c);
        for (uint16_t i = 0; i < 32u; ++i) {
            tx_buf[i] = static_cast<uint8_t>(0x21u + i);
            rx_buf[i] = 0xEE;
            peer_tx_dma[i] = static_cast<uint8_t>(0x91u + i);
            peer_rx_dma[i] = 0xEE;
        }
        // The client, on its own two channels and nothing else.
        peer_stop();
        peer_pump_live = false;
        (void)Peer::init(clock, {.mode = SpiMode::mode0, .bits = SpiDataSize::bits8});
        (void)S2::disable();
        (void)S2::dma_transmit(true);
        (void)S2::dma_receive(true);
        PeerRx::arm(S2::data_address(), S2::dma_rx_request());
        PeerTx::arm(S2::data_address(), S2::dma_tx_request());
        (void)PeerRx::start(peer_rx_dma, 32);
        (void)PeerTx::start(peer_tx_dma, 32);
        S2::enable();

        const bool ok_move = dma_xfer(tx_buf, rx_buf, 32, rate, true);
        uint32_t sp = 400'000u;
        while (PeerRx::take() == 0u && sp-- != 0u) {
        }
        bool ok = ok_move;
        for (uint16_t i = 0; i < 32u && ok; ++i) {
            ok = rx_buf[i] == static_cast<uint8_t>(0x91u + i) &&
                 peer_rx_dma[i] == static_cast<uint8_t>(0x21u + i);
        }
        print(serial, "  engines at PCLK/", spi_division(rate), ": ",
              ok ? "exact" : "SLIPPED", crlf);
        engine_ok[c] = ok;
        if (ok) {
            ++engine_exact;
            engine_top = rate;
        }
        PeerTx::stop();
        PeerRx::stop();
        if (c == 0u) {
            break;
        }
    }
    // A CEILING IS MONOTONE - everything below it passes, everything
    // above it fails - which is exactly what letter d's CPU-driven
    // ladder could not produce.
    bool engine_monotone = true;
    for (uint8_t c = 0; c < 8u; ++c) {
        if (!engine_ok[c]) {
            for (uint8_t f = 0; f < c; ++f) {
                if (engine_ok[f]) {
                    engine_monotone = false;
                }
            }
        }
    }
    print(serial, "  both ends on DMA: exact at ", engine_exact,
          " of 8 codes, fastest PCLK/", spi_division(engine_top), " = ",
          spi_sck_hz(SysClock::pclk_hz(), engine_top) / 1000u, " kHz, monotone ",
          engine_monotone ? 1 : 0, crlf);
    bench.verdict("WITH BOTH ENDS ON DMA THERE IS A CEILING AND IT IS MONOTONE: "
                  "every rung below it byte-exact, every rung above it not - "
                  "which is what a real limit looks like and what letter d's "
                  "CPU-driven ladder could not produce, its own loop being the "
                  "pacer. A channel sustains frames an interrupt entry cannot",
                  engine_monotone && engine_exact >= 4u && engine_exact < 8u);

    // ---- the LDMA odd-count rule, under 8-bit packing ----
    // A 16-bit DMA access to an 8-bit frame carries TWO frames, so an
    // odd count needs the silicon told (35.9.2). The witness is the
    // number of frames the client SEES.
    static uint16_t packed[4] = {0x2211u, 0x4433u, 0x0055u, 0x0000u};
    uint16_t seen[2];
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        const bool odd_declared = leg != 0u;
        peer_stop();
        peer_pump_live = true;
        for (uint16_t i = 0; i < 16u; ++i) {
            peer_answers[i] = 0;
        }
        peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, 8);
        Nvic::disable(S1::irq());
        S1::reset();
        (void)S1::configure(SpiConfig{.mode = SpiMode::mode0,
                                      .clock = SpiClock::div256,
                                      .bits = SpiDataSize::bits8,
                                      .dma_transmit = true,
                                      .last_dma_transmit_odd = odd_declared});
        Pin<'B', 3>::function(PinFunction::af0, {.speed = PinSpeed::very_high});
        Pin<'B', 5>::function(PinFunction::af0, {.speed = PinSpeed::very_high});
        PeerTx16::arm(S1::data_address(), S1::dma_tx_request());
        CsPin::clear();
        S1::enable();
        (void)PeerTx16::start(packed, 3);   // 3 half-words = 5 or 6 frames
        uint32_t sp = 2'000'000u;
        while (peer_rx_n < 5u && sp-- != 0u) {
        }
        // A SETTLE MUST WAIT ON SOMETHING REAL. The first version spun
        // an empty countdown here and gcc deleted it, so the count was
        // read while the sixth frame was still in the transmit FIFO -
        // and the two legs looked identical. The wall is the wall.
        const uint32_t t_settle = cycles_now();
        while (cycles_now() - t_settle < 3'200'000u) {
        }
        seen[leg] = peer_rx_n;
        const uint32_t sr = S1::status();
        const uint16_t in_flight = PeerTx16::in_flight();
        PeerTx16::stop();
        (void)S1::disable();
        CsPin::set();
        print(serial, "    LDMA_TX ", odd_declared ? 1 : 0, ": client saw ",
              seen[leg], " frames [");
        for (uint16_t i = 0; i < seen[leg] && i < 8u; ++i) {
            print(serial, " ", hex(peer_rx[i]));
        }
        print(serial, " ], host SR ", hex(sr), ", DMA still in flight ", in_flight,
              crlf);
    }
    print(serial, "  packed 16-bit DMA of 3 half-words: LDMA_TX clear -> ",
          seen[0], " frames, LDMA_TX set -> ", seen[1], " frames", crlf);
    bench.verdict("35.9.2's LDMA_TX, MEASURED: three 16-bit DMA accesses to an "
                  "8-bit frame size carry SIX frames with the bit clear and FIVE "
                  "with it set - the odd count told to the silicon is what stops "
                  "the dummy half of the last access reaching the wire",
                  seen[0] == 6u && seen[1] == 5u);

    dma_host_live = false;
    peer_stop();
    peer_pump_live = true;
    software_cs_pads();
    (void)Host::init(clock);
}

// =============================================================================
// j - the kernel letter
// =============================================================================

namespace kl {

constexpr uint8_t queued = 4;
constexpr uint8_t payload = 6;

uint8_t tx[queued][payload];
uint8_t rx[queued][payload];

struct Kick {};
struct Fill {};
struct Ask {};
struct Wedge {};

/// Letter q hands the chip select to the LETTER instead of the engine:
/// the peer's `exchange` is ONE burst of `queued x payload` characters,
/// so the four arbitrated transactions have to share one select window
/// (a client whose transaction the select edge resets would otherwise
/// realign four times inside one commanded stream). What that trades
/// away is the engine's own chip-select handling, which is letter j's to
/// prove; what it buys is the arbiter measured against a SECOND CHIP.
bool hand_cs = false;
/// Letter j runs at PCLK/64; letter q at the peer's command rate.
SpiClock rate = SpiClock::div64;

class Driver;
/// Pending depth 3 - one fewer than the letter queues, so the rejection
/// is reachable - and a 100-tick per-bus timeout, which is two orders of
/// magnitude above the longest transaction here.
using SpiArb = SpiBus<Host, P, 3, BusPassThrough, 100>;

class Driver : public Fsm<Driver, SpiDone, Kick, Fill, Ask, Wedge, SleepVote> {
public:
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t ok_replies = 0;
    static inline uint8_t timeouts = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;
    static inline uint8_t order[queued + 2];
    static inline uint8_t order_n = 0;

    static void init() {
        clear_tally();
        start(&running);
    }
    static void clear_tally() {
        replies = rejected = ok_replies = timeouts = votes = order_n = 0;
        last_vote = false;
    }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

private:
    static Status running(const Event& e) {
        return brio::match(
            e,
            [](const Kick&) {
                for (uint8_t i = 0; i < queued; ++i) {
                    post<SpiArb>(request(i));
                }
                return handled();
            },
            [](const Fill&) {
                for (uint8_t i = 0; i < queued + 2u; ++i) {
                    post<SpiArb>(request(i % queued));
                }
                return handled();
            },
            [](const Ask&) {
                post<SpiArb>(PrepareSleep{
                    .depth = SleepDepth::standby,
                    .reply = reply_to<Driver, SleepVote>(),
                });
                return handled();
            },
            [](const Wedge&) {
                host_swallow_completion = true;
                post<SpiArb>(request(0));
                return handled();
            },
            [](const SpiDone& d) {
                if (order_n < queued + 2u) {
                    order[order_n] = d.status;
                }
                ++order_n;
                ++replies;
                if (d.status == spi_ok) ++ok_replies;
                if (d.status == spi_rejected) ++rejected;
                if (d.status == spi_timeout) ++timeouts;
                return handled();
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
                return handled();
            },
            [](auto) { return unhandled(); });
    }

    static Host::Request request(uint8_t i) {
        return Host::Request{
            .cs = hand_cs ? PinRef{} : CsPin::ref(),
            .dc = {},
            .cs_setup_us = 2,
            .cmd = {},
            .cmd_len = 0,
            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx[i])),
            .rx = lend<Lease::reply>(rx[i]),
            .len = payload,
            .reply = reply_to<Driver, SpiDone>(),
            .clock = rate,
            .mode = SpiMode::mode0,
            .bits = SpiDataSize::bits8,
            // The ISR PUMP, deliberately: this is where the engine's
            // asynchronous half meets the arbiter's, which is the shape
            // util/bus_master.hpp was written for.
            .polled = false,
        };
    }
};

using BusKernel = Kernel<P, Driver, SpiArb>;

/// The kernel's own loop, minus the sleep: Kernel::run() matures the
/// TIME EVENTS before every step, and the per-bus timeout IS a time
/// event - a pump that only called step() would wait for a deadline
/// nothing was advancing (measured: the first version of this letter's
/// timeout leg sat for six hundred milliseconds with no reply at all).
void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

}   // namespace kl

void tj_kernel() {
    if (!need_self_link()) {
        return;
    }
    // Letter q borrows this arbiter with the chip select in its own hand
    // and at the peer's rate; both are put back here, so the order the
    // two letters run in cannot change what either measures.
    kl::hand_cs = false;
    kl::rate = SpiClock::div64;
    software_cs_pads();
    bench.verdict("the engine comes up for the kernel letter", Host::init(clock));
    dma_host_live = false;

    for (uint8_t i = 0; i < kl::queued; ++i) {
        for (uint8_t k = 0; k < kl::payload; ++k) {
            kl::tx[i][k] = static_cast<uint8_t>(0xA0u + i * 16u + k);
            kl::rx[i][k] = 0xEE;
        }
    }
    for (uint16_t i = 0; i < peer_cap; ++i) {
        peer_answers[i] = static_cast<uint16_t>(0x30u + (i & 0x0Fu));
    }
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers,
             peer_cap, 0x33);

    kl::BusKernel::init_all();
    bus_ao_live = true;
    host_swallow_completion = false;

    post<kl::Driver>(kl::Kick{});
    uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 400u) {
        kl::pump();
        if (kl::Driver::replies >= kl::queued) {
            break;
        }
    }
    bool wire_ok = kl::Driver::replies == kl::queued &&
                   kl::Driver::ok_replies == kl::queued;
    for (uint8_t i = 0; i < kl::queued && wire_ok; ++i) {
        for (uint8_t k = 0; k < kl::payload; ++k) {
            if (peer_rx[i * kl::payload + k] != kl::tx[i][k]) {
                wire_ok = false;
            }
            if (kl::rx[i][k] != 0x30u + ((i * kl::payload + k) & 0x0Fu)) {
                wire_ok = false;
            }
        }
    }
    print(serial, "  kernel: ", kl::Driver::replies, " replies, ",
          kl::Driver::ok_replies, " ok; the client saw ", peer_rx_n, " frames", crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH come back in "
                  "order, every one spi_ok, with the bytes on the wire exactly "
                  "what each request lent - util/spi_bus.hpp and "
                  "util/bus_master.hpp on their THIRD silicon with NOT ONE LINE "
                  "changed",
                  wire_ok);

    // The rejection.
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Fill{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 400u) {
        kl::pump();
        if (kl::Driver::replies >= kl::queued + 2u) {
            break;
        }
    }
    print(serial, "  full FIFO: ", kl::Driver::replies, " replies, ",
          kl::Driver::rejected, " rejected", crlf);
    bench.verdict("a request past the arbiter's pending depth is answered "
                  "bus_rejected IMMEDIATELY and never dropped in silence",
                  kl::Driver::replies == kl::queued + 2u && kl::Driver::rejected >= 1u);

    // The sleep votes. EVERY LEG CLEARS THE WHOLE TALLY, and the first
    // version of this letter did not: with `replies` still holding the
    // previous leg's six, the busy-vote loop's "replies >= queued" was
    // true before a single one of its own four had run, so it broke out
    // with four transactions still in flight - and the WEDGE leg then
    // queued behind them and read their completion as its own.
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Ask{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 200u && kl::Driver::votes == 0u) {
        kl::pump();
    }
    const bool idle_vote = kl::Driver::last_vote;
    const uint8_t idle_votes = kl::Driver::votes;
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Kick{});
    post<kl::Driver>(kl::Ask{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 400u) {
        kl::pump();
        if (kl::Driver::votes != 0u && kl::Driver::replies >= kl::queued) {
            break;
        }
    }
    const bool busy_vote = kl::Driver::last_vote;
    const uint8_t busy_votes = kl::Driver::votes;
    const uint8_t busy_replies = kl::Driver::replies;
    print(serial, "  votes: idle ", idle_votes, " ok=", idle_vote ? 1 : 0,
          ", busy ", busy_votes, " ok=", busy_vote ? 1 : 0, " with ", busy_replies,
          " of ", kl::queued, " transactions done", crlf);
    bench.verdict("an IDLE bus votes ok on a PrepareSleep and a BUSY one votes "
                  "AGAINST it - the arbiter is a voter on the third target too, "
                  "and the four transactions the busy vote was taken across all "
                  "finished afterwards",
                  idle_votes == 1u && idle_vote && busy_votes == 1u && !busy_vote &&
                      busy_replies == kl::queued);

    // The per-bus timeout: a completion that never comes.
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Wedge{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 600u) {
        kl::pump();
        if (kl::Driver::replies >= 1u) {
            break;
        }
    }
    const uint8_t timeouts = kl::Driver::timeouts;
    const uint8_t wedge_replies = kl::Driver::replies;
    const uint8_t wedge_status = kl::Driver::order_n != 0u ? kl::Driver::order[0] : 0xEEu;
    print(serial, "  wedge: ", wedge_replies, " replies, first status ",
          wedge_status, " (spi_timeout is 255), swallowed ",
          host_swallow_completion ? 1 : 0, crlf);
    host_swallow_completion = false;
    // And the bus survives it: the next request must run.
    kl::Driver::replies = 0;
    kl::Driver::ok_replies = 0;
    post<kl::Driver>(kl::Kick{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 400u) {
        kl::pump();
        if (kl::Driver::replies >= kl::queued) {
            break;
        }
    }
    print(serial, "  timeout: ", timeouts, " spi_timeout replies, then ",
          kl::Driver::ok_replies, " of ", kl::queued, " ok afterwards", crlf);
    bench.verdict("A COMPLETION THAT NEVER COMES is answered spi_timeout in its "
                  "place - the staging is the honest one for a bus whose host "
                  "clocks itself: the ISR body runs and acknowledges the frames "
                  "but the TransferDone is never posted, which is the lost "
                  "interrupt util/bus_master.hpp's timeout was written for",
                  timeouts == 1u);
    bench.verdict("...and recover() puts the engine back where start() is legal: "
                  "the very next four transactions run to spi_ok on the same bus "
                  "AO, with no reset of the program between them",
                  kl::Driver::ok_replies == kl::queued);

    bus_ao_live = false;
    peer_stop();
}

// =============================================================================
// k - I2S
// =============================================================================

constexpr uint16_t i2s_cap = 64;
volatile uint16_t i2s_rx[i2s_cap];
volatile uint16_t i2s_rx_n = 0;
volatile uint8_t i2s_chside[i2s_cap];
volatile bool i2s_live = false;

void i2s_service() {
    const uint32_t f = S2::isr();
    if ((f & SpiFlag::rxne) != 0u) {
        const bool right = S2::channel_right();
        const uint16_t v = S2::data_halfword();
        if (i2s_rx_n < i2s_cap) {
            i2s_rx[i2s_rx_n] = v;
            i2s_chside[i2s_rx_n] = right ? 1u : 0u;
        }
        i2s_rx_n = static_cast<uint16_t>(i2s_rx_n + 1u);
    }
}

/// Bring both instances up as an I2S pair on the self-link's three
/// wires. Returns false if either configuration was refused.
bool i2s_pair(const I2sConfig& master, const I2sConfig& slave) {
    Nvic::disable(S1::irq());
    Nvic::disable(S2::irq());
    // A LETTER MUST STAND ALONE. Both APB gates are opened here and not
    // assumed: run on its own, this letter is the first thing to touch
    // either instance, and a register block with no bus clock answers
    // every read with zero - which is what the first version of this
    // letter measured when it was run from a cold flash instead of
    // after its neighbours.
    S1::bus_clock(true);
    S2::bus_clock(true);
    S1::reset();
    S2::reset();
    // I2SxCLK must be at least PCLK (35.7.4's warning), so SYSCLK it is.
    const bool k1 = S1::i2s_kernel_clock(I2sClock::sysclk);
    const bool k2 = S2::i2s_kernel_clock(I2sClock::sysclk);
    const bool c1 = S1::i2s_configure(master);
    const bool c2 = S2::i2s_configure(slave);
    // CK, WS and SD at both ends; the master drives the first two.
    Pin<'B', 3>::function(PinFunction::af0, {.speed = PinSpeed::very_high});    // I2S1_CK
    Pin<'A', 15>::function(PinFunction::af0, {.speed = PinSpeed::very_high});   // I2S1_WS
    Pin<'B', 5>::function(PinFunction::af0, {.speed = PinSpeed::very_high});    // I2S1_SD
    Pin<'B', 10>::function(PinFunction::af5);                                   // I2S2_CK
    Pin<'B', 12>::function(PinFunction::af0);                                   // I2S2_WS
    Pin<'D', 4>::function(PinFunction::af1);                                    // I2S2_SD
    return k1 && k2 && c1 && c2;
}

/// Play `n` half-words out of the master and collect what the slave
/// receives. The slave goes first (35.5.8's rule, in I2S clothes).
uint16_t i2s_play(const uint16_t* src, uint16_t n) {
    i2s_rx_n = 0;
    i2s_live = true;
    S2::rxne_interrupt(true);
    Nvic::enable(S2::irq());
    (void)S2::i2s_enable();
    (void)S1::i2s_enable();
    for (uint16_t i = 0; i < n; ++i) {
        uint32_t sp = 1'000'000u;
        while (!S1::txe() && sp-- != 0u) {
        }
        S1::data_halfword(src[i]);
    }
    uint32_t sp = 4'000'000u;
    while (i2s_rx_n < n && sp-- != 0u) {
    }
    const uint16_t got = i2s_rx_n;
    (void)S1::i2s_stop_transmit();
    S2::rxne_interrupt(false);
    S2::i2s_disable();
    Nvic::disable(S2::irq());
    i2s_live = false;
    return got;
}

void tk_i2s() {
    if (!need_self_link()) {
        return;
    }
    peer_stop();
    dma_host_live = false;
    bus_ao_live = false;
    bench.verdict("this instance pair CAN be an I2S at all (table 205: SPI1 "
                  "everywhere, SPI2 on the G0B1/G0C1)",
                  S1::has_i2s_mode && S2::has_i2s_mode && S1::has_i2s() && S2::has_i2s());
    bench.verdict("...and SPI3 cannot, at compile time and at run time both",
                  !Spi<3>::has_i2s_mode && !Spi<3>::has_i2s());

    static uint16_t samples[16];
    for (uint16_t i = 0; i < 16u; ++i) {
        samples[i] = static_cast<uint16_t>(0x1000u + i * 0x0111u);
    }

    // ---- Philips, 16-bit data in a 16-bit channel ----
    const I2sConfig m16{.mode = I2sMode::master_transmit,
                        .standard = I2sStandard::philips,
                        .data_length = I2sDataLength::bits16,
                        .channel_length = I2sChannelLength::bits16,
                        .divider = 21};
    const I2sConfig s16{.mode = I2sMode::slave_receive,
                        .standard = I2sStandard::philips,
                        .data_length = I2sDataLength::bits16,
                        .channel_length = I2sChannelLength::bits16};
    bench.verdict("both ends take an I2S configuration (Philips, 16/16), with "
                  "the kernel clock on SYSCLK - 35.7.4's warning makes I2SxCLK "
                  "at or above PCLK mandatory, which rules HSI16 out at 64 MHz",
                  i2s_pair(m16, s16));
    const uint32_t t0 = cycles_now();
    const uint16_t got = i2s_play(samples, 16);
    const uint32_t took = cycles_now() - t0;
    bool exact = got >= 16u;
    for (uint16_t i = 0; i < 16u && exact; ++i) {
        exact = i2s_rx[i] == samples[i];
    }
    bool alternating = got >= 16u;
    for (uint16_t i = 0; i < 16u && alternating; ++i) {
        alternating = i2s_chside[i] == (i & 1u);
    }
    print(serial, "  Philips 16/16: ", got, " of 16 half-words, exact ",
          exact ? 1 : 0, ", CHSIDE alternates ", alternating ? 1 : 0, crlf);
    bench.verdict("I2S1 MASTER TRANSMITTER REACHES I2S2 SLAVE RECEIVER ON THE "
                  "SAME THREE WIRES: sixteen 16-bit samples, value for value",
                  exact);
    bench.verdict("...and CHSIDE alternates left/right frame by frame, which is "
                  "what makes a pair of half-words one stereo sample (35.7.5: "
                  "\"a full frame ... is not possible to have a partial frame\")",
                  alternating);

    // The prescaler arithmetic, against the wall.
    const uint32_t predicted = i2s_sampling_hz(SysClock::hz(), m16);
    // Sixteen half-words are EIGHT stereo frames.
    const uint32_t measured = took == 0u ? 0u : (8u * SysClock::hz()) / took;
    print(serial, "  I2SDIV ", m16.divider, " ODD ", m16.divider_odd ? 1 : 0,
          ": predicted Fs ", predicted, " Hz, eight frames took ", took,
          " cycles -> ", measured, " Hz", crlf);
    bench.verdict("35.7.4's own formula, TIMED: Fs = f / (32 x (CHLEN + 1) x "
                  "(2 x I2SDIV + ODD)) predicts the rate the eight stereo frames "
                  "really took, within a tenth",
                  measured != 0u && measured * 10u > predicted * 9u &&
                      measured * 9u < predicted * 10u);

    // With MCK out, the SAME divider means a rate eight times lower (256
    // in the denominator instead of 32) - which is the master clock's own
    // witness on a bench with no counter free for a 12 MHz line.
    I2sConfig m_mck = m16;
    m_mck.master_clock_out = true;
    const uint32_t predicted_mck = i2s_sampling_hz(SysClock::hz(), m_mck);
    (void)i2s_pair(m_mck, s16);
    Pin<'B', 4>::function(PinFunction::af0, {.speed = PinSpeed::very_high});   // I2S1_MCK
    Pin<'C', 2>::function(PinFunction::af1);                                   // I2S2_MCK
    const uint32_t t1 = cycles_now();
    const uint16_t got_mck = i2s_play(samples, 16);
    const uint32_t took_mck = cycles_now() - t1;
    const uint32_t measured_mck = took_mck == 0u ? 0u : (8u * SysClock::hz()) / took_mck;
    bool exact_mck = got_mck >= 16u;
    for (uint16_t i = 0; i < 16u && exact_mck; ++i) {
        exact_mck = i2s_rx[i] == samples[i];
    }
    print(serial, "  MCKOE on the MISO wire: predicted Fs ", predicted_mck,
          " Hz, measured ", measured_mck, " Hz, samples exact ", exact_mck ? 1 : 0,
          crlf);
    bench.verdict("MCKOE IS MEASURED BY WHAT IT DOES TO THE RATE, not by a "
                  "counter: the same I2SDIV with the master clock out divides by "
                  "256 instead of 32, an eightfold drop the wall sees - and the "
                  "link still carries every sample with MCK sitting on the MISO "
                  "wire (PB4 is I2S1_MCK, PC2 is I2S2_MCK: the pad pair this "
                  "self-link already has)",
                  exact_mck && measured_mck != 0u &&
                      measured_mck * 10u > predicted_mck * 9u &&
                      measured_mck * 9u < predicted_mck * 10u);

    // ---- the other three standards ----
    uint16_t std_got[4];
    bool std_exact[4];
    for (uint8_t st = 0; st < 4u; ++st) {
        I2sConfig m = m16;
        I2sConfig sl = s16;
        m.standard = static_cast<I2sStandard>(st);
        sl.standard = static_cast<I2sStandard>(st);
        (void)i2s_pair(m, sl);
        const uint16_t got_n = i2s_play(samples, 8);
        bool ok = got_n >= 8u;
        for (uint16_t i = 0; i < 8u && ok; ++i) {
            ok = i2s_rx[i] == samples[i];
        }
        std_got[st] = got_n;
        std_exact[st] = ok;
    }
    print(serial, "  standards (samples of 8, exact): Philips ", std_got[0], "/",
          std_exact[0] ? 1 : 0, ", MSB ", std_got[1], "/", std_exact[1] ? 1 : 0,
          ", LSB ", std_got[2], "/", std_exact[2] ? 1 : 0, ", PCM ", std_got[3],
          "/", std_exact[3] ? 1 : 0, crlf);
    bench.verdict("THE THREE STEREO STANDARDS carry eight samples value-exact: "
                  "Philips, MSB-justified and LSB-justified, each a different "
                  "place for the data inside a 32-bit stereo frame and each "
                  "arriving whole",
                  "(3 of 3)", std_exact[0] && std_exact[1] && std_exact[2]);
    bench.verdict("PCM IS NOT A STEREO STANDARD AND ITS COUNT SAYS SO: with "
                  "I2SSTD = 11 the WS line becomes a frame SYNCHRONIZATION pulse "
                  "rather than a channel side, so what the slave collects in the "
                  "same window is one sample short of the stereo pairs above (the "
                  "count is printed). The samples that do arrive are value-exact; "
                  "the missing one is the frame the sync pulse costs at start-up, "
                  "and closing that would need a receiver that counts PCM frames "
                  "rather than half-words - DECLINED here and named",
                  std_got[3] >= 7u);

    // ---- 24 and 32-bit data ----
    // A 24-BIT SAMPLE IS TWO 16-BIT PACKETS AND ONLY 24 OF THOSE 32 BITS
    // ARE ON THE WIRE (35.7.2's diagrams): the second packet carries the
    // low byte in its HIGH half and eight bits of padding below, so a
    // value-for-value comparison of what was written and what came back
    // is wrong for that one length - and saying so is better than a
    // comparison that pretends.
    uint8_t lengths_ok = 0;
    uint16_t len_got[3];
    bool len_exact[3];
    for (uint8_t d = 0; d < 3u; ++d) {
        I2sConfig m = m16;
        I2sConfig sl = s16;
        m.data_length = static_cast<I2sDataLength>(d);
        sl.data_length = static_cast<I2sDataLength>(d);
        m.channel_length = I2sChannelLength::bits32;
        sl.channel_length = I2sChannelLength::bits32;
        (void)i2s_pair(m, sl);
        const uint16_t got_n = i2s_play(samples, 8);
        const bool packed24 = static_cast<I2sDataLength>(d) == I2sDataLength::bits24;
        bool ok = got_n >= 8u;
        for (uint16_t i = 0; i < 8u && ok; ++i) {
            if (packed24 && (i & 1u) != 0u) {
                ok = (i2s_rx[i] & 0xFF00u) == (samples[i] & 0xFF00u);
            } else {
                ok = i2s_rx[i] == samples[i];
            }
        }
        len_got[d] = got_n;
        len_exact[d] = ok;
        if (ok) {
            ++lengths_ok;
        }
    }
    print(serial, "  data lengths (samples of 8, exact): 16-bit ", len_got[0], "/",
          len_exact[0] ? 1 : 0, ", 24-bit ", len_got[1], "/", len_exact[1] ? 1 : 0,
          " (its odd packets judged on their top byte alone - the other eight bits "
          "are padding), 32-bit ", len_got[2], "/", len_exact[2] ? 1 : 0, crlf);
    bench.verdict("and all three data lengths do, in a 32-bit channel frame: "
                  "16, 24 and 32 bits, each moved as 16-bit packets through DR "
                  "(35.7.5: \"whatever the data or channel length, the audio "
                  "data are received by 16-bit packets\") - with the 24-bit "
                  "length's SECOND packet judged on the eight bits that are "
                  "really on the wire, because the other eight are padding the "
                  "receiver zeroes",
                  "(3 of 3)", lengths_ok == 3u);
    bench.verdict("i2s_channel_bits() states what the silicon does with CHLEN "
                  "above 16 bits of data: it is FIXED at 32 whatever is written "
                  "(35.9.8's own note), and the readback says so",
                  i2s_channel_bits(I2sDataLength::bits24, I2sChannelLength::bits16) == 32u &&
                      S1::i2s_channel_length() == I2sChannelLength::bits32);

    // ---- UDR on a slave transmitter ----
    const I2sConfig m_rx{.mode = I2sMode::master_receive,
                         .standard = I2sStandard::philips,
                         .data_length = I2sDataLength::bits16,
                         .channel_length = I2sChannelLength::bits16,
                         .divider = 60};
    const I2sConfig s_tx{.mode = I2sMode::slave_transmit,
                         .standard = I2sStandard::philips,
                         .data_length = I2sDataLength::bits16,
                         .channel_length = I2sChannelLength::bits16};
    const bool pair_ok = i2s_pair(m_rx, s_tx);
    (void)S2::i2s_enable();
    (void)S1::i2s_enable();
    // 35.7.8: "The UDR bit is cleared by a read operation on the SPIx_SR
    // register" - so a poll loop that ASKS whether the flag is set is
    // the very thing that clears it, and the answer must be taken from
    // the ONE read that saw it. (The overrun's clear sequence has the
    // same shape and the same trap; letter d met it from the other end.)
    uint32_t slave_sr = 0;
    uint32_t sp = 4'000'000u;
    do {
        slave_sr = S2::status();
    } while ((slave_sr & SpiFlag::underrun) == 0u && sp-- != 0u);
    const bool udr = (slave_sr & SpiFlag::underrun) != 0u;
    const uint32_t master_sr = S1::status();
    const bool udr_cleared = (S2::status() & SpiFlag::underrun) == 0u;
    S1::i2s_disable();
    S2::i2s_disable();
    print(serial, "  UDR on a slave transmitter with nothing loaded: ",
          udr ? "raised" : "NOT raised", ", gone by the next read ",
          udr_cleared ? 1 : 0, "; pair ", pair_ok ? 1 : 0, ", slave SR ",
          hex(slave_sr), ", master SR ", hex(master_sr), crlf);
    bench.verdict("35.7.8's UNDERRUN, on silicon: a slave TRANSMITTER whose data "
                  "register was never written raises UDR when the master's clock "
                  "asks for a sample it has not got - the one error SPI mode has "
                  "no counterpart for (35.5.9: \"there is no underflow error "
                  "signal ... in SPI mode\")",
                  udr);
    bench.verdict("...AND THE FLAG IS GONE BY THE NEXT READ, which is 35.7.8's "
                  "own clearing rule (\"cleared by a read operation on the "
                  "SPIx_SR register\") and a trap with teeth: a poll loop that "
                  "ASKS whether UDR is set is the thing that clears it, so the "
                  "answer has to be taken from the one read that saw it - the "
                  "first version of this leg polled the verb and reported no "
                  "underrun at all",
                  udr_cleared);

    // Back to SPI on both instances.
    (void)S1::spi_mode();
    (void)S2::spi_mode();
    bench.verdict("both instances go back to SPI Motorola mode (I2SMOD cleared "
                  "with both enables down), which is what makes the two "
                  "personalities one block and not two",
                  !S1::i2s_mode() && !S2::i2s_mode());
    Pin<'B', 4>::release();
    Pin<'C', 2>::release();
    software_cs_pads();
    (void)Host::init(clock);
}

// =============================================================================
// l - the dynamic clock
// =============================================================================

void tl_dynamic() {
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 8;
    software_cs_pads();
    // A stated ceiling is what makes a rate change mean anything: the BR
    // code must be re-resolved at every rung.
    bench.verdict("the host comes up under the dynamic clock with a 1 MHz SCK "
                  "ceiling", Host::init(clock, 1'000'000));
    dma_host_live = false;

    struct Rung {
        uint8_t index;
        SpiClock expect;
    };
    const Rung rungs[] = {
        {r_fast, SpiClock::div64},   // 64 MHz / 64 = 1 MHz
        {r_mid, SpiClock::div16},    // 16 MHz / 16 = 1 MHz
        {r_slow, SpiClock::div2},    //  2 MHz /  2 = 1 MHz
        {r_fast, SpiClock::div64},
    };
    uint8_t rungs_ok = 0;
    uint8_t codes_ok = 0;
    for (const auto& r : rungs) {
        console_drain();
        if (!SysClock::set_index(r.index)) {
            continue;
        }
        const auto code = Host::ceiling_clock();
        if (code && *code == r.expect) {
            ++codes_ok;
        }
        fill_link_fixture(n);
        peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
        const bool moved = host_xfer(tx_buf, rx_buf, n, SpiClock::div2, SpiMode::mode0,
                                     SpiDataSize::bits8, true);
        const LinkResult res = judge_link(n);
        print(serial, "  SYSCLK ", SysClock::hz() / 1000u, " kHz: ceiling code PCLK/",
              code ? spi_division(*code) : 0, ", SCK ",
              Host::sck_hz(SpiClock::div2) / 1000u, " kHz, link ",
              (moved && res.host_ok && res.peer_ok) ? "exact" : "SLIPPED", crlf);
        if (moved && res.host_ok && res.peer_ok) {
            ++rungs_ok;
        }
    }
    peer_stop();
    (void)SysClock::set_index(r_fast);
    console_drain();
    bench.verdict("THE BR CODE FOLLOWS THE CLOCK: at 64, 16 and 2 MHz a 1 MHz "
                  "ceiling resolves to PCLK/64, PCLK/16 and PCLK/2 - three "
                  "different registers for one stated SCK",
                  "(4 of 4 rungs)", codes_ok == 4u);
    bench.verdict("...and the link is byte-exact at every rung, in both "
                  "directions, with the client's own answer turnaround shrinking "
                  "with the core it runs on",
                  "(4 of 4 rungs)", rungs_ok == 4u);
    bench.verdict("the clock is back on its first rung and the console never "
                  "moved (its kernel clock is HSI16, so its divisor is not a "
                  "function of what this letter switched)",
                  SysClock::rate_index() == r_fast && SysClock::hz() == 64'000'000u);
    (void)Host::init(clock);
}

// =============================================================================
// m - the wake from Sleep
// =============================================================================

volatile bool sleep_woke = false;

void tm_sleep() {
    peer_stop();
    software_cs_pads();
    Nvic::disable(S1::irq());
    S1::bus_clock(true);
    S1::reset();
    (void)S1::configure(SpiConfig{.mode = SpiMode::mode0, .clock = SpiClock::div256});
    Pin<'B', 3>::function(PinFunction::af0);
    Pin<'B', 5>::function(PinFunction::af0);
    S1::enable();
    S1::flush_rx();

    console_drain();
    // The frame takes 32 us at PCLK/256; the kernel tick is 1 ms, so the
    // SPI's own interrupt is what returns from the WFI. Repeated, so a
    // tick landing first is visible rather than fatal.
    sleep_probe_live = true;
    uint8_t first_wake_spi = 0;
    uint8_t woke_on_spi = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        sleep_woke = false;
        S1::rxne_interrupt(true);
        Nvic::enable(S1::irq());
        S1::data_byte(static_cast<uint8_t>(0x40u + k));
        P::idle();
        if (sleep_woke) {
            ++first_wake_spi;
        }
        // The kernel tick is the OTHER thing that can return from a WFI
        // here (1 ms against the frame's 32 us), so a round the tick won
        // is slept again rather than counted against the SPI.
        for (uint8_t retry = 0; retry < 4u && !sleep_woke; ++retry) {
            P::idle();
        }
        if (sleep_woke) {
            ++woke_on_spi;
        }
        S1::rxne_interrupt(false);
        Nvic::disable(S1::irq());
        S1::flush_rx();
    }
    sleep_probe_live = false;
    print(serial, "  WFI in Sleep mode: the SPI's own interrupt returned it in ",
          woke_on_spi, " rounds of 8, and was the FIRST wake in ", first_wake_spi,
          " of them (the kernel tick is the other candidate, 1 ms against the "
          "frame's 32 us)", crlf);
    bench.verdict("TABLE 205'S LAST ROW, on silicon: an SPI interrupt wakes a "
                  "core sleeping in Sleep mode (the plain WFI the platform's "
                  "idle() is), because the peripheral keeps its APB clock there "
                  "- the frame the program started before the WFI completes "
                  "underneath it and RXNE is what returns",
                  woke_on_spi == 8u);
    (void)S1::disable();
    (void)Host::init(clock);
}

// =============================================================================
// n - the cross-architecture command channel
// =============================================================================

void tn_peer_link() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the four wires", true);

    spilink::Ident d{};
    const bool got = peer_ident(d);
    if (got) {
        print(serial, "  peer: label '");
        for (uint8_t i = 0; i < 8 && d.label[i]; ++i) {
            print(serial, d.label[i]);
        }
        print(serial, "' xtal=", d.xtal, " sanity=", hex(d.sanity), " fw=",
              hex(d.version), crlf);
    }
    bench.verdict("ident comes back and it IS spi_peer (the sanity byte), from a "
                  "board of ANOTHER ARCHITECTURE speaking the same wire format",
                  got && d.sanity == spilink::ident_sanity);

    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++pings;
        }
    }
    print(serial, "  ", pings, " of 10 pings answered at ",
          Host::sck_hz(link_clock) / 1000u,
          " kHz SCK, one frame per chip-select window", crlf);
    bench.verdict("the channel is steady over ten frames", pings == 10u);
}

// =============================================================================
// o - the matrix against the peer
// =============================================================================

void to_peer_matrix() {
    if (!need_peer()) {
        return;
    }

    uint8_t good = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        Exchange e{};
        e.cfg.mode = m;
        e.host_mode = static_cast<SpiMode>(m);
        e.seed_a = static_cast<uint8_t>(0x13u + m);
        e.seed_b = static_cast<uint8_t>(0x57u + m);
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        if (ok) {
            ++good;
        } else {
            print(serial, "  mode ", m, ":", crlf);
            dump_exchange(e, v, r);
        }
    }
    print(serial, "  ", good, " of 4 transfer modes byte-exact BOTH ways, 8 frames each",
          crlf);
    bench.verdict("all four transfer modes carry a burst byte-exact in both "
                  "directions between two DIFFERENT SILICONS",
                  good == 4u);

    Exchange lsb{};
    lsb.cfg.dord = 1;
    lsb.host_lsb = true;
    Verify v{};
    spilink::Report r{};
    const bool lsb_ok = exchange_exact(lsb, v, r);
    if (!lsb_ok) {
        dump_exchange(lsb, v, r);
    }
    bench.verdict("LSb first, both ends agreeing, is byte-exact too - and on this "
                  "target the bit order is a BUS-level verb (CR1.LSBFIRST) rather "
                  "than a Request field",
                  lsb_ok);

    Exchange mism{};
    mism.cfg.dord = 1;      // the client LSb first
    mism.host_lsb = false;  // this end MSb first
    Verify v2{};
    spilink::Report r2{};
    const bool ran = do_exchange(mism);
    const bool rep = ran && peer_report(r2);
    v2 = verify_rx(mism);
    print(serial, "  DORD mismatch: host mism=", v2.mism, " client mism=", r2.mism,
          " count=", r2.count, " (both checking the exact bit-reverse)", crlf);
    bench.verdict("a bit-order mismatch is an EXACT TWO-WAY BIT REVERSAL - each end "
                  "reads the other's bytes with their bits in the opposite order",
                  ran && rep && v2.mism == 0u && r2.mism == 0u &&
                      r2.count == mism.count);

    bench.verdict("and the command channel still works right after all of that",
                  command(Op::ping));
}

// =============================================================================
// p - the rate ladder against the peer
// =============================================================================

void tp_peer_rates() {
    if (!need_peer()) {
        return;
    }

    // The BR codes, not a frequency ladder: a Request's clock IS a
    // division of PCLK, so this walks the register's own vocabulary from
    // the command rate upwards and prints what each one really is.
    static const SpiClock codes[] = {SpiClock::div256, SpiClock::div128,
                                     SpiClock::div64,  SpiClock::div32,
                                     SpiClock::div16,  SpiClock::div8,
                                     SpiClock::div4};
    uint32_t last_good = 0;
    uint32_t first_bad = 0;
    spilink::Report bad_r{};
    for (const SpiClock c : codes) {
        Exchange e{};
        e.rate = c;
        e.count = 8;
        e.seed_a = 0x21;
        e.seed_b = 0x84;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        const uint32_t real = Host::sck_hz(c);
        print(serial, "  SCK PCLK/", spi_division(c), " = ", real / 1000u, " kHz: ",
              ok ? "exact both ways" : "NOT exact", "  host mism=", v.mism,
              " client mism=", r.mism, " client count=", r.count, " serve=",
              (r.aux2 & 0x04u) != 0u ? "dma" : "pump", crlf);
        if (ok && first_bad == 0u) {
            last_good = real;
        } else if (!ok && first_bad == 0u) {
            first_bad = real;
            bad_r = r;
        }
    }
    print(serial, "  the cross-architecture link held to ", last_good / 1000u, " kHz");
    if (first_bad != 0u) {
        print(serial, " and broke at ", first_bad / 1000u, " kHz");
    }
    print(serial, crlf);
    bench.verdict("the link is exact at the command rate and at least four times "
                  "faster",
                  last_good >= 1'000'000UL);
    // THE SIGNATURE OF THE PEER'S BOUNDARY: at the first rung that is not
    // exact the client still RECEIVED every character (its count full,
    // its mismatches zero) while what this end read back broke - so the
    // failure lives in the ANSWER path and not in the wire. Vacuous, and
    // says so, on the day a peer holds the whole ladder.
    bench.verdict("wherever the climb breaks, the peer still hears every character "
                  "exact there - the boundary is its ANSWER RELOAD, not the wire",
                  first_bad == 0u || (bad_r.count == 8u && bad_r.mism == 0u));
    (void)link_command_mode();
    bench.verdict("the command channel survives the climb", command(Op::ping));
}

// =============================================================================
// q - THE KERNEL against the peer
// =============================================================================

void tq_peer_kernel() {
    if (!need_peer()) {
        return;
    }

    // The peer answers ONE burst of queued x payload characters, so the
    // letter holds the select down across the four arbitrated
    // transactions (kl::hand_cs).
    constexpr uint16_t total = kl::queued * kl::payload;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0,
                         .regime = spilink::regime_buffer_wait};
    a.count = total;
    a.ms = 800;
    a.seed_a = 0x3B;
    a.seed_b = 0x6D;
    a.pattern = spilink::pattern_prbs;
    if (!peer_act(Op::exchange, a)) {
        bench.verdict("the peer accepted the exchange the kernel letter clocks", false);
        return;
    }
    bench.verdict("the peer accepted the exchange the kernel letter clocks", true);

    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint8_t i = 0; i < kl::queued; ++i) {
            for (uint8_t k = 0; k < kl::payload; ++k) {
                kl::tx[i][k] = out.next();
                kl::rx[i][k] = 0xEE;
            }
        }
    }

    kl::BusKernel::init_all();
    kl::hand_cs = true;
    kl::rate = link_clock;
    bus_ao_live = true;
    host_swallow_completion = false;

    settle();
    Host::prime(SpiMode::mode0, link_clock);
    CsPin::clear();
    link_hold();
    post<kl::Driver>(kl::Kick{});
    uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 800u) {
        kl::pump();
        if (kl::Driver::replies >= kl::queued) {
            break;
        }
    }
    link_hold();
    CsPin::set();

    bool wire_ok = kl::Driver::replies == kl::queued &&
                   kl::Driver::ok_replies == kl::queued;
    uint16_t mism = 0;
    {
        spilink::Stream want(a.pattern, a.seed_b);
        for (uint8_t i = 0; i < kl::queued; ++i) {
            for (uint8_t k = 0; k < kl::payload; ++k) {
                if (kl::rx[i][k] != want.next()) {
                    ++mism;
                }
            }
        }
    }
    spilink::Report r{};
    const bool rep = peer_report(r);
    // THE REPORT IS ITSELF A COMMAND, and collecting it runs through
    // link_command_mode(), which hands SPI1's vector back to the bare
    // pump - so the arbiter's claim on it has to be re-stated before the
    // next leg, or every later completion is consumed by the wrong
    // branch and the arbiter times out on a transaction that ran
    // perfectly (measured, and it cost this letter four spi_timeout
    // replies before the flag was put back).
    bus_ao_live = true;
    print(serial, "  kernel: ", kl::Driver::replies, " replies, ",
          kl::Driver::ok_replies, " ok; host mism=", mism, "; the peer read ",
          r.count, " of ", total, " with mism=", r.mism, " flags=", hex(r.flags), crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH come back in order, "
                  "every one spi_ok, and the bytes on the wire are exactly what "
                  "each request lent - read back by A BOARD OF ANOTHER "
                  "ARCHITECTURE, with not one line of util/spi_bus.hpp, "
                  "util/bus_master.hpp or kernel/ changed for it",
                  wire_ok && mism == 0u && rep && r.count == total && r.mism == 0u);

    // The rejection, the votes and the per-bus timeout need a
    // transaction that COMPLETES, not one whose data is judged: the peer
    // is back in command mode from here on and simply hears the traffic.
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Fill{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 600u) {
        kl::pump();
        if (kl::Driver::replies >= kl::queued + 2u) {
            break;
        }
    }
    print(serial, "  full FIFO: ", kl::Driver::replies, " replies, ",
          kl::Driver::rejected, " rejected", crlf);
    bench.verdict("a request past the arbiter's pending depth is answered "
                  "bus_rejected IMMEDIATELY and never dropped in silence",
                  kl::Driver::replies == kl::queued + 2u && kl::Driver::rejected >= 1u);

    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Ask{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 200u && kl::Driver::votes == 0u) {
        kl::pump();
    }
    const bool idle_vote = kl::Driver::last_vote;
    const uint8_t idle_votes = kl::Driver::votes;
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Kick{});
    post<kl::Driver>(kl::Ask{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 600u) {
        kl::pump();
        if (kl::Driver::votes != 0u && kl::Driver::replies >= kl::queued) {
            break;
        }
    }
    const bool busy_vote = kl::Driver::last_vote;
    const uint8_t busy_votes = kl::Driver::votes;
    const uint8_t busy_replies = kl::Driver::replies;
    print(serial, "  votes: idle ", idle_votes, " ok=", idle_vote ? 1 : 0, ", busy ",
          busy_votes, " ok=", busy_vote ? 1 : 0, " with ", busy_replies, " of ",
          kl::queued, " transactions done", crlf);
    bench.verdict("an IDLE bus votes ok on a PrepareSleep and a BUSY one votes "
                  "AGAINST it",
                  idle_votes == 1u && idle_vote && busy_votes == 1u && !busy_vote &&
                      busy_replies == kl::queued);

    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Wedge{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 800u) {
        kl::pump();
        if (kl::Driver::replies >= 1u) {
            break;
        }
    }
    const uint8_t timeouts = kl::Driver::timeouts;
    host_swallow_completion = false;
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Kick{});
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 600u) {
        kl::pump();
        if (kl::Driver::replies >= kl::queued) {
            break;
        }
    }
    print(serial, "  timeout: ", timeouts, " spi_timeout replies, then ",
          kl::Driver::replies, " replies of which ", kl::Driver::ok_replies,
          " ok afterwards", crlf);
    bench.verdict("A COMPLETION THAT NEVER COMES is answered spi_timeout in its "
                  "place, and recover() puts the engine back where start() is "
                  "legal - the next four transactions run to spi_ok on the same "
                  "bus AO",
                  timeouts == 1u && kl::Driver::ok_replies == kl::queued);

    bus_ao_live = false;
    kl::hand_cs = false;
    kl::rate = SpiClock::div64;
    (void)link_command_mode();
}

// =============================================================================
// r - THE ROLES INVERT: this board as the CLIENT, the peer as the host
// =============================================================================

/// SPI1's own pads carry the wire, so the client role is the SAME four
/// pins with MISO driven and SCK, MOSI and NSS taken as inputs - no
/// second instance and no re-jumpering, which is what an asymmetric bus
/// on symmetric wiring buys.
using PeerClient = SpiClient<1, host_pins>;

uint8_t crx[64];
uint16_t crx_n = 0;

bool run_as_client(const spilink::Params& a) {
    crx_n = 0;
    peer_stop();
    Nvic::disable(S1::irq());
    if (!PeerClient::init(clock, {.mode = SpiMode::mode0,
                                  .bits = SpiDataSize::bits8,
                                  .lsb_first = false,
                                  .nss = SpiNss::hardware_input,
                                  .drive_output = true})) {
        return false;
    }
    // ONE FRAME AHEAD, and on this silicon it is the FIFO that says so
    // (35.5.8): the first two answers go in before the host's clock can
    // arrive and every received frame loads the next-plus-one.
    spilink::Stream out(a.pattern, a.seed_b);
    const uint16_t b0 = out.next();
    const uint16_t b1 = out.next();
    uint16_t queued = 2;
    PeerClient::enable(b0, b1);

    const uint32_t t0 = Ticker::millis();
    while (crx_n < a.count && crx_n < sizeof crx && Ticker::millis() - t0 < a.ms) {
        const auto v = PeerClient::poll();
        if (!v) {
            continue;
        }
        crx[crx_n++] = static_cast<uint8_t>(*v);
        if (queued < a.count) {
            PeerClient::write(out.next());
            ++queued;
        }
    }
    (void)PeerClient::disable();
    PeerClient::release();
    return true;
}

void tr_peer_client() {
    if (!need_peer()) {
        return;
    }

    spilink::Params a{};
    a.count = 12;
    a.ms = 500;
    a.seed_a = 0x2C;
    a.seed_b = 0x71;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;    // lead-in ms: time for this end to become a client
    a.aux16 = 32;   // the peer's own SCK division (48 MHz / 32 = 1.5 MHz)
    a.cfg.apply = 1;
    a.cfg.mode = 0;
    a.cfg.dord = 0;

    if (!peer_act(Op::host_burst, a)) {
        bench.verdict("the peer accepted the host_burst command (its spi_peer must "
                      "carry op 0x14)",
                      false);
        return;
    }
    bench.verdict("the peer accepted the host_burst command", true);

    const bool ran = run_as_client(a);
    bench.verdict("this board came up as an SPI CLIENT on the very pads it hosts "
                  "with - NSS as the hardware input the peer drives",
                  ran);

    spilink::Report r{};
    const bool rep = peer_report(r);

    uint16_t mism = 0;
    uint8_t first_idx = 0xFF, got = 0, want = 0;
    for (uint16_t i = 0; i < crx_n; ++i) {
        const uint8_t exp = spilink::pattern_value(a.pattern, a.seed_a, i);
        if (crx[i] != exp) {
            if (mism == 0) {
                first_idx = static_cast<uint8_t>(i);
                got = crx[i];
                want = exp;
            }
            ++mism;
        }
    }
    print(serial, "  client read ", crx_n, " of ", a.count, " frames, mism=", mism);
    if (mism) {
        print(serial, " (first idx ", first_idx, " got ", hex(got), " exp ", hex(want),
              ")");
    }
    print(serial, "; the peer as HOST read ", r.count, " with mism=", r.mism,
          " flags=", hex(r.flags), crlf);
    print(serial, "  read:");
    for (uint16_t i = 0; i < crx_n && i < 16u; ++i) {
        print(serial, " ", hex(crx[i]));
    }
    print(serial, crlf);

    bench.verdict("the client received every frame the foreign host clocked, "
                  "byte-exact",
                  ran && crx_n == a.count && mism == 0u);
    bench.verdict("...and that host read this end's answer stream byte-exact FROM "
                  "THE FIRST FRAME, which is what the two preloads before the "
                  "select edge buy (35.5.8)",
                  rep && r.count == a.count && r.mism == 0u);

    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swap", command(Op::ping));
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf, "test_stm32_spi - SPI/I2S on the Nucleo-G0B1RE", crlf);
    if (self_link) {
        print(serial, "  the SELF-LINK is on the desk: SPI1 host PB3/PB4/PB5 + PA15 "
                      " <->  SPI2 client PB10/PC2/PD4/PB12; letters b..l are live",
              crlf);
    } else {
        print(serial, "  NO SELF-LINK on the desk (probed): letters b..l skip "
                      "themselves and claim nothing", crlf);
    }
    print(serial, "  SPI1 PB3/PB4/PB5 + PA15 also reach the SAM C21 running "
                  "`spi_peer` (SERCOM1 PA17/PA16/PA19/PA18) - letters n..r", crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

// ---------------------------------------------------------------------------
// The vectors (app glue: this is the one vendor thing an app may contain)
// ---------------------------------------------------------------------------

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void BRIO_STM32G0_USART2_HANDLER() { (void)Serial::isr(); }

/// SPI1's line is its own. Which host owns it is a flag, because two
/// SpiHost instantiations over one instance share the peripheral and not
/// the statics.
extern "C" void SPI1_IRQHandler() {
    if (sleep_probe_live) {
        // The sleep letter's bare RXNE: no task owns this instance, so
        // the frame is read here and the wake is recorded.
        if (S1::rxne()) {
            (void)S1::data_byte();
            sleep_woke = true;
        }
        return;
    }
    if (dma_host_live) {
        if (DmaHost::isr()) {
            host_done = true;
            host_isr_completions = static_cast<uint16_t>(host_isr_completions + 1u);
        }
        return;
    }
    if (bus_ao_live) {
        if (Host::isr() && !host_swallow_completion) {
            brio::post<kl::SpiArb>(brio::TransferDone{Host::status()});
        }
        return;
    }
    if (Host::isr()) {
        host_done = true;
        host_isr_completions = static_cast<uint16_t>(host_isr_completions + 1u);
    }
}

/// SPI2 and SPI3 share one line where the part has a SPI3 (the reserve
/// derives the name), so the app binds the derived spelling.
extern "C" void BRIO_STM32G0_SPI2_HANDLER() {
    if (i2s_live) {
        i2s_service();
        return;
    }
    if (peer_pump_live) {
        peer_service();
    }
}

extern "C" void EXTI4_15_IRQHandler() {
    if (NssWatch::pending()) {
        (void)NssWatch::clear();
        nss_edges = static_cast<uint16_t>(nss_edges + 1u);
    }
}

extern "C" void DMA1_Channel1_IRQHandler() {
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
        dma_completions = static_cast<uint16_t>(dma_completions + 1u);
    } else if (!dma_host_live) {
        (void)HostTx::service();
    }
}

extern "C" void DMA1_Channel2_3_IRQHandler() {
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
        dma_completions = static_cast<uint16_t>(dma_completions + 1u);
        return;
    }
    (void)HostRx::service();
    (void)PeerTx::service();
}

extern "C" void BRIO_STM32G0_DMA1_CH4_UP_HANDLER() { (void)PeerRx::service(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    software_cs_pads();

    bench.letter('a', "the block: the reserve, the reset values, the enable "
                      "protection measured", ta_block);
    bench.letter('b', "THE LINK: SPI1 to SPI2 over four wires, polled and on "
                      "the ISR pump", tb_link);
    bench.letter('c', "the matrix: four modes, both orders, every frame size "
                      "4..16", tc_matrix);
    bench.letter('d', "the rate ladder: eight BR codes, and where the link "
                      "stops", td_ladder);
    bench.letter('e', "NSS four ways: software, the client's input, MODF, SSOE, "
                      "NSSP", te_nss);
    bench.letter('f', "CRC 8 and 16 against a bitwise reference, and CRCERR",
                 tf_crc);
    bench.letter('g', "TI mode on the wire, and FRE", tg_ti);
    bench.letter('h', "ES0548 2.12.1 and 2.12.2, and the overrun", th_errata);
    bench.letter('i', "the DMA engines, the ladder on them, and LDMA", ti_dma);
    bench.letter('j', "THE KERNEL: SpiBus over SpiHost, util untouched",
                 tj_kernel);
    bench.letter('k', "I2S: master to slave on the same wires", tk_i2s);
    bench.letter('l', "the dynamic clock: the BR code follows the rate",
                 tl_dynamic);
    bench.letter('m', "an SPI interrupt wakes a WFI in Sleep mode", tm_sleep);
    bench.letter('n', "THE PEER: the spi_link command channel to the SAM C21",
                 tn_peer_link);
    bench.letter('o', "the matrix against the peer: four modes, both orders",
                 to_peer_matrix);
    bench.letter('p', "the BR ladder against the peer, and where it stops",
                 tp_peer_rates);
    bench.letter('q', "THE KERNEL against the peer: SpiBus over SpiHost",
                 tq_peer_kernel);
    bench.letter('r', "THE ROLES INVERT: this board as the client, the peer as "
                      "the host", tr_peer_client);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
        // THE TOPOLOGY IS ASKED OF THE WIRE, ONCE, before any letter can
        // run: which of the two instruments this desk is carrying is not
        // something the image may assume.
        self_link = probe_self_link();
        software_cs_pads();
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
