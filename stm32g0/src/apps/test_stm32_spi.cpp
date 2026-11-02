// test_stm32_spi - the reference bench suite for the STM32G0's SPI/I2S
// block: stm32g0/spi.hpp over RM0444 ch. 35, both roles, in ONE image on
// ONE board.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// TWO INSTRUMENTS, ONE SET OF PADS, AND THE SUITE ASKS THE WIRE WHICH
// ONE IS ON THE DESK.
//
// (1) THE BOARD'S OWN SELF-LINK: SPI1 the host and SPI2 the client,
// four wires between
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
// (2) THE PEER LINK: the SAME four SPI1 pads reach a SECOND BOARD
// running `spi_peer`, and ANY of its ports is valid - the peer names
// itself in its `ident` (fw 0x01xx = the AVR, 0x02xx = the SAM C21,
// 0x03xx = another STM32G0, 0x04xx = an STM32F446), and the desk says
// which one is fitted -
//
//   SCK   PB3  AF0  ->  a SAM C21's PA19 SERCOM1 PAD[3], a G0's PB3  AF0, an F446's PB13 AF5
//   MOSI  PB5  AF0  ->  a SAM C21's PA16 SERCOM1 PAD[0], a G0's PB5  AF0, an F446's PB15 AF5
//   MISO  PB4  AF0  <-  a SAM C21's PA18 SERCOM1 PAD[2], a G0's PB4  AF0, an F446's PB14 AF5
//   NSS   PA15 GPIO ->  a SAM C21's PA17 SERCOM1 PAD[1], a G0's PA15 AF0, an F446's PB12 AF5
//
// - an stm32g0 peer answers on the SAME PIN NAMES this board hosts on,
// an stm32f4 one on its SPI2 by function (SPI's own MOSI/MISO naming
// carries the direction, so nothing is crossed), plus a dedicated GND,
// both boards at 3.3 V. On the peer link the host's MOSI pad goes out
// one class below the driver's default (peer_link_config(), with the
// measurement that asks for it). The instrument
// is commanded IN BAND over the bus under test, over
// avrxt/src/apps/spi_link.hpp included by relative path - one source of
// truth for the wire format, whatever the peer's architecture. Letters
// n..r and t..y are this instrument's.
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
// ON THE LQFP32 ONLY ONE OF THE TWO IS EVEN POSSIBLE: SPI2's four pads
// reach no pin of that package (DS12992 table 12), so its letters are
// compiled out there and the probe is not asked - while SPI1's own four
// pads are bonded, so the PEER letters run on that part like on any
// other, against a Nucleo-64 at the other end of the same four wires.
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
//   b  THE LINK: 8-bit mode 0 both ways, host polled then on its DMA
//      engines, a command in the loop before its data block
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
//   i  THE DMA ENGINES: the host's data phase on both, a command in the
//      loop before the data block, the ladder with the client on raw
//      engines, and the LDMA odd-count rule under 8-bit packing
//   j  THE KERNEL: SpiBus (= BusMaster) over the engined SpiHost with
//      the client answering on the wire - ordered replies (on the
//      engines, one polled among them, one a command in the loop before
//      its data block), rejection, the sleep votes, and the per-bus
//      timeout with recover()
//   k  I2S: I2S1 master to I2S2 slave on three of the same wires, the
//      four standards, three data lengths, CHSIDE, the prescaler
//      arithmetic timed, and UDR on a slave transmitter
//   l  THE DYNAMIC CLOCK: the link exact at 64, 16 and 2 MHz with the
//      BR code re-resolved against a stated ceiling
//   m  sleep: an SPI interrupt waking a WFI in Sleep mode
//   s  THE HOST'S SHAPES, WIRELESS: the transmit-only polled loop, the
//      packed receive, the frame-by-frame short receive, the half-word
//      loop, a two-phase request - each judged by the FIFO levels and OVR
//      after it, with nothing on MISO - a request naming the engines
//      refused on the engineless host (the select never falls, CR1
//      untouched) - and THE RATE ON THE WIRE:
//      a frame inside a block timed against its SCK periods at the rate
//      asked, at every BR code on the engines and the polled loop, in
//      16-bit frames, and under a 1 MHz ceiling at the dynamic clock's
//      16 and 2 MHz. A client is clocked by the host's SCK and cannot
//      tell a wrong divisor; SCK is the host's whatever is wired, so this
//      wireless timing stands for d, i, l and p, which set the same
//      divisors over wires
//   n  THE PEER: the spi_link command channel to the peer board, its
//      ident and ten frames
//   o  the matrix against the peer: four modes, both bit orders, and a
//      DORD mismatch as an exact two-way bit reversal
//   p  the BR ladder against the peer, and where its answer reload stops;
//      a rung above the peer's documented client ceiling (by the kind its
//      ident names) is printed and not judged
//   q  THE KERNEL against the peer: SpiBus (= BusMaster) over the
//      engined SpiHost with a SECOND CHIP reading the bytes back (letter
//      j's mix of ways) - the rejection, both sleep votes and the per-bus
//      timeout with recover()
//   S  THE ENGINE A REQUEST NAMES against the peer: one frame read and one
//      written on the engines at rates where the loop costs the core less,
//      exact at both ends; what the engines cannot carry refused, the
//      select, the D/C and the block untouched and no edge on SCK; a command
//      in the loop before its data block, D/C moving between the two
//      phases, and a command alone on the engines
//   E  WHEN A REQUEST ENDS THE WIRE HAS ENDED: the SCK pad read right after
//      the request ended, all four modes, every rate, read and write, both
//      engines, through a select and through a null one
//   r  THE ROLES INVERT: this board as the CLIENT on the pads it hosts
//      with, the peer clocking a bounded burst as the bus host
//   u  16-BIT FRAMES against the peer (spi_link's extended exchange,
//      spare_wide): the polled loop and the half-word engines at every
//      BR code, the wire's time from two window lengths,
//      and the peer's own 16-bit client run
//   v  RECEIVE-ONLY and ODD COUNTS (spare_receive_only): this host's
//      transmit-only shapes witnessed by a client whose answer line is
//      dark, the peer's receive() run at 7 and 33 frames in both widths,
//      and odd-count exchanges both ways
//   w  THE CRC against the peer's unit (spare_crc): both widths, both of
//      the peer's serves, every BR code, this end's unit on the resource
//      with the last two data frames written as one burst and CRCNEXT
//      after them; a wrong CRC frame caught as report_crcerr
//   y  THE SIMPLEX LINES, THE SOFTWARE SELECT AND prime() against the
//      peer: a client with no select pad (SSM) deaf with SSI high and exact
//      with it low, the host's single line out (BIDIMODE) and its
//      receive-only clock (RXONLY), the client's single line out on MISO
//      and its receive-only run, and every pair of modes applied inside an
//      open select window - the slip prime() exists to prevent, counted by
//      the F446's client into CPHA = 0 alone
//   K  THE NEXT REQUEST AFTER A STALL OR A TRANSFER ERROR, recover() never
//      called: every stall path and a bus error on either channel, directly
//      and queued through SpiBus, the next polled and asynchronous windows
//      exact at both ends; the clock gate closed, which no reset opens
//   t  (OUTSIDE z) THE CRCNEXT WINDOW: the store walked away from the
//      last data store one loop turn at a time, one frame in flight -
//      the instrument behind w's burst (spi.md)
//   x  (OUTSIDE z) slip statistics against the peer: each mode ten times,
//      the bursts that were not byte-exact and where each broke - the
//      instrument that found the peer's MISO edge rate (spi.md)
//
// NOTHING WRITES FLASH, no option byte is touched, and the RTC domain is
// not reset. The pads this suite moves are the eight of the self-link
// plus the console's two.
//
// build: boards = g0b1re,g071rb,g031k8
// build: groups = abcdefghijklmnopqsK,rtuvwxySE
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "kernel/tenuto.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/lpuart.hpp"
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

// THE PROTOCOL HEADER IS SHARED, NOT COPIED. spi_link.hpp
// is pure encoding - it names no register and includes nothing of brio -
// so every architecture on this link compiles the same file. A copy here
// would be a second source of truth for a wire format, which is exactly
// the thing that drifts.
#include "../../../avrxt/src/apps/spi_link.hpp"

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

// AND ON A PART WHOSE USART2 HAS NO KERNEL-CLOCK MULTIPLEXER, THAT COSTS
// A DIFFERENT PERIPHERAL. Table 183's FULL/BASIC split moves with the
// part: where USART2 is BASIC it runs on PCLK, full stop - its divisor
// would follow every switch this suite makes and the report would be a
// function of its own subject. The instance that always has a
// multiplexer is the LPUART (34.4.6), and LPUART1_TX/RX reach THE SAME
// TWO PADS at AF6. So the console moves to LPUART1 exactly where USART2's
// multiplexer is missing. The HANDLER has to be chosen by the
// preprocessor, which cannot call a constexpr function, so the same
// header symbol the reserve probes for usart_has_clock_select(2) is
// probed here and the static_assert keeps the two answers one answer.
#if defined(RCC_CCIPR_USART2SEL_Pos)
#define BRIO_SUITE_CONSOLE_HANDLER BRIO_STM32G0_USART2_HANDLER
constexpr bool console_on_lpuart = false;
constexpr PinFunction console_af = PinFunction::af1;
#else
#define BRIO_SUITE_CONSOLE_HANDLER BRIO_STM32G0_LPUART1_HANDLER
constexpr bool console_on_lpuart = true;
constexpr PinFunction console_af = PinFunction::af6;
#endif
static_assert(console_on_lpuart == !usart_has_clock_select(2),
              "the console's instance and the reserve's own column must be "
              "one answer");

constexpr UartPins console_pins{
    .tx = {'A', 2, console_af},
    .rx = {'A', 3, console_af},
};
using Serial = std::conditional_t<
    console_on_lpuart,
    LpUart<1, console_pins, 64, 512, NoDmaEngine, NoDmaEngine, console_opts>,
    Uart<2, console_pins, 64, 512, NoDmaEngine, NoDmaEngine, console_opts>>;

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
// pads: a second set of statics, one live at a time. Half-word engines,
// so a frame of either width rides the DMA with its own beat.
using HostTx = DmaTxEngine<1, 1, uint16_t>;
using HostRx = DmaRxEngine<1, 2, uint16_t>;
using DmaHost = SpiHost<1, host_pins, HostTx, HostRx>;
/// Letter S's byte engines on the same channels: a 16-bit frame has no beat on
/// them whatever its buffer, so this host refuses it and never starts one.
using ByteDmaHost = SpiHost<1, host_pins, DmaTxEngine<1, 1>, DmaRxEngine<1, 2>>;
// The client's own engines, RAW (SpiClient has no slots - see the
// report): letter i uses them to take the client's ISR turnaround out of
// the ladder, which is the whole point of that measurement.
using PeerTx = DmaTxEngine<1, 3>;
using PeerRx = DmaRxEngine<1, 4>;
// The packed pair of the LDMA leg: the SAME channels at half-word width.
using PeerTx16 = DmaTxEngine<1, 3, uint16_t>;

using SysClock = DynamicClock<Rates<Fast, Mid, Slow>, Ticker, Serial, Host, DmaHost, ByteDmaHost>;
constexpr SysClock clock;
static_assert(SysClock::rate_count == 3);

constexpr uint8_t r_fast = 0;
constexpr uint8_t r_mid = 1;
constexpr uint8_t r_slow = 2;

Serial serial;
// The table holds every letter registered in main(), 28 of them: a
// letter past its capacity is refused (letter() returns false), and the
// suite does not check the return, so it would be missing in silence.
TestBench<Serial, 28> bench;

using CsPin = Pin<'A', 15>;          ///< the host's chip select, a GPIO
using ClientNssPin = Pin<'B', 12>;   ///< the client's NSS - and letter e's witness
using NssWatch = ExtInt<ClientNssPin>;
/// The host's own select seen from its pad: letter s counts its falling
/// edges around a refused request (none) and a polled one (one).
using CsWatch = ExtInt<CsPin>;

// ---------------------------------------------------------------------------
// Instruments
// ---------------------------------------------------------------------------

/// A cycle count that spans ticks: the kernel ticker's own cycles(), the
/// tick count and the counter's position composed right across the reload
/// (util/cycle_count.hpp); it wraps at 2^32, so only differences are used.
uint32_t cycles_now() {
    return Ticker::cycles();
}

/// A measurement window a transmit interrupt walks through is not a
/// measurement. Drain, then count.
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
/// data frame there instead (measured: without the limit, 0xFF lands
/// where the client's TXCRCR belongs).
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
    // handler that leaves one standing re-enters for ever.
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
/// The engined host's completions (TransferDone edges) and the receive
/// channel's block ends under them - one of each, whatever the phases.
volatile uint16_t dma_completions = 0;
volatile uint16_t dma_blocks = 0;
/// Letter K's stopwatch on the completing vector, the ticker's cycles.
volatile bool time_vector = false;
volatile uint32_t vector_cycles = 0;
/// Letter K's witness of the order: SCK's edges on its own pad, each counted by
/// where the select's output latch stood when it was seen.
using SckWatch = ExtInt<Pin<host_pins.sck.port, host_pins.sck.pin>>;
volatile uint16_t sck_edges_selected = 0;
volatile uint16_t sck_edges_released = 0;
/// The select's falling edges, letter s's CsWatch.
volatile uint16_t cs_falls = 0;
/// Letter s stretches the tick's handler to prove the receive loops lose
/// nothing under a handler longer than every frame they keep in flight:
/// `tick_stretch` turns of an empty loop after the tick, zero otherwise.
volatile uint32_t tick_stretch = 0;
volatile uint32_t ticks_stretched = 0;
/// Letter j stages a LOST INTERRUPT: the DMA vector's body still runs (so
/// the block ends and the engines are put away) but the completion is
/// never posted, which is exactly the wedge util/bus_master.hpp's timeout
/// is for.
volatile bool host_swallow_completion = false;
/// Whether the engined host owns the DMA vectors right now, and whether
/// its completions go to the kernel letters' arbiter.
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;
/// Letter m owns SPI1's vector for its own bare RXNE: no host arms the
/// block's interrupts, and the letter reads the frame there itself.
volatile bool sleep_probe_live = false;

uint8_t tx_buf[96];
uint8_t rx_buf[96];

/// One transaction through the ENGINELESS host, waiting for it however
/// it completes. On this host an asynchronous request is refused at once
/// (start() true, spi_rejected), so only a polled one moves frames.
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
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
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
              bool polled, uint8_t cmd_len = 0, const uint8_t* cmd = nullptr,
              SpiDataSize bits = SpiDataSize::bits8) {
    DmaHost::Request r{};
    r.cs = CsPin::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = frames;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.bits = bits;
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
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
// they have already moved: the same SPI1 pads now carry the link to a
// PEER BOARD running `spi_peer`, and SPI2's four pads are on nothing
// (the two wirings are the same jumpers at different ends). So the
// suite ASKS THE WIRE which desk it is on, once, before any letter runs,
// and the letters
// that need two peripherals on four wires SKIP THEMSELVES - by name,
// with the reason printed - when the answer is no. A skipped letter
// claims nothing and scores no verdict either way.

bool self_link = false;

// THE PACKAGE DECIDES THE SELF-LINK AND THE DESK DECIDES THE PEER, which
// are two questions and not one. SPI2's four pads (PB10/PC2/PD4/PB12)
// reach no pin of the LQFP32 at all (DS12992 table 12: its port B stops
// at PB9 and it has no port D), so on that package the self-link letters
// are not merely skipped at run time - they are COMPILED OUT, each still
// printing its own reason and claiming nothing, which is also what keeps
// this suite inside 64 KB of flash. SPI1's own four pads, on the other
// hand, are bonded on every package this stratum has met, so THE PEER
// LETTERS ARE COMPILED EVERYWHERE and what settles them is the wiring:
// the probe below says whether these pads carry the self-link instead,
// and a peer that does not answer with the wires in place fails LOUDLY,
// because that is firmware to flash and not a fact about the plastic.
// Which device header is compiled is the one question only the
// preprocessor can ask, and what package goes with it is this suite's own
// knowledge.
#if defined(STM32G031xx)
constexpr bool self_link_possible = false;
#else
constexpr bool self_link_possible = true;
#endif

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
    if constexpr (!self_link_possible) {
        print(serial, "  self-link probe: not asked - SPI2's four pads are "
              "bonded to no pin of this package (DS12992 table 12), so there "
              "is nothing at the far end to drive or read", crlf);
        return false;
    }
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
    if constexpr (!self_link_possible) {
        print(serial,
              "  SKIPPED, no verdict claimed: this letter's instrument is a "
              "SELF-LINK between SPI1's pads and SPI2's, and SPI2's four "
              "(PB10/PC2/PD4/PB12) are bonded to no pin of this package "
              "(DS12992 table 12). There is no wiring of this board that "
              "could carry it: the letter is compiled out here, not merely "
              "skipped.",
              crlf);
        return false;
    }
    if (self_link) {
        return true;
    }
    print(serial,
          "  SKIPPED, no verdict claimed: this letter's instrument is the board's "
          "OWN self-link (SPI1 PB3/PB4/PB5/PA15 to SPI2 PB10/PC2/PD4/PB12) and "
          "the probe says those four wires are not on the desk. They carry the "
          "link to the PEER BOARD today - letters n..r are the instrument this "
          "desk has.",
          crlf);
    return false;
}

// =============================================================================
// The peer: spi_link.hpp over the bus under test
// =============================================================================
//
// The wire format is avrxt/src/apps/spi_link.hpp, included by relative
// path and NOT copied - it names no register, includes nothing of brio
// and every architecture that speaks the link compiles it. The
// instrument at the other end is `spi_peer`, in whichever of its ports
// the wires reach: the samc21 one answers on SERCOM1 fn C, the stm32g0
// one on SPI1 AF0 at the same pin names this board hosts on, the stm32f4
// one on the F446's SPI2 AF5. The peer's
// own `ident` says which (fw 0x01xx, 0x02xx, 0x03xx, 0x04xx).

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

/// THE HOLD AROUND EACH SELECT WINDOW, and it is a measurement rather
/// than a precaution: the engine releases CS about a
/// microsecond after the last SCK edge, and a client whose transaction
/// the select edge RESETS loses a character it has not fetched yet. The
/// protocol therefore owns the chip select for these windows (the
/// Request carries a null PinRef) and pays 30 us on each side of them.
void link_hold() { (void)delay_us(clock, 30); }

/// SPI1 as the command channel's host, PA15 as its GPIO chip select.
/// THE PEER LINK'S DATA PAD ONE NOTCH DOWN. The driver's default class is
/// very-high, and on a bus of jumper wires that edge is an AGGRESSOR on
/// the clock at the far end: against the stm32f4 peer, modes 0 and 3 -
/// the two that SAMPLE on the rising edge, the host's MOSI changing while
/// SCK is LOW - slipped one bit (both ends disagreeing from the same frame
/// on) in 9 and 10 of 40 bursts of letter x, and in 0 of 40 with this pad
/// at high, medium or low (docs/stm32g0/spi.md). The clock pad keeps its
/// class: it is the edge the client samples on. The Config every peer
/// letter's init() is handed; letter x moves the MOSI class for its
/// back-to-back rung and puts it back.
PinSpeed peer_mosi_speed = PinSpeed::high;
Host::Config peer_link_config() { return {.mosi_speed = peer_mosi_speed}; }

bool link_command_mode() {
    peer_stop();
    dma_host_live = false;
    bus_ao_live = false;
    sleep_probe_live = false;
    software_cs_pads();
    const bool ok = Host::init(clock, peer_link_config());
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
    r.engine = SpiEngine::polled;
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
          "      the peer board must be running `spi_peer` (brio "
          "flash F spi_peer); its console '0' forces the dark client back.",
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
          "(brio flash F spi_peer); its console '0' forces the "
          "dark client back. Check the five wires in this file's header.",
          crlf);
    return false;
}

/// The opening line of every letter whose instrument is the PEER BOARD.
/// THE TWO WIRINGS ARE THE SAME JUMPERS AT DIFFERENT ENDS, so a desk
/// carrying the self-link cannot be carrying the peer: that is a
/// topology and it skips, exactly as the self-link letters skip on the
/// other desk. An absent peer with the wires in place is a different
/// thing - firmware to flash - and fails loudly.
bool need_peer() {
    if (self_link) {
        print(serial,
              "  SKIPPED, no verdict claimed: this letter's instrument is the PEER "
              "BOARD running `spi_peer`, and the probe says these four pads carry "
              "the board's OWN self-link today - the two wirings are the same "
              "jumpers at different ends. Letters b..l are the "
              "instrument this desk has.",
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

// ---- the third instance, asked only where the part has one ------------------
// A part whose header declares no SPI3 must not SPELL `Spi<3>` (the
// driver's static_assert is the refusal), and `if constexpr` inside a
// plain function still instantiates the branch it discards - while a
// NON-DEPENDENT `Spi<3>` inside a template body is looked up when the
// template is DEFINED. So the instance NUMBER depends on the same
// reserve fact that gates the branch.
template <bool present>
using Spi3 = Spi<present ? uint8_t{3} : uint8_t{2}>;

/// The reserve's I2S row and the device header's own macro, compared on a
/// third instance. Where there is none there is nothing to compare, and
/// the reserve already says so (spi_has_i2s(3) is false with no I2S3).
template <bool present = spi_present(3)>
bool third_instance_i2s_agrees() {
    if constexpr (present) {
        return spi_has_i2s(3) == Spi3<present>::has_i2s();
    } else {
        return !spi_has_i2s(3);
    }
}

/// "A third instance is not an I2S" - at compile time and at run time
/// both where one exists, and at compile time alone where it does not.
template <bool present = spi_present(3)>
bool third_instance_has_no_i2s() {
    if constexpr (present) {
        return !Spi3<present>::has_i2s_mode && !Spi3<present>::has_i2s();
    } else {
        return !spi_has_i2s(3);
    }
}

void ta_block() {
    peer_stop();
    Nvic::disable(S1::irq());
    S1::bus_clock(true);
    S2::bus_clock(true);
    S1::reset();
    S2::reset();

    // ---- the reserve against the device header ----
    print(serial, "  instances: SPI1 ", spi_present(1), " SPI2 ", spi_present(2),
          " SPI3 ", spi_present(3), crlf);
    bench.verdict("the reserve knows this part's SPI roster: SPI1 and SPI2 "
                  "everywhere, a third on the G0B1/G0C1 alone",
                  spi_present(1) && spi_present(2) && !spi_present(4));
    print(serial, "  vectors: SPI1 ", static_cast<int32_t>(spi_irq(1)), ", SPI2 ",
          static_cast<int32_t>(spi_irq(2)), crlf);
    bench.verdict("SPI2's line is SPI1's no more, and it is SHARED with a "
                  "third instance exactly where the part has one - which is "
                  "how the reserve derives the name",
                  spi_irq(1) == SPI1_IRQn && spi_irq(1) != spi_irq(2) &&
                      (!spi_present(3) || spi_irq(2) == spi_irq(3)));
    bench.verdict("the DMAMUX request pairs are table 56's (16/17, 18/19, 66/67)",
                  S1::dma_rx_request() == 16 && S1::dma_tx_request() == 17 &&
                      S2::dma_rx_request() == 18 && S2::dma_tx_request() == 19 &&
                      spi_dma_rx_request(3) == 66 && spi_dma_tx_request(3) == 67);
    print(serial, "  I2S: reserve says SPI1 ", spi_has_i2s(1) ? "yes" : "no", " SPI2 ",
          spi_has_i2s(2) ? "yes" : "no", " SPI3 ", spi_has_i2s(3) ? "yes" : "no",
          "; the header's IS_I2S_ALL_INSTANCE says SPI1 ", S1::has_i2s() ? "yes" : "no",
          " SPI2 ", S2::has_i2s() ? "yes" : "no", crlf);
    bench.verdict("table 205's I2S row: the reserve's constant and the device "
                  "header's own macro agree on every instance the part has",
                  spi_has_i2s(1) == S1::has_i2s() && spi_has_i2s(2) == S2::has_i2s() &&
                      third_instance_i2s_agrees());
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
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
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

    // The same on the engined host's DMA engines.
    Dma<1>::bus_clock(true);
    bench.verdict("the ENGINED host comes up over the same instance and pads",
                  DmaHost::init(clock));
    dma_host_live = true;
    fill_link_fixture(n);
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    dma_completions = 0;
    dma_blocks = 0;
    const bool moved_dma = dma_xfer(tx_buf, rx_buf, n, SpiClock::div64, false);
    const LinkResult dma = judge_link(n);
    print(serial, "  asynchronous host: completions ", dma_completions, ", blocks ", dma_blocks,
          ", peer frames ", dma.peer_frames, crlf);
    bench.verdict("the SAME sixteen frames ASYNCHRONOUS, on the host's DMA engines - one "
                  "block, one completion at the end",
                  moved_dma && dma.host_ok && dma.peer_ok && dma_completions == 1u &&
                      dma_blocks == 1u);

    // A command phase in front of the data: the command in the loop, the
    // data on a block.
    static const uint8_t cmd[3] = {0x9F, 0x01, 0x02};
    fill_link_fixture(n);
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    dma_completions = 0;
    dma_blocks = 0;
    const bool with_cmd = dma_xfer(tx_buf, rx_buf, 8, SpiClock::div64, false, 3, cmd);
    bool cmd_ok = with_cmd && peer_rx_n == 11u && dma_completions == 1u && dma_blocks == 1u;
    for (uint16_t i = 0; i < 3u && cmd_ok; ++i) {
        cmd_ok = peer_rx[i] == cmd[i];
    }
    for (uint16_t i = 0; i < 8u && cmd_ok; ++i) {
        cmd_ok = peer_rx[3u + i] == static_cast<uint16_t>(0xA0u + i);
    }
    print(serial, "  cmd+data: peer got ", peer_rx_n, " frames (3 + 8 expected), blocks ",
          dma_blocks, ", completions ", dma_completions, crlf);
    bench.verdict("a two-phase request is ONE select window: three command "
                  "frames in the loop, then eight data ones on a block, in "
                  "order, with the D/C flip between them - one block, one "
                  "completion",
                  cmd_ok);
    dma_host_live = false;
    DmaHost::release();
    (void)Host::init(clock);
    peer_stop();
}

// =============================================================================
// c - the matrix
// =============================================================================

void tc_matrix() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
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
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    constexpr uint16_t n = 16;
    software_cs_pads();
    (void)Host::init(clock);
    dma_host_live = false;

    // ---- the PACED ladder: the engine's own polled loop, its write-ahead
    // and its refill setting the pace, which is what a device driver really
    // does ----
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
    print(serial, "  paced by the engine's polled loop: ", paced, " of 8 codes exact both "
                  "ways, up to PCLK/2 = ", spi_sck_hz(SysClock::pclk_hz(), SpiClock::div2) / 1000u,
          " kHz", crlf);
    bench.verdict("ALL EIGHT BR CODES CARRY THE LINK BYTE-EXACT WHEN THE HOST "
                  "PACES IT - up to PCLK/2, the peripheral's own maximum - the "
                  "pace being the polled loop's write-ahead and refill",
                  paced == 8u);

    // ---- the BURST ladder: the transmit FIFO kept full, so that the
    // clock does not stop between frames and the client has ONE FRAME
    // TIME to answer.
    //
    // NOT ONE CHARACTER IS PRINTED INSIDE THIS LOOP. A console byte is
    // an interrupt, an interrupt inside a burst is a gap the host's own
    // receive FIFO can overrun, so a printing loop measures its own
    // printing: one rung stalls outright, spending four million turns
    // waiting for a frame the overrun has thrown away. Collect the
    // eight rungs, THEN say what happened ----
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
    bench.verdict("THIS LOOP CANNOT MAKE A CONTINUOUS SPI CLOCK BESIDE A CLIENT ON "
                  "THE SAME CORE. A hand loop that keeps the transmit FIFO full and "
                  "reads every frame back spends about three hundred cycles a frame "
                  "while the client's vector answers each one on this core, so from "
                  "PCLK/32 upward the frame period STOPS FOLLOWING THE RATE and the "
                  "clock has a gap in it at every rung - where the driver's own "
                  "transmit-only loop, with no client to serve, holds the clock "
                  "continuous at PCLK/2 (letter s times it). A ladder driven this way "
                  "never presses the far end, however fast BR is set",
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
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
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
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
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
                  "instead",
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
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
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
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
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

void ti_dma() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
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

    // A command before the data block: the command in the loop.
    static const uint8_t cmd[2] = {0x0B, 0x00};
    for (uint16_t i = 0; i < n; ++i) {
        rx_buf[i] = 0xEE;
    }
    peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits8}, peer_answers, n);
    dma_completions = 0;
    dma_blocks = 0;
    const bool with_cmd = dma_xfer(tx_buf, rx_buf, 16, SpiClock::div256, false, 2, cmd);
    bool handover = with_cmd && peer_rx_n == 18u && peer_rx[0] == cmd[0] &&
                    peer_rx[1] == cmd[1] && dma_blocks == 1u && dma_completions == 1u;
    for (uint16_t i = 0; i < 16u && handover; ++i) {
        handover = peer_rx[2u + i] == static_cast<uint16_t>(0x11u + i * 3u);
    }
    print(serial, "  handover: peer got ", peer_rx_n, " frames (2 + 16 expected), blocks ",
          dma_blocks, ", completions ", dma_completions, crlf);
    bench.verdict("the COMMAND phase in the loop and the data block after it "
                  "- one select window, the D/C flip in the middle, no frame "
                  "lost or repeated at the seam, one block and one completion",
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
                  "start_fixed() and start_discard() of stm32g0/dma.hpp",
                  dummies && discarded);

    // THE BEAT IS THE FRAME: sixteen 16-bit frames on the half-word
    // engines - one completion, the receive block's, and byte-exact both
    // ways with each frame low byte first in the buffers - and then the
    // same request from an ODD address, which the engines cannot carry
    // (10.4.3: a half-word beat cannot read across one): refused, nothing on
    // the wire.
    {
        alignas(2) static uint8_t wide_tx[2u * 16u + 2u];
        alignas(2) static uint8_t wide_rx[2u * 16u + 2u];
        uint16_t wide_answers[16];
        for (uint16_t i = 0; i < 16u; ++i) {
            const uint16_t out = static_cast<uint16_t>(0x1203u + i * 0x0111u);
            wide_answers[i] = static_cast<uint16_t>(0xA05Au + i * 7u);
            wide_tx[2u * i] = static_cast<uint8_t>(out);
            wide_tx[2u * i + 1u] = static_cast<uint8_t>(out >> 8);
            wide_rx[2u * i] = 0xEE;
            wide_rx[2u * i + 1u] = 0xEE;
        }
        bool wide_ok[2] = {};
        uint16_t completions[2][2] = {};
        for (uint8_t leg = 0; leg < 2u; ++leg) {
            // Leg 1 shifts both buffers by one byte: an odd address.
            uint8_t* const tx = leg == 0u ? wide_tx : wide_tx + 1;
            uint8_t* const rx = leg == 0u ? wide_rx : wide_rx + 1;
            if (leg == 1u) {
                for (int16_t i = 31; i >= 0; --i) {
                    tx[i] = wide_tx[i];   // the same frames, one byte up
                }
                for (uint16_t i = 0; i < 32u; ++i) {
                    rx[i] = 0xEE;
                }
            }
            peer_arm({.mode = SpiMode::mode0, .bits = SpiDataSize::bits16}, wide_answers, 16);
            dma_completions = 0;
            DmaHost::Request wr{};
            wr.engine = SpiEngine::dma;
            wr.cs = CsPin::ref();
            wr.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
            wr.rx = lend<Lease::reply>(rx);
            wr.len = 16;
            wr.clock = SpiClock::div256;
            wr.bits = SpiDataSize::bits16;
            host_done = false;
            const bool inside = DmaHost::start(wr);
            const uint32_t w0 = Ticker::millis();
            while (!inside && !host_done && Ticker::millis() - w0 < 500u) {
            }
            const bool moved = inside || host_done;
            if (leg == 1u) {
                // Refused: nothing reached the client, nothing came back.
                bool untouched = inside && DmaHost::status() == spi_rejected && peer_rx_n == 0u;
                for (uint16_t i = 0; i < 32u && untouched; ++i) {
                    untouched = rx[i] == 0xEE;
                }
                wide_ok[leg] = untouched;
                completions[leg][0] = dma_completions;
                completions[leg][1] = inside ? 1u : 0u;
                continue;
            }
            bool ok = moved && peer_rx_n == 16u && DmaHost::status() == spi_ok;
            for (uint16_t i = 0; i < 16u && ok; ++i) {
                const uint16_t in = static_cast<uint16_t>(rx[2u * i] | (rx[2u * i + 1u] << 8));
                const uint16_t out = static_cast<uint16_t>(tx[2u * i] | (tx[2u * i + 1u] << 8));
                ok = in == wide_answers[i] && peer_rx[i] == out;
            }
            wide_ok[leg] = ok;
            completions[leg][0] = dma_completions;
            completions[leg][1] = inside ? 1u : 0u;
        }
        print(serial, "  16-bit frames: aligned exact=", wide_ok[0], " (dma completions ",
              completions[0][0], ", inside start() ", completions[0][1], "), odd address refused=",
              wide_ok[1], " (dma ", completions[1][0], ", inside start() ", completions[1][1], ")",
              crlf);
        bench.verdict("16-BIT FRAMES ON THE DMA: sixteen half-word beats out and back, "
                      "byte-exact both ways, ONE completion",
                      wide_ok[0] && completions[0][0] == 1u && completions[0][1] == 0u);
        bench.verdict("and the same request from an ODD address is refused inside start(), "
                      "spi_rejected, nothing reaching the client and nothing written back, "
                      "with no completion on a channel - the engines cannot carry it",
                      wide_ok[1] && completions[1][0] == 0u && completions[1][1] == 1u);
    }

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
        (void)PeerRx::start(std::span<uint8_t>(peer_rx_dma, 32));
        (void)PeerTx::start(std::span<const uint8_t>(peer_tx_dma, 32));
        S2::enable();

        // On the engines: thirty-two frames at every code.
        const bool ok_move = dma_xfer(tx_buf, rx_buf, 32, rate, false);
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
        (void)PeerTx16::start(std::span<const uint16_t>(packed, 3));   // 5 or 6 frames
        uint32_t sp = 2'000'000u;
        while (peer_rx_n < 5u && sp-- != 0u) {
        }
        // A SETTLE MUST WAIT ON SOMETHING REAL. An empty countdown here
        // is one gcc deletes, and the count is then read while the sixth
        // frame is still in the transmit FIFO - the two legs look
        // identical. The wall is the wall.
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
/// Letter K: two requests posted from one dispatch, the first bound to fail.
struct Pair {
    bool fault;    ///< the first ends spi_dma_fault; else it ends spi_stalled
    bool polled;   ///< the way the second one runs
};

/// Letter K's window.
constexpr uint16_t way_frames = 32;
constexpr SpiClock way_rate = SpiClock::div32;
uint8_t way_tx[way_frames];
uint8_t way_rx[way_frames];
/// 0x3000 0000 is reserved on every part of the family (RM0444 2.2, the three
/// memory maps), and 10.4.7 makes a read or a write of reserved space a transfer
/// error that clears the channel's EN. Nothing reads or writes this but a channel.
inline uint8_t* way_bad_address() { return reinterpret_cast<uint8_t*>(uintptr_t{0x3000'0000u}); }
SpiMode way_mode = SpiMode::mode0;

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

/// Request 3's command phase (Driver::request()), whose echoes are not
/// read back: whether byte k of request i is one the host reads.
constexpr uint8_t cmd_frames = 3;
constexpr bool read_back(uint8_t i, uint8_t k) { return i != 3u || k >= cmd_frames; }

class Driver;
/// Pending depth 3 - one fewer than the letter queues, so the rejection
/// is reachable - and a 100-tick per-bus timeout, which is two orders of
/// magnitude above the longest transaction here.
using SpiArb = SpiBus<DmaHost, P, 3, BusPassThrough, 100>;

class Driver : public Fsm<Driver, SpiDone, Kick, Fill, Ask, Wedge, Pair, SleepVote> {
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
            [](const Pair& p) {
                post<SpiArb>(bound_to_fail(p.fault));
                post<SpiArb>(the_good_one(p.polled));
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

    /// The ways one arbiter runs a request on this host: 0 and 2 on the
    /// engines, 1 POLLED inside start() - the synchronous completion the
    /// arbiter answers in its own dispatch - and 3 on the engines with its
    /// first `cmd_frames` as a command phase, in the loop before the data
    /// block.
    static DmaHost::Request request(uint8_t i) {
        const bool split = i == 3u;
        return DmaHost::Request{
            .cs = hand_cs ? PinRef{} : CsPin::ref(),
            .dc = {},
            .cmd = split ? lend<Lease::reply>(static_cast<const uint8_t*>(tx[i]))
                         : Borrowed<const uint8_t, Lease::reply>{},
            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx[i] + (split ? cmd_frames : 0u))),
            .rx = lend<Lease::reply>(rx[i] + (split ? cmd_frames : 0u)),
            .reply = reply_to<Driver, SpiDone>(),
            .len = static_cast<uint16_t>(split ? payload - cmd_frames : payload),
            .cmd_len = static_cast<uint8_t>(split ? cmd_frames : 0u),
            .cs_setup_us = 2,
            .clock = rate,
            .mode = SpiMode::mode0,
            .bits = SpiDataSize::bits8,
            .engine = i == 1u ? SpiEngine::polled : SpiEngine::dma,
        };
    }

    /// Letter K's first request: a polled write over a block whose enable the
    /// letter took down, or an asynchronous one out of a reserved address. The
    /// select is null on a fault's: the frames a receive error lets out must
    /// not reach the window the peer holds for the good one.
    static DmaHost::Request bound_to_fail(bool fault) {
        DmaHost::Request r{};
        r.cs = fault ? PinRef{} : CsPin::ref();
        r.tx = lend<Lease::reply>(fault ? static_cast<const uint8_t*>(way_bad_address())
                                        : static_cast<const uint8_t*>(way_tx));
        r.len = fault ? way_frames : 9u;
        r.clock = way_rate;
        r.mode = way_mode;
        r.engine = fault ? SpiEngine::dma : SpiEngine::polled;
        r.reply = reply_to<Driver, SpiDone>();
        return r;
    }
    /// The good one: the whole window the peer holds, its own select, read back.
    static DmaHost::Request the_good_one(bool polled) {
        DmaHost::Request r{};
        r.cs = CsPin::ref();
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(way_tx));
        r.rx = lend<Lease::reply>(way_rx);
        r.len = way_frames;
        r.cs_setup_us = 30;
        r.clock = way_rate;
        r.mode = way_mode;
        r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
        r.reply = reply_to<Driver, SpiDone>();
        return r;
    }
};

using BusKernel = Tenuto<P, Driver, SpiArb>;

/// The kernel's own loop, minus the sleep: Tenuto::run() matures the
/// TIME EVENTS before every step, and the per-bus timeout IS a time
/// event - a pump that only called step() would wait for a deadline
/// nothing was advancing (measured: a step()-only pump leaves the
/// timeout leg sitting six hundred milliseconds with no reply at all).
void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

}   // namespace kl

void tj_kernel() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    // Letter q borrows this arbiter with the chip select in its own hand
    // and at the peer's rate; both are put back here, so the order the
    // two letters run in cannot change what either measures.
    kl::hand_cs = false;
    kl::rate = SpiClock::div64;
    software_cs_pads();
    Dma<1>::bus_clock(true);
    bench.verdict("the engined host comes up for the kernel letter", DmaHost::init(clock));
    dma_host_live = true;

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
            if (kl::read_back(i, k) &&
                kl::rx[i][k] != 0x30u + ((i * kl::payload + k) & 0x0Fu)) {
                wire_ok = false;
            }
        }
    }
    print(serial, "  kernel: ", kl::Driver::replies, " replies, ",
          kl::Driver::ok_replies, " ok; the client saw ", peer_rx_n, " frames", crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH come back in "
                  "order, every one spi_ok, with the bytes on the wire exactly "
                  "what each request lent - two on the engines, one polled "
                  "inside start(), one a command in the loop before its data block",
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
                  "AGAINST it - the arbiter is a voter on this target too, "
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
                  "clocks itself: the DMA vector's body runs and ends the block "
                  "but the TransferDone is never posted, which is the lost "
                  "interrupt util/bus_master.hpp's timeout was written for",
                  timeouts == 1u);
    bench.verdict("...and recover() puts the engine back where start() is legal: "
                  "the very next four transactions run to spi_ok on the same bus "
                  "AO, with no reset of the program between them",
                  kl::Driver::ok_replies == kl::queued);

    bus_ao_live = false;
    dma_host_live = false;
    DmaHost::release();
    (void)Host::init(clock);
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
    // every read with zero, which is what this letter would measure run
    // from a cold flash instead of after its neighbours.
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
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    peer_stop();
    dma_host_live = false;
    bus_ao_live = false;
    bench.verdict("this instance pair CAN be an I2S at all (table 205: SPI1 "
                  "everywhere, SPI2 on the G0B1/G0C1)",
                  S1::has_i2s_mode && S2::has_i2s_mode && S1::has_i2s() && S2::has_i2s());
    bench.verdict("...and a third instance cannot, at compile time and at run "
                  "time both",
                  third_instance_has_no_i2s());

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
                  "answer has to be taken from the one read that saw it - a leg "
                  "that polls the verb reports no underrun at all",
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
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
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
// s - the host above the wire, WIRELESS: every shape of the two loops, and
//     what the block holds after each
// =============================================================================
/// 256 bytes each: the packed receive run of the polled loop and the
/// engines' longest transaction here, with nothing on MISO - what is
/// judged is the block's state after the run and the way it ran, not the
/// bytes.
uint8_t shape_tx[256];
uint8_t shape_rx[256];

struct ShapeResult {
    bool done;
    uint8_t ftlvl;
    uint8_t frlvl;
    bool ovr;
    bool inside;            ///< start() answered true: it ended inside the call
    uint16_t completions;   ///< the engines' completions (TransferDone edges)
    uint16_t blocks;        ///< the engines' blocks under them
};

/// One polled request through the engineless host (`engined` false), or
/// one asynchronous request through the engined host, waited for however
/// it completes; the block's state after it and the way it ran.
ShapeResult run_shape(const uint8_t* tx, uint8_t* rx, uint16_t frames, SpiClock rate,
                      SpiDataSize bits, bool engined, uint8_t cmd_len = 0,
                      const uint8_t* cmd = nullptr) {
    dma_completions = 0;
    dma_blocks = 0;
    host_done = false;
    bool inside = false;
    bool done = false;
    if (engined) {
        DmaHost::Request r{};
        r.engine = SpiEngine::dma;
        r.cs = CsPin::ref();
        r.cmd = lend<Lease::reply>(cmd);
        r.cmd_len = cmd_len;
        r.tx = lend<Lease::reply>(tx);
        r.rx = lend<Lease::reply>(rx);
        r.len = frames;
        r.clock = rate;
        r.bits = bits;
        inside = DmaHost::start(r);
        const uint32_t t0 = Ticker::millis();
        while (!inside && !host_done && Ticker::millis() - t0 < 500u) {
        }
        done = inside || host_done;
    } else {
        done = host_xfer(tx, rx, frames, rate, SpiMode::mode0, bits, true, cmd_len, cmd);
        inside = true;
    }
    const ShapeResult r{done, S1::tx_level(), S1::rx_level(), S1::overrun(), inside,
                        dma_completions, dma_blocks};
    if (!done) {
        if (engined) {
            (void)DmaHost::recover();
        } else {
            (void)Host::recover();
        }
    }
    print(serial, "  ", engined ? "asynchronous " : "polled ", frames,
          bits == SpiDataSize::bits16 ? " 16-bit" : " 8-bit", rx != nullptr ? " read" : " write");
    if (cmd_len != 0u) {
        print(serial, " after a command of ", cmd_len);
    }
    print(serial, " at PCLK/", spi_division(rate), ": done ", r.done ? 1u : 0u, " inside start() ",
          r.inside ? 1u : 0u, " FTLVL ", r.ftlvl, " FRLVL ", r.frlvl, " OVR ", r.ovr ? 1u : 0u,
          " completions ", r.completions, " blocks ", r.blocks, crlf);
    return r;
}

/// A transaction that moved every frame it was given leaves both FIFOs
/// empty and the overrun flag clear - the transmit-only shape clears the
/// one its unread echoes raise - and ran the way it was meant to: inside
/// start() with no completion, or on the engines with one completion over
/// `blocks` blocks.
bool shape_clean(const ShapeResult& r, bool inside, uint16_t blocks = 0) {
    return r.done && r.ftlvl == 0u && r.frlvl == 0u && !r.ovr && r.inside == inside &&
           r.completions == (inside ? 0u : 1u) && r.blocks == blocks;
}

// ---- THE RATE ON THE WIRE, which a self-link cannot tell ------------------
//
// The client of a self-link or a peer is clocked BY the host's SCK, so a
// wrong divisor is a slower or a faster link and every byte still lands.
// So the host's frame is TIMED against its SCK periods at the rate ASKED:
// the difference of two blocks of 2048 and 256 frames - so a
// transaction's fixed cost cancels - on the ticker's cycle count, neither
// buffer named (the transmit side pours 0xFF, the receive side drains
// into a held sink), the best of three of each; 35.5.9 makes the clock
// continuous while TXFIFO holds a frame ("the clock signal is provided
// continuously by the master until TXFIFO becomes empty"), so the frames
// of a block carry no gap and the frame is its SCK periods alone. The
// timing needs no wire - SCK is the host's whatever is on the far end -
// so it runs here, at every BR code and at the dynamic clock's rungs, and
// stands for the letters whose wires set the same divisor (d, i, l, p).

/// One block of `n` frames through the polled host or the engined one,
/// neither buffer named, the best of three, in core cycles.
uint32_t block_cycles(bool engines, uint16_t n, SpiClock rate, SpiDataSize bits) {
    uint32_t least = 0xFFFFFFFFu;
    for (uint8_t k = 0; k < 3u; ++k) {
        const uint32_t t0 = cycles_now();
        if (engines) {
            // Asynchronous: both block lengths ride the engines at every
            // code, the completion waited for in the same call.
            (void)dma_xfer(nullptr, nullptr, n, rate, false, 0, nullptr, bits);
        } else {
            (void)host_xfer(nullptr, nullptr, n, rate, SpiMode::mode0, bits, true);
        }
        const uint32_t took = cycles_now() - t0;
        least = took < least ? took : least;
    }
    return least;
}

/// A frame inside a block, in thousandths of its SCK periods at `sck_hz`
/// - the rate ASKED, never the one the code produced.
uint32_t frame_permille(bool engines, SpiClock rate, SpiDataSize bits, uint32_t sck_hz) {
    const uint32_t big = block_cycles(engines, 2048, rate, bits);
    const uint32_t small = block_cycles(engines, 256, rate, bits);
    const uint32_t width = bits == SpiDataSize::bits16 ? 16u : 8u;
    const uint64_t want = 1792ull * width * SysClock::hz() / sck_hz;
    return big <= small ? 0u : static_cast<uint32_t>(1000ull * (big - small) / want);
}

bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

void ts_shapes() {
    peer_stop();
    software_cs_pads();
    dma_host_live = false;
    bus_ao_live = false;
    bench.verdict("the host comes up on SPI1 with the self-link's pads, nothing wired",
                  Host::init(clock));
    for (uint16_t i = 0; i < 256u; ++i) {
        shape_tx[i] = static_cast<uint8_t>(0x30u + i);
    }
    static const uint8_t cmd[3] = {0x2C, 0x01, 0x02};

    const ShapeResult a = run_shape(shape_tx, nullptr, 16, SpiClock::div4, SpiDataSize::bits8, false);
    bench.verdict("THE TRANSMIT-ONLY POLLED SHAPE: 16 bytes at PCLK/4 paced on TXE, the "
                  "echoes nobody read raised OVR and the tail cleared it with 35.5.11's "
                  "sequence and drained them - both FIFOs empty, the flag clear",
                  shape_clean(a, true));
    const ShapeResult b = run_shape(shape_tx, shape_rx, 255, SpiClock::div4, SpiDataSize::bits8, false);
    bench.verdict("THE PACKED RECEIVE SHAPE: 255 bytes at PCLK/4 read two frames an access "
                  "with two pairs in flight, the odd last one by a byte access with the "
                  "threshold moved for it and moved back - empty FIFOs, no overrun, "
                  "FRXTH at a quarter again",
                  shape_clean(b, true) && S1::rx_threshold() == SpiRxThreshold::quarter);
    const ShapeResult c = run_shape(shape_tx, shape_rx, 3, SpiClock::div4, SpiDataSize::bits8, false);
    bench.verdict("a short receive runs frame by frame, three in flight: three bytes at "
                  "PCLK/4, empty FIFOs after",
                  shape_clean(c, true));
    const ShapeResult d = run_shape(shape_tx, shape_rx, 64, SpiClock::div4, SpiDataSize::bits16, false);
    bench.verdict("half-word frames polled, two in flight: 64 at PCLK/4, empty FIFOs after",
                  shape_clean(d, true));
    const ShapeResult h = run_shape(shape_tx, shape_rx, 8, SpiClock::div16, SpiDataSize::bits8, false, 3, cmd);
    bench.verdict("a two-phase request polled - three command frames transmit-only, the "
                  "D/C flip at their tail, eight data frames read back: empty FIFOs after",
                  shape_clean(h, true));
    const ShapeResult j = run_shape(shape_tx, nullptr, 1, SpiClock::div16, SpiDataSize::bits8, false, 1, nullptr);
    bench.verdict("a command alone with a null buffer clocks one all-ones frame and ends "
                  "clean", shape_clean(j, true));

    // ---- THE ASYNCHRONOUS WAY ----
    // Refused on the engineless host: start() answers at once with
    // spi_rejected, before the block, the select or the D/C is touched.
    // The witnesses: the select's falling edges counted on its own pad
    // (EXTI line 15), none for the refused request and one for the same
    // request polled as the control; CR1 after a mode-0 prime and a mode-3
    // request refused, unchanged - no CPOL flip on the wire; SCK idle low.
    {
        Host::prime(SpiMode::mode0, SpiClock::div4);
        const uint32_t cr1_before = S1::regs().CR1;
        cs_falls = 0;
        (void)CsWatch::select();
        (void)CsWatch::configure(ExtiSense::falling);
        (void)CsWatch::clear();
        (void)CsWatch::arm(true);
        Nvic::enable(CsWatch::irq());
        Host::Request r{};
        r.engine = SpiEngine::dma;
        r.cs = CsPin::ref();
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(shape_tx));
        r.len = 16;
        r.clock = SpiClock::div16;
        r.mode = SpiMode::mode3;
        const bool ended = Host::start(r);
        const uint8_t status = Host::status();
        settle_ms(2);
        const uint16_t refused_falls = cs_falls;
        const uint32_t cr1_after = S1::regs().CR1;
        const bool sck_low = !Pin<'B', 3>::read();
        const uint8_t ftlvl = S1::tx_level();
        const uint8_t frlvl = S1::rx_level();
        r.engine = SpiEngine::polled;
        (void)Host::start(r);
        settle_ms(2);
        const uint16_t polled_falls = static_cast<uint16_t>(cs_falls - refused_falls);
        (void)CsWatch::arm(false);
        (void)CsWatch::clear();
        print(serial, "  refused: start() ", ended ? 1u : 0u, " status ", status,
              " (spi_rejected ", spi_rejected, "), select falls ", refused_falls,
              ", CR1 ", hex(cr1_before), " -> ", hex(cr1_after), ", SCK ", sck_low ? "low" : "HIGH",
              ", FTLVL ", ftlvl, " FRLVL ", frlvl, "; the same request polled: select falls ",
              polled_falls, crlf);
        bench.verdict("AN ASYNCHRONOUS REQUEST ON A HOST WITH NO ENGINES IS REFUSED: start() "
                      "answers at once with spi_rejected, the select never falls (and falls "
                      "once for the same request polled), CR1 is untouched - no CPOL flip for "
                      "the mode-3 request - SCK idles low, and no frame reached the FIFOs",
                      ended && status == spi_rejected && refused_falls == 0u &&
                          polled_falls == 1u && cr1_after == cr1_before && sck_low &&
                          ftlvl == 0u && frlvl == 0u);
    }

    Dma<1>::bus_clock(true);
    // THE RECEIVE SHAPES UNDER A HANDLER LONGER THAN WHAT THEY KEEP IN
    // FLIGHT: the tick's handler stretched to some 1500 cycles - four
    // byte frames at PCLK/2 are 64 - across a hundred receive runs at
    // three rates polled and on the engines at PCLK/4. What the loop keeps
    // in flight never exceeds the RXFIFO's capacity, the transmit FIFO
    // drains into it and the clock stops, so a handler that holds the loop
    // off delays it and loses nothing; the engines' receive channel
    // outranks their transmit one, so no handler stands between them:
    // every run completes (a lost frame never completes: the count never
    // arrives), the levels are zero after, OVR clear.
    struct Stretch {
        const char* name;
        SpiClock rate;
        bool engined;
    };
    constexpr Stretch stretches[4] = {
        {"polled at PCLK/2", SpiClock::div2, false},
        {"polled at PCLK/4", SpiClock::div4, false},
        {"polled at PCLK/16", SpiClock::div16, false},
        {"asynchronous on the engines at PCLK/4", SpiClock::div4, true},
    };
    tick_stretch = 500;   // ~3 cycles a turn: about 1500 cycles a tick
    for (const Stretch& st : stretches) {
        if (st.engined) {
            dma_host_live = true;
            (void)DmaHost::init(clock);
        }
        ticks_stretched = 0;
        uint16_t completed = 0;
        uint16_t ovr_seen = 0;
        uint16_t leftovers = 0;
        for (uint16_t k = 0; k < 100u; ++k) {
            dma_completions = 0;
            const bool done = st.engined ? dma_xfer(shape_tx, shape_rx, 256, st.rate, false)
                                         : host_xfer(shape_tx, shape_rx, 256, st.rate,
                                                     SpiMode::mode0, SpiDataSize::bits8, true);
            const uint8_t status = st.engined ? DmaHost::status() : Host::status();
            if (done && status == spi_ok && dma_completions == (st.engined ? 1u : 0u)) {
                ++completed;
            } else if (st.engined) {
                (void)DmaHost::recover();
            } else {
                (void)Host::recover();
            }
            if (S1::overrun()) {
                ++ovr_seen;
            }
            if (S1::tx_level() != 0u || S1::rx_level() != 0u) {
                ++leftovers;
            }
        }
        print(serial, "  256-byte receive runs, ", st.name, ", the tick's handler stretched: ",
              completed, " of 100 complete, OVR seen ", ovr_seen, ", FIFOs not empty after ",
              leftovers, ", ticks that landed stretched ", ticks_stretched, crlf);
        bench.verdict("A HUNDRED RECEIVE RUNS OF 256 BYTES UNDER A STRETCHED TICK HANDLER lose "
                      "nothing: every one completes, OVR never stands, both FIFOs empty after "
                      "each - what is in flight never exceeds the RXFIFO and the clock stops "
                      "when the TXFIFO has drained into it",
                      completed == 100u && ovr_seen == 0u && leftovers == 0u);
        if (st.engined) {
            dma_host_live = false;
            DmaHost::release();
            (void)Host::init(clock);
        }
    }
    tick_stretch = 0;

    // ---- THE RATE ON THE WIRE: every BR code, timed ----
    Dma<1>::bus_clock(true);
    const bool engined_up = DmaHost::init(clock);
    dma_host_live = true;
    uint32_t engined[8];
    uint32_t polled[8];
    bool engined_ok = engined_up;
    bool polled_ok = true;
    for (uint8_t c = 0; c < 8u; ++c) {
        const auto rate = static_cast<SpiClock>(c);
        const uint32_t sck = SysClock::pclk_hz() / spi_division(rate);
        engined[c] = frame_permille(true, rate, SpiDataSize::bits8, sck);
        engined_ok = engined_ok && wire_ok(engined[c]);
    }
    const uint32_t w16_div2 = frame_permille(true, SpiClock::div2, SpiDataSize::bits16,
                                             SysClock::pclk_hz() / 2u);
    const uint32_t w16_div16 = frame_permille(true, SpiClock::div16, SpiDataSize::bits16,
                                              SysClock::pclk_hz() / 16u);
    dma_host_live = false;
    DmaHost::release();
    (void)Host::init(clock);
    for (uint8_t c = 0; c < 8u; ++c) {
        const auto rate = static_cast<SpiClock>(c);
        polled[c] = frame_permille(false, rate, SpiDataSize::bits8,
                                   SysClock::pclk_hz() / spi_division(rate));
        polled_ok = polled_ok && wire_ok(polled[c]);
    }
    print(serial, "  a frame inside a block, in thousandths of its eight SCK periods at "
          "PCLK/2 .. PCLK/256: engines");
    for (uint8_t c = 0; c < 8u; ++c) {
        print(serial, " ", engined[c]);
    }
    print(serial, "; the polled transmit-only loop");
    for (uint8_t c = 0; c < 8u; ++c) {
        print(serial, " ", polled[c]);
    }
    print(serial, "; 16-bit frames on the engines at PCLK/2 ", w16_div2, ", PCLK/16 ",
          w16_div16, crlf);
    bench.verdict("THE ENGINES RUN AT THE RATE ASKED: a frame inside a block takes its "
                  "SCK periods at PCLK over the divisor, every one of the eight BR codes "
                  "and 16-bit frames at PCLK/2 and /16, each -1 % to +3 % - with no gap "
                  "between frames, the clock continuous while TXFIFO holds one (35.5.9)",
                  engined_ok && wire_ok(w16_div2) && wire_ok(w16_div16));
    bench.verdict("and so does the polled TRANSMIT-ONLY loop, PCLK/2 included: two byte "
                  "frames packed into each half-word store on TXE, which stands while "
                  "TXFIFO is at most half full (35.5.9), outrun sixteen cycles a frame - "
                  "each code -1 % to +3 %",
                  polled_ok);

    // ---- and at the dynamic clock's other rungs: a 1 MHz ceiling ----
    uint32_t rung_pm[2] = {};
    const uint8_t rung_ix[2] = {r_mid, r_slow};
    bool rungs_ok = true;
    for (uint8_t k = 0; k < 2u; ++k) {
        console_drain();
        if (!SysClock::set_index(rung_ix[k])) {
            rungs_ok = false;
            continue;
        }
        (void)DmaHost::init(clock, 1'000'000);
        dma_host_live = true;
        rung_pm[k] = frame_permille(true, SpiClock::div2, SpiDataSize::bits8, 1'000'000u);
        dma_host_live = false;
        DmaHost::release();
        rungs_ok = rungs_ok && wire_ok(rung_pm[k]);
    }
    (void)SysClock::set_index(r_fast);
    console_drain();
    (void)Host::init(clock);
    print(serial, "  a 1 MHz ceiling on the engines, a PCLK/2 request clamped to it: at "
          "16 MHz ", rung_pm[0], ", at 2 MHz ", rung_pm[1], " thousandths of 1 MHz", crlf);
    bench.verdict("AND THE CEILING IS THE RATE ON THE WIRE at every rung: 1 MHz asked, the "
                  "code re-resolved at 16 and at 2 MHz, a frame in its eight periods of "
                  "1 MHz, -1 % to +3 %", rungs_ok);
}

// =============================================================================
// n - the peer's command channel
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
                  "SECOND BOARD speaking the same wire format",
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
                  "directions between TWO SEPARATE CHIPS",
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

/// THE PEER'S DOCUMENTED CLIENT CEILING, by the implementation its ident
/// names (spi_link.hpp lets a host use the kind to expect a capability,
/// never to decide correctness - and a rung above the ceiling is exactly
/// a rung whose correctness no data sheet promises). The stm32f4 peer
/// answers on the F446's SPI2, whose slave mode DS10693 table 63 gives
/// 22.5 MHz (note 2: tv(SO) and the host's setup inside half a clock);
/// the stm32g0 peer's SPI1 is 32 MHz in full duplex at 2.7..3.6 V on
/// every G0 data sheet (DS13560 table 76, DS12232 and DS12992 the same) -
/// this ladder's top, so nothing above it. Zero for a kind whose data
/// sheet this suite has not read: every rung judged.
uint32_t peer_client_ceiling_hz(uint8_t kind) {
    switch (kind) {
        case spilink::peer_stm32f4: return 22'500'000u;
        case spilink::peer_stm32g0: return 32'000'000u;
        default: return 0u;
    }
}

void tp_peer_rates() {
    if (!need_peer()) {
        return;
    }
    spilink::Ident id{};
    const uint32_t ceiling = peer_ident(id) ? peer_client_ceiling_hz(spilink::peer_kind(id.version))
                                            : 0u;
    if (ceiling != 0u) {
        print(serial, "  the peer (fw ", hex(id.version), ") is specified as a client to ",
              ceiling / 1000u, " kHz: a rung above it is printed, not judged", crlf);
    } else {
        print(serial, "  no client ceiling known for this peer: every rung judged", crlf);
    }

    // The BR codes, not a frequency ladder: a Request's clock IS a
    // division of PCLK, so this walks the register's own vocabulary from
    // the command rate upwards and prints what each one really is. The
    // top rung, PCLK/2 = 32 MHz, is only ASKED when everything below it
    // was exact - the ladder's job is to FIND the boundary, and a rung
    // above a break measures nothing.
    static const SpiClock codes[] = {SpiClock::div256, SpiClock::div128,
                                     SpiClock::div64,  SpiClock::div32,
                                     SpiClock::div16,  SpiClock::div8,
                                     SpiClock::div4,   SpiClock::div2};
    uint32_t last_good = 0;
    uint32_t first_bad = 0;
    spilink::Report bad_r{};
    for (const SpiClock c : codes) {
        if (c == SpiClock::div2 && first_bad != 0u) {
            break;
        }
        Exchange e{};
        e.rate = c;
        e.count = 8;
        e.seed_a = 0x21;
        e.seed_b = 0x84;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        const uint32_t real = Host::sck_hz(c);
        const bool above = ceiling != 0u && real > ceiling;
        print(serial, above ? "  [informational, above the peer's ceiling] SCK PCLK/" : "  SCK PCLK/",
              spi_division(c), " = ", real / 1000u, " kHz: ",
              ok ? "exact both ways" : "NOT exact", "  host mism=", v.mism,
              " client mism=", r.mism, " client count=", r.count, " serve=",
              (r.aux2 & 0x04u) != 0u ? "dma" : "pump", crlf);
        if (above) {
            continue;
        }
        if (ok && first_bad == 0u) {
            last_good = real;
        } else if (!ok && first_bad == 0u) {
            first_bad = real;
            bad_r = r;
        }
    }
    print(serial, "  the link to the peer held to ", last_good / 1000u, " kHz");
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

    // THE SECOND CLIMB IS INFORMATIONAL AND CLAIMS NOTHING: the same
    // ladder with `spare_polled_pump` set, which asks the peer for its
    // SOFTWARE serve instead of whatever engine it has. Two boundaries
    // printed side by side is what the bit was added for, and a peer
    // with no engine at all simply repeats its first answer here.
    uint32_t soft_last = 0;
    uint32_t soft_first_bad = 0;
    for (const SpiClock c : codes) {
        if (soft_first_bad != 0u) {
            break;
        }
        Exchange e{};
        e.rate = c;
        e.count = 8;
        e.seed_a = 0x35;
        e.seed_b = 0x9A;
        e.spare = spilink::spare_polled_pump;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        const uint32_t real = Host::sck_hz(c);
        print(serial, "  [informational] software pump at PCLK/", spi_division(c),
              " = ", real / 1000u, " kHz: ", ok ? "exact" : "SLIPPED",
              "  host mism=", v.mism, " client mism=", r.mism, " count=", r.count,
              " serve=", (r.aux2 & 0x04u) != 0u ? "dma" : "pump", crlf);
        if (ok) {
            soft_last = real;
        } else {
            soft_first_bad = real;
        }
    }
    print(serial, "  the peer's SOFTWARE pump held to ", soft_last / 1000u, " kHz");
    if (soft_first_bad != 0u) {
        print(serial, " and broke at ", soft_first_bad / 1000u, " kHz");
    }
    print(serial, " (no verdict: this is the other boundary, printed)", crlf);

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
    Dma<1>::bus_clock(true);
    (void)DmaHost::init(clock, {.mosi_speed = peer_mosi_speed});
    dma_host_live = true;
    bus_ao_live = true;
    host_swallow_completion = false;

    settle();
    DmaHost::prime(SpiMode::mode0, link_clock);
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
                const uint8_t w = want.next();
                if (kl::read_back(i, k) && kl::rx[i][k] != w) {
                    ++mism;
                }
            }
        }
    }
    spilink::Report r{};
    const bool rep = peer_report(r);
    // THE REPORT IS ITSELF A COMMAND, and collecting it runs through
    // link_command_mode(), which brings the engineless host up on the same
    // instance - a reset of the block - and takes the DMA vectors from the
    // arbiter's engined host: so the engined host is brought up again and
    // its claim on the vectors re-stated before the next leg, or every
    // later completion is consumed by the wrong branch and the arbiter
    // times out on a transaction that ran perfectly.
    (void)DmaHost::init(clock, {.mosi_speed = peer_mosi_speed});
    dma_host_live = true;
    bus_ao_live = true;
    print(serial, "  kernel: ", kl::Driver::replies, " replies, ",
          kl::Driver::ok_replies, " ok; host mism=", mism, "; the peer read ",
          r.count, " of ", total, " with mism=", r.mism, " flags=", hex(r.flags), crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH come back in order, "
                  "every one spi_ok, and the bytes on the wire are exactly what "
                  "each request lent - read back by A SECOND BOARD: two on the "
                  "engines, one polled inside start(), one a command in the "
                  "loop before its data block",
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
    dma_host_live = false;
    DmaHost::release();
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

uint8_t cout_[64];
PeerClient::Run client_run{};

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
    // THE DRIVER'S RUN (SpiClient::exchange): the TXFIFO filled with the
    // first four answers before the host's clock can arrive (35.5.8), a
    // packed pair on every TXE after, and each pair read back on RXNE -
    // one call, the select pad its window.
    const uint16_t n = a.count < sizeof crx ? a.count : sizeof crx;
    spilink::Stream out(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < n; ++i) {
        cout_[i] = out.next();
    }
    PeerClient::enable();
    client_run = PeerClient::exchange(std::span<const uint8_t>(cout_, n),
                                      std::span<uint8_t>(crx, n),
                                      static_cast<uint32_t>(a.ms) * (Fast::hz / 24'000u));
    crx_n = client_run.count;
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
    // The peer's own SCK division OF ITS OWN CLOCK, so the same number
    // names a different rate at each end of the family: 750 kHz on a
    // 24 MHz AVR, 1.5 MHz on a 48 MHz SAM, 2 MHz on a 64 MHz G0.
    a.aux16 = 32;
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
    print(serial, "; the run ended ",
          client_run.end == PeerClient::RunEnd::complete ? "complete"
          : client_run.end == PeerClient::RunEnd::closed ? "closed"
                                                         : "STALLED",
          "; the peer as HOST read ", r.count, " with mism=", r.mism, " (first idx ", r.idx,
          " got ", hex(r.got), " exp ", hex(r.exp), ") flags=", hex(r.flags), crlf);
    print(serial, "  read:");
    for (uint16_t i = 0; i < crx_n && i < 16u; ++i) {
        print(serial, " ", hex(crx[i]));
    }
    print(serial, crlf);

    bench.verdict("the client received every frame the foreign host clocked, "
                  "byte-exact",
                  ran && crx_n == a.count && mism == 0u);
    bench.verdict("...and that host read this end's answer stream byte-exact FROM "
                  "THE FIRST FRAME, which is what the TXFIFO filled before the "
                  "select edge buys (35.5.8)",
                  rep && r.count == a.count && r.mism == 0u);

    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swap", command(Op::ping));
}

// =============================================================================
// x - the slip statistics against the peer (a diagnostic, OUTSIDE z)
// =============================================================================
//
// Letter o runs each transfer mode ONCE; this letter runs each of them
// `slip_rounds` times with the seeds moved every round, counts the bursts
// that were not byte-exact and prints where each one broke at both ends.
// No verdict: it is the instrument that measures the falling-edge slip
// (spi.md), not a claim about it.

constexpr uint8_t slip_rounds = 10;
constexpr uint8_t b2b_rounds = 30;

void tx_peer_slips() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  each mode ", slip_rounds, " times, 8 frames at PCLK/",
          spi_division(link_clock), ": bursts not byte-exact, and the first bad "
          "frame at each end (host/client)", crlf);
    uint8_t total = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        uint8_t slips = 0;
        print(serial, "  mode ", m, ":");
        for (uint8_t k = 0; k < slip_rounds; ++k) {
            Exchange e{};
            e.cfg.mode = m;
            e.host_mode = static_cast<SpiMode>(m);
            e.seed_a = static_cast<uint8_t>(0x13u + m + 7u * k);
            e.seed_b = static_cast<uint8_t>(0x57u + m + 11u * k);
            Verify v{};
            spilink::Report r{};
            if (!exchange_exact(e, v, r)) {
                ++slips;
                print(serial, " [", v.idx, "/", r.idx, r.count != e.count ? "!" : "", "]");
            }
        }
        print(serial, "  -> ", slips, " of ", slip_rounds, crlf);
        total = static_cast<uint8_t>(total + slips);
    }
    print(serial, "  ", total, " of ", 4u * slip_rounds, " bursts slipped", crlf);

    // THE BACK-TO-BACK RUNG: the polled host at PCLK/4, whose TXFIFO keeps
    // the frames running without a gap - the rung where the far F446
    // host's MOSI class showed - thirty bursts of thirty-two frames at each
    // MOSI class in turn, the peer link's own class last.
    static constexpr PinSpeed classes[] = {PinSpeed::very_high, PinSpeed::medium, PinSpeed::low, PinSpeed::high};
    static constexpr const char* names[] = {"very_high", "medium", "low", "high"};
    const PinSpeed keep = peer_mosi_speed;
    for (uint8_t c = 0; c < 4u; ++c) {
        peer_mosi_speed = classes[c];
        uint8_t b2b = 0;
        print(serial, "  MOSI at ", names[c], ", PCLK/4, ", b2b_rounds, " bursts of 32 frames:");
        for (uint8_t k = 0; k < b2b_rounds; ++k) {
            Exchange e{};
            e.rate = SpiClock::div4;
            e.count = 32;
            e.seed_a = static_cast<uint8_t>(0x43u + 5u * k);
            e.seed_b = static_cast<uint8_t>(0x29u + 3u * k);
            Verify v{};
            spilink::Report r{};
            if (!exchange_exact(e, v, r)) {
                ++b2b;
                print(serial, " [", v.idx, "/", r.idx, r.count != e.count ? "!" : "", "]");
            }
        }
        print(serial, "  -> ", b2b, " of ", b2b_rounds, crlf);
    }
    peer_mosi_speed = keep;
    (void)link_command_mode();
    print(serial, "  the command channel afterwards: ",
          command(Op::ping) ? "answers" : "DEAD", crlf);
}

// =============================================================================
// u, v, w - THE EXTENDED EXCHANGE against the peer
// =============================================================================
//
// spi_link.hpp's three extended bits give this host's own 16-bit frames,
// its transmit-only shapes and its CRC unit a FOREIGN WITNESS: the peer
// answers or listens in 16-bit frames (`spare_wide`), listens with its
// answer line dark (`spare_receive_only`), or runs its CRC unit beside
// this one and checks the CRC frame this end sends (`spare_crc`). A peer
// that serves them raises `report_extended`; one that does not ignores
// the bits, and every verdict here asks for the flag first. Every letter
// climbs the BR codes from the command rate up: a rung above the peer's
// documented client ceiling (the kind its ident names, letter p's
// table) is printed and not judged.

namespace ext {

/// Which of this host's two ways clocks the window: the polled loop, or
/// the DMA engines (a request naming them on the engined host of letter
/// i, its half-word channels).
enum class Path : uint8_t { polled, engines };

const char* path_name(Path p) {
    switch (p) {
        case Path::polled: return "polled loop";
        default: return "DMA engines";
    }
}

constexpr uint16_t cap = 64;   ///< frames this letter set clocks at most
alignas(4) uint8_t tx[2u * cap + 4u];
alignas(4) uint8_t rx[2u * cap + 4u];

struct Ext {
    Path path = Path::polled;
    SpiClock rate = link_clock;
    uint16_t count = 32;
    bool wide = false;
    bool receive_only = false;
    uint8_t spare = 0;   ///< spare_polled_pump, on top of the extended bits
    uint8_t seed_a = 0x4D;
    uint8_t seed_b = 0xB2;
};

struct Outcome {
    bool accepted = false;    ///< the peer took the command
    bool clocked = false;     ///< this host's transaction completed
    bool reported = false;
    spilink::Report r{};
    uint16_t mism = 0;        ///< this end's read-back, bytes wrong
    uint16_t idx = 0xFFFF;
    uint8_t got = 0, exp = 0;
    uint32_t cycles = 0;      ///< start() to completion, core cycles
};

/// Whether the peer served the extended exchange and heard every frame.
bool peer_heard(const Ext& e, const Outcome& o) {
    return o.accepted && o.reported && (o.r.flags & spilink::report_extended) != 0u &&
           o.r.count == e.count && o.r.mism == 0u;
}

/// The whole window both ways, judged at both ends.
bool exact(const Ext& e, const Outcome& o) {
    return peer_heard(e, o) && o.clocked && (e.receive_only || o.mism == 0u);
}

/// Command one extended exchange and clock it on `e.path`.
Outcome run(const Ext& e) {
    Outcome o{};
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = e.count;
    a.ms = 400;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = static_cast<uint8_t>(e.spare | (e.wide ? spilink::spare_wide : 0u) |
                                   (e.receive_only ? spilink::spare_receive_only : 0u));
    o.accepted = peer_act(Op::exchange, a);
    if (!o.accepted) {
        return o;
    }
    const uint16_t n = e.count < cap ? e.count : cap;
    const uint16_t bytes = static_cast<uint16_t>(e.wide ? 2u * n : n);
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < sizeof tx; ++i) {
            tx[i] = i < bytes ? out.next() : 0u;
            rx[i] = 0xEE;
        }
    }
    settle();
    const SpiDataSize bits = e.wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    uint8_t* const in = e.receive_only ? nullptr : rx;
    host_done = false;
    if (e.path == Path::engines) {
        Dma<1>::bus_clock(true);
        dma_host_live = true;
        (void)DmaHost::init(clock, {.mosi_speed = peer_mosi_speed});
        DmaHost::prime(SpiMode::mode0, e.rate);
        CsPin::clear();
        link_hold();
        DmaHost::Request r{};
        r.cs = {};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
        r.rx = lend<Lease::reply>(in);
        r.len = n;
        r.clock = e.rate;
        r.mode = SpiMode::mode0;
        r.bits = bits;
        r.engine = SpiEngine::dma;
        const uint32_t t0 = cycles_now();
        bool done = DmaHost::start(r);
        const uint32_t m0 = Ticker::millis();
        while (!done && !host_done && Ticker::millis() - m0 < 200u) {
        }
        o.cycles = cycles_now() - t0;
        o.clocked = done || host_done;
        link_hold();
        CsPin::set();
        DmaHost::release();
        dma_host_live = false;
    } else {
        Host::prime(SpiMode::mode0, e.rate);
        CsPin::clear();
        link_hold();
        Host::Request r{};
        r.cs = {};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
        r.rx = lend<Lease::reply>(in);
        r.len = n;
        r.clock = e.rate;
        r.mode = SpiMode::mode0;
        r.bits = bits;
        r.engine = SpiEngine::polled;
        const uint32_t t0 = cycles_now();
        bool done = Host::start(r);
        const uint32_t m0 = Ticker::millis();
        while (!done && !host_done && Ticker::millis() - m0 < 200u) {
        }
        o.cycles = cycles_now() - t0;
        o.clocked = done || host_done;
        link_hold();
        CsPin::set();
    }
    (void)link_command_mode();
    o.reported = peer_report(o.r);
    if (!e.receive_only) {
        for (uint16_t i = 0; i < bytes; ++i) {
            const uint8_t want = spilink::wide_byte(a.pattern, a.seed_b, i, e.wide, false);
            if (rx[i] != want) {
                if (o.mism == 0u) {
                    o.idx = i;
                    o.got = rx[i];
                    o.exp = want;
                }
                ++o.mism;
            }
        }
    }
    return o;
}

void print_outcome(const Ext& e, const Outcome& o, bool above) {
    print(serial, above ? "    [informational, above the peer's ceiling] " : "    ",
          path_name(e.path), " PCLK/", spi_division(e.rate), " = ", Host::sck_hz(e.rate) / 1000u,
          " kHz, ", e.count, e.wide ? " 16-bit" : " 8-bit", " frames: ",
          exact(e, o) ? "exact" : "NOT exact");
    if (!e.receive_only) {
        print(serial, "  host mism=", o.mism);
        if (o.mism != 0u) {
            print(serial, " (byte ", o.idx, " got ", hex(o.got), " exp ", hex(o.exp), ")");
        }
    }
    print(serial, "  peer count=", o.r.count, " mism=", o.r.mism);
    if (o.r.mism != 0u) {
        print(serial, " (byte ", o.r.idx, " got ", hex(o.r.got), " exp ", hex(o.r.exp), ")");
    }
    print(serial, " flags=", hex(o.r.flags), " serve=", (o.r.aux2 & 0x04u) != 0u ? "dma" : "run",
          o.clocked ? "" : " HOST NEVER COMPLETED", crlf);
}

/// The peer's documented client ceiling, from its ident (letter p's).
uint32_t ceiling = 0;

void learn_ceiling() {
    spilink::Ident id{};
    ceiling = peer_ident(id) ? peer_client_ceiling_hz(spilink::peer_kind(id.version)) : 0u;
    if (ceiling != 0u) {
        print(serial, "  the peer (fw ", hex(id.version), ") is specified as a client to ",
              ceiling / 1000u, " kHz: a rung above it is printed, not judged", crlf);
    }
}

bool above_ceiling(SpiClock c) { return ceiling != 0u && Host::sck_hz(c) > ceiling; }

constexpr SpiClock rungs[] = {SpiClock::div256, SpiClock::div128, SpiClock::div64,
                              SpiClock::div32,  SpiClock::div16,  SpiClock::div8,
                              SpiClock::div4,   SpiClock::div2};

/// Climb every BR code with `e` and say whether every JUDGED rung was
/// exact; `judged` counts them.
bool climb(Ext e, uint8_t& judged) {
    bool all = true;
    judged = 0;
    uint8_t k = 0;
    for (const SpiClock c : rungs) {
        e.rate = c;
        e.seed_a = static_cast<uint8_t>(0x4Du + 9u * k);
        e.seed_b = static_cast<uint8_t>(0xB2u + 5u * k);
        ++k;
        const Outcome o = run(e);
        const bool above = above_ceiling(c);
        print_outcome(e, o, above);
        if (!above) {
            ++judged;
            all = all && exact(e, o);
        }
    }
    return all && judged != 0u;
}

/// THE WIRE'S TIME, from the difference of two windows of 16 and 48
/// frames at the same rung - the transaction's fixed cost cancels - in
/// permille of the frame's SCK periods (PCLK = HCLK here, so a bit is
/// `division` core cycles). Zero when either window was not exact.
uint32_t frame_permille(Ext e) {
    Ext shorter = e;
    shorter.count = 16;
    Ext longer = e;
    longer.count = 48;
    longer.seed_a = static_cast<uint8_t>(e.seed_a + 3u);
    const Outcome a = run(shorter);
    const Outcome b = run(longer);
    if (!exact(shorter, a) || !exact(longer, b) || b.cycles <= a.cycles) {
        return 0u;
    }
    const uint32_t per_frame_x1000 = (b.cycles - a.cycles) * 1000u / 32u;
    const uint32_t wire = (e.wide ? 16u : 8u) * spi_division(e.rate);
    return per_frame_x1000 / wire;
}

}  // namespace ext

void tu_wide() {
    if (!need_peer()) {
        return;
    }
    ext::learn_ceiling();
    print(serial, "  16-bit frames (CR2.DS = 1111, a half-word access a frame), 32 a window, "
                  "against the peer's DMA serve in 16-bit beats:", crlf);
    bool path_ok[2] = {};
    for (uint8_t p = 0; p < 2u; ++p) {
        ext::Ext e{};
        e.path = static_cast<ext::Path>(p);
        e.wide = true;
        uint8_t judged = 0;
        path_ok[p] = ext::climb(e, judged);
        print(serial, "  -> ", ext::path_name(e.path), ": ", path_ok[p] ? "exact" : "NOT exact",
              " at every judged rung (", judged, ")", crlf);
    }
    bench.verdict("16-BIT FRAMES ON THE POLLED LOOP: thirty-two frames byte-exact both ways at "
                  "every judged code, the peer serving them in 16-bit beats (report_extended)",
                  path_ok[0]);
    bench.verdict("...and on the DMA ENGINES, a half-word beat a frame (the beat is the frame), "
                  "likewise",
                  path_ok[1]);

    // THE WIRE'S TIME: the frame period on the engines and the polled
    // loop, against the frame's sixteen SCK periods, at PCLK/16 and PCLK/4.
    static constexpr SpiClock timed[] = {SpiClock::div16, SpiClock::div4};
    bool engines_wire = true;
    bool polled_wire = true;
    for (const SpiClock c : timed) {
        for (uint8_t p = 0; p < 2u; ++p) {
            ext::Ext e{};
            e.path = static_cast<ext::Path>(p);
            e.wide = true;
            e.rate = c;
            const uint32_t pm = ext::frame_permille(e);
            print(serial, "  ", ext::path_name(e.path), " at PCLK/", spi_division(c),
                  ": a 16-bit frame takes ", pm, " permille of its 16 SCK periods", crlf);
            if (e.path == ext::Path::engines) {
                engines_wire = engines_wire && wire_ok(pm);
            } else {
                polled_wire = polled_wire && wire_ok(pm);
            }
        }
    }
    bench.verdict("THE ENGINES' 16-BIT FRAMES RUN AT THE WIRE'S TIME: a frame inside the "
                  "window is its sixteen SCK periods, -1 % to +3 %, at PCLK/16 and PCLK/4",
                  engines_wire);
    bench.verdict("...and so do the polled loop's, its two half-words in flight", polled_wire);

    // The peer's own 16-bit RUN (spare_polled_pump): the other end's
    // SpiClient::exchange in uint16_t frames, judged at the same rungs.
    ext::Ext run{};
    run.wide = true;
    run.spare = spilink::spare_polled_pump;
    uint8_t judged = 0;
    const bool run_ok = ext::climb(run, judged);
    bench.verdict("the PEER'S CLIENT RUN in 16-bit frames (its exchange() over uint16_t) is "
                  "exact both ways at every judged code",
                  run_ok);
    (void)link_command_mode();
    bench.verdict("the command channel survives the 16-bit climbs", command(Op::ping));
}

void tv_receive_only() {
    if (!need_peer()) {
        return;
    }
    ext::learn_ceiling();
    // THE HOST'S TRANSMIT-ONLY SHAPES, WITNESSED: `rx` null is the loop that
    // reads nothing back (frames in on TXE, the unread echoes' overrun
    // cleared at the tail) and the engines' held sink - and the peer
    // listens with its answer line DARK, so what it
    // judges is this end's transmit path alone. An ODD count: thirty-three.
    print(serial, "  transmit-only windows of 33 frames, the peer listening with MISO dark "
                  "(its receive DMA):", crlf);
    bool shapes[2][2] = {};
    for (uint8_t p = 0; p < 2u; ++p) {
        for (uint8_t w = 0; w < 2u; ++w) {
            ext::Ext e{};
            e.path = static_cast<ext::Path>(p);
            e.wide = w != 0u;
            e.receive_only = true;
            e.count = 33;
            uint8_t judged = 0;
            shapes[p][w] = ext::climb(e, judged);
        }
    }
    bench.verdict("THE POLLED TRANSMIT-ONLY LOOP, witnessed by a listening client: 33 frames, "
                  "8-bit and 16-bit, every one heard exact at every judged code",
                  shapes[0][0] && shapes[0][1]);
    bench.verdict("...and the engines' write shape (the receive side drained into a held "
                  "sink) likewise",
                  shapes[1][0] && shapes[1][1]);

    // THE PEER'S receive() RUN (spare_polled_pump with the dark line), at
    // an odd count either side of its pairs and in both widths.
    print(serial, "  the peer's receive() run, 7 and 33 frames, from the polled loop:", crlf);
    bool receive_ok = true;
    static constexpr uint16_t counts[] = {7, 33};
    for (const uint16_t n : counts) {
        for (uint8_t w = 0; w < 2u; ++w) {
            ext::Ext e{};
            e.wide = w != 0u;
            e.receive_only = true;
            e.count = n;
            e.spare = spilink::spare_polled_pump;
            uint8_t judged = 0;
            receive_ok = ext::climb(e, judged) && receive_ok;
        }
    }
    bench.verdict("the PEER'S receive() RUN hears 7 and 33 frames exact, 8-bit and 16-bit, at "
                  "every judged code",
                  receive_ok);

    // ODD-COUNT EXCHANGES both ways, both serves.
    print(serial, "  exchanges of 7 and 33 8-bit frames, both ways:", crlf);
    bool odd_ok = true;
    for (const uint16_t n : counts) {
        for (uint8_t s = 0; s < 2u; ++s) {
            ext::Ext e{};
            e.count = n;
            e.spare = s != 0u ? spilink::spare_polled_pump : 0u;
            uint8_t judged = 0;
            odd_ok = ext::climb(e, judged) && odd_ok;
        }
    }
    bench.verdict("ODD COUNTS both ways: 7 and 33 frames exact at every judged code, against "
                  "the peer's DMA serve and its client run",
                  odd_ok);
    (void)link_command_mode();
    bench.verdict("the command channel survives the receive-only climbs", command(Op::ping));
}

// ---- w: the CRC ------------------------------------------------------------------

namespace ext {

/// The bitwise CRC spi_link.hpp names: of the frame's width, MSb first,
/// unreflected, from zero, over `frames` frames of a byte stream (a 16-bit
/// frame is two bytes, low first).
uint16_t crc_of(uint16_t poly, bool wide, const uint8_t* bytes, uint16_t frames) {
    const uint8_t width = wide ? 16u : 8u;
    const uint32_t top = 1u << (width - 1u);
    const uint32_t mask = (1u << width) - 1u;
    uint32_t crc = 0;
    for (uint16_t k = 0; k < frames; ++k) {
        const uint16_t f = wide ? static_cast<uint16_t>(bytes[2u * k] | (bytes[2u * k + 1u] << 8))
                                : bytes[k];
        for (uint8_t b = width; b-- > 0;) {
            const bool in = ((f >> b) & 1u) != 0u;
            const bool out = (crc & top) != 0u;
            crc = (crc << 1) & mask;
            if (in != out) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint16_t>(crc);
}

struct Crc {
    Outcome o{};
    uint16_t frame = 0;          ///< the frame read where the peer's CRC belongs
    uint16_t tx_crc = 0, rx_crc = 0;
    uint16_t tx_ref = 0, rx_ref = 0, peer_ref = 0;
    bool host_crcerr = false;
    bool frames_ok = false;      ///< every frame's RXNE came inside its budget
};

/// Letter t's three stamps, SysTick's down-count at HCLK: before the last
/// data store, after the CRCNEXT store, and when that frame's RXNE is
/// first seen.
uint32_t walk_store = 0, walk_next = 0, walk_rxne = 0;

uint32_t walk_since(uint32_t from, uint32_t to) {
    const uint32_t period = SysTick->LOAD + 1u;
    return (from + period - to) % period;
}

[[gnu::always_inline]] inline void spin_loops(uint32_t n) {
    if (n != 0u) {
        __asm__ volatile("1: sub %0, #1\n\tbne 1b" : "+l"(n) : : "cc");
    }
}

/// One CRC exchange, this end BY HAND on the resource - SpiHost's Request
/// has no CRC phase: `count` data frames, CRCNEXT right after the last is
/// written (35.9.1), then the frame the hardware puts on the wire by
/// itself - the frames kept ahead in the TXFIFO and the last two written
/// as one burst (below), or, with `one_in_flight`, each into an idle
/// shifter and CRCNEXT `next_delay` loop turns after the last (letter t).
/// `corrupt` leaves this end's unit OFF and sends the CRC frame as one
/// more DATA frame, its last bit flipped: a wrong CRC on a real wire, for
/// the peer to catch.
Crc run_crc(bool wide, uint16_t poly, SpiClock rate, uint8_t spare, bool corrupt,
            uint8_t seed_a, uint8_t seed_b, bool one_in_flight = false,
            uint32_t next_delay = 0) {
    Crc c{};
    constexpr uint16_t n = 16;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = n;
    a.ms = 400;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = static_cast<uint8_t>(spare | spilink::spare_crc | (wide ? spilink::spare_wide : 0u));
    a.aux16 = poly;
    c.o.accepted = peer_act(Op::exchange, a);
    if (!c.o.accepted) {
        return c;
    }
    const uint8_t w = wide ? 2u : 1u;
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < n * w; ++i) {
            tx[i] = out.next();
            rx[i] = 0xEE;
        }
    }
    c.tx_ref = crc_of(poly, wide, tx, n);
    settle();
    const SpiDataSize bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    (void)S1::disable();
    c.o.clocked = S1::configure({.role = SpiRole::host,
                                 .mode = SpiMode::mode0,
                                 .clock = rate,
                                 .bits = bits,
                                 .crc = !corrupt,
                                 .crc_length = wide ? SpiCrcLength::crc16 : SpiCrcLength::crc8,
                                 .crc_polynomial = poly});
    S1::enable();
    S1::flush_rx();
    CsPin::clear();
    link_hold();
    bool frames = true;
    auto frame_at = [&](uint16_t i) -> uint16_t {
        if (i == n) {   // the corrupted CRC frame, sent as data
            return static_cast<uint16_t>(c.tx_ref ^ 0x0001u);
        }
        return wide ? static_cast<uint16_t>(tx[2u * i] | (tx[2u * i + 1u] << 8)) : tx[i];
    };
    auto keep = [&](uint16_t i, uint16_t got) {
        if (i == n) {
            c.frame = got;
            return;
        }
        rx[w * i] = static_cast<uint8_t>(got);
        if (wide) {
            rx[2u * i + 1u] = static_cast<uint8_t>(got >> 8);
        }
    };
    const uint16_t written = static_cast<uint16_t>(corrupt ? n + 1u : n);
    if (one_in_flight) {
        // THE WALK'S SHAPE (letter t): a frame written into an idle shifter
        // and read back before the next, CRCNEXT `next_delay` loop turns
        // after the last data store.
        for (uint16_t i = 0; i <= n; ++i) {
            const bool last = i == n - 1u && !corrupt;
            if (i < written) {
                const uint16_t v = frame_at(i);
                if (last) {
                    walk_store = SysTick->VAL;
                    S1::data(bits, v);
                    spin_loops(next_delay);
                    S1::crc_next();
                    walk_next = SysTick->VAL;
                } else {
                    S1::data(bits, v);
                }
            }
            uint32_t spins = 400'000u;
            while (!S1::rxne() && --spins != 0u) {   // a time-out leaves 0, never a wrap
            }
            if (last) {
                walk_rxne = SysTick->VAL;
            }
            frames = frames && spins != 0u;
            keep(i, S1::data(bits));
        }
    } else {
        // THE WRITE-AHEAD SHAPE: frames into the TXFIFO while TXE admits
        // them, so the last data store - and the CRCNEXT right after it -
        // happens with the frames before it still queued: the bit must
        // land about two SCK periods before the last data frame ENDS
        // (letter t's walk), which a store into an idle shifter cannot
        // promise at the fast codes. In flight never exceeds the TXFIFO and
        // the shifter, inside the RXFIFO's four frames.
        // In flight - written and not read back - stays inside the RXFIFO
        // (four byte frames, two half-words: 35.5.9), and the LAST TWO
        // data frames go in as one burst, CRCNEXT the store after: the
        // shifter then still holds the second-to-last frame when the bit
        // lands, whatever this loop's own pace.
        const uint16_t depth = wide ? 2u : 3u;
        uint16_t wr = 0;
        uint16_t rd = 0;
        uint32_t spins = 400'000u;
        while (rd <= n && spins-- != 0u) {
            const uint32_t sr = S1::status();
            if (wr < written && (sr & SPI_SR_TXE) != 0u) {
                if (!corrupt && wr + 2u == n) {
                    if (wr - rd + 2u <= depth) {
                        S1::data(bits, frame_at(wr));
                        S1::data(bits, frame_at(static_cast<uint16_t>(wr + 1u)));
                        S1::crc_next();
                        wr = static_cast<uint16_t>(wr + 2u);
                    }
                } else if (wr - rd < depth) {
                    S1::data(bits, frame_at(wr));
                    ++wr;
                }
            }
            if ((sr & SPI_SR_RXNE) != 0u) {
                keep(rd, S1::data(bits));
                ++rd;
            }
        }
        frames = rd == n + 1u;
    }
    link_hold();
    CsPin::set();
    c.frames_ok = frames;
    c.tx_crc = S1::tx_crc();
    c.rx_crc = S1::rx_crc();
    c.host_crcerr = S1::crc_error();
    c.rx_ref = crc_of(poly, wide, rx, n);
    {
        uint8_t want[2u * n];
        for (uint16_t i = 0; i < n * w; ++i) {
            want[i] = spilink::pattern_value(a.pattern, a.seed_b, i);
            if (rx[i] != want[i]) {
                if (c.o.mism == 0u) {
                    c.o.idx = i;
                    c.o.got = rx[i];
                    c.o.exp = want[i];
                }
                ++c.o.mism;
            }
        }
        c.peer_ref = crc_of(poly, wide, want, n);
    }
    (void)link_command_mode();
    c.o.reported = peer_report(c.o.r);
    return c;
}

/// A matched CRC exchange, judged whole: the frames both ways, this end's
/// two accumulators against the arithmetic, the CRC frame it read against
/// the peer's stream's, and neither end's CRCERR.
bool crc_agrees(const Crc& c, bool wide) {
    const uint16_t mask = wide ? 0xFFFFu : 0x00FFu;
    return c.o.accepted && c.o.clocked && c.o.reported && c.frames_ok && c.o.mism == 0u &&
           (c.o.r.flags & spilink::report_extended) != 0u && c.o.r.count == 16u &&
           c.o.r.mism == 0u && (c.o.r.flags & spilink::report_crcerr) == 0u &&
           c.tx_crc == c.tx_ref && c.rx_crc == c.rx_ref && (c.frame & mask) == c.peer_ref &&
           !c.host_crcerr && c.o.r.aux3 == static_cast<uint8_t>(c.tx_ref);
}

void print_crc(const char* label, SpiClock rate, bool above, const Crc& c) {
    print(serial, above ? "    [informational, above the peer's ceiling] " : "    ", label,
          " PCLK/", spi_division(rate), ": TXCRCR ", hex(c.tx_crc), " (bitwise ", hex(c.tx_ref),
          "), RXCRCR ", hex(c.rx_crc), " (bitwise over what came ", hex(c.rx_ref),
          "), the CRC frame ", hex(c.frame), " (the peer's, bitwise ", hex(c.peer_ref),
          "), CRCERR here ", c.host_crcerr ? "SET" : "clear", "; host mism ", c.o.mism,
          "; the peer heard ", c.o.r.count, " mism ", c.o.r.mism, " flags ", hex(c.o.r.flags),
          " RXCRCR low ", hex(c.o.r.aux3), " serve=", (c.o.r.aux2 & 0x04u) != 0u ? "dma" : "run",
          crlf);
}

}  // namespace ext

// t - THE CRCNEXT WINDOW (OUTSIDE z): the instrument behind letter w's
// write-ahead. One frame in flight, the CRCNEXT store walked a loop turn
// at a time away from the last data store; each point printed as
// next/rxne:TRP - the core cycles (SysTick) from before the last data
// store to after the CRCNEXT store, and to that frame's RXNE first seen;
// T this end's TXCRCR the arithmetic, R its RXCRCR, P the peer's check
// clean, F every frame read back, the CRC frame among them (lower case:
// not - f where the store came past the last frame's end and no CRC frame
// was clocked). No verdict.
void tt_crcnext_walk() {
    if (!need_peer()) {
        return;
    }
    const uint32_t t0 = cycles_now();
    ext::spin_loops(10000u);
    print(serial, "  a loop turn is ", (cycles_now() - t0) / 10000u, " core cycles", crlf);
    static constexpr SpiClock rates[] = {SpiClock::div2, SpiClock::div4, SpiClock::div8, SpiClock::div16};
    for (uint8_t w = 0; w < 2u; ++w) {
        for (const SpiClock c : rates) {
            const uint32_t frame = (w ? 16u : 8u) * spi_division(c);
            print(serial, "  ", w ? "16" : "8", "-bit PCLK/", spi_division(c), " (frame ", frame, " cycles):");
            for (uint32_t d = 0; d <= frame / 3u + 6u; ++d) {
                const ext::Crc r = ext::run_crc(w != 0u, w ? 0x1021u : 0x0007u, c, 0, false,
                                                static_cast<uint8_t>(0x5Bu + d), 0xB9, true, d);
                const bool tx_ok = r.tx_crc == r.tx_ref;
                const bool rx_ok = r.rx_crc == r.rx_ref;
                const bool peer_ok = (r.o.r.flags & spilink::report_crcerr) == 0u;
                print(serial, " ", ext::walk_since(ext::walk_store, ext::walk_next), "/",
                      ext::walk_since(ext::walk_store, ext::walk_rxne), ":", tx_ok ? "T" : "t",
                      rx_ok ? "R" : "r", peer_ok ? "P" : "p", r.frames_ok ? "F" : "f");
            }
            print(serial, crlf);
        }
    }
    (void)link_command_mode();
}

void tw_crc() {
    if (!need_peer()) {
        return;
    }
    ext::learn_ceiling();
    // Both widths, both serves - the peer's engines (35.5.14's "CRC
    // transfer managed by DMA" on the G0, 26.4.2's on the F4) and its CRC
    // run over the resource - at every BR code, this end's unit by hand.
    struct Leg {
        bool wide;
        uint16_t poly;
        uint8_t spare;
        const char* name;
    };
    static const Leg legs[] = {
        {false, 0x0007u, 0, "CRC-8 0x07, the peer's DMA serve,"},
        {true, 0x1021u, 0, "CRC-16 0x1021, the peer's DMA serve,"},
        {false, 0x0007u, spilink::spare_polled_pump, "CRC-8 0x07, the peer's CRC run,"},
        {true, 0x1021u, spilink::spare_polled_pump, "CRC-16 0x1021, the peer's CRC run,"},
    };
    bool leg_ok[4] = {};
    for (uint8_t l = 0; l < 4u; ++l) {
        bool all = true;
        uint8_t k = 0;
        for (const SpiClock c : ext::rungs) {
            const ext::Crc r = ext::run_crc(legs[l].wide, legs[l].poly, c, legs[l].spare, false,
                                            static_cast<uint8_t>(0x31u + 7u * k),
                                            static_cast<uint8_t>(0xA7u + 3u * k));
            ++k;
            const bool above = ext::above_ceiling(c);
            ext::print_crc(legs[l].name, c, above, r);
            if (!above) {
                all = all && ext::crc_agrees(r, legs[l].wide);
            }
        }
        leg_ok[l] = all;
    }
    bench.verdict("THE CRC UNIT AGAINST A FOREIGN ONE, 8-bit: at every judged code this end's "
                  "TXCRCR and RXCRCR are the bitwise arithmetic, the CRC frame it reads is the "
                  "peer's stream's CRC, and NEITHER END raises CRCERR (the peer's engines "
                  "managing its CRC by themselves)",
                  leg_ok[0]);
    bench.verdict("...and with 16-bit frames and a 16-bit CRC likewise", leg_ok[1]);
    bench.verdict("...and against the peer's CRC run (CRCNEXT after its last answer), both "
                  "widths",
                  leg_ok[2] && leg_ok[3]);

    // A WRONG CRC ON A REAL WIRE: this end's unit off, the CRC frame sent as
    // data with its last bit flipped. The peer must flag it; its own CRC
    // frame must still be the arithmetic of its stream.
    bool caught = true;
    static constexpr SpiClock fast_judged[] = {SpiClock::div256, SpiClock::div4};
    for (uint8_t l = 0; l < 4u; ++l) {
        for (const SpiClock c : fast_judged) {
            const ext::Crc r = ext::run_crc(legs[l].wide, legs[l].poly, c, legs[l].spare, true,
                                            0x5E, 0x3B);
            print(serial, "  CORRUPTED, ");
            ext::print_crc(legs[l].name, c, false, r);
            const uint16_t mask = legs[l].wide ? 0xFFFFu : 0x00FFu;
            caught = caught && r.o.accepted && r.o.reported && r.frames_ok &&
                     (r.o.r.flags & spilink::report_crcerr) != 0u && r.o.r.count == 16u &&
                     r.o.r.mism == 0u && r.o.mism == 0u && (r.frame & mask) == r.peer_ref;
        }
    }
    bench.verdict("A WRONG CRC FRAME IS CAUGHT BY THE FOREIGN UNIT: one flipped bit in the "
                  "frame where the CRC belongs and the peer reports CRCERR (report_crcerr), "
                  "every data frame still exact, its own CRC frame still the arithmetic - both "
                  "widths, both serves, at the command rate and at PCLK/4",
                  caught);
    (void)link_command_mode();
    bench.verdict("the command channel survives the CRC", command(Op::ping));
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

// =============================================================================
// y - THE SIMPLEX LINES, THE SOFTWARE SELECT AND prime() against the peer
// =============================================================================
//
// Every other peer letter runs both data lines and frames on a select pad.
// This one takes each of those away in turn and asks the FOREIGN end what
// arrived: a client under SSM (no NSS pad, SSI the select), the host's
// single line out (BIDIMODE + BIDIOE), the host's receive-only clock
// (RXONLY), the client's single line out (BIDIMODE on MISO), the client's
// receive-only run - and prime(), staged by its absence: the mode applied
// INSIDE an open select window.
//
// What the wiring cannot carry, and why it is not here: a host RECEIVING
// on its single line (BIDIMODE, BIDIOE clear) needs the far end to drive
// THIS end's MOSI, and the peer's MOSI pad is an input on every op; a
// client receiving on its single line needs the far HOST to drive MISO,
// the host's input. Both are wire facts, not driver ones.

namespace simplex {

/// The client role WITHOUT AN NSS PAD: SPI1's three data pads alone, the
/// select the software bit (SSM, and SSI driven by select()).
constexpr SpiPins ssm_pins{
    .sck = host_pins.sck, .miso = host_pins.miso, .mosi = host_pins.mosi, .nss = {}};
using SsmClient = SpiClient<1, ssm_pins>;
static_assert(!SsmClient::has_nss_pad);
/// The peer's select wire, read as a plain input: the program's own
/// knowledge of when it is addressed - what an SSM client stands for.
using SelectWire = Pin<'A', 15>;

/// The host_burst every client leg of this letter is clocked by: twelve
/// frames at the peer's own clock / 32 (1.406 MHz on the F446's 45 MHz
/// APB1), 20 us between frames, 40 ms of lead-in (spi_link.hpp).
spilink::Params burst_params(uint8_t seed_a, uint8_t seed_b) {
    spilink::Params a{};
    a.count = 12;
    a.ms = 300;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;
    a.aux16 = 32;
    a.cfg.apply = 1;
    return a;
}

/// A run's spin budget for `ms` milliseconds of window (letter r's own).
uint32_t budget(uint16_t ms) { return static_cast<uint32_t>(ms) * (Fast::hz / 24'000u); }

/// Bytes of `got` that differ from the pattern stream `seed`.
uint16_t mismatches(const uint8_t* got, uint16_t n, uint8_t pattern, uint8_t seed) {
    uint16_t m = 0;
    for (uint16_t i = 0; i < n; ++i) {
        if (got[i] != spilink::pattern_value(pattern, seed, i)) {
            ++m;
        }
    }
    return m;
}

void print_bytes(const char* label, const uint8_t* p, uint16_t n) {
    print(serial, label);
    for (uint16_t i = 0; i < n && i < 16u; ++i) {
        print(serial, " ", hex(p[i]));
    }
    print(serial, crlf);
}

template <typename E>
const char* end_name(E e) {
    return e == E::complete ? "complete" : e == E::closed ? "closed" : "stalled";
}

// ---- the client under SSM ---------------------------------------------------

struct SsmLeg {
    bool accepted = false;
    bool fell = false;       ///< the peer's select was seen to fall on PA15
    SsmClient::Run run{0, SsmClient::RunEnd::stalled};
    bool reported = false;
    spilink::Report r{};
};

/// One burst against the client with no select pad. `select_it`: drive SSI
/// low once the wire's select is seen to fall (the program's software
/// select), or leave it high for the whole burst (a deselected client the
/// clock passes by).
SsmLeg ssm_leg(bool select_it, uint8_t seed_a, uint8_t seed_b) {
    SsmLeg L{};
    const spilink::Params a = burst_params(seed_a, seed_b);
    L.accepted = peer_act(Op::host_burst, a);
    if (!L.accepted) {
        return L;
    }
    peer_stop();
    Nvic::disable(S1::irq());
    (void)SsmClient::init(clock, {.mode = SpiMode::mode0,
                                  .bits = SpiDataSize::bits8,
                                  .lsb_first = false,
                                  .nss = SpiNss::software,
                                  .drive_output = true});
    // init() leaves a software-NSS client SELECTED (SSI low, the role's
    // default); the window is this program's, so it opens deselected.
    SsmClient::select(false);
    SelectWire::input();
    const uint16_t n = a.count;
    spilink::Stream out(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < n; ++i) {
        cout_[i] = out.next();
        crx[i] = 0xEE;
    }
    SsmClient::enable();
    const uint32_t m0 = Ticker::millis();
    while (Ticker::millis() - m0 < a.aux8 + 100u) {
        if (!SelectWire::read()) {
            L.fell = true;
            break;
        }
    }
    if (select_it && L.fell) {
        SsmClient::select(true);
    }
    // The run has no pad to close it: it ends on its count, or on the
    // budget when no frame comes.
    L.run = SsmClient::exchange(std::span<const uint8_t>(cout_, n), std::span<uint8_t>(crx, n),
                                budget(a.ms));
    SsmClient::select(false);
    (void)SsmClient::disable();
    SsmClient::release();
    SelectWire::release();
    L.reported = peer_report(L.r);
    return L;
}

// ---- the host's simplex shapes, through the resource ------------------------

/// The host's configuration at `rate` (prime() writes it whole), then the
/// direction the leg wants: SpiHost's Request is full duplex by
/// construction, so a simplex window is the resource's.
void host_direction(SpiDirection d, SpiClock rate) {
    Host::prime(SpiMode::mode0, rate);
    (void)S1::disable();
    (void)S1::direction(d);
}

/// BIDIMODE + BIDIOE: `n` frames out on MOSI alone, polled on TXE, then
/// 35.5.9's disable (FTLVL, then BSY) so the last frame is through
/// before the select rises. Returns the frames written; `rx_left` the
/// RXFIFO level before the disable drains it.
uint16_t host_bidi_out(const uint8_t* tx, uint16_t n, SpiClock rate, uint8_t& rx_left,
                       bool& drained) {
    host_direction(SpiDirection::bidi_transmit, rate);
    S1::enable();
    CsPin::clear();
    link_hold();
    uint16_t w = 0;
    for (; w < n; ++w) {
        uint32_t spins = 100'000u;
        while (!S1::txe() && spins-- != 0u) {
        }
        if (spins == UINT32_MAX) {
            break;
        }
        S1::data_byte(tx[w]);
    }
    uint32_t spins = 100'000u;
    while (S1::tx_level() != 0u && spins-- != 0u) {
    }
    rx_left = S1::rx_level();
    drained = S1::disable();
    link_hold();
    CsPin::set();
    return w;
}

/// RXONLY: SPE starts a clock that only SPE stops (35.5.9's receive-only
/// procedure). `n` frames read on RXNE, then SPE cleared at once - inside
/// frame n + 1, between its first sampling edge and its last, the window
/// the chapter names: a frame at PCLK/16 is 128 core cycles and the read
/// plus the store are a few tens. Returns the frames read.
uint16_t host_rx_only(uint8_t* rx, uint16_t n, SpiClock rate, bool& stopped) {
    host_direction(SpiDirection::receive_only, rate);
    CsPin::clear();
    link_hold();
    S1::enable();
    uint16_t got = 0;
    for (; got < n; ++got) {
        uint32_t spins = 100'000u;
        while (!S1::rxne() && spins-- != 0u) {
        }
        if (spins == UINT32_MAX) {
            break;
        }
        rx[got] = S1::data_byte();
    }
    stopped = S1::disable_receive_only();
    link_hold();
    CsPin::set();
    return got;
}

// ---- the client's simplex shapes ---------------------------------------------

/// SPI1 as a client on its own four pads: letter r's PeerClient.
using Client1 = PeerClient;

struct ClientLeg {
    bool accepted = false;
    Client1::Run run{0, Client1::RunEnd::stalled};
    bool reported = false;
    spilink::Report r{};
};

/// One burst against SPI1 as a client with `dir`: bidi_transmit answers
/// on MISO alone (answer(), nothing to read), receive_only listens with
/// MISO dark (receive()).
ClientLeg client_leg(SpiDirection dir, uint8_t seed_a, uint8_t seed_b) {
    ClientLeg L{};
    const spilink::Params a = burst_params(seed_a, seed_b);
    L.accepted = peer_act(Op::host_burst, a);
    if (!L.accepted) {
        return L;
    }
    peer_stop();
    Nvic::disable(S1::irq());
    const bool talks = dir == SpiDirection::bidi_transmit;
    (void)Client1::init(clock, {.mode = SpiMode::mode0,
                                .bits = SpiDataSize::bits8,
                                .lsb_first = false,
                                .nss = SpiNss::hardware_input,
                                .direction = dir,
                                .drive_output = talks});
    const uint16_t n = a.count;
    spilink::Stream out(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < n; ++i) {
        cout_[i] = out.next();
        crx[i] = 0xEE;
    }
    Client1::enable();
    if (talks) {
        L.run = Client1::answer(std::span<const uint8_t>(cout_, n), budget(a.ms));
    } else {
        L.run = Client1::receive(std::span<uint8_t>(crx, n), budget(a.ms));
    }
    (void)Client1::disable();
    Client1::release();
    L.reported = peer_report(L.r);
    return L;
}

// ---- prime(), by its absence ----------------------------------------------------

/// The protocol window of link_xfer() with ONE difference: the select falls
/// BEFORE the request's mode is applied, so start() flips CPOL inside the
/// open window - the order prime() exists to prevent.
bool link_xfer_unprimed(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate, SpiMode mode) {
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
    r.engine = SpiEngine::polled;
    const bool done = Host::start(r);
    link_hold();
    CsPin::set();
    return done;
}

/// One exchange at mode `next` (the client exact from the first frame,
/// regime_buffer_wait), the host primed at mode `prev` with the select
/// high first, then the window primed or not.
bool mode_exchange(uint8_t prev, uint8_t next, bool primed, Verify& v, spilink::Report& r) {
    Exchange e{};
    e.cfg.mode = next;
    e.host_mode = static_cast<SpiMode>(next);
    e.seed_a = 0x6E;
    e.seed_b = 0xA1;
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    if (!peer_act(Op::exchange, a)) {
        return false;
    }
    spilink::Stream out(e.pattern, e.seed_a);
    for (uint8_t i = 0; i < max_exchange; ++i) {
        xtx[i] = i < e.count ? out.next() : 0u;
        xrx[i] = 0xEE;
    }
    settle();
    Host::prime(static_cast<SpiMode>(prev), link_clock);
    settle();
    if (primed) {
        (void)link_xfer(xtx, xrx, e.count, link_clock, e.host_mode);
    } else {
        (void)link_xfer_unprimed(xtx, xrx, e.count, link_clock, e.host_mode);
    }
    (void)link_command_mode();
    const bool rep = peer_report(r);
    v = verify_rx(e);
    return rep;
}

}   // namespace simplex

void ty_simplex() {
    using namespace simplex;
    if (!need_peer()) {
        return;
    }

    // ---- 1. the client under SSM ------------------------------------------------
    print(serial, "  1. THE CLIENT WITH NO SELECT PAD (SSM): the peer's 12-frame host_burst "
                  "at its /32",
          crlf);
    const SsmLeg deaf = ssm_leg(false, 0x35, 0x9C);
    print(serial, "    SSI left HIGH: the wire's select fell=", deaf.fell, ", the run read ",
          deaf.run.count, " frames and ended ", end_name(deaf.run.end),
          "; the peer as host read ", deaf.r.count, " frames, mism=", deaf.r.mism, crlf);
    if (deaf.r.mism != 0u) {
        print(serial, "    what the peer read off MISO from the deselected client: first "
                      "mismatch at ", deaf.r.idx, " got ", hex(deaf.r.got), " exp ",
              hex(deaf.r.exp), crlf);
    }
    bench.verdict("a client under SSM with SSI HIGH is deaf to a clock the wire carries: "
                  "the peer clocked its burst and not one frame arrived (the run without a "
                  "pad ends on its budget)",
                  deaf.accepted && deaf.fell && deaf.run.count == 0u &&
                      deaf.run.end == SsmClient::RunEnd::stalled && deaf.reported &&
                      deaf.r.count == 12u);
    const SsmLeg sel = ssm_leg(true, 0x35, 0x9C);
    const uint16_t sel_mism = mismatches(crx, sel.run.count, spilink::pattern_prbs, 0x35);
    print(serial, "    SSI driven low after the wire's select fell: the run read ", sel.run.count,
          " frames, mism=", sel_mism, ", ended ", end_name(sel.run.end),
          "; the peer as host read ", sel.r.count, " frames, mism=", sel.r.mism, " (first idx ",
          sel.r.idx, " got ", hex(sel.r.got), " exp ", hex(sel.r.exp), ")", crlf);
    bench.verdict("selected by SOFTWARE (SSI low), the client with no select pad receives "
                  "the foreign host's burst byte-exact and its run ends COMPLETE on its count",
                  sel.accepted && sel.fell && sel.run.count == 12u && sel_mism == 0u &&
                      sel.run.end == SsmClient::RunEnd::complete);
    bench.verdict("...and the foreign host read its answer stream byte-exact from the first "
                  "frame - the TXFIFO filled before SSI fell",
                  sel.reported && sel.r.count == 12u && sel.r.mism == 0u);
    (void)link_command_mode();

    // ---- 2. the host's single line out -----------------------------------------------
    print(serial, "  2. THE HOST'S SINGLE LINE OUT (BIDIMODE + BIDIOE), the peer listening "
                  "with its answer line dark (spare_receive_only)",
          crlf);
    static constexpr SpiClock bidi_rates[] = {SpiClock::div64, SpiClock::div16};
    bool bidi_ok = true;
    bool bidi_quiet = true;
    for (const SpiClock c : bidi_rates) {
        constexpr uint16_t n = 32;
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0,
                             .regime = spilink::regime_buffer_wait};
        a.count = n;
        a.ms = 400;
        a.seed_a = 0x47;
        a.seed_b = 0x11;
        a.pattern = spilink::pattern_prbs;
        a.spare = spilink::spare_receive_only;
        const bool acc = peer_act(Op::exchange, a);
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < n; ++i) {
            xtx[i] = out.next();
        }
        settle();
        uint8_t rx_left = 0xFF;
        bool drained = false;
        const uint16_t w = acc ? host_bidi_out(xtx, n, c, rx_left, drained) : 0u;
        (void)link_command_mode();
        spilink::Report r{};
        const bool rep = acc && peer_report(r);
        const bool ok = rep && w == n && drained && r.count == n && r.mism == 0u &&
                        (r.flags & spilink::report_extended) != 0u;
        print(serial, "    PCLK/", spi_division(c), " = ", Host::sck_hz(c) / 1000u, " kHz: ", w,
              " frames written, the disable ", drained ? "clean" : "TIMED OUT",
              ", RXFIFO level before it ", rx_left, "; the peer heard ", r.count, " mism=",
              r.mism, " flags=", hex(r.flags), ok ? "" : "  <-- NOT exact", crlf);
        bidi_ok = bidi_ok && ok;
        bidi_quiet = bidi_quiet && rx_left == 0u;
    }
    bench.verdict("the host's SINGLE LINE OUT carries 32 frames to a foreign client "
                  "byte-exact at PCLK/64 and PCLK/16, and 35.5.9's disable (FTLVL, then BSY) "
                  "lets the last one through before the select rises",
                  bidi_ok);
    bench.verdict("...and with BIDIOE set the receiver takes nothing: the RXFIFO is empty "
                  "after every window",
                  bidi_quiet);

    // ---- 3. the host's receive-only clock ------------------------------------------------
    print(serial, "  3. THE HOST'S RECEIVE-ONLY CLOCK (RXONLY), the peer answering alone "
                  "(spare_answer_only)",
          crlf);
    static constexpr SpiClock rxo_rates[] = {SpiClock::div64, SpiClock::div16};
    bool rxo_ok = true;
    for (const SpiClock c : rxo_rates) {
        constexpr uint16_t n = 16;
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0,
                             .regime = spilink::regime_buffer_wait};
        a.count = 24;   // answers past the host's 16: the frame it clocks while it stops
        a.ms = 400;
        a.seed_a = 0x00;
        a.seed_b = 0xC3;
        a.pattern = spilink::pattern_prbs;
        a.spare = spilink::spare_answer_only;
        const bool acc = peer_act(Op::exchange, a);
        for (uint16_t i = 0; i < max_exchange; ++i) {
            xrx[i] = 0xEE;
        }
        settle();
        bool stopped = false;
        const uint16_t got = acc ? host_rx_only(xrx, n, c, stopped) : 0u;
        (void)link_command_mode();
        spilink::Report r{};
        const bool rep = acc && peer_report(r);
        const uint16_t m = mismatches(xrx, got, a.pattern, a.seed_b);
        const bool ok = rep && got == n && m == 0u && stopped;
        print(serial, "    PCLK/", spi_division(c), " = ", Host::sck_hz(c) / 1000u, " kHz: read ",
              got, " frames, mism=", m, ", SPE cleared and BSY ", stopped ? "fell" : "STOOD",
              "; the peer loaded ", r.count, " answers", ok ? "" : "  <-- NOT exact", crlf);
        if (!ok) {
            print_bytes("      read:", xrx, got);
        }
        rxo_ok = rxo_ok && ok;
    }
    bench.verdict("the host's RECEIVE-ONLY clock reads a foreign client's answer stream "
                  "byte-exact at PCLK/64 and PCLK/16, and clearing SPE inside the next frame "
                  "stops it (35.5.9's receive-only procedure)",
                  rxo_ok);

    // ---- 4. the client's single line out, 5. its receive-only run ----------------------
    print(serial, "  4. THE CLIENT'S SINGLE LINE OUT (BIDIMODE + BIDIOE on MISO) under the "
                  "peer's host_burst",
          crlf);
    const ClientLeg bo = client_leg(SpiDirection::bidi_transmit, 0x29, 0x6B);
    print(serial, "    answer() wrote ", bo.run.count, " answers, ended ",
          end_name(bo.run.end),
          "; the peer as host read ", bo.r.count, " mism=", bo.r.mism, " (first idx ", bo.r.idx,
          " got ", hex(bo.r.got), " exp ", hex(bo.r.exp), ")", crlf);
    bench.verdict("a CLIENT on its single line (BIDIMODE, BIDIOE set) answers a foreign "
                  "host on MISO byte-exact from the first frame, and its run ends when the "
                  "select rises",
                  bo.accepted && bo.run.count == 12u && bo.run.end == Client1::RunEnd::closed &&
                      bo.reported && bo.r.count == 12u && bo.r.mism == 0u);
    print(serial, "  5. THE CLIENT'S RECEIVE-ONLY RUN (RXONLY, MISO dark) under the peer's "
                  "host_burst",
          crlf);
    const ClientLeg ro = client_leg(SpiDirection::receive_only, 0x5A, 0x00);
    const uint16_t ro_mism = mismatches(crx, ro.run.count, spilink::pattern_prbs, 0x5A);
    print(serial, "    receive() read ", ro.run.count, " frames, mism=", ro_mism, ", ended ",
          end_name(ro.run.end),
          "; the peer as host clocked ", ro.r.count, crlf);
    bench.verdict("a CLIENT in RXONLY receives a foreign host's burst byte-exact with its "
                  "answer line never driven",
                  ro.accepted && ro.run.count == 12u && ro_mism == 0u &&
                      ro.run.end == Client1::RunEnd::complete && ro.reported &&
                      ro.r.count == 12u);
    (void)link_command_mode();

    // ---- 6. prime(), by its absence ------------------------------------------------------
    // Every ordered pair of modes: the host left at the first by a prime()
    // with the select HIGH, then a window at the second with the select
    // falling BEFORE start() applies it. A pair that keeps CPOL moves no
    // edge at all. A pair that flips it puts one edge inside the window,
    // and the F446's client divides those by the NEW mode's phase: landing
    // in CPHA = 0 (modes 0 and 2) the flip is counted as one more bit - the
    // client's frames arrive shifted one place right and the host reads
    // the client one place early, both from the first frame - and landing
    // in CPHA = 1 (modes 1 and 3) it is not counted at all. A one-bit slip
    // is read off
    // the first mismatching frame at each end: the low seven bits of what
    // the client got are the expected frame shifted right, and the top
    // seven of what the host got are the expected one shifted left (the
    // bit that came in at the far end is the wire's, and not judged).
    print(serial, "  6. prime() BY ITS ABSENCE: every ordered pair of modes, the select "
                  "falling before the second is applied",
          crlf);
    uint8_t slips_as_said = 0, exact_as_said = 0;
    for (uint8_t prev = 0; prev < 4u; ++prev) {
        for (uint8_t next = 0; next < 4u; ++next) {
            if (prev == next) {
                continue;
            }
            Verify v{};
            spilink::Report r{};
            const bool ran = mode_exchange(prev, next, false, v, r);
            const bool flips = ((prev ^ next) & 2u) != 0u;
            const bool lands_cpha0 = flips && (next & 1u) == 0u;
            const bool slip = ran && r.count == 8u && r.mism != 0u && r.idx == 0u &&
                              v.mism != 0u && v.idx == 0u &&
                              (r.got & 0x7Fu) == (r.exp >> 1) &&
                              (v.got >> 1) == (v.exp & 0x7Fu);
            const bool exact = ran && r.count == 8u && r.mism == 0u && v.mism == 0u;
            if (lands_cpha0 ? slip : exact) {
                if (lands_cpha0) {
                    ++slips_as_said;
                } else {
                    ++exact_as_said;
                }
            } else {
                print(serial, "    mode ", prev, " -> ", next, ": host mism=", v.mism, " (idx ",
                      v.idx, " got ", hex(v.got), " exp ", hex(v.exp), "), client count=",
                      r.count, " mism=", r.mism, " (idx ", r.idx, " got ", hex(r.got), " exp ",
                      hex(r.exp), ")  <-- not as the phase says", crlf);
            }
        }
    }
    print(serial, "    unprimed: ", slips_as_said, " of the 4 CPOL flips into CPHA = 0 "
                  "slipped one bit at both ends, ", exact_as_said, " of the 8 other pairs (4 "
                  "flips into CPHA = 1, 4 with no flip) stayed exact",
          crlf);
    bench.verdict("UNPRIMED, a CPOL flip with the select already low is ONE MORE BIT to a "
                  "client landing in CPHA = 0 (modes 0 and 2): every such pair slips exactly "
                  "one place at both ends from the first frame",
                  slips_as_said == 4u);
    bench.verdict("...while the same flip into CPHA = 1 (modes 1 and 3) is not counted, and "
                  "a pair that keeps CPOL moves no edge: those eight stay exact",
                  exact_as_said == 8u);
    uint8_t primed_ok = 0;
    static constexpr uint8_t primed_pairs[][2] = {{0, 2}, {3, 0}, {1, 2}, {2, 0}};
    for (const auto& pr : primed_pairs) {
        Verify v{};
        spilink::Report r{};
        if (mode_exchange(pr[0], pr[1], true, v, r) && r.count == 8u && r.mism == 0u &&
            v.mism == 0u) {
            ++primed_ok;
        } else {
            print(serial, "    primed ", pr[0], " -> ", pr[1], ": host mism=", v.mism,
                  ", client count=", r.count, " mism=", r.mism, crlf);
        }
    }
    bench.verdict("PRIMED (the mode applied with the select high), the four slipping pairs "
                  "are byte-exact both ways",
                  primed_ok == 4u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the simplex shapes", command(Op::ping));
}


// =============================================================================
// K - THE NEXT REQUEST AFTER A STALL OR A TRANSFER ERROR
// =============================================================================
//
// The host puts its own block right, so nothing here calls recover() between a
// failure and the request judged after it, and nothing re-initializes the host:
// the peer's exchange is commanded FIRST, through the engineless host on the
// same instance, then the engined host comes up and is not touched again until
// the window has been clocked. The peer's DMA serve counts frames and takes no
// notice of a select that falls and rises with no clock under it, so a stalled
// request may frame itself with the real select and the window survives it.
//
// A STALL IS STAGED WITH SPE DOWN UNDER THE HOST (S1::disable() on the resource):
// the request's compare finds nothing to change, so the enable stays down, the
// frames written stand in the TXFIFO - the write-ahead's own worst case - and the
// flag the loop waits for never rises. THE FRAMES STANDING ARE THE TEST OF THE
// RESET: one left in the TXFIFO would go out in front of the next request and
// the window would be frames out of place at both ends. MODE 3 WITH A PULL-DOWN
// ON SCK IS THE TEST OF THE ORDER, as a bus with a pull on that wire would be:
// this block drives SCK only while SPE is set (measured here), so the pad falls
// when SPE does and the reset's enable raises it, a rising edge; were the select
// still low the peer's client would count it. SCK's edges are also counted on the
// pad itself, each by where the select's latch stood: none with it low and at
// least one with it high is the order read directly.
//
// A TRANSFER ERROR IS A BUFFER IN RESERVED SPACE (kl::way_bad_address()). A bad
// transmit buffer stops the channel on its first beat: nothing reaches DR, no
// clock, so the real select can frame it. A bad receive buffer stops the
// receive channel on the first frame back, with the transmit channel already
// ahead of it: frames still clock, others stand in the FIFOs, so its request
// carries no select - the peer's window stays deaf - and the one after it is the
// first thing the peer hears. A frame of the faulted block read in front of the
// window would put this end's read-back out of place.
//
// The select's level is read on its output latch, the store the host made. The
// walls are printed against the budget, not judged: a handler can only lengthen
// them, and the figure is read beside the arithmetic in docs/stm32g0/spi.md. The
// vector's cost is the ticker's cycles, around DmaHost::dma_isr().
//
// THE BOUNDS. The peer holds its window for 400 ms from the command: in front of
// the window stand at most two stalls, 58 ms each at the longest wall measured, or
// one of 64 ms, and a millisecond of settling. A fault's completion waits 100 ms
// for a first beat or a first frame back, 8 x 32 PCLK cycles, 4 us; a window's
// waits 200 ms for 32 such frames, 128 us; the arbiter's 100 ticks are not spent
// on a stall, whose wall is inside start() and untimed. The verdicts themselves
// are exact counts, with no tolerance to derive.

namespace way {

using kl::way_frames;
using kl::way_rate;
using kl::way_rx;
using kl::way_tx;

enum class Shape : uint8_t {
    write,     ///< polled, nothing read back: the transmit-only loop
    read,      ///< polled, read back: the receive loop
    command,   ///< on the engines with a command phase: the command in the loop before its block
};

struct Failed {
    bool ended = false;       ///< start() answered true
    bool completed = false;   ///< ended, or the vector answered true within the wait
    uint16_t completions = 0;
    uint8_t status = 0xFFu;
    bool released = false;
    uint32_t wall = 0;        ///< cycles inside start()
};

/// Clear the window, then command the peer. False when the peer refused.
bool arm_peer(spilink::Params& a, uint8_t seed, SpiMode mode) {
    (void)link_command_mode();
    a = spilink::Params{};
    a.cfg = spilink::Cfg{.apply = 1,
                         .mode = static_cast<uint8_t>(mode),
                         .dord = 0,
                         .regime = spilink::regime_buffer_wait};
    a.count = way_frames;
    a.ms = 400;
    a.seed_a = seed;
    a.seed_b = static_cast<uint8_t>(seed ^ 0x6Cu);
    a.pattern = spilink::pattern_prbs;
    spilink::Stream out(a.pattern, a.seed_a);
    for (uint16_t i = 0; i < way_frames; ++i) {
        way_tx[i] = out.next();
        way_rx[i] = 0xEE;
    }
    return peer_act(Op::exchange, a);
}

/// The engined host up at the letter's rate, the select high.
void host_up(SpiMode mode) {
    kl::way_mode = mode;
    software_cs_pads();
    dma_host_live = true;
    (void)DmaHost::init(clock, {.mosi_speed = peer_mosi_speed});
    DmaHost::prime(mode, way_rate);
    settle_ms(1);
}

/// Letter's pull on SCK, which the next init() takes away with the pad's
/// function: what makes the pad's being undriven with SPE clear an edge.
void pull_sck_down() {
    Pin<host_pins.sck.port, host_pins.sck.pin>::pull(PinPull::down);
    settle_ms(1);
}

/// SCK's edges counted over one call: the pad is an input to the EXTI whatever
/// its function, on line 3.
void watch_sck(bool on) {
    if (on) {
        (void)SckWatch::select();
        (void)SckWatch::configure(ExtiSense::both);
        (void)SckWatch::clear();
        sck_edges_selected = 0;
        sck_edges_released = 0;
    }
    (void)SckWatch::arm(on);
    if (on) {
        Nvic::enable(SckWatch::irq());
    } else {
        Nvic::disable(SckWatch::irq());
        (void)SckWatch::clear();
    }
}

/// A request made to stall, the block already unable to clock.
Failed run_stalled(Shape shape, PinRef cs, SpiMode mode) {
    DmaHost::Request q{};
    q.cs = cs;
    q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(way_tx));
    q.len = 9;
    q.clock = way_rate;
    q.mode = mode;
    q.engine = SpiEngine::polled;
    if (shape == Shape::read) {
        q.rx = lend<Lease::reply>(way_rx);
    }
    if (shape == Shape::command) {
        q.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(way_tx));
        q.cmd_len = 1;
        q.rx = lend<Lease::reply>(way_rx);
        q.len = way_frames;
        q.engine = SpiEngine::dma;
    }
    Failed f{};
    const uint32_t t0 = cycles_now();
    f.ended = DmaHost::start(q);
    f.wall = cycles_now() - t0;
    f.completed = f.ended;
    f.status = DmaHost::status();
    f.released = CsPin::read_out();
    return f;
}

/// The completion, however it comes: `ended` when start() already answered, or
/// the vector's answer within `ms`.
bool completed_within(bool ended, uint32_t ms) {
    const uint32_t m0 = Ticker::millis();
    while (!ended && !host_done && Ticker::millis() - m0 < ms) {
    }
    return ended || host_done;
}

/// A block whose buffer gives a bus error, and the vector's answer.
Failed run_faulted(bool bad_tx, PinRef cs, SpiMode mode) {
    DmaHost::Request q{};
    q.cs = cs;
    q.tx = lend<Lease::reply>(bad_tx ? static_cast<const uint8_t*>(kl::way_bad_address())
                                     : static_cast<const uint8_t*>(way_tx));
    q.rx = lend<Lease::reply>(bad_tx ? way_rx : kl::way_bad_address());
    q.len = way_frames;
    q.clock = way_rate;
    q.mode = mode;
    q.engine = SpiEngine::dma;
    Failed f{};
    dma_completions = 0;
    host_done = false;
    time_vector = true;
    const uint32_t t0 = cycles_now();
    f.ended = DmaHost::start(q);
    f.wall = cycles_now() - t0;
    f.completed = completed_within(f.ended, 100u);
    time_vector = false;
    f.completions = dma_completions;
    f.status = DmaHost::status();
    f.released = CsPin::read_out();
    return f;
}

struct Window {
    bool ended = false;       ///< start() answered true
    bool completed = false;
    uint8_t status = 0xFFu;
    bool reported = false;
    spilink::Report r{};
    uint16_t mism = 0;        ///< this end's read-back, frames wrong
};

/// What every window ends in: the host put away, the peer's report, this end's
/// read-back against what the peer owes.
void close_window(const spilink::Params& a, Window& o) {
    DmaHost::release();
    dma_host_live = false;
    (void)link_command_mode();
    o.reported = peer_report(o.r);
    spilink::Stream want(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < way_frames; ++i) {
        if (way_rx[i] != want.next()) {
            ++o.mism;
        }
    }
}

/// The whole window, the first request the host has seen since the failure.
Window next_window(const spilink::Params& a, SpiMode mode, bool polled) {
    Window o{};
    DmaHost::Request q{};
    q.cs = CsPin::ref();
    q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(way_tx));
    q.rx = lend<Lease::reply>(way_rx);
    q.len = way_frames;
    q.cs_setup_us = 30;
    q.clock = way_rate;
    q.mode = mode;
    q.engine = polled ? SpiEngine::polled : SpiEngine::dma;
    host_done = false;
    o.ended = DmaHost::start(q);
    o.completed = completed_within(o.ended, 200u);
    o.status = DmaHost::status();
    close_window(a, o);
    return o;
}

/// Exact at both ends, and run the way it asked: inside start() when polled, on
/// the engines when not.
bool exact(const Window& o, bool polled) {
    return o.completed && o.status == spi_ok && o.ended == polled && o.reported && o.r.count == way_frames &&
           o.r.mism == 0u && o.mism == 0u;
}

void print_window(const char* what, const Window& o) {
    print(serial, "      then ", what, ": status ", o.status, ", inside start() ", o.ended ? 1u : 0u, ", host mism ",
          o.mism, ", the peer read ", o.r.count, " mism ", o.r.mism, o.completed ? "" : " NEVER COMPLETED", crlf);
}

void print_failed(const char* what, const Failed& f) {
    print(serial, "    ", what, ": start() ", f.ended ? "true" : "false", ", status ", f.status, " (stalled ",
          spi_stalled, ", fault ", spi_dma_fault, "), ", f.wall, " cycles (", f.wall / (SysClock::hz() / 1'000'000u),
          " us) in start(), select ", f.released ? "high" : "LOW", crlf);
}

/// One request queued behind a failing one, through the arbiter. The replies and
/// the good one's window, which the peer has held since before the failure.
struct Queued {
    uint8_t replies = 0;
    uint8_t first = 0;
    uint8_t second = 0;
    Window o{};
};
Queued run_queued(bool fault, bool polled, uint8_t seed) {
    Queued q{};
    spilink::Params a{};
    const bool armed = arm_peer(a, seed, SpiMode::mode0);
    kl::BusKernel::init_all();
    host_up(SpiMode::mode0);
    bus_ao_live = true;
    if (!fault) {
        (void)S1::disable();
    }
    post<kl::Driver>(kl::Pair{.fault = fault, .polled = polled});
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 800u && kl::Driver::replies < 2u) {
        kl::pump();
    }
    bus_ao_live = false;
    q.replies = kl::Driver::replies;
    q.first = kl::Driver::order[0];
    q.second = kl::Driver::order[1];
    q.o.completed = armed;
    q.o.ended = polled;
    q.o.status = q.second;
    close_window(a, q.o);
    return q;
}

bool queued_clean(const Queued& q, uint8_t first, bool polled) {
    return q.replies == 2u && q.first == first && exact(q.o, polled);
}

void print_queued(const char* what, const Queued& q) {
    print(serial, "    ", what, ": ", q.replies, " replies, status ", q.first, " then ", q.second, ", host mism ", q.o.mism,
          ", the peer read ", q.o.r.count, " mism ", q.o.r.mism, crlf);
}

}  // namespace way

void tK_way_back() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  a stall: 32 frames at PCLK/32, the enable taken down under the host first", crlf);

    // ---- a stall on each path, the next request exact --------------------------
    spilink::Params a{};
    bool armed = way::arm_peer(a, 0x21, SpiMode::mode3);
    way::host_up(SpiMode::mode3);
    way::pull_sck_down();
    (void)S1::disable();
    way::watch_sck(true);
    const way::Failed f_write = way::run_stalled(way::Shape::write, CsPin::ref(), SpiMode::mode3);
    way::watch_sck(false);
    const uint16_t under_write = sck_edges_selected;
    const uint16_t after_write = sck_edges_released;
    const way::Window n_write = way::next_window(a, SpiMode::mode3, true);
    way::print_failed("a polled write, mode 3", f_write);
    print(serial, "      SCK's edges in start(): ", under_write, " with the select low, ", after_write, " with it high", crlf);
    way::print_window("a polled window", n_write);
    bench.verdict("A POLLED WRITE THAT STALLS, MODE 3, THEN A POLLED REQUEST WITH NO recover(): spi_stalled, the select "
                  "released, and the window exact at both ends - the frames that stood in the TXFIFO gone, SCK moved by "
                  "the reset behind a select already up",
                  armed && f_write.ended && f_write.status == spi_stalled && f_write.released && under_write == 0u && after_write != 0u &&
                      way::exact(n_write, true));

    armed = way::arm_peer(a, 0x34, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    (void)S1::disable();
    const way::Failed f_read = way::run_stalled(way::Shape::read, CsPin::ref(), SpiMode::mode0);
    const way::Window n_read = way::next_window(a, SpiMode::mode0, false);
    way::print_failed("a polled read, mode 0", f_read);
    way::print_window("an asynchronous window", n_read);
    bench.verdict("A POLLED READ THAT STALLS, THEN AN ASYNCHRONOUS REQUEST WITH NO recover(): spi_stalled, the select "
                  "released, and the window on the engines exact at both ends",
                  armed && f_read.ended && f_read.status == spi_stalled && f_read.released &&
                      way::exact(n_read, false));

    armed = way::arm_peer(a, 0x47, SpiMode::mode3);
    way::host_up(SpiMode::mode3);
    way::pull_sck_down();
    (void)S1::disable();
    way::watch_sck(true);
    const way::Failed f_cmd = way::run_stalled(way::Shape::command, CsPin::ref(), SpiMode::mode3);
    way::watch_sck(false);
    const uint16_t under_cmd = sck_edges_selected;
    const uint16_t after_cmd = sck_edges_released;
    const way::Window n_cmd = way::next_window(a, SpiMode::mode3, true);
    way::print_failed("an asynchronous request's polled command phase, mode 3", f_cmd);
    print(serial, "      SCK's edges in start(): ", under_cmd, " with the select low, ", after_cmd, " with it high", crlf);
    way::print_window("a polled window", n_cmd);
    bench.verdict("AN ASYNCHRONOUS REQUEST WHOSE POLLED COMMAND PHASE STALLS ends inside start(), spi_stalled, the select "
                  "released, no block started; the next request exact at both ends",
                  armed && f_cmd.ended && f_cmd.status == spi_stalled && f_cmd.released && under_cmd == 0u && after_cmd != 0u &&
                      way::exact(n_cmd, true));

    // ---- queued behind a stall, through the arbiter ----------------------------
    const way::Queued q_stall_p = way::run_queued(false, true, 0x5A);
    way::print_queued("queued behind a stall, polled", q_stall_p);
    const way::Queued q_stall_a = way::run_queued(false, false, 0x6D);
    way::print_queued("queued behind a stall, asynchronous", q_stall_a);
    bench.verdict("TWO REQUESTS FROM ONE DISPATCH, THE FIRST STALLS: the replies in order, spi_stalled then spi_ok, the "
                  "second's window exact at both ends, polled and on the engines, recover() never called",
                  way::queued_clean(q_stall_p, spi_stalled, true) && way::queued_clean(q_stall_a, spi_stalled, false));

    // ---- a transfer error, the next request exact ------------------------------
    print(serial, "  a transfer error: 32 frames at PCLK/32, a buffer at 0x30000000", crlf);
    armed = way::arm_peer(a, 0x71, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    const way::Failed e_tx = way::run_faulted(true, CsPin::ref(), SpiMode::mode0);
    const uint32_t tx_vector = vector_cycles;
    const way::Window n_tx = way::next_window(a, SpiMode::mode0, true);
    way::print_failed("a transmit buffer in reserved space", e_tx);
    print(serial, "      the vector's cost ", tx_vector, " cycles (the channel's error, one completion)", crlf);
    way::print_window("a polled window", n_tx);
    bench.verdict("A BUS ERROR ON THE TRANSMIT CHANNEL: dma_isr() answers true once, spi_dma_fault, the select released, no "
                  "recover(); the next polled request exact at both ends",
                  armed && !e_tx.ended && e_tx.completed && e_tx.completions == 1u &&
                      e_tx.status == spi_dma_fault && e_tx.released && way::exact(n_tx, true));

    armed = way::arm_peer(a, 0x83, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    const way::Failed e_rx = way::run_faulted(false, {}, SpiMode::mode0);
    const uint32_t rx_vector = vector_cycles;
    settle_ms(2);
    const uint8_t rx_stood = static_cast<uint8_t>(S1::rx_level());
    const uint8_t tx_stood = static_cast<uint8_t>(S1::tx_level());
    const way::Window n_rx = way::next_window(a, SpiMode::mode0, false);
    way::print_failed("a receive buffer in reserved space, no select", e_rx);
    print(serial, "      the vector's cost ", rx_vector, " cycles; RXFIFO level ", rx_stood, ", TXFIFO level ", tx_stood, " behind it",
          crlf);
    way::print_window("an asynchronous window", n_rx);
    bench.verdict("A BUS ERROR ON THE RECEIVE CHANNEL: dma_isr() answers true once, spi_dma_fault, no recover(); the next "
                  "asynchronous request exact at both ends, no frame of the faulted block read back in front of it",
                  armed && !e_rx.ended && e_rx.completed && e_rx.completions == 1u &&
                      e_rx.status == spi_dma_fault && way::exact(n_rx, false));

    // The same error with the select the request names; no window is armed, so
    // the peer's command listener hears a frame or two of nothing it knows.
    (void)link_command_mode();
    way::host_up(SpiMode::mode0);
    const way::Failed e_sel = way::run_faulted(false, CsPin::ref(), SpiMode::mode0);
    settle_ms(1);
    way::print_failed("a receive buffer in reserved space, the select named", e_sel);
    DmaHost::release();
    dma_host_live = false;
    bench.verdict("...and the vector itself releases the select the request named",
                  !e_sel.ended && e_sel.completed && e_sel.status == spi_dma_fault && e_sel.released);

    // ---- queued behind a transfer error, through the arbiter -------------------
    const way::Queued q_fault_p = way::run_queued(true, true, 0x95);
    way::print_queued("queued behind a transfer error, polled", q_fault_p);
    const way::Queued q_fault_a = way::run_queued(true, false, 0xA7);
    way::print_queued("queued behind a transfer error, asynchronous", q_fault_a);
    bench.verdict("TWO REQUESTS FROM ONE DISPATCH, THE FIRST A BUS ERROR: the replies in order, spi_dma_fault then spi_ok, "
                  "the second's window exact at both ends, polled and on the engines, recover() never called",
                  way::queued_clean(q_fault_p, spi_dma_fault, true) && way::queued_clean(q_fault_a, spi_dma_fault, false));

    // ---- what a reset cannot do: the clock gate closed under the host ----------
    armed = way::arm_peer(a, 0xB9, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    S1::bus_clock(false);
    const way::Failed g_first = way::run_stalled(way::Shape::write, {}, SpiMode::mode0);
    const way::Failed g_second = way::run_stalled(way::Shape::write, {}, SpiMode::mode0);
    const bool answer_closed = DmaHost::recover();
    S1::bus_clock(true);
    const bool answer_open = DmaHost::recover();
    const way::Window n_gate = way::next_window(a, SpiMode::mode0, true);
    way::print_failed("the gate closed, a polled write", g_first);
    way::print_failed("the gate still closed, the next", g_second);
    print(serial, "      recover() with the gate closed ", answer_closed ? "true" : "false", ", opened ",
          answer_open ? "true" : "false", crlf);
    way::print_window("a polled window", n_gate);
    bench.verdict("A RESET DOES NOT OPEN A CLOSED GATE: the request after a stall stalls too, on its own budget, and "
                  "recover() answers false until the gate is open",
                  g_first.status == spi_stalled && g_second.status == spi_stalled && !answer_closed && answer_open);
    bench.verdict("...and with the gate open and recover() called, the next request exact at both ends",
                  armed && way::exact(n_gate, true));

    armed = way::arm_peer(a, 0xCB, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    S1::bus_clock(false);
    const way::Failed h_first = way::run_stalled(way::Shape::write, {}, SpiMode::mode0);
    S1::bus_clock(true);
    const uint32_t cr1_open = S1::regs().CR1;
    const way::Failed h_second = way::run_stalled(way::Shape::write, {}, SpiMode::mode0);
    const way::Window n_open = way::next_window(a, SpiMode::mode0, true);
    way::print_failed("the gate closed, a polled write", h_first);
    print(serial, "      the gate opened: CR1 ", hex(cr1_open), crlf);
    way::print_failed("the gate open, no recover(), a polled write", h_second);
    way::print_window("a polled window", n_open);
    bench.verdict("WITH THE GATE REOPENED AND NO recover(), the block found at its reset values: one more request stalls, "
                  "and that stall's reset is the way back, the next request exact at both ends",
                  armed && h_first.status == spi_stalled && (cr1_open & SPI_CR1_SPE) == 0u &&
                      h_second.status == spi_stalled && way::exact(n_open, true));

    // recover() over frames standing in the TXFIFO, SPE down: its wall.
    way::host_up(SpiMode::mode0);
    (void)S1::disable();
    S1::data_byte(0xFFu);
    S1::data_byte(0xFFu);
    S1::data_byte(0xFFu);
    const uint8_t standing = static_cast<uint8_t>(S1::tx_level());
    const uint32_t r0 = cycles_now();
    (void)DmaHost::recover();
    const uint32_t recover_wall = cycles_now() - r0;
    print(serial, "    recover() over TXFIFO level ", standing, ", the enable down: ", recover_wall, " cycles (",
          recover_wall / (SysClock::hz() / 1'000'000u), " us)", crlf);
    DmaHost::release();
    dma_host_live = false;

    (void)link_command_mode();
    bench.verdict("the command channel survives the letter", command(Op::ping));
}

void banner() {
    print(serial, crlf, "test_stm32_spi - SPI/I2S on a G0 Nucleo", crlf);
    if (self_link) {
        print(serial, "  the SELF-LINK is on the desk: SPI1 host PB3/PB4/PB5 + PA15 "
                      " <->  SPI2 client PB10/PC2/PD4/PB12; letters b..l are live",
              crlf);
    } else if constexpr (!self_link_possible) {
        print(serial, "  NO SELF-LINK is possible on this package: SPI2's four pads "
                      "are bonded to no pin of it, so letters b..l are compiled out "
                      "and claim nothing", crlf);
    } else {
        print(serial, "  NO SELF-LINK on the desk (probed): letters b..l skip "
                      "themselves and claim nothing", crlf);
    }
    print(serial, "  SPI1 PB3/PB4/PB5 + PA15 also reach a PEER BOARD running "
                  "`spi_peer` (its ident says which port) - letters n..r", crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

// ---------------------------------------------------------------------------
// The vectors (app glue: this is the one vendor thing an app may contain)
// ---------------------------------------------------------------------------

extern "C" void SysTick_Handler() {
    brio::Ticker::tick();
    const uint32_t turns = tick_stretch;
    if (turns != 0u) {
        for (uint32_t i = 0; i < turns; ++i) {
            asm volatile("" ::: "memory");
        }
        ticks_stretched = ticks_stretched + 1u;
    }
}
extern "C" void BRIO_SUITE_CONSOLE_HANDLER() { (void)Serial::isr(); }

// =============================================================================
// S - THE ENGINE A REQUEST NAMES, against the peer
// =============================================================================
//
// A request naming the engines runs on them at any length. One frame, read and
// written, at PCLK/4 and PCLK/8: the loop would cost the core less there
// (docs/stm32g0/spi.md has the arithmetic) and nothing asks. What they cannot
// carry is refused before the block, the select or the D/C is touched, every
// witness read: the select's falling edges on its pad, D/C's latch, CR1 and
// CR2, SCK's edges. A command before data runs in the loop, one block and one
// completion, with D/C rising on a wire at rest between the two phases - the
// handler of D/C's own edge finds BSY low, the transmit FIFO empty and SCK at
// its idle level, and it runs before the data block is launched, the rise
// being main context's - and a command with no data is the block. (SCK's edges
// are not counted one by one here: at 64 MHz the handler of an edge outlasts
// half a bit at every rung.) A polled request of any
// length stays the loop: letters o, p, u and K run them.

namespace named {

/// PB6, a pad on no recorded wire (the bench's survey): D/C for this letter,
/// and the select of the one refusal whose select does fall.
using DcPad = Pin<'B', 6>;
using DcWatch = ExtInt<DcPad>;
/// D/C's rise as its handler saw the wire: 0 none yet, 1 at rest, 2 not.
volatile uint8_t dc_rise = 0;

constexpr uint16_t cap = 40;
alignas(2) uint8_t tx[cap];
alignas(2) uint8_t rx[cap];

struct Window {
    bool accepted = false;
    bool clocked = false;
    bool inside = false;
    uint16_t completions = 0;
    uint16_t blocks = 0;
    bool reported = false;
    spilink::Report r{};
    uint16_t mism = 0;
    uint8_t dc_rise = 0;   ///< 0 D/C never rose, 1 on a wire at rest, 2 not
    bool dc_up = false;
};

DmaHost::Config link_config() { return {.mosi_speed = peer_mosi_speed}; }

/// One exchange of `cmd_len + len` 8-bit frames commanded of the peer and
/// clocked on the engined host at `rate`, the first `cmd_len` the command
/// phase: the peer hears every frame, this end reads the data phase back when
/// `read`. D/C a real pad parked high, its rise judged by its own handler.
Window run(uint8_t cmd_len, uint16_t len, SpiClock rate, uint8_t seed, bool read = true) {
    Window o{};
    const uint16_t n = static_cast<uint16_t>(cmd_len + len);
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = n;
    a.ms = 200;
    a.seed_a = seed;
    a.seed_b = static_cast<uint8_t>(seed ^ 0x6Cu);
    a.pattern = spilink::pattern_prbs;
    o.accepted = peer_act(Op::exchange, a);
    if (!o.accepted) {
        return o;
    }
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < cap; ++i) {
            tx[i] = i < n ? out.next() : 0u;
            rx[i] = 0xEE;
        }
    }
    settle();
    Dma<1>::bus_clock(true);
    CsPin::output(true);
    DcPad::output(true);
    (void)DmaHost::init(clock, link_config());
    dma_host_live = true;
    DmaHost::prime(SpiMode::mode0, rate);
    CsPin::clear();
    link_hold();
    dc_rise = 0;
    (void)DcWatch::select();
    (void)DcWatch::configure(ExtiSense::rising);
    (void)DcWatch::clear();
    (void)DcWatch::arm(true);
    Nvic::enable(DcWatch::irq());
    DmaHost::Request r{};
    r.cs = {};
    r.dc = DcPad::ref();
    if (cmd_len != 0u) {
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
        r.cmd_len = cmd_len;
    }
    if (len != 0u) {
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx + cmd_len));
        if (read) {
            r.rx = lend<Lease::reply>(rx + cmd_len);
        }
        r.len = len;
    }
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.engine = SpiEngine::dma;
    dma_completions = 0;
    dma_blocks = 0;
    host_done = false;
    const bool done = DmaHost::start(r);
    o.inside = done;
    const uint32_t m0 = Ticker::millis();
    while (!done && !host_done && Ticker::millis() - m0 < 200u) {
    }
    o.clocked = done || host_done;
    o.completions = dma_completions;
    o.blocks = dma_blocks;
    o.dc_up = DcPad::read_out();
    (void)DcWatch::arm(false);
    Nvic::disable(DcWatch::irq());
    (void)DcWatch::clear();
    o.dc_rise = dc_rise;
    link_hold();
    CsPin::set();
    dma_host_live = false;
    DmaHost::release();
    DcPad::release();
    (void)link_command_mode();
    o.reported = peer_report(o.r);
    if (read) {
        spilink::Stream want(a.pattern, a.seed_b);
        for (uint16_t i = 0; i < n; ++i) {
            const uint8_t w = want.next();
            if (i >= cmd_len && rx[i] != w) {
                ++o.mism;
            }
        }
    }
    return o;
}

/// Exact at both ends, and on the engines: not inside start(), one block, one
/// completion.
bool clean(const Window& o, uint16_t n) {
    return o.accepted && o.clocked && o.reported && o.r.count == n && o.r.mism == 0u && o.mism == 0u &&
           !o.inside && o.completions == 1u && o.blocks == 1u;
}

void print_window(const char* what, uint16_t n, SpiClock rate, const Window& o) {
    print(serial, "    ", what, ", ", n, " frames at PCLK/", spi_division(rate), ": inside start() ",
          o.inside ? 1u : 0u, ", completions ", o.completions, ", blocks ", o.blocks, ", D/C rose ",
          o.dc_rise == 0u ? "never" : o.dc_rise == 1u ? "at rest" : "ON A MOVING WIRE", ", host mism ", o.mism, ", the peer read ", o.r.count, " mism ",
          o.r.mism, o.clocked ? "" : " HOST NEVER COMPLETED", crlf);
}

struct Refusal {
    bool ended = false;
    uint8_t status = 0xFFu;
    uint16_t falls = 0;
    uint16_t edges = 0;
    bool regs_same = false;
    bool dc_low = false;
    bool select_up = false;
};

/// One request the host must refuse. The block sits at mode 0, PCLK/16 and 8
/// bits and the request names mode 3, PCLK/8 and 16 bits; a data phase would
/// raise the parked D/C.
template <typename H>
Refusal refuse(typename H::Request q) {
    Refusal f{};
    q.cs = CsPin::ref();
    q.dc = DcPad::ref();
    q.clock = SpiClock::div8;
    q.mode = SpiMode::mode3;
    q.engine = SpiEngine::dma;
    H::prime(SpiMode::mode0, SpiClock::div16);
    DcPad::clear();
    const uint32_t cr1 = S1::regs().CR1;
    const uint32_t cr2 = S1::regs().CR2;
    cs_falls = 0;
    (void)CsWatch::select();
    (void)CsWatch::configure(ExtiSense::falling);
    (void)CsWatch::clear();
    (void)CsWatch::arm(true);
    Nvic::enable(CsWatch::irq());
    way::watch_sck(true);
    f.ended = H::start(q);
    f.status = H::status();
    settle_ms(1);
    way::watch_sck(false);
    (void)CsWatch::arm(false);
    (void)CsWatch::clear();
    f.falls = cs_falls;
    f.edges = static_cast<uint16_t>(sck_edges_selected + sck_edges_released);
    f.regs_same = S1::regs().CR1 == cr1 && S1::regs().CR2 == cr2;
    f.dc_low = !DcPad::read_out();
    f.select_up = CsPin::read_out();
    DcPad::set();
    return f;
}

bool refused(const Refusal& f) {
    return f.ended && f.status == spi_rejected && f.falls == 0u && f.edges == 0u && f.regs_same && f.dc_low &&
           f.select_up;
}

void print_refusal(const char* what, const Refusal& f) {
    print(serial, "    ", what, ": start() ", f.ended ? 1u : 0u, ", status ", f.status, " (spi_rejected ",
          spi_rejected, "), select falls ", f.falls, ", SCK edges ", f.edges, ", CR1 and CR2 ",
          f.regs_same ? "untouched" : "MOVED", ", D/C ", f.dc_low ? "untouched" : "MOVED", crlf);
}

}  // namespace named

void tS_named() {
    if (!need_peer()) {
        return;
    }
    using named::Window;

    // PROOF ONE. PCLK/4 and /8: a frame's 32 and 64 cycles, under the
    // engines' fixed cost.
    bool one_ok = true;
    uint8_t seed = 0x31;
    for (const SpiClock rate : {SpiClock::div4, SpiClock::div8}) {
        const Window read = named::run(0, 1, rate, seed);
        named::print_window("one frame read", 1, rate, read);
        const Window written = named::run(0, 1, rate, static_cast<uint8_t>(seed + 7u), false);
        named::print_window("one frame written", 1, rate, written);
        one_ok = one_ok && named::clean(read, 1) && named::clean(written, 1);
        seed = static_cast<uint8_t>(seed + 23u);
    }
    bench.verdict("A REQUEST NAMING THE ENGINES RUNS ON THEM AT THE SHORTEST LENGTH: one frame, read and "
                  "written, at PCLK/4 and PCLK/8 - start() false, dma_isr() the end, one block, one "
                  "completion, byte-exact at both ends",
                  one_ok);

    // PROOF TWO. Refused, every witness read; the next request exact.
    Dma<1>::bus_clock(true);
    CsPin::output(true);
    named::DcPad::output(true);
    (void)DmaHost::init(clock, named::link_config());
    dma_host_live = true;
    const auto wide = [](const uint8_t* out, uint8_t* in) {
        DmaHost::Request q{};
        q.tx = lend<Lease::reply>(out);
        q.rx = lend<Lease::reply>(in);
        q.len = 4;
        q.bits = SpiDataSize::bits16;
        return q;
    };
    const named::Refusal odd_tx = named::refuse<DmaHost>(wide(named::tx + 1, named::rx));
    const named::Refusal odd_rx = named::refuse<DmaHost>(wide(named::tx, named::rx + 1));
    DmaHost::Request lone{};
    lone.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(named::tx + 1));
    lone.cmd_len = 4;
    lone.bits = SpiDataSize::bits16;
    const named::Refusal odd_cmd = named::refuse<DmaHost>(lone);
    dma_host_live = false;
    DmaHost::release();
    CsPin::output(true);
    (void)ByteDmaHost::init(clock, ByteDmaHost::Config{.mosi_speed = peer_mosi_speed});
    ByteDmaHost::Request on_bytes{};
    on_bytes.tx = lend<Lease::reply>(static_cast<const uint8_t*>(named::tx));
    on_bytes.rx = lend<Lease::reply>(named::rx);
    on_bytes.len = 4;
    on_bytes.bits = SpiDataSize::bits16;
    const named::Refusal bytes = named::refuse<ByteDmaHost>(on_bytes);
    ByteDmaHost::release();
    named::print_refusal("a wide frame out of an odd address", odd_tx);
    named::print_refusal("a wide frame into an odd address", odd_rx);
    named::print_refusal("a wide command alone from an odd address", odd_cmd);
    named::print_refusal("a wide frame on byte engines", bytes);
    bench.verdict("WHAT THE ENGINES CANNOT CARRY IS REFUSED: a wide frame on an odd address either way or as a "
                  "command alone, a wide frame on byte engines - start() true, spi_rejected, the select never "
                  "fell, D/C, CR1 and CR2 untouched, no edge on SCK",
                  named::refused(odd_tx) && named::refused(odd_rx) && named::refused(odd_cmd) &&
                      named::refused(bytes));

    // A transmit channel a program holds refuses at the launch: the select has
    // fallen by then, and comes back; nothing is clocked. The select is PB6
    // here, so the peer's never moves.
    CsPin::output(true);
    named::DcPad::output(true);
    (void)DmaHost::init(clock, named::link_config());
    dma_host_live = true;
    DmaHost::prime(SpiMode::mode0, SpiClock::div16);
    const bool held = HostTx::claim();
    DmaHost::Request late{};
    late.cs = named::DcPad::ref();
    late.tx = lend<Lease::reply>(static_cast<const uint8_t*>(named::tx));
    late.len = 4;
    late.clock = SpiClock::div16;
    late.engine = SpiEngine::dma;
    way::watch_sck(true);
    const bool late_ended = DmaHost::start(late);
    const uint8_t late_status = DmaHost::status();
    settle_ms(1);
    way::watch_sck(false);
    const uint16_t late_edges = static_cast<uint16_t>(sck_edges_selected + sck_edges_released);
    const bool late_up = named::DcPad::read_out();
    HostTx::unclaim();
    print(serial, "    a transmit channel a program holds: claimed ", held ? 1u : 0u, ", start() ",
          late_ended ? 1u : 0u, ", status ", late_status, ", SCK edges ", late_edges, ", the select back up ",
          late_up ? 1u : 0u, crlf);
    bench.verdict("A CHANNEL THAT REFUSES THE BLOCK ENDS THE REQUEST: start() true, spi_rejected, no edge on SCK, "
                  "the select back up - it does not wait for a completion that will never come",
                  held && late_ended && late_status == spi_rejected && late_edges == 0u && late_up);
    dma_host_live = false;
    DmaHost::release();
    named::DcPad::release();
    (void)link_command_mode();

    const Window after = named::run(0, 32, SpiClock::div8, 0x52);
    named::print_window("the next request", 32, SpiClock::div8, after);
    bench.verdict("the next request after every refusal is exact at both ends", named::clean(after, 32));

    // PROOF THREE. Sixteen edges a frame: D/C rises at exactly the command's.
    const Window before = named::run(4, 8, SpiClock::div256, 0x5B);
    named::print_window("a 4-frame command before 8 data frames", 12, SpiClock::div256, before);
    const Window one_cmd = named::run(1, 32, SpiClock::div128, 0x6D);
    named::print_window("a 1-frame command before 32 data frames", 33, SpiClock::div128, one_cmd);
    bench.verdict("A COMMAND BEFORE DATA RUNS IN THE LOOP AND THE DATA ON THE ENGINES: one block, one completion, D/C "
                  "rising on a wire at rest between the two phases, exact at both ends",
                  named::clean(before, 12) && before.dc_rise == 1u && named::clean(one_cmd, 33) &&
                      one_cmd.dc_rise == 1u);
    const Window alone = named::run(3, 0, SpiClock::div16, 0x77);
    named::print_window("a 3-frame command alone", 3, SpiClock::div16, alone);
    const Window alone_slow = named::run(3, 0, SpiClock::div256, 0x79);
    named::print_window("a 3-frame command alone", 3, SpiClock::div256, alone_slow);
    bench.verdict("A COMMAND WITH NO DATA RIDES THE ENGINES AS THE WHOLE REQUEST: one block, one completion, exact at "
                  "the peer, D/C high when the completion answers, on a wire at rest",
                  named::clean(alone, 3) && alone.dc_up && alone.dc_rise == 1u && named::clean(alone_slow, 3) &&
                      alone_slow.dc_up && alone_slow.dc_rise == 1u);
    bench.verdict("the command channel survives the letter", command(Op::ping));
}

// =============================================================================
// E - WHEN A REQUEST ENDS THE WIRE HAS ENDED
// =============================================================================
//
// The SCK pad's input, read right after the request ended - after start()
// returned for a polled one, masked, in the handler right after dma_isr()
// returned true for the engines' - must be at the mode's idle level. WHY AN
// EARLY END CANNOT BE MISSED, AND WHERE: half a bit is 2^code core cycles here
// (PCLK is HCLK), 128 at /256; from the flag to the pad's read is some 40 core
// cycles on the polled path and the vector's whole body on the engines' (the
// figures below), so the instrument sees an early end where the half bit
// outlasts that, and the control run names the rungs. THE VERDICT IS ONE-SIDED:
// SCK at its idle level only ever follows the last edge, so a pad read at the
// active level is an early end and nothing else, and the bound is zero at every
// rung. PB6 is the select, a pad on no recorded wire; the link's own stays high,
// so the peer hears none of this.

namespace wire_end {

using Sck = Pin<host_pins.sck.port, host_pins.sck.pin>;
using Select = Pin<'B', 6>;

volatile bool watching = false;
volatile bool pad_high = false;
volatile bool select_up = false;

[[gnu::always_inline]] inline void sampled() {
    pad_high = Sck::read();
    select_up = Select::read_out();
}

enum class Way : uint8_t { polled_read_1, polled_read_8, polled_write, dma_read, dma_write, dma_command, dma_both };
constexpr uint8_t way_count = 7;
const char* way_name(uint8_t w) {
    constexpr const char* names[way_count] = {"polled read, 1 frame",  "polled read, 8 frames", "polled write, 8 frames",
                                              "dma read, 1 frame",     "dma write, 1 frame",    "dma command alone",
                                              "dma command then data"};
    return names[w];
}
constexpr bool way_is_dma(uint8_t w) { return w >= static_cast<uint8_t>(Way::dma_read); }

uint8_t out[8];
uint8_t in[8];
uint8_t early[way_count][4];
uint32_t reads = 0;
uint32_t bad_status = 0;
uint32_t no_end = 0;
uint32_t vector_most[2][8];
uint32_t vector_least[2][8];

DmaHost::Request request(uint8_t w, SpiMode m, SpiClock c, PinRef cs) {
    DmaHost::Request r{};
    r.cs = cs;
    r.clock = c;
    r.mode = m;
    r.bits = SpiDataSize::bits8;
    r.engine = way_is_dma(w) ? SpiEngine::dma : SpiEngine::polled;
    const uint8_t* const o = out;
    switch (static_cast<Way>(w)) {
        case Way::polled_read_1:
        case Way::dma_read:
            r.tx = lend<Lease::reply>(o);
            r.rx = lend<Lease::reply>(in);
            r.len = 1;
            break;
        case Way::polled_read_8:
            r.tx = lend<Lease::reply>(o);
            r.rx = lend<Lease::reply>(in);
            r.len = 8;
            break;
        case Way::polled_write:
            r.tx = lend<Lease::reply>(o);
            r.len = 8;
            break;
        case Way::dma_write:
            r.tx = lend<Lease::reply>(o);
            r.len = 1;
            break;
        case Way::dma_command:
            r.cmd = lend<Lease::reply>(o);
            r.cmd_len = 1;
            break;
        case Way::dma_both:
            r.cmd = lend<Lease::reply>(o);
            r.cmd_len = 1;
            r.tx = lend<Lease::reply>(o);
            r.rx = lend<Lease::reply>(in);
            r.len = 1;
            break;
    }
    return r;
}

bool once(uint8_t w, SpiMode m, SpiClock c, PinRef cs, bool& high, bool& up) {
    const DmaHost::Request r = request(w, m, c, cs);
    if (!way_is_dma(w)) {
        {
            InterruptGuard guard;
            (void)DmaHost::start(r);
            high = Sck::read();
            up = Select::read_out();
        }
        return true;
    }
    host_done = false;
    watching = true;
    const bool inside = DmaHost::start(r);
    const uint32_t m0 = Ticker::millis();
    while (!inside && !host_done && Ticker::millis() - m0 < 20u) {
    }
    watching = false;
    high = pad_high;
    up = select_up;
    return !inside && host_done;
}

void sweep() {
    for (auto& per_way : early) {
        for (uint8_t& per_mode : per_way) {
            per_mode = 0;
        }
    }
    for (uint8_t i = 0; i < 2u; ++i) {
        for (uint8_t k = 0; k < 8u; ++k) {
            vector_most[i][k] = 0;
            vector_least[i][k] = UINT32_MAX;
        }
    }
    reads = 0;
    bad_status = 0;
    no_end = 0;
    time_vector = true;
    for (uint8_t w = 0; w < way_count; ++w) {
        for (uint8_t mode = 0; mode < 4u; ++mode) {
            const SpiMode m = static_cast<SpiMode>(mode);
            const bool idle_high = spi_cpol(m);
            for (uint8_t code = 0; code < 8u; ++code) {
                const SpiClock c = static_cast<SpiClock>(code);
                for (uint8_t rep = 0; rep < 6u; ++rep) {
                    const PinRef cs = rep < 3u ? Select::ref() : PinRef{};
                    bool high = idle_high;
                    bool up = true;
                    const bool ended = once(w, m, c, cs, high, up);
                    ++reads;
                    if (!ended) {
                        ++no_end;
                        (void)DmaHost::recover();
                        continue;
                    }
                    if (DmaHost::status() != spi_ok) {
                        ++bad_status;
                    }
                    if (high != idle_high || !up) {
                        early[w][mode] = static_cast<uint8_t>(early[w][mode] | (1u << code));
                    }
                    if (rep != 0u && mode < 2u &&
                        (w == static_cast<uint8_t>(Way::dma_read) || w == static_cast<uint8_t>(Way::dma_write))) {
                        const uint32_t v = vector_cycles;
                        uint32_t& most = vector_most[mode][code];
                        uint32_t& least = vector_least[mode][code];
                        most = v > most ? v : most;
                        least = v < least ? v : least;
                    }
                }
            }
        }
    }
    time_vector = false;
}

bool report(uint32_t& worst_over) {
    uint32_t seen = 0;
    print(serial, "  SPI1: ", reads, " requests ended, ", no_end, " with no end, ", bad_status, " not spi_ok", crlf);
    for (uint8_t w = 0; w < way_count; ++w) {
        for (uint8_t mode = 0; mode < 4u; ++mode) {
            if (early[w][mode] != 0u) {
                print(serial, "    EARLY: ", way_name(w), ", mode ", mode, ", rungs");
                for (uint8_t code = 0; code < 8u; ++code) {
                    if ((early[w][mode] & (1u << code)) != 0u) {
                        print(serial, " /", 2u << code);
                        ++seen;
                    }
                }
                print(serial, crlf);
            }
        }
    }
    worst_over = 0;
    print(serial, "    the vector, core cycles, read or write, mode 0 then mode 1, least..most:", crlf);
    for (uint8_t code = 0; code < 8u; ++code) {
        const uint32_t half = 1u << code;
        print(serial, "      /", 2u << code, " half a bit ", half, ": ", vector_least[0][code], "..",
              vector_most[0][code], "  ", vector_least[1][code], "..", vector_most[1][code], crlf);
        const uint32_t floor = vector_least[1][code];
        const uint32_t over = vector_most[0][code] > floor + half ? vector_most[0][code] - (floor + half) : 0u;
        worst_over = over > worst_over ? over : worst_over;
    }
    print(serial, "    the longest wait past half a bit, mode 0 over mode 1's least: ", worst_over, " cycles", crlf);
    return seen == 0u && no_end == 0u && bad_status == 0u;
}

}  // namespace wire_end

void tE_wire_ended() {
    using namespace wire_end;
    for (uint8_t i = 0; i < 8u; ++i) {
        out[i] = static_cast<uint8_t>(0xA5u ^ (i * 11u));
    }
    Select::output(true);
    CsPin::output(true);
    Dma<1>::bus_clock(true);
    (void)DmaHost::init(clock, named::link_config());
    dma_host_live = true;
    sweep();
    dma_host_live = false;
    uint32_t over = 0;
    const bool ok = report(over);
    DmaHost::release();
    Select::release();
    bench.verdict("WHEN A REQUEST ENDS THE WIRE HAS ENDED: all four modes, every rate, polled and on the engines, "
                  "read and write, through a select and a null one - SCK at its idle level every time (the bound is "
                  "zero: a pad read at the active level is an early end and nothing else)",
                  ok);
    (void)link_command_mode();
}

/// SPI1's line is letter m's alone: no host arms the block's interrupts,
/// so the sleep letter's bare RXNE is the one thing that raises it, and
/// the frame is read here and the wake recorded.
extern "C" void SPI1_IRQHandler() {
    if (sleep_probe_live && S1::rxne()) {
        (void)S1::data_byte();
        sleep_woke = true;
    }
}

/// The engined host's completion: to the kernel letters' arbiter, or -
/// swallowed - nowhere (letter j's lost interrupt), or to the waiting
/// letter.
void dma_host_done() {
    if (bus_ao_live) {
        if (!host_swallow_completion) {
            brio::post<kl::SpiArb>(brio::TransferDone{DmaHost::status()});
        }
        return;
    }
    host_done = true;
    dma_completions = static_cast<uint16_t>(dma_completions + 1u);
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

/// Letter K's witness of SCK.
extern "C" void EXTI2_3_IRQHandler() {
    if (SckWatch::pending()) {
        (void)SckWatch::clear();
        if (CsPin::read_out()) {
            sck_edges_released = static_cast<uint16_t>(sck_edges_released + 1u);
        } else {
            sck_edges_selected = static_cast<uint16_t>(sck_edges_selected + 1u);
        }
    }
}

extern "C" void EXTI4_15_IRQHandler() {
    if (NssWatch::pending()) {
        (void)NssWatch::clear();
        nss_edges = static_cast<uint16_t>(nss_edges + 1u);
    }
    if (CsWatch::pending()) {
        (void)CsWatch::clear();
        cs_falls = static_cast<uint16_t>(cs_falls + 1u);
    }
    if (named::DcWatch::pending()) {
        (void)named::DcWatch::clear();
        // Letter S's windows are mode 0: SCK idles low.
        if (named::dc_rise == 0u) {
            const bool rest = !S1::busy() && S1::tx_level() == 0u &&
                              !Pin<host_pins.sck.port, host_pins.sck.pin>::read();
            named::dc_rise = rest ? 1u : 2u;
        }
    }
}

/// The engined host's body, from either channel's vector; letter K times it.
void dma_host_vector() {
    const uint32_t t0 = time_vector ? cycles_now() : 0u;
    const bool ended = DmaHost::dma_isr();
    if (ended && wire_end::watching) {
        wire_end::sampled();
    }
    if (ended && time_vector) {
        vector_cycles = cycles_now() - t0;
    }
    if (ended) {
        dma_host_done();
    }
}

extern "C" void DMA1_Channel1_IRQHandler() {
    if (dma_host_live) {
        dma_host_vector();
    } else {
        (void)HostTx::service();
    }
}

/// The receive channel's vector: every entry under the engined host is
/// one of its blocks ending (its transmit channel interrupts for an error
/// alone, on DMA1_Channel1).
extern "C" void DMA1_Channel2_3_IRQHandler() {
    if (dma_host_live) {
        dma_blocks = static_cast<uint16_t>(dma_blocks + 1u);
        dma_host_vector();
        (void)PeerTx::service();
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
                      "the DMA engines", tb_link);
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
    bench.letter('s', "the host's shapes, WIRELESS: the two loops in every shape, a "
                      "request naming the engines refused with none, the rate on the wire",
                 ts_shapes);
    bench.letter('n', "THE PEER: the spi_link command channel to the peer board",
                 tn_peer_link);
    bench.letter('o', "the matrix against the peer: four modes, both orders",
                 to_peer_matrix);
    bench.letter('p', "the BR ladder against the peer, and where it stops",
                 tp_peer_rates);
    bench.letter('q', "THE KERNEL against the peer: SpiBus over SpiHost",
                 tq_peer_kernel);
    bench.letter('r', "THE ROLES INVERT: this board as the client, the peer as "
                      "the host", tr_peer_client);
    bench.letter('u', "16-BIT FRAMES against the peer: the loop, the engines, "
                      "the wire's time", tu_wide);
    bench.letter('v', "RECEIVE-ONLY and ODD COUNTS against the peer: the transmit-only "
                      "shapes witnessed", tv_receive_only);
    bench.letter('w', "THE CRC against the peer's unit, and a wrong CRC caught", tw_crc);
    bench.letter('y', "THE SIMPLEX LINES, THE SOFTWARE SELECT AND prime() against the peer",
                 ty_simplex);
    bench.letter('K', "THE NEXT REQUEST AFTER A STALL OR A TRANSFER ERROR, recover() never called",
                 tK_way_back);
    bench.letter('S', "THE ENGINE A REQUEST NAMES against the peer: one frame on the engines, "
                      "the refusals, the command rule",
                 tS_named);
    bench.letter('E', "WHEN A REQUEST ENDS THE WIRE HAS ENDED: the SCK pad, every mode and rate, "
                      "both engines",
                 tE_wire_ended);
    bench.letter('t', "THE CRCNEXT WINDOW against the peer: the walk, no verdict",
                 tt_crcnext_walk, false);
    bench.letter('x', "SLIP STATISTICS against the peer: each mode ten times, no "
                      "verdict", tx_peer_slips, false);

    if (serial_ok) {
        const auto idcode = brio::DeviceIdcode::read();
        print(serial, crlf, "part DEV_ID ", hex(idcode.dev_id),
              " REV_ID ", hex(idcode.rev_id), crlf);
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
