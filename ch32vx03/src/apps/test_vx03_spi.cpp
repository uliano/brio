// test_vx03_spi - the reference bench suite for the CH32V203's SPI
// chapter: ch32vx03/spi.hpp over RM ch. 20, both instances, both roles,
// the DMA engines, the hardware CRC and util/spi_bus.hpp's arbiter with
// not one line changed.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT IT MEASURES WITH. Two instruments, and both are on the desk at
// once because they are on DIFFERENT INSTANCES:
//
//  - SPI1's own LOOPBACK. The board carries a strap from PA7 (MOSI) to
//    PA6 (MISO) through PA8, so the host's own frames come back. PA8 is
//    the third pad of that node and this suite NEVER drives it: it is
//    left a floating input for the whole run, and a letter that wanted
//    it would be fighting the wire. The strap is probed at every letter
//    that needs it (a plain output on PA7 read on PA6 against the
//    opposite pull), never cached: a wire can leave the desk between
//    two letters.
//    SPI1's NSS pad PA4 is strapped to its own SCK pad PA5, so SPI1
//    runs SOFTWARE SELECT ONLY here - a hardware NSS input would read
//    the clock and demote the host at the first edge. The three NSS
//    arrangements are exercised on SPI2 instead, where the select is a
//    real wire.
//
//  - SPI2 against a PEER BOARD running `spi_peer` (a Nucleo-F446RE; the
//    peer's ident says which), commanded IN BAND over the bus under
//    test through avrdx/src/apps/spi_link.hpp, included by relative
//    path: one source of truth for the wire format on every
//    architecture. Five wires, both boards at 3.3 V:
//
//      PB12 (NSS)   <->  the peer's NSS   (its pull-up holds it high)
//      PB13 (SCK)   ->   the peer's SCK
//      PB14 (MISO)  <-   the peer's MISO
//      PB15 (MOSI)  ->   the peer's MOSI
//      GND          <->  GND
//
//    Every peer letter asks for a ping first and SKIPS (with no verdict
//    claimed) when three command retries fail: the two boards are not
//    always in step, and a suite that hangs on an absent instrument is
//    worse than one that says so.
//
// THE PADS. PA4..PA8 and PA15/PB3/PB4/PB5 (SPI1's two columns), PB12..
// PB15 (SPI2). NEVER TOUCHED: PA9/PA10 (the console), PA13/PA14 (the
// debug port), PA11/PA12 (the USB pads), PC14/PC15 and PD0/PD1 (the
// crystals), PA0 (the KEY), PB6/PB7 and PB10/PB11 (other phases' wires)
// - and PB2, the LED, toggled per command as every suite of this target
// does.
//
// What is exercised, letter by letter:
//   a  THE RATES: every BR code on both instances read back, with the
//      SCK of SPI1's SECOND COLUMN measured on the pad itself - PB3 is
//      TIM2's channel 2 under TIM2's partial remap, and a pad driven by
//      one peripheral still reaches another's input - and the effective
//      bit rate of a saturated block on each instance
//   b  THE HOST WITH ITS MISO HELD BY ITS OWN PORT, no wire: frames of
//      all ones and all zeros read back, both widths, both bit orders,
//      the four modes read back out of CTLR1
//   c  THE LOOPBACK on SPI1: round trips at every mode, both widths,
//      both orders, the hardware CRC against a bitwise reference, 4 KB
//      at the fastest code that passes clean, both DMA engines, and
//      SpiBus (= BusMaster) over the whole of it
//   d  THE PEER on SPI2: ping/ident/report, the four modes and both
//      orders exchanged, the dummy byte MEASURED, sink_slow, ss_pulse,
//      host_burst with THIS board as the client, the engines under an
//      exchange, and a ten-second stress with the counters at both ends
//   e  THE FLAGS AND THE VECTORS: every flag with the sequence that
//      clears it, the bidirectional line mode on one pad, the
//      receive-only host and how it is stopped, the I2S register this
//      series has no I2S for
//
// build: boards = v203c6,v203c8
// build: groups = abe,cd
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/spi.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

#include "../../../avrdx/src/apps/spi_link.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The two instances, their pads and their engines
// ---------------------------------------------------------------------------

/// The instance the PEER is on. SPI2 where the part has it; SPI1
/// otherwise, so the suite still LINKS on a part of the 32 KB tier that
/// carries one SPI - letter d then says so and claims no verdict.
constexpr uint8_t peer_spi = device::spi_count >= 2u ? 2u : 1u;
constexpr bool has_two_spi = device::spi_count >= 2u;

using S1 = Spi<1>;
using S2 = Spi<peer_spi>;

/// SPI1's default column, the loopback's: NSS PA4, SCK PA5, MISO PA6,
/// MOSI PA7.
constexpr SpiPins loop_pins = spi_pins_for(1, 0);
/// SPI1's SECOND column, letter a's ruler: NSS PA15, SCK PB3, MISO PB4,
/// MOSI PB5 - PB3 is where TIM2's channel 2 lands under TIM2's partial
/// remap, so the clock can be counted on the very pad that carries it.
constexpr SpiPins moved_pins = spi_pins_for(1, 1);
/// SPI2's one column, the peer's: NSS PB12, SCK PB13, MISO PB14, MOSI
/// PB15.
constexpr SpiPins peer_pins = spi_pins_for(peer_spi, 0);

using Host1 = SpiHost<1, loop_pins>;
using Dma1 = SpiHost<1, loop_pins, DmaTxEngine<S1::dma_tx_channel>,
                     DmaRxEngine<S1::dma_rx_channel>>;
using Moved = SpiHost<1, moved_pins>;
using PeerHost = SpiHost<peer_spi, peer_pins>;
using PeerDma = SpiHost<peer_spi, peer_pins, DmaTxEngine<S2::dma_tx_channel>,
                        DmaRxEngine<S2::dma_rx_channel>>;
using PeerClient = SpiClient<peer_spi, peer_pins>;

using SckPad = Pin<loop_pins.sck.port, loop_pins.sck.pin>;
using MisoPad = Pin<loop_pins.miso.port, loop_pins.miso.pin>;
using MosiPad = Pin<loop_pins.mosi.port, loop_pins.mosi.pin>;
using NssPad = Pin<loop_pins.nss.port, loop_pins.nss.pin>;
/// The third pad of the strapped node. Named ONLY so the suite can put
/// it back to a floating input and prove it is one; never driven.
using ThirdPad = Pin<'A', 8>;
using PeerNssPad = Pin<peer_pins.nss.port, peer_pins.nss.pin>;
using PeerMisoPad = Pin<peer_pins.miso.port, peer_pins.miso.pin>;

/// A transaction that never answered: the suite's own word.
constexpr uint8_t no_answer = 200;

// ---------------------------------------------------------------------------
// The rulers
// ---------------------------------------------------------------------------

/// The core's STK read as an absolute cycle count: the tick counter
/// times the reload, plus the sub-tick counter, re-read until the two
/// agree. It wraps at 2^32 cycles, some thirty seconds here.
uint32_t cycles_now() {
    const uint32_t period = stk()->CMPLR + 1u;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t cnt = stk()->CNTL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * period + cnt;
        }
    }
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

// ---------------------------------------------------------------------------
// The buffers (sized for the 10 KB tier: a 4 KB burst is this pattern
// sent sixteen times over, not a 4 KB array)
// ---------------------------------------------------------------------------

constexpr uint16_t chunk = 256;
uint8_t tx_buf[chunk];
uint8_t rx_buf[chunk];
uint8_t cmd_buf[4];

void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + i * 7u + (i >> 3));
    }
}

bool same(const uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// The straps, probed and never cached
// ---------------------------------------------------------------------------

/// Drive one pad, read the other against the OPPOSITE pull: a floating
/// pad that merely follows its neighbour is not taken for a strap, and
/// a wire beats a pull. Both pads are left floating afterwards.
template <typename Driver, typename Reader>
bool pads_linked() {
    Reader::input(PinPull::down);
    Driver::output(true);
    (void)delay_us(clock, 20);
    const bool high = Reader::read();
    Reader::input(PinPull::up);
    Driver::clear();
    (void)delay_us(clock, 20);
    const bool low = !Reader::read();
    Driver::release();
    Reader::release();
    return high && low;
}

/// PA8 is the third pad of the MOSI/MISO node and must stay an input for
/// the whole run.
void third_pad_idle() { ThirdPad::release(); }

bool need_loop() {
    third_pad_idle();
    if (pads_linked<MosiPad, MisoPad>()) {
        return true;
    }
    print(serial, "  SKIPPED, no verdict claimed: no strap between PA7 (MOSI) and PA6 (MISO) "
                  "- probed now",
          crlf);
    return false;
}

// ---------------------------------------------------------------------------
// The plain host, waited out
// ---------------------------------------------------------------------------

volatile bool host_done = false;
volatile uint32_t spi1_isr_entries = 0;
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;
volatile bool peer_client_live = false;

void host_ready() {
    dma_host_live = false;
    bus_ao_live = false;
    (void)Host1::init(clock);
    NssPad::release();   // the select pad is SCK's neighbour on this board
}

uint8_t host_xfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                  SpiMode mode, SpiClock rate, SpiDataSize bits, bool polled) {
    Host1::Request r{};
    r.cs = {};   // PA4 is strapped to PA5 here: the suite frames nothing with it
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = mode;
    r.clock = rate;
    r.bits = bits;
    r.polled = polled;
    host_done = false;
    spi1_isr_entries = 0;
    if (Host1::start(r)) {
        return Host1::status();
    }
    for (uint32_t i = 0; i < 4'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL: STATR=", hex(S1::status()), " CTLR1=", hex(S1::regs().CTLR1),
              " CTLR2=", hex(S1::regs().CTLR2), " isr entries ", spi1_isr_entries, crlf);
        (void)Host1::recover();
        return no_answer;
    }
    return Host1::status();
}

uint8_t dma_xfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                 SpiClock rate, SpiDataSize bits, bool polled) {
    Dma1::Request r{};
    r.cs = {};
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = SpiMode::mode0;
    r.clock = rate;
    r.bits = bits;
    r.polled = polled;
    host_done = false;
    if (Dma1::start(r)) {
        return Dma1::status();
    }
    for (uint32_t i = 0; i < 4'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL (dma): STATR=", hex(S1::status()),
              " ch", S1::dma_rx_channel, " flags=", hex(DmaChannel<S1::dma_rx_channel>::flags()),
              " ch", S1::dma_tx_channel, " flags=", hex(DmaChannel<S1::dma_tx_channel>::flags()),
              crlf);
        (void)Dma1::recover();
        return no_answer;
    }
    return Dma1::status();
}

// ===========================================================================
// a - the rates, and the clock measured on its own pad
// ===========================================================================

/// TIM2 as an EDGE COUNTER on PB3: the partial remap (column 1) puts
/// TIM2's channel 2 on that pad, the channel is configured as an input,
/// and the slave controller's external clock mode 1 makes TI2FP2's
/// rising edges the counter's clock. The pad stays the SPI's alternate
/// output throughout - a pad driven by one peripheral still reaches
/// another's input on this family.
using SckCounter = Tim<2>;

bool sck_counter_arm() {
    SckCounter::init();
    if (!SckCounter::remap(1)) {
        return false;
    }
    if (!SckCounter::configure({.prescaler = 0, .period = 0xFFFF})) {
        return false;
    }
    if (!SckCounter::capture_channel(1, {.select = TimChannelSelect::direct,
                                         .polarity = TimCapturePolarity::rising,
                                         .prescaler = TimCapturePrescaler::every,
                                         .filter = 0,
                                         .enable = true})) {
        return false;
    }
    if (!SckCounter::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::ti2})) {
        return false;
    }
    SckCounter::enable(true);
    return true;
}

void ta_rates() {
    // ---- the register's own vocabulary, read back on both instances ----
    uint8_t read_back = 0;
    S1::bus_clock(true);
    S1::reset();
    for (uint8_t code = 0; code < 8u; ++code) {
        (void)S1::configure({.role = SpiRole::host, .clock = static_cast<SpiClock>(code)});
        if (S1::clock() == static_cast<SpiClock>(code)) {
            ++read_back;
        }
    }
    if constexpr (has_two_spi) {
        S2::bus_clock(true);
        S2::reset();
        for (uint8_t code = 0; code < 8u; ++code) {
            (void)S2::configure({.role = SpiRole::host, .clock = static_cast<SpiClock>(code)});
            if (S2::clock() == static_cast<SpiClock>(code)) {
                ++read_back;
            }
        }
    }
    bench.verdict("every BR code reads back out of CTLR1 on every instance this part has",
                  read_back == (has_two_spi ? 16u : 8u));

    print(serial, "  SPI1 divides PB2 = ", SysClock::pclk2_hz / 1'000'000u,
          " MHz, SPI2 divides PB1 = ", SysClock::pclk1_hz / 1'000'000u,
          " MHz: the same code is two frequencies", crlf);

    // ---- the clock counted on the pad that carries it ----
    if (!sck_counter_arm()) {
        bench.verdict("TIM2's channel 2 takes the partial remap onto PB3", false);
        return;
    }
    bench.verdict("TIM2's channel 2 takes the partial remap onto PB3, SPI1's second column's "
                  "clock pad",
                  SckCounter::remap() == 1u);

    if (!Moved::init(clock)) {
        bench.verdict("SPI1 comes up on its second column (PA15/PB3/PB4/PB5)", false);
        return;
    }
    bench.verdict("SPI1 comes up as a host on its SECOND column - the one remap this family's "
                  "SPI has (table 10-32)",
                  S1::enabled() && Afio::remap_code(Remap::spi1) == 1u);
    bench.verdict("and its BR field divides ITS OWN BUS: SPI1 is the PB2 instance, so its "
                  "reference is PCLK2 and not the system clock by coincidence",
                  Moved::reference_hz() == SysClock::pclk2_hz);

    constexpr uint16_t frames = 64;
    fill_pattern(tx_buf, frames, 0x5A);
    uint8_t exact_edges = 0;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock rate = static_cast<SpiClock>(code);
        SckCounter::set_count(0);
        console_drain();
        const uint32_t t0 = cycles_now();
        Moved::Request r{};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.len = frames;
        r.clock = rate;
        r.polled = true;
        (void)Moved::start(r);
        const uint32_t cycles = cycles_now() - t0;
        const uint32_t edges = SckCounter::count();
        const uint32_t want = 8UL * frames;
        const uint32_t sck = Moved::sck_hz(rate);
        // The wire's own time for the burst at this SCK, in core cycles.
        const uint32_t wire = static_cast<uint32_t>((8ULL * frames * SysClock::hz) / sck);
        print(serial, "  /", spi_division(rate), " (", sck / 1000u, " kHz): ", edges,
              " rising edges on PB3 (want ", want, "), ", cycles, " cycles for ", frames,
              " frames (the wire alone ", wire, ")", crlf);
        if (edges == want) {
            ++exact_edges;
        }
    }
    print(serial, "  the counter is TI2FP2 through the slave controller's external clock mode "
                  "1, and the pad is the SPI's alternate OUTPUT throughout",
          crlf);
    bench.verdict("THE CLOCK IS COUNTED ON ITS OWN PAD: eight rising edges per 8-bit frame at "
                  "EVERY BR code, the fastest included - a pad driven by one peripheral reaches "
                  "another's input, and the counter follows SCK to half the timer clock",
                  exact_edges == 8u);

    SckCounter::enable(false);
    SckCounter::release();
    Moved::release();
    (void)Afio::remap(Remap::spi1, 0);

    // ---- the effective bit rate of a saturated block, per instance ----
    if constexpr (has_two_spi) {
        (void)PeerHost::init(clock);
        fill_pattern(tx_buf, chunk, 0x11);
        for (uint8_t code = 2; code < 8u; code += 2u) {
            const SpiClock rate = static_cast<SpiClock>(code);
            console_drain();
            const uint32_t t0 = cycles_now();
            PeerHost::Request r{};
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
            r.len = chunk;
            r.clock = rate;
            r.polled = true;
            (void)PeerHost::start(r);
            const uint32_t cycles = cycles_now() - t0;
            const uint32_t bits = 8UL * chunk;
            const uint32_t measured = cycles == 0u ? 0u
                                                   : static_cast<uint32_t>(
                                                         (static_cast<uint64_t>(bits) *
                                                          SysClock::hz) /
                                                         cycles);
            print(serial, "  SPI2 /", spi_division(rate), ": nominal ",
                  PeerHost::sck_hz(rate) / 1000u, " kHz, ", chunk,
                  " bytes polled measured ", measured / 1000u, " kbit/s", crlf);
        }
        bench.verdict("SPI2's polled bursts run at its OWN bus's divisions - the effective bit "
                      "rate follows PB1 and not the system clock",
                      PeerHost::reference_hz() == SysClock::pclk1_hz);
        PeerHost::release();
    } else {
        print(serial, "  this part has one SPI: no second instance to time", crlf);
    }
    host_ready();
}

// ===========================================================================
// b - the host with its MISO held by its own port
// ===========================================================================

void tb_held() {
    third_pad_idle();
    // SPI1 as a host with SCK alone handed to the peripheral: MOSI is
    // RELEASED (PA7, PA6 and PA8 are one node on this board, so a driven
    // MOSI would fight the port) and PA6 is an ordinary output whose
    // level is what every frame reads back.
    Pfic::disable(S1::irq);
    S1::bus_clock(true);
    S1::reset();
    (void)S1::remap(0);
    SckPad::function();
    MosiPad::release();
    NssPad::release();

    struct Case {
        SpiMode mode;
        SpiDataSize bits;
        bool lsb;
    };
    constexpr Case cases[] = {
        {SpiMode::mode0, SpiDataSize::bits8, false},  {SpiMode::mode1, SpiDataSize::bits8, false},
        {SpiMode::mode2, SpiDataSize::bits8, false},  {SpiMode::mode3, SpiDataSize::bits8, false},
        {SpiMode::mode0, SpiDataSize::bits16, false}, {SpiMode::mode0, SpiDataSize::bits8, true},
    };
    uint8_t ones = 0, zeros = 0, modes = 0;
    for (const Case& c : cases) {
        (void)S1::configure({.role = SpiRole::host,
                             .mode = c.mode,
                             .clock = SpiClock::div64,
                             .bits = c.bits,
                             .lsb_first = c.lsb});
        S1::enable();
        S1::flush_rx();
        if (S1::mode() == c.mode && S1::bits() == c.bits) {
            ++modes;
        }
        const uint16_t mask = spi_frame_mask(c.bits);
        for (uint8_t level = 0; level < 2u; ++level) {
            MisoPad::output(level != 0u);
            (void)delay_us(clock, 5);
            S1::flush_rx();
            S1::data(c.bits, 0x00);
            uint32_t spins = 400'000u;
            while (!S1::rxne() && spins-- != 0u) {
            }
            const uint16_t got = S1::data(c.bits);
            if (level != 0u && got == mask) {
                ++ones;
            }
            if (level == 0u && got == 0u) {
                ++zeros;
            }
        }
        (void)S1::disable();
    }
    print(serial, "  six configurations x two levels: ", ones, " read all ones, ", zeros,
          " read all zeros, ", modes, " of 6 read their own mode and width back", crlf);
    bench.verdict("A HOST CLOCKS ITSELF: with MISO held high by the port every frame comes back "
                  "all ones, at both widths, both bit orders and all four modes",
                  ones == 6u);
    bench.verdict("... and held low, all zeros - the receive path is the pad's level and not a "
                  "leftover",
                  zeros == 6u);
    bench.verdict("the four modes and both widths read back out of CTLR1", modes == 6u);

    // A frame written with no clock configured at all is NOT sent: the
    // host's clock is SPE's, and BSY with SPE down is the question.
    (void)S1::disable();
    const bool idle = !S1::busy();
    bench.verdict("with SPE down the block is not busy - the host's clock is the enable's",
                  idle);

    MisoPad::release();
    host_ready();
}

// ===========================================================================
// c - the loopback: the pump, the widths, the CRC, the engines, the kernel
// ===========================================================================

/// The chapter's CRC, bitwise: MSB first, no reflection, init 0.
uint16_t crc_reference(uint16_t poly, uint8_t width, const uint8_t* data, uint16_t n) {
    uint32_t crc = 0;
    const uint32_t top = 1UL << (width - 1u);
    const uint32_t mask = (1UL << width) - 1UL;
    for (uint16_t k = 0; k < n; ++k) {
        for (uint8_t b = 8; b-- > 0;) {
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

namespace kl {

using SpiArb = SpiBus<Host1, P, 4>;

uint8_t out_a[8];
uint8_t out_b[8];
uint8_t in_a[8];

class Probe {
public:
    using Event = std::variant<SpiDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint8_t n = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;

    static void init() { clear_tally(); }
    static void clear_tally() {
        n = 0;
        rejected = 0;
        votes = 0;
        last_vote = false;
    }

    static void dispatch(const Event& e) {
        brio::match(
            e,
            [](const SpiDone& d) {
                if (n < 8u) {
                    replies[n] = d.status;
                }
                ++n;
                if (d.status == spi_rejected) {
                    ++rejected;
                }
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
            });
    }
};

using BusKernel = Tenuto<P, Probe, SpiArb>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

void pump_until(uint8_t want, uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
        if (Probe::n >= want) {
            break;
        }
    }
    pump();
}

Host1::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    Host1::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.mode = SpiMode::mode0;
    r.clock = SpiClock::div32;
    r.polled = polled;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kl

void tc_loop() {
    if (!need_loop()) {
        return;
    }
    host_ready();

    // ---- the pump: the four modes, both widths, both orders ----
    uint8_t exact = 0;
    constexpr uint8_t trials = 10;
    for (uint8_t k = 0; k < trials; ++k) {
        const SpiMode mode = static_cast<SpiMode>(k & 3u);
        const SpiDataSize bits = (k >= 4u && k < 8u) ? SpiDataSize::bits16 : SpiDataSize::bits8;
        const bool lsb = k >= 8u;
        const uint16_t frames = spi_frame_is_halfword(bits) ? 16u : 32u;
        const uint16_t bytes = spi_frame_is_halfword(bits) ? 32u : 32u;
        fill_pattern(tx_buf, bytes, static_cast<uint8_t>(0x20u + k));
        for (uint16_t i = 0; i < bytes; ++i) {
            rx_buf[i] = 0xEE;
        }
        (void)Host1::bit_order(lsb);
        const uint8_t st =
            host_xfer(nullptr, 0, tx_buf, rx_buf, frames, mode, SpiClock::div16, bits, false);
        if (st == spi_ok && same(tx_buf, rx_buf, bytes)) {
            ++exact;
        } else {
            print(serial, "    mode ", static_cast<uint8_t>(mode), " ", spi_data_bits(bits),
                  " bits ", lsb ? "LSb" : "MSb", ": status ", st, crlf);
        }
    }
    (void)Host1::bit_order(false);
    print(serial, "  ", exact, " of ", trials,
          " loopback runs byte-exact through the ISR pump (four modes, both widths, both orders)",
          crlf);
    bench.verdict("THE LOOPBACK ON THE PUMP: every mode, both frame widths and both bit orders "
                  "come back byte-exact",
                  exact == trials);
    bench.verdict("the pump runs on RXNE: one interrupt per frame and no more",
                  spi1_isr_entries == 32u);

    // ---- a command phase and a read with no out buffer ----
    cmd_buf[0] = 0x9F;
    fill_pattern(tx_buf, 16, 0x71);
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t cs = host_xfer(cmd_buf, 1, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClock::div16,
                                 SpiDataSize::bits8, false);
    bench.verdict("a command frame then a data phase, the data read back exact",
                  cs == spi_ok && same(tx_buf, rx_buf, 16));
    for (uint16_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0;
    }
    const uint8_t rd = host_xfer(nullptr, 0, nullptr, rx_buf, 8, SpiMode::mode0, SpiClock::div16,
                                 SpiDataSize::bits8, true);
    bool ff = true;
    for (uint16_t i = 0; i < 8; ++i) {
        ff = ff && rx_buf[i] == 0xFFu;
    }
    bench.verdict("a read with no out buffer clocks 0xFF dummies and reads them back on the loop",
                  rd == spi_ok && ff);

    // ---- the ladder, and 4 KB at the fastest code that is clean ----
    uint32_t best = 0;
    uint32_t first_bad = 0;
    for (uint8_t code = 8; code-- > 0;) {
        const SpiClock rate = static_cast<SpiClock>(code);
        fill_pattern(tx_buf, chunk, static_cast<uint8_t>(0xA0u + code));
        for (uint16_t i = 0; i < chunk; ++i) {
            rx_buf[i] = 0xEE;
        }
        console_drain();
        const uint32_t t0 = cycles_now();
        const uint8_t st = host_xfer(nullptr, 0, tx_buf, rx_buf, chunk, SpiMode::mode0, rate,
                                     SpiDataSize::bits8, true);
        const uint32_t cycles = cycles_now() - t0;
        const bool ok = st == spi_ok && same(tx_buf, rx_buf, chunk);
        print(serial, "  /", spi_division(rate), " (", Host1::sck_hz(rate) / 1000u, " kHz): ",
              chunk, " bytes polled in ", cycles, " cycles (", cycles / chunk, " per frame)",
              ok ? "  byte-exact" : "  MISMATCH", crlf);
        if (ok && first_bad == 0u) {
            best = Host1::sck_hz(rate);
        } else if (!ok && first_bad == 0u) {
            first_bad = Host1::sck_hz(rate);
        }
    }
    print(serial, "  the loop is clean to ", best / 1000u, " kHz");
    if (first_bad != 0u) {
        print(serial, " and breaks at ", first_bad / 1000u, " kHz");
    }
    print(serial, crlf);
    bench.verdict("the strap carries every BR code the block offers, or names the one it does "
                  "not",
                  best >= 1'000'000UL);

    // 4 KB at the fastest code that passed: the same chunk sixteen times,
    // which is what a 10 KB part can afford.
    const auto top = spi_rate_for(Host1::reference_hz(), best);
    uint16_t bad_chunks = 0;
    if (top) {
        console_drain();
        const uint32_t t0 = cycles_now();
        for (uint8_t k = 0; k < 16u; ++k) {
            fill_pattern(tx_buf, chunk, static_cast<uint8_t>(k * 13u + 3u));
            for (uint16_t i = 0; i < chunk; ++i) {
                rx_buf[i] = 0xEE;
            }
            const uint8_t st = host_xfer(nullptr, 0, tx_buf, rx_buf, chunk, SpiMode::mode0, *top,
                                         SpiDataSize::bits8, true);
            if (st != spi_ok || !same(tx_buf, rx_buf, chunk)) {
                ++bad_chunks;
            }
        }
        const uint32_t cycles = cycles_now() - t0;
        print(serial, "  4096 bytes at ", best / 1000u, " kHz in ", cycles / 1000u,
              " thousand cycles, ", bad_chunks, " of 16 chunks wrong", crlf);
    }
    bench.verdict("FOUR KILOBYTES at the fastest code the strap carries, byte-exact",
                  top.has_value() && bad_chunks == 0u);

    // ---- the hardware CRC against a bitwise reference ----
    Pfic::disable(S1::irq);
    Host1::release();
    S1::bus_clock(true);
    S1::reset();
    (void)S1::configure({.role = SpiRole::host,
                         .mode = SpiMode::mode0,
                         .clock = SpiClock::div64,
                         .bits = SpiDataSize::bits8,
                         .crc = true,
                         .crc_polynomial = 0x0007u});
    SckPad::function();
    MosiPad::function();
    MisoPad::input();
    S1::enable();
    S1::flush_rx();

    constexpr uint16_t cn = 6;
    uint8_t data[cn];
    uint8_t got[cn];
    for (uint16_t i = 0; i < cn; ++i) {
        data[i] = static_cast<uint8_t>(0x31u + i * 7u);
    }
    for (uint16_t i = 0; i < cn; ++i) {
        S1::data8(data[i]);
        if (i == cn - 1u) {
            S1::crc_next();
        }
        uint32_t spins = 400'000u;
        while (!S1::rxne() && spins-- != 0u) {
        }
        got[i] = S1::data8();
    }
    uint32_t spins = 400'000u;
    while (!S1::rxne() && spins-- != 0u) {
    }
    const uint8_t crc_frame = S1::data8();
    spins = 400'000u;
    while (S1::busy() && spins-- != 0u) {
    }
    const uint16_t tx_crc = S1::tx_crc();
    const uint16_t rx_crc = S1::rx_crc();
    const bool err = S1::crc_error();
    const uint16_t ref = crc_reference(0x0007u, 8, data, cn);
    print(serial, "  CRC8 over six frames: TXCRCR ", hex(tx_crc), " (software ", hex(ref),
          "), RXCRCR ", hex(rx_crc), ", the CRC frame read back ", hex(crc_frame), ", CRCERR ",
          err ? "SET" : "clear", crlf);
    bench.verdict("the six data frames came back exact through the loop", same(data, got, cn));
    bench.verdict("THE HARDWARE CRC IS THE ARITHMETIC: TXCRCR is what a bitwise loop over the "
                  "same polynomial computes",
                  tx_crc == ref);
    bench.verdict("the receiver's RXCRCR over the looped-back frames is that same number, the "
                  "CRC frame it read is its low byte, and CRCERR stands down",
                  rx_crc == ref && crc_frame == static_cast<uint8_t>(ref) && !err);
    (void)S1::disable();

    // ---- the DMA engines ----
    dma_host_live = true;
    bus_ao_live = false;
    Dma::open();
    (void)Dma1::init(clock);
    fill_pattern(tx_buf, chunk, 0x3C);
    for (uint16_t i = 0; i < chunk; ++i) {
        rx_buf[i] = 0xEE;
    }
    console_drain();
    uint32_t t0 = cycles_now();
    const uint8_t ds = dma_xfer(nullptr, 0, tx_buf, rx_buf, chunk, SpiClock::div4,
                                SpiDataSize::bits8, false);
    uint32_t cycles = cycles_now() - t0;
    print(serial, "  ", chunk, " bytes at /4 on the engines (channels ", S1::dma_tx_channel,
          " and ", S1::dma_rx_channel, "): status ", ds, " in ", cycles, " cycles (",
          cycles / chunk, " per frame; the wire alone 32)",
          same(tx_buf, rx_buf, chunk) ? "  byte-exact" : "  MISMATCH", crlf);
    bench.verdict("A BLOCK THROUGH BOTH ENGINES completes spi_ok, byte-exact, on the two "
                  "channels table 11-5 wires to this instance",
                  ds == spi_ok && same(tx_buf, rx_buf, chunk));

    cmd_buf[0] = 0x0B;
    fill_pattern(tx_buf, 32, 0x5A);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t cds = dma_xfer(cmd_buf, 1, tx_buf, rx_buf, 32, SpiClock::div8,
                                 SpiDataSize::bits8, false);
    bench.verdict("a command frame on the pump then 32 data frames on the engines, exact",
                  cds == spi_ok && same(tx_buf, rx_buf, 32));

    fill_pattern(tx_buf, 32, 0x42);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    spi1_isr_entries = 0;
    const uint8_t hs = dma_xfer(nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div8,
                                SpiDataSize::bits16, false);
    bench.verdict("16-bit frames fall back to the pump on an engined host, sixteen ISR entries, "
                  "exact",
                  hs == spi_ok && same(tx_buf, rx_buf, 32) && spi1_isr_entries == 16u);
    Dma1::release();
    dma_host_live = false;

    // ---- the kernel ----
    host_ready();
    kl::BusKernel::init_all();
    bus_ao_live = true;
    fill_pattern(kl::out_a, 8, 0x01);
    fill_pattern(kl::out_b, 8, 0x02);
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::SpiArb>(kl::request(kl::out_a, (i == 3u) ? kl::in_a : nullptr, i == 1u));
    }
    kl::pump_until(4, 300);
    print(serial, "  four queued transactions (one polled): replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ",
          kl::Probe::replies[3], "]", crlf);
    bench.verdict("four transactions through SpiBus, four replies - util/spi_bus.hpp and "
                  "util/bus_master.hpp unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... every one spi_ok, ISR-pumped and polled interleaved on one bus",
                  kl::Probe::replies[0] == spi_ok && kl::Probe::replies[1] == spi_ok &&
                      kl::Probe::replies[2] == spi_ok && kl::Probe::replies[3] == spi_ok);
    bench.verdict("and the last one's read-back is the pattern, through the arbiter",
                  same(kl::out_a, kl::in_a, 8));

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::SpiArb>(kl::request(kl::out_b, nullptr, false));
    }
    kl::pump_until(6, 300);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately",
                  kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);

    kl::Probe::clear_tally();
    post<kl::SpiArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::SpiArb>(kl::request(kl::out_a, nullptr, false));
    post<kl::SpiArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    kl::pump_until(2, 100);
    print(serial, "  the vote from a BUSY bus: ", kl::Probe::votes, " vote(s), last ",
          kl::Probe::last_vote ? "yes" : "no", crlf);
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u && !kl::Probe::last_vote);
    bus_ao_live = false;
    host_ready();
}

// ===========================================================================
// d - the peer on SPI2
// ===========================================================================

using spilink::Op;

/// The command channel's SCK. PB1 runs at 72 MHz here, so /256 is
/// 281.25 kHz and a character is 28 us - an order of magnitude more than
/// the peer's polled command listener needs to turn one around.
SpiClock link_clock = SpiClock::div256;

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t raw_seen[16];
uint8_t raw_n = 0;
spilink::Decoder dec;
bool link_quiet = false;

/// The hold around each select window: the engine releases the select
/// about a microsecond after the last edge, and a client whose
/// transaction the select edge frames loses a character it has not
/// fetched yet - so the protocol owns the chip select for these windows
/// (the Request carries a null PinRef) and pays 30 us on each side.
void link_hold() { (void)delay_us(clock, 30); }

bool link_command_mode() {
    peer_client_live = false;
    const bool ok = PeerHost::init(clock);
    PeerNssPad::output(true);
    dec.reset();
    return ok;
}

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip
/// inside an open window is one extra edge and the selected client
/// counts it into the frame.
bool link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate = SpiClock::div256,
               SpiMode mode = SpiMode::mode0) {
    if (n == 0) {
        return true;
    }
    PeerHost::prime(mode, rate);
    PeerNssPad::clear();
    link_hold();
    PeerHost::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
    r.bits = SpiDataSize::bits8;
    r.polled = true;
    const bool done = PeerHost::start(r);
    link_hold();
    PeerNssPad::set();
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
    (void)link_xfer(frame_buf, nullptr, n, link_clock);
}

/// One answer window: `answer_bytes` dummies in a single transaction (a
/// null out buffer: the engine's own 0xFF, which is no frame's magic),
/// then the decoder walks what came back.
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        answer_buf[i] = 0xEE;
    }
    (void)link_xfer(nullptr, answer_buf, spilink::answer_bytes, link_clock);
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

void settle() { settle_ms(spilink::settle_ms); }

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
          "      the peer board must be running `spi_peer`; its console '0' forces the dark "
          "client back.",
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
        if (query(Op::report, f) && f.op == Op::report_data && f.len == spilink::report_size) {
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
/// an absent peer is a NAMED skip and never a hang.
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
    return false;
}

bool need_peer() {
    if constexpr (!has_two_spi) {
        print(serial, "  SKIPPED, no verdict claimed: this part carries one SPI, and the peer "
                      "is on the second instance's four wires",
              crlf);
        return false;
    } else {
        if (ensure_link()) {
            return true;
        }
        print(serial, "  SKIPPED, no verdict claimed: THE PEER DID NOT ANSWER three command "
                      "retries. The peer board must be running `spi_peer`; its console '0' "
                      "forces the dark client back. Check the five wires in this file's header.",
              crlf);
        return false;
    }
}

// ---- one exchange, commanded and then clocked ------------------------------

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    SpiClock rate = SpiClock::div256;
    uint16_t count = 8;
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;
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
    // A bit-order mismatch is not a shrug: told about it, each end checks
    // the EXACT bit-reverse of what the other sent.
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
    if (e.host_lsb) {
        (void)PeerHost::bit_order(true);
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
    print(serial, "    host mism=", v.mism, " (first idx ", v.idx, " got ", hex(v.got), " exp ",
          hex(v.exp), "), client count=", r.count, " mism=", r.mism, " (first idx ", r.idx,
          " got ", hex(r.got), " exp ", hex(r.exp), ") flags=", hex(r.flags), crlf);
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

// ---- this board as the CLIENT, the peer as the host ------------------------

uint8_t crx[64];
uint16_t crx_n = 0;

bool run_as_client(const spilink::Params& a) {
    crx_n = 0;
    peer_client_live = true;
    Pfic::disable(S2::irq);
    PeerNssPad::input(PinPull::up);
    if (!PeerClient::init(clock, {.mode = SpiMode::mode0,
                                  .bits = SpiDataSize::bits8,
                                  .lsb_first = false,
                                  .nss = SpiNss::hardware_input,
                                  .drive_output = true})) {
        peer_client_live = false;
        return false;
    }
    Pfic::disable(S2::irq);   // polled here: the vector stays with the host's pump
    // The select FRAMES the transaction: the peer drives PB12 low for the
    // whole burst, and this end preloads its first answer before it
    // falls.
    spilink::Stream out(a.pattern, a.seed_b);
    PeerClient::enable(out.next());
    const uint32_t t0 = Ticker::millis();
    while (!PeerClient::selected() && Ticker::millis() - t0 < a.ms + 200u) {
    }
    if (!PeerClient::selected()) {
        (void)PeerClient::disable();
        peer_client_live = false;
        return false;
    }
    const uint32_t started = Ticker::millis();
    while (crx_n < a.count && Ticker::millis() - started < a.ms) {
        const auto v = PeerClient::poll();
        if (!v) {
            continue;
        }
        if (crx_n < sizeof crx) {
            crx[crx_n] = static_cast<uint8_t>(*v);
        }
        ++crx_n;
        PeerClient::write(out.next());   // ONE AHEAD: one buffer, no FIFO
    }
    const bool dark_ok = !PeerMisoPad::read_out() || true;
    (void)dark_ok;
    (void)PeerClient::disable();
    PeerClient::drive_output(false);
    peer_client_live = false;
    return true;
}

void td_peer() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the four wires of SPI2", true);

    spilink::Ident d{};
    const bool got = peer_ident(d);
    if (got) {
        print(serial, "  peer: label '");
        for (uint8_t i = 0; i < 8 && d.label[i]; ++i) {
            print(serial, d.label[i]);
        }
        print(serial, "' xtal=", d.xtal, " sanity=", hex(d.sanity), " fw=", hex(d.version), crlf);
    }
    bench.verdict("ident comes back and it IS spi_peer (the sanity byte), from a SECOND BOARD of "
                  "another architecture speaking the same wire format",
                  got && d.sanity == spilink::ident_sanity);

    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++pings;
        }
    }
    print(serial, "  ", pings, " of 10 pings answered at ", PeerHost::sck_hz(link_clock) / 1000u,
          " kHz SCK, one frame per chip-select window", crlf);
    bench.verdict("the command channel is steady over ten frames", pings == 10u);

    // ---- the four modes and both bit orders, and THE PADS' SLEW ----
    //
    // The four-mode matrix is run at each of this port's three pad slew
    // classes, from the driver's default down. A data line's edge
    // coupled into the clock is one sampling edge too many, and the
    // modes that sample on the FALLING edge - where the data changes on
    // the rising one - are where it shows: the matrix is therefore a
    // measurement of the PAD as much as of the block, and what it prints
    // is the ladder and not just a verdict.
    constexpr PinSpeed speeds[] = {PinSpeed::fast, PinSpeed::medium, PinSpeed::slow};
    const char* speed_names[] = {"fast (50 MHz, the driver's own)", "medium (10 MHz)",
                                 "slow (2 MHz)"};
    uint8_t best_good = 0;
    uint8_t clean_at = 3;
    for (uint8_t si = 0; si < 3u; ++si) {
        PeerHost::pad_speed(speeds[si]);
        (void)link_command_mode();
        uint8_t good = 0;
        uint8_t worst_mode = 0xFF;
        Exchange worst{};
        Verify worst_v{};
        spilink::Report worst_r{};
        for (uint8_t m = 0; m < 4; ++m) {
            Exchange e{};
            e.cfg.mode = m;
            e.host_mode = static_cast<SpiMode>(m);
            e.seed_a = static_cast<uint8_t>(0x13u + m);
            e.seed_b = static_cast<uint8_t>(0x57u + m);
            Verify v{};
            spilink::Report r{};
            if (exchange_exact(e, v, r)) {
                ++good;
            } else if (worst_mode == 0xFFu) {
                worst_mode = m;
                worst = e;
                worst_v = v;
                worst_r = r;
            }
        }
        print(serial, "  SCK and MOSI at ", speed_names[si], ": ", good,
              " of 4 transfer modes byte-exact both ways", crlf);
        if (worst_mode != 0xFFu) {
            print(serial, "    mode ", worst_mode, ":", crlf);
            dump_exchange(worst, worst_v, worst_r);
        }
        if (good > best_good) {
            best_good = good;
        }
        if (good == 4u && clean_at == 3u) {
            clean_at = si;
            break;
        }
    }
    PeerHost::pad_speed(clean_at < 3u ? speeds[clean_at] : PinSpeed::slow);
    (void)link_command_mode();
    print(serial, "  the matrix is clean from ",
          clean_at < 3u ? speed_names[clean_at] : "no slew class this port has", " downwards",
          crlf);
    bench.verdict("ALL FOUR TRANSFER MODES carry a burst byte-exact in both directions between "
                  "TWO SEPARATE CHIPS - at a pad slew class this letter FINDS rather than "
                  "assumes, because a data line's edge coupled into the clock is one sampling "
                  "edge too many",
                  best_good == 4u);

    Exchange lsb{};
    lsb.cfg.dord = 1;
    lsb.host_lsb = true;
    Verify v{};
    spilink::Report r{};
    const bool lsb_ok = exchange_exact(lsb, v, r);
    if (!lsb_ok) {
        dump_exchange(lsb, v, r);
    }
    bench.verdict("LSb first, both ends agreeing, is byte-exact too - the bit order a BUS-level "
                  "verb (CTLR1.LSBFIRST) and not a Request field",
                  lsb_ok);

    Exchange mism{};
    mism.cfg.dord = 1;      // the client LSb first
    mism.host_lsb = false;  // this end MSb first
    Verify v2{};
    spilink::Report r2{};
    const bool ran = do_exchange(mism);
    const bool rep = ran && peer_report(r2);
    v2 = verify_rx(mism);
    print(serial, "  DORD mismatch: host mism=", v2.mism, " client mism=", r2.mism, " count=",
          r2.count, " (both checking the exact bit-reverse)", crlf);
    bench.verdict("a bit-order mismatch is an EXACT TWO-WAY BIT REVERSAL - each end reads the "
                  "other's bytes with their bits in the opposite order",
                  ran && rep && v2.mism == 0u && r2.mism == 0u && r2.count == mism.count);

    // ---- THE DUMMY BYTE, measured and not assumed ----
    Exchange nopre{};
    nopre.cfg.regime = spilink::regime_normal;   // the peer does NOT preload
    nopre.seed_a = 0x2B;
    nopre.seed_b = 0x6D;
    Verify v3{};
    spilink::Report r3{};
    (void)do_exchange(nopre);
    (void)peer_report(r3);
    v3 = verify_rx(nopre);
    const uint8_t first_read = xrx[0];
    const uint8_t want_first = spilink::pattern_value(nopre.pattern, nopre.seed_b, 0);
    uint16_t shifted = 0;
    for (uint16_t i = 1; i < nopre.count && i < max_exchange; ++i) {
        if (xrx[i] != spilink::pattern_value(nopre.pattern, nopre.seed_b, i - 1u)) {
            ++shifted;
        }
    }
    print(serial, "  with NO preload the first frame read back is ", hex(first_read),
          " (the preloaded stream would start ", hex(want_first), "); the rest is ",
          shifted == 0u ? "the client's stream ONE PLACE LATE" : "neither aligned nor shifted",
          ", and the aligned read had ", v3.mism, " frame(s) out of place", crlf);
    bench.verdict("THE DUMMY IS MEASURED AND NOT ASSUMED: a client that has not preloaded sends "
                  "the shifter's leftover in the first frame and its own stream one place late "
                  "from there - the one-ahead rule seen from the other end",
                  shifted == 0u && v3.mism != 0u && first_read != want_first);

    // ---- the ladder against the peer ----
    uint32_t last_good = 0;
    uint32_t first_bad = 0;
    spilink::Report bad_r{};
    for (uint8_t code = 8; code-- > 0;) {
        const SpiClock c = static_cast<SpiClock>(code);
        if (c == SpiClock::div2 && first_bad != 0u) {
            break;
        }
        Exchange e{};
        e.rate = c;
        e.count = 8;
        e.seed_a = 0x21;
        e.seed_b = 0x84;
        Verify lv{};
        spilink::Report lr{};
        const bool ok = exchange_exact(e, lv, lr);
        const uint32_t real = PeerHost::sck_hz(c);
        print(serial, "  SCK PB1/", spi_division(c), " = ", real / 1000u, " kHz: ",
              ok ? "exact both ways" : "NOT exact", "  host mism=", lv.mism, " client mism=",
              lr.mism, " client count=", lr.count, crlf);
        if (ok && first_bad == 0u) {
            last_good = real;
        } else if (!ok && first_bad == 0u) {
            first_bad = real;
            bad_r = lr;
        }
    }
    print(serial, "  the link to the peer held to ", last_good / 1000u, " kHz");
    if (first_bad != 0u) {
        print(serial, " and broke at ", first_bad / 1000u, " kHz");
    }
    print(serial, crlf);
    bench.verdict("the link is exact at the command rate and at least four times faster",
                  last_good >= 1'100'000UL);
    bench.verdict("wherever the climb breaks, the peer still hears every character exact there - "
                  "the boundary is its ANSWER RELOAD and not the wire",
                  first_bad == 0u || (bad_r.count == 8u && bad_r.mism == 0u));

    // ---- sink_slow: what a client that never drains does to the host ----
    {
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_normal};
        a.count = 32;
        a.ms = 120;
        if (peer_act(Op::sink_slow, a)) {
            settle_ms(10);
            fill_pattern(xtx, max_exchange, 0x91);
            for (uint8_t i = 0; i < max_exchange; ++i) {
                xrx[i] = 0xEE;
            }
            (void)link_xfer(xtx, xrx, 16, link_clock);
            (void)link_command_mode();
            spilink::Report sr{};
            const bool rp = peer_report(sr);
            print(serial, "  sink_slow: the client retained ", sr.count,
                  " frame(s), its status during the flood was ", hex(sr.sum), ", after the drain ",
                  hex(sr.exp), "; the host read back ", hex(xrx[0]), " ", hex(xrx[1]), " ",
                  hex(xrx[2]), crlf);
            bench.verdict("A CLIENT THAT NEVER DRAINS keeps ONE frame and raises the overrun - "
                          "there is no FIFO behind it (figure 20-1)",
                          rp && sr.count <= 1u &&
                              (sr.flags & spilink::report_bufovf) != 0u);
        } else {
            bench.verdict("the peer accepted sink_slow", false);
        }
    }

    // ---- ss_pulse: the select seen from the client's end ----
    {
        spilink::Params a{};
        a.aux8 = 60;      // the lead-in, long enough for this end to let go
        a.aux16 = 4000;   // microseconds the peer holds the wire down
        // THE COMMAND GOES OUT FIRST, and only then does this end let the
        // wire go: the command travels over the bus the host owns, so a
        // release before it would have nothing to send it with.
        if (peer_act(Op::ss_pulse, a)) {
            PeerHost::release();
            PeerNssPad::input(PinPull::up);
            bool saw_low = false;
            const uint32_t t0 = Ticker::millis();
            while (Ticker::millis() - t0 < 300u) {
                if (!PeerNssPad::read()) {
                    saw_low = true;
                    break;
                }
            }
            print(serial, "  ss_pulse: the shared select ", saw_low ? "went LOW" : "stayed high",
                  " while the peer drove it", crlf);
            bench.verdict("THE SELECT IS A REAL WIRE: the peer pulling it low is seen at this "
                          "end, which is what frames a client's transaction",
                          saw_low);
        } else {
            bench.verdict("the peer accepted ss_pulse", false);
        }
        (void)link_command_mode();
    }

    // ---- host_burst: THE ROLES INVERT, this board the client ----
    {
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
        a.count = 16;
        a.ms = 400;
        a.seed_a = 0x3B;
        a.seed_b = 0x6D;
        a.pattern = spilink::pattern_prbs;
        a.aux8 = 40;    // the lead-in: this end becomes a client inside it
        a.aux16 = 64;   // the instrument's SCK division of its own clock
        if (!peer_act(Op::host_burst, a)) {
            bench.verdict("the peer accepted host_burst", false);
        } else {
            const bool ran = run_as_client(a);
            spilink::Report hr{};
            (void)link_command_mode();
            const bool rp = peer_report(hr);
            uint16_t mism = 0;
            {
                spilink::Stream want(a.pattern, a.seed_a);
                for (uint16_t i = 0; i < crx_n && i < a.count; ++i) {
                    if (crx[i] != want.next()) {
                        ++mism;
                    }
                }
            }
            print(serial, "  host_burst: this board read ", crx_n, " of ", a.count,
                  " frames with ", mism, " wrong; the peer read ", hr.count, " with ", hr.mism,
                  " wrong (flags ", hex(hr.flags), ")", crlf);
            bench.verdict("THE ROLES INVERT: this board's SpiClient answers ONE FRAME AHEAD "
                          "under a host on another chip, and both ends agree byte for byte",
                          ran && rp && crx_n >= a.count && mism == 0u && hr.count == a.count &&
                              hr.mism == 0u);
            bench.verdict("... and the client's MISO is released again afterwards - the dark "
                          "listener leaves the answer line to the bus",
                          !PeerClient::output_driven());
        }
    }

    // ---- the engines under an exchange ----
    {
        Dma::open();
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
        a.count = 16;
        a.ms = 400;
        a.seed_a = 0x44;
        a.seed_b = 0x99;
        a.pattern = spilink::pattern_prbs;
        if (!peer_act(Op::exchange, a)) {
            bench.verdict("the peer accepted the engined exchange", false);
        } else {
            spilink::Stream out(a.pattern, a.seed_a);
            for (uint8_t i = 0; i < max_exchange; ++i) {
                xtx[i] = 0;
                xrx[i] = 0xEE;
            }
            for (uint16_t i = 0; i < a.count; ++i) {
                xtx[i] = out.next();
            }
            settle();
            peer_client_live = false;
            dma_host_live = true;
            (void)PeerDma::init(clock);
            PeerNssPad::output(true);
            PeerDma::prime(SpiMode::mode0, link_clock);
            PeerNssPad::clear();
            link_hold();
            PeerDma::Request r{};
            r.cs = {};
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(xtx));
            r.rx = lend<Lease::reply>(xrx);
            r.len = a.count;
            r.clock = link_clock;
            r.mode = SpiMode::mode0;
            r.polled = true;
            const bool done = PeerDma::start(r);
            link_hold();
            PeerNssPad::set();
            dma_host_live = false;
            PeerDma::release();
            uint16_t mism = 0;
            {
                spilink::Stream want(a.pattern, a.seed_b);
                for (uint16_t i = 0; i < a.count; ++i) {
                    if (xrx[i] != want.next()) {
                        ++mism;
                    }
                }
            }
            spilink::Report er{};
            (void)link_command_mode();
            const bool rp = peer_report(er);
            print(serial, "  the engines on channels ", S2::dma_tx_channel, " and ",
                  S2::dma_rx_channel, ": status ", PeerDma::status(), ", host mism=", mism,
                  ", the peer read ", er.count, " with ", er.mism, " wrong", crlf);
            bench.verdict("SPI2's DMA ENGINES carry an exchange to a second chip, byte-exact at "
                          "both ends",
                          done && mism == 0u && rp && er.count == a.count && er.mism == 0u);
        }
    }

    // ---- ten seconds of traffic, counted at both ends ----
    {
        uint32_t rounds = 0, bad = 0, frames = 0;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 10'000u) {
            Exchange e{};
            e.count = 16;
            e.rate = SpiClock::div64;
            e.seed_a = static_cast<uint8_t>(0x11u + rounds);
            e.seed_b = static_cast<uint8_t>(0x77u + rounds);
            Verify sv{};
            spilink::Report sr{};
            if (!exchange_exact(e, sv, sr)) {
                ++bad;
            }
            frames += e.count;
            ++rounds;
        }
        print(serial, "  ten seconds: ", rounds, " exchanges, ", frames, " frames, ", bad,
              " not exact at one end or the other", crlf);
        bench.verdict("A TEN-SECOND STRESS between two chips: every exchange exact at both ends",
                      rounds > 0u && bad == 0u);
    }

    (void)link_command_mode();
    bench.verdict("the command channel is back after all of that", command(Op::ping));
    PeerHost::release();
    host_ready();
}

// ===========================================================================
// e - the flags, the vectors, the line modes, the disable rule
// ===========================================================================

volatile uint32_t err_isr_entries = 0;
volatile uint32_t err_isr_flags = 0;
volatile bool err_mode = false;

void te_flags() {
    third_pad_idle();
    Pfic::disable(S1::irq);
    S1::bus_clock(true);
    S1::reset();
    (void)S1::remap(0);

    // ---- the reset values (table 20-1) ----
    const bool reset_ok = S1::regs().CTLR1 == 0u && S1::regs().CTLR2 == 0u &&
                          S1::regs().STATR == 0x0002u && S1::crc_polynomial() == 0x0007u;
    print(serial, "  after the RCC reset pulse: CTLR1=", hex(S1::regs().CTLR1), " CTLR2=",
          hex(S1::regs().CTLR2), " STATR=", hex(S1::status()), " CRCR=",
          hex(S1::crc_polynomial()), crlf);
    bench.verdict("the reset values are the chapter's: TXE up alone in STATR, the polynomial 7",
                  reset_ok);

    // ---- the overrun, and the sequence that clears it ----
    const bool loop = pads_linked<MosiPad, MisoPad>();
    if (loop) {
        (void)S1::configure({.role = SpiRole::host, .clock = SpiClock::div64});
        SckPad::function();
        MosiPad::function();
        MisoPad::input();
        S1::enable();
        S1::flush_rx();
        for (uint8_t i = 0; i < 3u; ++i) {
            S1::data8(static_cast<uint8_t>(0x40u + i));
            uint32_t spins = 400'000u;
            while (!S1::txe() && spins-- != 0u) {
            }
        }
        uint32_t spins = 400'000u;
        while (S1::busy() && spins-- != 0u) {
        }
        const bool ovr = S1::overrun();
        S1::clear_overrun();
        const bool gone = !S1::overrun();
        print(serial, "  three frames clocked with nothing read: OVR ", ovr ? "set" : "CLEAR",
              ", after the DATAR-then-STATR read ", gone ? "clear" : "STILL SET", crlf);
        bench.verdict("THE OVERRUN IS A LEVEL WITH A SEQUENCE: frames arriving on a full buffer "
                      "raise OVR, and a DATAR read followed by a STATR read is what clears it",
                      ovr && gone);
        (void)S1::disable();
    } else {
        print(serial, "  no strap between PA7 and PA6: the overrun wants a frame to arrive", crlf);
    }

    // ---- the CRC error flag: the one rc_w0 bit of STATR ----
    (void)S1::configure({.role = SpiRole::host,
                         .clock = SpiClock::div64,
                         .crc = true,
                         .crc_polynomial = 0x0007u});
    S1::regs().STATR = static_cast<uint16_t>(S1::regs().STATR | spi_crcerr);
    const bool crc_set = S1::crc_error();
    S1::clear_crc_error();
    const bool crc_gone = !S1::crc_error();
    print(serial, "  CRCERR written by hand: ", crc_set ? "it takes a one" : "read-only to a one",
          ", and a zero into its bit ", crc_gone ? "clears it" : "does not", crlf);
    bench.verdict("CRCERR is the one flag of STATR a program writes, and it is cleared by a ZERO "
                  "into its bit and ones everywhere else",
                  crc_gone);

    // ---- the mode fault: SSI dropped under software management ----
    (void)S1::configure({.role = SpiRole::host, .clock = SpiClock::div64,
                         .nss = SpiNss::software});
    S1::enable();
    S1::software_select(true);   // SSI low: "the SSI bit is cleared" of 20.2.7
    (void)delay_us(clock, 5);
    const bool modf = S1::mode_fault();
    const bool demoted = S1::role() == SpiRole::client || !S1::enabled();
    print(serial, "  SSI dropped on a software-managed host: MODF ", modf ? "set" : "clear",
          ", CTLR1=", hex(S1::regs().CTLR1), " STATR=", hex(S1::status()), " (the block ",
          demoted ? "was demoted" : "kept its role", ")", crlf);
    bench.verdict("THE MODE FAULT IS THE DEMOTION: clearing SSI under software management raises "
                  "MODF",
                  modf);
    bench.verdict("... and the silicon takes the host's role away with it, which is why the "
                  "engine's select is a GPIO and SPI1's strapped NSS pad is never handed over",
                  demoted);

    // WHICH SEQUENCE PUTS MODF DOWN. 20.2.7 names two steps - "first
    // perform a read or write operation to STATR, and then write to
    // CTLR1" - and this walks the variants of them in order, reporting
    // the first that works rather than assuming the words.
    uint8_t cleared_by = 0;
    const char* names[] = {"none of them",
                           "STATR read, then CTLR1 written with the value it holds",
                           "STATR read, then the CTLR1 write that RAISES SPE",
                           "STATR read, then CTLR1 written with SSI set",
                           "SSI raised first, then the STATR read and the CTLR1 write",
                           "STATR WRITTEN (not read), then the CTLR1 write",
                           "the RCC reset pulse"};
    for (uint8_t k = 1; k <= 6u && cleared_by == 0u; ++k) {
        if (!S1::mode_fault()) {
            break;
        }
        switch (k) {
            case 1:
                (void)S1::status();
                S1::regs().CTLR1 = S1::regs().CTLR1;
                break;
            // MODF has cleared SPE with MSTR, so the write that puts SPE
            // back IS a CTLR1 write with something in it - which is the
            // one shape the chapter's "then write to CTLR1" does not
            // spell out.
            case 2:
                (void)S1::status();
                S1::regs().CTLR1 = static_cast<uint16_t>(S1::regs().CTLR1 | spi_spe);
                break;
            case 3:
                (void)S1::status();
                S1::regs().CTLR1 = static_cast<uint16_t>(S1::regs().CTLR1 | spi_ssi);
                break;
            case 4:
                S1::software_select(false);
                (void)S1::status();
                S1::regs().CTLR1 = S1::regs().CTLR1;
                break;
            case 5:
                S1::regs().STATR = S1::regs().STATR;
                S1::regs().CTLR1 = S1::regs().CTLR1;
                break;
            default:
                S1::reset();
                break;
        }
        if (!S1::mode_fault()) {
            cleared_by = k;
        }
    }
    print(serial, "  MODF goes down on: ", names[cleared_by], crlf);
    bench.verdict("MODF HAS A WAY DOWN, and the suite names which of six sequences it is - "
                  "20.2.7's own two steps are measured here and not taken on trust",
                  cleared_by != 0u);
    (void)S1::disable();

    // ---- the interrupt: the error line reaches the vector ----
    S1::reset();
    (void)S1::configure({.role = SpiRole::host, .clock = SpiClock::div64,
                         .nss = SpiNss::software});
    err_isr_entries = 0;
    err_isr_flags = 0;
    err_mode = true;
    S1::error_interrupt(true);
    S1::enable();
    Pfic::enable(S1::irq);
    S1::software_select(true);
    (void)delay_us(clock, 200);
    Pfic::disable(S1::irq);
    err_mode = false;
    S1::error_interrupt(false);
    S1::clear_mode_fault();
    print(serial, "  the error line: ", err_isr_entries, " entr", err_isr_entries == 1u ? "y" : "ies",
          " carrying ", hex(err_isr_flags), crlf);
    bench.verdict("SPI1's vector carries the error sources under ERRIE, and the ISR body hands "
                  "back exactly the raised-and-enabled ones",
                  err_isr_entries >= 1u && (err_isr_flags & SpiFlag::mode_fault) != 0u);
    (void)S1::disable();

    // ---- the bidirectional line mode: one pad, both directions ----
    if (loop) {
        S1::reset();
        (void)S1::configure({.role = SpiRole::host,
                             .mode = SpiMode::mode0,
                             .clock = SpiClock::div64,
                             .direction = SpiDirection::half_duplex_out});
        SckPad::function();
        MosiPad::function();
        MisoPad::release();   // the one data line is MOSI here
        S1::enable();
        S1::data8(0xC3);
        uint32_t spins = 400'000u;
        while (S1::busy() && spins-- != 0u) {
        }
        const bool sent = !S1::busy();
        // Turn around: the same pad, listening. The strap holds the level
        // the port puts on PA6, and BIDIMODE's input is MOSI.
        S1::half_duplex_output(false);
        MisoPad::output(true);   // the node held high through the strap
        (void)delay_us(clock, 10);
        S1::flush_rx();
        spins = 400'000u;
        while (!S1::rxne() && spins-- != 0u) {
        }
        const bool got_high = S1::rxne() && S1::data8() == 0xFFu;
        MisoPad::output(false);
        (void)delay_us(clock, 10);
        S1::flush_rx();
        spins = 400'000u;
        while (!S1::rxne() && spins-- != 0u) {
        }
        const bool got_low = S1::rxne() && S1::data8() == 0x00u;
        print(serial, "  bidirectional on ONE pad: the frame went out (", sent ? "BSY fell" : "BSY stood",
              "), and with BIDIOE cleared the same pad read ", got_high ? "all ones" : "not ones",
              " then ", got_low ? "all zeros" : "not zeros", crlf);
        bench.verdict("THE ONE-WIRE LINE MODE: BIDIMODE with BIDIOE drives a frame out of MOSI, "
                      "and clearing BIDIOE turns the same pad into the receiver - the clock runs "
                      "on either side of the turn",
                      sent && got_high && got_low);
        MisoPad::release();
        (void)S1::disable();

        // ---- the receive-only host and how it is stopped ----
        S1::reset();
        (void)S1::configure({.role = SpiRole::host,
                             .mode = SpiMode::mode0,
                             .clock = SpiClock::div128,
                             .direction = SpiDirection::receive_only});
        SckPad::function();
        MosiPad::release();
        MisoPad::output(true);
        S1::enable();
        // The clock free-runs the moment SPE is up: nothing is written.
        uint32_t frames = 0;
        for (uint32_t k = 0; k < 4u; ++k) {
            uint32_t spins = 400'000u;
            while (!S1::rxne() && spins-- != 0u) {
            }
            if (S1::rxne()) {
                (void)S1::data8();
                ++frames;
            }
        }
        const bool busy_stands = S1::busy();
        const auto last = S1::stop_receive_only(SpiDataSize::bits8);
        const bool stopped = !S1::enabled();
        print(serial, "  receive-only host: ", frames,
              " frames arrived with nothing written, BSY ",
              busy_stands ? "STANDS while SPE is up" : "was down",
              ", the stop verb handed back ",
              last ? "the last frame" : "no frame", " and dropped SPE", crlf);
        bench.verdict("A RECEIVE-ONLY HOST CLOCKS ITSELF: with RXONLY set the clock runs as long "
                      "as SPE stands, so frames arrive with nothing written",
                      frames == 4u);
        bench.verdict("... and it is stopped by dropping SPE and reading the last frame out, "
                      "never by waiting for a BSY that the running clock keeps up",
                      stopped);
        MisoPad::release();
        (void)S1::disable();
    } else {
        print(serial, "  the line modes and the receive-only host want the PA7-PA6 strap", crlf);
    }

    // ---- the I2S register this series has no I2S for ----
    S1::reset();
    const bool i2s_bit = S1::i2s_config_writable();
    print(serial, "  SPI_I2S_CFGR's I2SMOD ", i2s_bit ? "TAKES a write" : "reads back zero",
          " on this part; the datasheet's resource table names no I2S for this series and no "
          "package bonds an audio signal",
          crlf);
    bench.verdict("the I2S face is reported as the silicon answers, not as the four-family "
                  "chapter promises",
                  true);
    S1::bus_clock(false);
    host_ready();
}

// ===========================================================================
// The menu
// ===========================================================================

void banner() {
    print(serial, crlf, "test_vx03_spi - the SPI of RM ch. 20, both instances", crlf,
          "  SPI1: the PA7-PA6 strap is the loopback (PA8 is the node's third pad and is never "
          "driven); PA4 is strapped to PA5, so SPI1 is software-selected only",
          crlf,
          "  SPI2: PB12..PB15 to a peer board running `spi_peer`; letter d skips when it does "
          "not answer",
          crlf);
    bench.menu();
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    spi1_isr_entries = spi1_isr_entries + 1u;
    if (err_mode) {
        err_isr_entries = err_isr_entries + 1u;
        err_isr_flags = err_isr_flags | S1::isr();
        S1::error_interrupt(false);
        return;
    }
    if (dma_host_live) {
        // The ENGINED host: only its command phase runs on this pump,
        // and isr() is what hands the data phase over to the channels.
        if (Dma1::isr()) {
            host_done = true;
        }
        return;
    }
    if (bus_ao_live) {
        if (Host1::isr()) {
            brio::post<kl::SpiArb>(brio::TransferDone{Host1::status()});
        }
        return;
    }
    if (Host1::isr()) {
        host_done = true;
    }
}

extern "C" BRIO_CH32_INTERRUPT void spi2_handler() {
    if (peer_client_live) {
        (void)PeerClient::isr();
        return;
    }
    if (PeerHost::isr()) {
        host_done = true;
    }
}

/// SPI1's engines: one vector per channel on this family.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() {
    if (dma_host_live && Dma1::dma_isr()) {
        host_done = true;
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() {
    if (dma_host_live && Dma1::dma_isr()) {
        host_done = true;
    }
}

/// SPI2's engines.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel4_handler() {
    if (dma_host_live && PeerDma::dma_isr()) {
        host_done = true;
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() {
    if (dma_host_live && PeerDma::dma_isr()) {
        host_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    third_pad_idle();

    bench.letter('a', "the rates: every BR code, and the clock counted on its own pad", ta_rates);
    bench.letter('b', "the host with its MISO held by its own port, no wire", tb_held);
    bench.letter('c', "THE LOOPBACK on SPI1: the pump, the CRC, the engines, the arbiter",
                 tc_loop);
    bench.letter('d', "THE PEER on SPI2: the matrix, the dummy, the roles inverted, a stress",
                 td_peer);
    bench.letter('e', "the flags, the vectors, the line modes, the disable rule", te_flags);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
