// test_x035_rubato_i2c - the bench suite of the CH32X035's I2C stack under
// Rubato, the preemptive kernel (kernel/rubato.hpp): util/i2c_bus.hpp's
// arbiter over ch32x035/i2c.hpp's I2cHost and its DMA engines, run against
// the desk's CH32V006K8U6 and its unchanged instrument
// (ch32v00x/src/apps/twi_peer.cpp) while an object of the highest priority
// preempts it at moments the traffic does not choose. Every verdict is the
// traffic judged byte-exact and every status the one the wire produced, as
// the Tenuto suite's peer letters (test_x035_i2c) judge them; nothing here
// changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests: the program is one Rubato pack of
// five objects - Pressure, Client, I2cArb, Minor and Console, from the
// highest priority down - and everything of it that names no register, the
// pack, the passes, the command channel and the letters, is the I2C suites'
// shared body (avrxt/src/apps/rubato_i2c.hpp, whose header describes each
// object, each letter and where preemption can reach a tenure). What is
// this chip's is here, each choice from the reference manual's chapters
// (RM 15 the I2C, RM 9 the DMA, RM 12 the timer) and the driver header:
//
//   THE BLOCK FIRST. The datasheet withdraws the I2C from some lots (table
//   2-1's note 3) and no register names a die's lot, so main() asks
//   I2c<1>::present() before Host::init() - which refuses a block that does
//   not hold its timing as well - and a die without the block registers no
//   letter and says so.
//   THE HOST. I2cHost<1> on its column 010, SCL PC16 and SDA PC17 (the
//   F8's one column off the debug port), with the DMA engines on the
//   channels that ARE its requests (figure 9-1: 6 transmits, 7 receives),
//   both at high, in the default placement (InFlash: the InRam pump is the
//   Tenuto suite's letter x, not this key's subject). Its engine, from the
//   chapter (ch32x035/i2c.hpp's class comment, docs/ch32x035/i2c.md): the
//   STM32F1's event machine on two vectors, no RTR, no byte counter, no
//   FIFO; the first two bytes of a pumped write loaded in the entry that
//   clears ADDR; a write's end at BTF; the repeated START of a
//   write-then-read requested on the last written byte's TxE; a refused
//   last byte closed by a void write from the repeated START's SB; a plain
//   write of three bytes or more and a read of two or more on the engines
//   (the write ending on BTF with the channel's count at zero and no DMA
//   interrupt, the read on channel 7's completion under LAST), a one-byte
//   read and the write half of a write-then-read on the pump (EVT6_1 has no
//   DMA form; the repeated START wants the last byte's TxE). start() waits,
//   bounded, for the last STOP to leave CTLR1 (stop_drain_us 5 ms) and for
//   BUSY (bus_free_us 100 us), each a count of delay_us(1) turns, takes
//   SWRST for a BUSY standing over an idle wire (unwedges()), and stores
//   START last: nothing is on the wire before that store, so a preemption
//   inside start() is a later START - a gap between tenures - or a wait it
//   lands in lasting longer, never shorter; inside a tenure the master
//   holds SCL at SB, at ADDR until its two reads and at BTF (15.3, 15.6)
//   while a vector is late.
//   THE PREDICTION: strict at every speed, path and pass; unwedges() and the
//   engines' faults printed (counts, not verdicts); letter x's landings
//   inside start() find MSL clear - the START store is start()'s last
//   register write, and MSL rises only at SB, some tens of microseconds on.
//   THE HELD WIRE: a START into an SDA the peer holds, from a block that saw
//   the line taken, goes out and its address loses at its first one bit -
//   i2c_arb_lost at once (docs/ch32x035/i2c.md, "A START into a held SDA is
//   not refused"); from a block brought up over the held line (SWRST:
//   recover(); a PE cycle) the START stays pending and only the arbiter's
//   20 ms answers (i2c_timeout). Letter h meets the hold at the speed in
//   force (the first signature predicted) and once at another, whose
//   timing write cycles PE before the START (the second predicted); the
//   requirement judged either way.
//   THE RELEASE CONTRACT (docs/ch32x035/dma.md, "A released requester"):
//   channels 6 and 7 are ORs this block shares with USART2 - the console,
//   engine-free here, its DMA enables off - with TIM1's channel 3 (TIM1
//   never opened) and with TIM2's channels 2 and 4 (the pressure's timer,
//   every DMA enable of DMAINTENR clear): letter i reads every one of them
//   so, and both channels with nothing left to move.
//   THE PRESSURE. TIM2's update interrupt, 16 bits on HCLK = 48 MHz at
//   prescaler 0: 97 / 193 / 389 us are 4656 / 9264 / 18672 counts. Its
//   dispatch, a 3000-cycle spin, road 1 (180 to 182 cycles with the
//   scheduler in SRAM, 372 from the flash: docs/ch32x035/platform.md)
//   and the way back, is shorter than the shortest period's 4656 cycles,
//   so no tick finds the last one still queued; at 97 us x 3000 it takes
//   64 % of the core.
//   THE RULER. The kernel ticker's composed cycles() (the tick count and
//   the STK counter's position, ch32x035/ticker.hpp), monotone across a
//   tick; spin() reads the STK counter bare, one load, the restart at the
//   compare folded in. The pressure's timer and the STK count one clock,
//   HCLK, so judge_pressure()'s arithmetic is exact.
//   THE PLACEMENT. This image runs the scheduler path from SRAM, the
//   project's default (`// build: scheduler` absent):
//   test_x035_rubato_i2c_flash is the same source with the path in the
//   flash. What the release map shows the compiler carried into SRAM with
//   activate() is the platform page's (docs/ch32x035/platform.md).
//   THE SUPPLY AND THE RAIL at boot: the voltage detector at 3.0 V
//   (pwr.hpp), then PB3 driven high push-pull - the bus's pull-ups - from
//   there to the image's end; under 3.0 V PB3 and the bus's pads stay
//   floating, no letter is registered and the banner says so.
//
// THE PEER, unchanged: twi_peer on the CH32V006K8U6 module, kind 0x06, its
// I2C1 on column 3 (PB5 SCL, PB6 SDA) a POLLED target that stretches one
// byte late (its header: the next byte enters the shifter while the last
// still sits in DATAR), its commanded hold spent at the address match and
// before every byte it takes or gives. It ends a counted serve once the
// count has moved and the wire has rested forty microseconds, never at the
// byte that reaches the count; it gives back the byte a closing NACK leaves
// in its DATAR (the count is the wire's). After a refused byte it keeps its
// acknowledge down until its action's window closes (the CH32 target's
// habit, docs/ch32x035/i2c.md): I2cTraits::refusal_holds_address below.
// Speeds: 100 and 400 kHz, the two of RM 15.1.
//
// THE WIRES, fixed on the desk (test_x035_i2c's pair, docs/boards/
// ch32x035-evt-f8u6.md, docs/boards/ch32v006k8.md): this board's PC16
// (SCL) to the CH32V006's PB5, PC17 (SDA) to its PB6, a 1.5 kOhm resistor
// from each line to this board's PB3 - NEVER to 3.3 V: PC17 high at
// power-up enters WCH's loader - and a ground. NOTHING LIT: no letter
// drives the LED's pad PA0.
//
// The letters (z: n o p q w h i; x and u outside it), each but i, x and u
// run as seven passes (the shared body's control and six pressures):
//   n  the command channel through the arbiter at 100 kHz: a ping, ident
//      (the sanity byte and the CH32V006's kind, 0x06), ten pings, every
//      tenure i2c_ok.
//   o  the tenure shapes at 400 kHz on both of the host's paths (I2cTraits
//      below names each shape's), one counted serve a pass: every status
//      i2c_ok, every read byte-exact, the peer's count, sum and address
//      matches exact.
//   p  100 and 400 kHz: a read of 16, a 1+4 and a write of 16 byte-exact,
//      the wire's time printed; the peer's commanded stretch, 1000 us a
//      byte at 100 kHz, priced from below by its holds (I2cTraits).
//   q  the vocabulary: a deaf client's i2c_nack_addr; the third of six
//      refused (i2c_nack_data, the write on the engines); the last written
//      byte of a 2+4 refused (i2c_nack_data, the controller let go by its
//      void write, the read half never begun); inside each refusing window
//      the probe refused at the address (this peer still deaf from its
//      refusal), after the deadline the next tenure i2c_ok, and the peer's
//      address count the one that implies.
//   w  three requesters of three priorities at 400 kHz: the Client's
//      write-then-reads of 1+6 (the read half on the engines), Minor's
//      writes of two (the pump: under the engine's three), Pressure's
//      probes: in order, i2c_ok, nothing rejected, no timeout, the peer's
//      tallies exact.
//   h  the held wire (the peer holds SDA 60 ms) with Pressure's flood of
//      probes, paced by the Console's turns: answered in its place, every
//      request once, the next tenure i2c_ok and the arbiter idle; then the
//      hold met at another speed (above). A flood at every tick regardless
//      asks more of this core at 97 us x 3000 than it serves - a rejected
//      probe costs some 1600 cycles beside Pressure's 3000 a period - and
//      starves the Console, which ends the flood and walks the time events:
//      letter u runs it so, by name.
//   i  dark: every object idle, the arbiter's vote, no time event armed,
//      TIM2 stopped with its line quiet and its DMA requests off, the host
//      idle with its enables at rest, its channels with nothing to move and
//      no request held on their ORs, BUSY and MSL clear, both lines high,
//      the rail up, the ceiling the Console's.
//   x  (outside z) 400 writes of 16 silent and 400 under 97 us x 3000:
//      the landings inside start() counted, and of them those with MSL set.
//   u  (outside z) letter h's held wire at 97 us x 3000 with the flood
//      unpaced - a probe at every tick, the pressure's vector marking the
//      turn letter h waits for - for 2000 ticks, after which the vector
//      stops the timer itself, since the Console the overload starves would
//      otherwise never end it: the free stack painted again first and read
//      after, its high-water against the preemptive kernel's bound of n runs
//      for a pack of n (the arithmetic beside the letter); every probe
//      answered at most once and the unanswered ones within what the
//      arbiter's full queue dropped; the next tenure i2c_ok and the arbiter
//      idle once the flood is over. The held tenure's own answer is printed
//      and not judged: the overload starves the time events its bound
//      rests on.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include <iterator>
#include <variant>

#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/i2c.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/schedule.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/tim.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/panic.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's instrument
// compiles twi_link.hpp too, and the controller's end of it is one reviewed
// file.
#include "../../../avrxt/src/apps/twi_link_client.hpp"

/// The one clock root, the HSI undivided: HCLK = 48 MHz, every bus's. FREQ
/// is 48 and CCR exact at both speeds (240 and, at DUTY 2, 40).
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Ch32x035RubatoPlatform<>;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

/// The composed cycle count: monotone across the ticker's restart, a
/// difference exact under 2^32 cycles (89 s at 48 MHz).
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
};

/// The STK counter's period: it counts HCLK and restarts at the ticker's
/// compare every millisecond.
constexpr uint32_t stk_period = SysClock::hz / Ticker::ticks_per_second;

/// A busy wait of `cycles` HCLK cycles: the STK counter read bare, one
/// load a turn, the restart folded in, for a span under half a period; the
/// composed count for a longer one. A preemption longer than a period only
/// lengthens a bare spin.
void spin(uint32_t cycles) {
    if (cycles < stk_period / 2u) {
        const uint32_t t0 = stk()->CNTL;
        for (;;) {
            const uint32_t t = stk()->CNTL;
            const uint32_t d = t >= t0 ? t - t0 : t + stk_period - t0;
            if (d >= cycles) {
                return;
            }
        }
    }
    const uint32_t t0 = Ticker::cycles();
    while (Ticker::cycles() - t0 < cycles) {
    }
}

// ---- the console -------------------------------------------------------------------

/// USART2 on PA2/PA3, the probe's serial, ENGINE-FREE: its requests are
/// ORed into channels 6 and 7, the I2C's (figure 9-1).
using Serial = Uart<2, P, 64, 128>;
constexpr Serial serial;
TestBench<Serial> bench;
using ConsoleUsart = Usart<2>;

// ---- the I2C host --------------------------------------------------------------------

using H = I2c<1>;
using TxEngine = DmaTxEngine<6>;
using RxEngine = DmaRxEngine<7>;
using Host = I2cHost<1, i2c1_default_pins, TxEngine, RxEngine>;
using TxChannel = DmaChannel<6>;
using RxChannel = DmaChannel<7>;
using SclPin = Pin<'C', 16>;
using SdaPin = Pin<'C', 17>;
/// The bus's pull-ups hang from this pad (the file header).
using Rail = Pin<'B', 3>;
static_assert(i2c1_default_pins.remap == 2u && i2c1_default_pins.scl == SclPin::pad &&
                  i2c1_default_pins.sda == SdaPin::pad,
              "the CH32X035F8's one column off the debug port is I2C1_RM 010, PC16/PC17");

constexpr uint8_t peer_kind = twilink::peer_ch32v00x;
/// RM 15.1's two speeds, what test_x035_i2c judges against this peer.
constexpr I2cSpeed speeds[] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k};
constexpr uint32_t speed_hz(I2cSpeed s) { return i2c_speed_hz(s); }

/// The boot's results, printed by the banner and read by bring_up().
bool clock_ok = false;
bool tick_ok = false;
bool supply_ok = false;
bool present_ok = false;
bool host_ok = false;
/// What the last reset left: RCC_RSTSCKR's flags (3.4.8), cleared once
/// read, and a panic's breadcrumb (kernel/panic.hpp) - none of either
/// after a jump to the reset vector.
uint32_t boot_flags = 0;
std::optional<PanicRecord> boot_panic;

/// A pad's MODE bits (its nibble's low two): 00 is an input.
template <typename Pad>
bool pad_is_output() {
    return (Pad::nibble() & 0x3u) != 0u;
}

/// What the shared body changes for this host and this peer.
struct Shape {
    uint8_t tx_len;
    uint8_t rx_len;
    const char* path;
};
struct I2cTraits {
    /// Letter o's table, each shape with the path this host takes: a plain
    /// write of three bytes or more and a read of two or more on the
    /// engines (the driver's dma_min_write, 3, private: the entry that
    /// clears ADDR loads two bytes with no interrupt, where an engine adds
    /// its block start; dma_serves_rx() from two), a one-byte read and a
    /// write half before a read on the pump. Ends on a plain write (the
    /// shared body's counted serve).
    static constexpr Shape shapes[] = {
        {0, 0, "the probe: SB, ADDR, the STOP in the ADDR entry"},
        {0, 1, "pump: EVT6_1, the STOP at ADDR, the byte at RxNE"},
        {0, 2, "engines: channel 7 under LAST, its completion the end"},
        {0, 3, "engines"},
        {0, 8, "engines"},
        {0, 32, "engines"},
        {1, 1, "pump both halves: the byte in the ADDR entry, Sr on its TxE, EVT6_1"},
        {1, 2, "pump write (Sr on the last byte's TxE), engine read"},
        {2, 8, "pump write, engine read"},
        {1, 0, "pump: the byte in the ADDR entry, BTF the end"},
        {2, 0, "pump: both bytes in the ADDR entry, BTF the end"},
        {3, 0, "engines: channel 6, BTF with its count at zero, no DMA interrupt"},
        {8, 0, "engines"},
        {32, 0, "engines"},
    };
    static_assert(Host::has_engines, "the table names engine paths");
    /// The CH32V006 peer's holds inside the stretched tenures (its
    /// run_serve): one at the address match and one before each byte it
    /// takes or gives. A read of 8: the address and all eight gives fall
    /// inside the tenure - the eighth is given at the end of the ninth
    /// hold and must reach the wire before the controller's NACK - nine
    /// holds. A write of 8: the peer is one byte late, so the hold before
    /// the take of byte k overlaps byte k + 1 entering its shifter; the
    /// tenure ends once byte 8 is acknowledged, which needs DATAR emptied
    /// of byte 6, the take after the seventh hold: the address and six,
    /// seven holds (test_x035_i2c's letter e measures seven at 2 ms). The
    /// wire adds at least the address byte before the first hold, 90 us at
    /// 100 kHz, more than this end's HSI, some 3000 ppm slow
    /// (docs/ch32x035/clock.md), takes off seven or nine milliseconds of
    /// the peer's crystal (27 us at most), so the bound is the holds alone.
    static constexpr uint8_t stretch_write_holds = 7;
    static constexpr uint8_t stretch_read_holds = 9;
    /// A refused LAST written byte of a write-then-read is closed by a void
    /// write from the repeated START's SB (the driver's close_refused()):
    /// one address more, which this peer, deaf after a refused byte, does
    /// not take.
    static constexpr uint8_t refused_close_hits = 1;
    /// The CH32V006 keeps its acknowledge down after a refused byte (its
    /// arm_ack(): ACK goes back up only at a Stop its loop sees, and a
    /// refused byte's Stop is not one it sees) until its action's window
    /// closes, so it refuses its own address there too - the Tenuto suite's
    /// letter r judges the same.
    static constexpr bool refusal_holds_address = true;
    /// The bus's rail and the block, before the first tenure of a letter:
    /// the supply above 3.0 V at boot, the block present and up, PB3 an
    /// output reading high.
    static bool bring_up() { return supply_ok && present_ok && host_ok && pad_is_output<Rail>() && Rail::read(); }
};

// ---- the pressure's clock ----------------------------------------------------------

using Pulse = Tim<2>;
static_assert(tim_clock_hz(SysClock{}) == SysClock::hz, "TIM2 counts HCLK at this clock");
constexpr uint32_t pulse_per_us = SysClock::hz / 1'000'000u;
static_assert(389u * pulse_per_us - 1u <= 0xFFFFu, "the longest period fits TIM2's 16 bits");
constexpr const char* pressure_name = "TIM2";
constexpr const char* peer_name = "the CH32V006K8U6";

// THE TARGET-FREE HALF IS SHARED, NOT COPIED: every family's I2C suite
// compiles the same body around its own host.
#include "../../../avrxt/src/apps/rubato_i2c.hpp"

// =============================================================================
// What the shared body asks of this family
// =============================================================================

void pressure_arm(uint16_t period_us) {
    Pulse::enable(false);
    Pulse::set_count(0);
    (void)Pulse::set_period(static_cast<uint16_t>(static_cast<uint32_t>(period_us) * pulse_per_us - 1u));
    Pulse::clear_flags(Pulse::update_flag);
    Pulse::interrupts(Pulse::update_interrupt, true);
}
void pressure_go() { Pulse::enable(true); }
void pressure_halt() { Pulse::enable(false); }
void pressure_quiet() {
    Pulse::interrupts(Pulse::update_interrupt, false);
    Pulse::clear_flags(Pulse::update_flag);
    Pfic::clear_pending(Pulse::update_irq());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

/// STAR2.MSL: the block is the bus's master, from SB to the STOP
/// (15.9.7). A STAR2 read completes ADDR's clearing sequence after a STAR1
/// read, but this is read only inside start(), where no ADDR can stand:
/// the last tenure ended and the next one's address is not out yet.
bool host_on_wire() { return H::host(); }

/// DMAINTENR's DMA request enables, bits 8 to 14: UDE, CC1DE..CC4DE, COMDE,
/// TDE (12.4.4) - CC2DE and CC4DE are channel 7's (figure 9-1).
constexpr uint16_t tim_dma_enables = 0x7F00u;

/// A channel with nothing left to move: disabled, or its count at zero
/// (the engines leave EN standing after a completed block, the next
/// start() clears it - dma.hpp).
template <typename Ch>
bool channel_at_rest() {
    return !Ch::enabled() || Ch::count() == 0u;
}

const char* host_dark() {
    if (Pulse::enabled() || (Pulse::interrupts() & Pulse::update_interrupt) != 0u ||
        (Pulse::interrupts() & tim_dma_enables) != 0u || Pfic::pending(Pulse::update_irq())) {
        return "TIM2 running, its update interrupt or a DMA request enabled, or its line pending";
    }
    if (Host::busy()) {
        return "the host's engine is not between tenures";
    }
    // finish() puts CTLR2 back to the error vector alone in one store, the
    // event and buffer vectors and DMAEN and LAST with it (the class
    // comment).
    const uint16_t c2 = H::regs().CTLR2;
    if ((c2 & (i2c_itevten | i2c_itbufen | i2c_dmaen | i2c_last)) != 0u || (c2 & i2c_iterren) == 0u) {
        return "the host's enables are not at rest (the error vector alone, DMAEN and LAST clear)";
    }
    if (TxEngine::busy() || !channel_at_rest<TxChannel>() || !channel_at_rest<RxChannel>()) {
        return "a DMA channel of the host's still has items to move";
    }
    // The release contract: no request held on channels 6 and 7's ORs.
    if ((ConsoleUsart::regs().CTLR3 & (usart_dmat | usart_dmar)) != 0u) {
        return "the console's DMA requests are enabled (USART2 shares channels 6 and 7)";
    }
    if (Rcc::enabled(Tim<1>::bus, Tim<1>::gate)) {
        return "TIM1's gate is open (its channel 3 shares channel 6)";
    }
    if (H::busy() || H::host()) {
        return "the bus monitor reads BUSY or MSL";
    }
    if (!SclPin::read() || !SdaPin::read()) {
        return "a line reads low on its pad";
    }
    if (!pad_is_output<Rail>() || !Rail::read()) {
        return "the rail PB3 is not driven high";
    }
    return nullptr;
}

void print_host() {
    print(serial, "; the arbiter's queue overflows ", I2cArb::queue.overflows());
    print(serial, "; unwedges ", Host::unwedges(), ", transmit tails abandoned ", TxEngine::faults(),
          ", receive faults ", RxEngine::faults(), " (since boot); stack never touched ", stack_untouched(), " B");
}

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=", tick_ok ? "STK" : "FAILED",
          " ruler=STK supply=", supply_ok ? "above 3.0 V" : "LOW", " i2c1=",
          !supply_ok ? "not tried" : (!present_ok ? "ABSENT (the lot note)" : (host_ok ? "host" : "FAILED")),
          " rail=", pad_is_output<Rail>() ? "PB3 high" : "floating", " scheduler=", scheduler_in_ram ? "SRAM" : "flash",
          crlf);
    print(serial, "reset: RSTSCKR flags ", hex(boot_flags), ", panic record ");
    if (boot_panic.has_value()) {
        print(serial, "code ", boot_panic->code, " context ", boot_panic->context, crlf);
    } else {
        print(serial, "none", crlf);
    }
    if (!supply_ok) {
        print(serial, "SUPPLY UNDER 3.0 V (the PVD at PLS 10): no letter registered, PB3 and the bus pads left "
                      "floating",
              crlf);
    } else if (!present_ok || !host_ok) {
        print(serial, "NO I2C BLOCK UP: no letter registered", crlf);
    }
    print(serial, "test_x035_rubato_i2c - the I2C stack under Rubato on the ", device::part_name, " against ",
          peer_name, " (twi_peer)", crlf);
    bench.menu();
}

// =============================================================================
// u - the held wire under an unpaced flood (by name, outside z)
// =============================================================================

/// Letter u's flood: while set, the pressure vector marks a turn before
/// every Tick it posts, so Pressure probes at every tick (the pacing of
/// letter h undone), and after `flood_n` ticks the vector stops the timer
/// itself - the overload starves the Console, which would end the flood
/// otherwise. Written by the letter, read and cleared by the vector.
volatile bool unpaced = false;
volatile uint32_t flood_ticks = 0;
/// 2000 ticks of 97 us, 194 ms: the hold's 60 ms three times over.
constexpr uint32_t flood_n = 2000;

/// The free stack below the caller's frame painted again with the crt's
/// pattern, from the floor to 64 bytes under the stack pointer (a handler
/// landing during the paint pushes nothing deeper than that; one that did
/// would count as used, the safe side). Main context.
void repaint_stack() {
    uint32_t sp;
    __asm__ volatile("mv %0, sp" : "=r"(sp));
    for (volatile uint32_t* p = __stack_floor; reinterpret_cast<uint32_t>(p) + 64u < sp; ++p) {
        *p = 0xA5A5A5A5UL;
    }
}

/// THE BOUND on the stack this letter may reach, by the release listing's
/// frames (-fstack-usage, the same in both placements) and the preemptive
/// kernel's count of runs (kernel/rubato.hpp, "The stack": a pack of n holds
/// at most n runs, each but the topmost suspended in a dispatch of a
/// strictly lower index, plus one empty run at an entry on this core):
///   main(), with run() in it                                          64
///   the base run's activate()                                        112
///   the Console's dispatch to this letter (TestBench::handle 48, run 16)
///   and the letter's deepest chain - its own frame 192, held_tenure() 80,
///   TwiLinkClient::command() 96, recv_frame() 80, arb_tenure() 64 and
///   the post to the Client 32 that asks for a run                    608
///   four more runs, each the software interrupt's block (16), the
///   entry's sixteen registers (64) and activate()'s frame (112) -
///   the Console's by-hand base turn (wait_turn() 16 and activate() 112)
///   is the smaller way to start one                                  768
///   the four dispatches above the Console in progress, at most one of
///   each object: Minor's 80 (its post_next() 48 and the post 32), the
///   arbiter's 176 (Fsm::dispatch() 16, follow() 32, a state handler
///   128 - busy() 32, begin_waiting() 48, Marked::start() 16,
///   I2cHost::start() 32 - its replies' thunks and posts 0), the
///   Client's 80 (post_next_w() 48 and the post 32), Pressure's 32 (its
///   post 32, or its spin 16)                                         368
///   one empty run at an entry: a block, an entry and an activate()
///   frame, counted over everything above                             192
///   an interrupt handler's own frame, the deepest i2c1_er's (the
///   hardware prologue pushes to the core's own stack)                  32
///                                                                     ----
///                                                                     2144
/// The old kernel opened a level of 192 bytes a tick under this flood
/// (26 levels, past 5 KB, before it crashed: docs/ch32x035/platform.md).
constexpr uint32_t unpaced_stack_bound = 2144;

void tu_unpaced_flood() {
    constexpr PassSpec s{97, 3000};
    begin(s);
    if (!need_peer()) {
        (void)pass_end(s);
        return;
    }
    repaint_stack();
    const uint16_t arb_drops0 = I2cArb::queue.overflows();
    const uint16_t press_drops0 = Pressure::queue.overflows();
    flood_ticks = 0;
    unpaced = true;
    const HeldOutcome o = held_tenure(speeds[0], true);
    unpaced = false;
    const PassStats st = pass_end(s);
    const uint32_t top = reinterpret_cast<uint32_t>(__stack_top);
    const uint32_t used = top - reinterpret_cast<uint32_t>(__stack_floor) - stack_untouched();
    const uint16_t arb_drops = static_cast<uint16_t>(I2cArb::queue.overflows() - arb_drops0);
    const uint16_t press_drops = static_cast<uint16_t>(Pressure::queue.overflows() - press_drops0);
    print_pass(s, st);
    print(serial, "    the unpaced flood: ", flood_ticks, " ticks, Pressure posted ", h::press.posted, ", answered ",
          h::press.replies, " (ok ", h::press.ok, ", rejected ", h::press.rejected, ", timeout ", h::press.timeouts,
          ", arb_lost ", h::press.arb_lost, ", nack_addr ", h::press.nack_addr, ", other ", h::press.other,
          "), dropped by the arbiter's full queue ", arb_drops, ", Pressure's queue dropped ", press_drops, crlf);
    print(serial, "    the held tenure: status ", o.status, status_name(o.status), " in ", us_of(o.from_start),
          " us from its start (not judged: an overload by design, no response time promised); next tenure ",
          o.next, ", the vote ", o.idle ? "idle" : "NOT IDLE", crlf);
    print(serial, "    the stack's high-water during the letter ", used, " bytes of ", top - reinterpret_cast<uint32_t>(__stack_floor),
          ", the bound ", unpaced_stack_bound, crlf);
    bench.verdict("the flood ran its ticks unpaced, a probe posted at each", flood_ticks == flood_n &&
                                                                                 h::press.posted > flood_n / 2u);
    // The arbiter answers every Request it takes (i2c_rejected when its
    // pending FIFO is full), so a probe goes unanswered only when its post
    // found the arbiter's queue full; that queue's count also takes the
    // other events dropped there (a TransferDone, a timeout's fire), so it
    // bounds the unanswered probes from above.
    bench.verdict("every probe answered at most once, and the unanswered ones within what the arbiter's full "
                  "queue dropped",
                  h::press.replies <= h::press.posted && h::press.posted - h::press.replies <= arb_drops &&
                      press_drops == 0u && h::press.other == 0u);
    bench.verdict("after the flood the next tenure i2c_ok, the arbiter idle with nothing pending (its own vote)",
                  o.next == i2c_ok && o.idle);
    bench.verdict("the stack's high-water within the bound of n runs (the arithmetic beside it)",
                  used <= unpaced_stack_bound);
}

} // namespace

// ---- target glue ------------------------------------------------------------

BRIO_CH32_VECTOR(usart2_handler) {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return<K>(); }

/// The pressure: TIM2's update, where it landed, and the post.
BRIO_CH32_VECTOR(tim2_up_handler) {
    Pulse::clear_flags(Pulse::update_flag);
    if (unpaced && h::live) {
        h::turned = true;   // letter u: a probe at every tick
        flood_ticks = flood_ticks + 1u;
        if (flood_ticks >= flood_n) {
            Pulse::enable(false);
            unpaced = false;
        }
    }
    pressure_landed();
}

// The host's two vectors (table 7-1's entries 30 and 31): every event of a
// tenure on the first, every error on the second, each ending a tenure
// with its status.
BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}
BRIO_CH32_VECTOR(i2c1_er_handler) {
    if (Host::error_isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

// The engines' two channels: a read block's completion on channel 7 ends
// its tenure, a transfer error on either ends it i2c_dma_fault; the
// transmit channel interrupts on an error alone (BTF ends a write).
[[gnu::always_inline]] inline void host_dma() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}
BRIO_CH32_VECTOR(dma1_channel6_handler) { host_dma(); }
BRIO_CH32_VECTOR(dma1_channel7_handler) { host_dma(); }

int main() {
    boot_flags = brio::Rcc::reset_flags();
    brio::Rcc::clear_reset_flags();
    boot_panic = brio::take_panic_record<P>();
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);

    // THE SUPPLY before any bus pad is driven (the desk's rule), the rail
    // only then, and the block asked before it is brought up.
    brio::Pwr::pvd(brio::PvdLevel::v3_0);
    (void)brio::delay_us(clock, 200);
    supply_ok = !brio::Pwr::supply_low();
    if (supply_ok) {
        Rail::output(true);
        present_ok = H::present();
        host_ok = present_ok && Host::init(clock);
    }
    if (host_ok) {
        Pulse::init();
        (void)Pulse::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFFu});
        brio::Pfic::enable(Pulse::update_irq());

        bench.letter('n', "the command channel through the arbiter", tn_command_channel);
        bench.letter('o', "the tenure shapes on both of the host's paths", to_shapes);
        bench.letter('p', "the speeds, and the peer's commanded stretch", tp_speeds);
        bench.letter('q', "the vocabulary under pressure: deaf, a refused byte, a refused last byte",
                     tq_vocabulary);
        bench.letter('w', "the arbiter with three requesters of three priorities", tw_arbiter);
        bench.letter('h', "the held wire, with requests arriving while it is held", th_held_wire);
        bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
        bench.letter('x', "the landings inside start() counted: 400 writes silent and pressed", tx_fill, false);
        bench.letter('u', "the held wire under an unpaced flood at 97 us x 3000, the stack's bound", tu_unpaced_flood,
                     false);
    }
    brio::scheduler_init();

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
