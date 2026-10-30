// test_vx03_rubato_i2c - the bench suite of the I2C stack of the CH32V203
// and CH32V303 stratum under Rubato, the preemptive kernel
// (kernel/rubato.hpp): util/i2c_bus.hpp's arbiter over ch32vx03/i2c.hpp's
// I2cHost and its DMA engines, run against the desk partner and its
// unchanged instrument (ch32vx03/src/apps/twi_peer.cpp) while an object of
// the highest priority preempts it at moments the traffic does not choose.
// ONE SOURCE, TWO IMAGES: the CH32V203C8T6 against the CH32V303VCT6 and the
// other way round, each board the other's peer, as test_vx03_i2c's peer
// letters are. Every verdict is the traffic judged byte-exact and every
// status the one the wire produced, as those Tenuto letters judge them;
// nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests: the program is one Rubato pack of
// five objects - Pressure, Client, I2cArb, Minor and Console, from the
// highest priority down - and everything of it that names no register, the
// pack, the passes, the command channel and the letters, is the I2C
// suites' shared body (avrxt/src/apps/rubato_i2c.hpp, whose header
// describes each object, each letter and where preemption can reach a
// tenure). What is this family's is here, each choice from the reference
// manual's chapters (RM 19 the I2C, RM 11 the DMA) and the driver header:
//
//   THE CLOCK IS 96 MHz, AND THAT IS THE I2C CHAPTER'S DOING. CTLR2.FREQ
//   states PB1 in whole megahertz and RM 19.12.2 confines it to 4..60 MHz,
//   so the 72 MHz PB1 of the family's 144 MHz has no legal timing: the
//   image runs HCLK at 96 MHz from the 8 MHz crystal (the far twi_peer's
//   clock too), PB1 at 48 MHz, CCR 240 and 40 - exactly 100 and 400 kHz
//   (test_vx03_i2c's clock, the crystal instead of the HSI).
//   THE HOST. I2C1 on its default column, PB6 (SCL) and PB7 (SDA), with the
//   DMA engines on its own two channels, DMA1's 6 (transmit) and 7
//   (receive): on this family the channel is the request (table 11-5).
//   Its engine (the driver's class comment, docs/ch32vx03/i2c.md): the F1
//   event machine with RTR, two vectors (events, errors), the four receive
//   procedures by count; the first two written bytes taken in the entry
//   that clears ADDR (EVT8_1: the data register and the shifter, TxE
//   polled sixteen STAR1 reads); a write phase of three bytes or more on
//   the transmit engine, its end BTF with the channel's count at zero; a
//   read phase of two or more on the receive engine under CTLR2.LAST, the
//   one-byte read on the pump; the repeated START requested on the last
//   written byte's TxE; a refused last byte closed with a void write; the
//   event vector off between tenures, the error vector on. start() runs in
//   the arbiter's dispatch and WAITS there, bounded, in delay_us(1) turns:
//   for the last STOP to leave CTLR1 (5 ms) and for BUSY (100 us, then
//   unwedge_busy(), SWRST where BUSY stands over an idle wire, counted by
//   unwedges()) - and its last act is the CTLR1 store that sets START. So a
//   preemption inside start() lengthens a wait it lands in (each turn is
//   timed on the STK, so a turn held off is a longer turn, never a shorter
//   bound) or moves the START: a gap between tenures. Inside a tenure the
//   machine holds SCL at SB, at ADDR until its two reads and at BTF (19.3)
//   while a vector is late. THE PREDICTION: strict at every speed, path and
//   pass; letter x's landings inside start() find MSL clear, because a START
//   on a free bus reaches the wire some 5 us after its store and start()
//   returns within a few instructions of it.
//   THE HELD WIRE (docs/ch32vx03/i2c.md, "A START on a busy bus waits for a
//   tick"): with SDA held low and SCL high the START set after start()'s
//   100 us BUSY wait leaves at the controller's next tick, some 85 us
//   later, into the held line and loses at its first one bit -
//   i2c_arb_lost, at once and by the silicon, which is the prediction for
//   letter h (the shared body judges the requirement either way). Measured
//   here, that is the FIRST START into a hold: a START set after one has
//   lost inside the same hold parks, and the arbiter's 20 ms answers it
//   (i2c_timeout), so whether the Client's tenure or one of Pressure's
//   probes went first decides which of the two the held tenure gets.
//   THE DMA RELEASE CONTRACT (docs/ch32vx03/dma.md, "A released
//   requester"): channel 7 is ORed with TIM2's channels 2 and 4 and with
//   USART2's transmit, so TIM2 - the pressure - never sets a DMA enable,
//   USART2 is never opened, and letter i reads both channels disabled,
//   I2C1's request enables down and TIM2's DMA enables clear.
//   THE PRESSURE. TIM2's update interrupt, 16 bits on TIMCLK = 2 x PB1 =
//   HCLK = 96 MHz at prescaler 0: 97 / 193 / 389 us are 9312 / 18528 /
//   37344 counts. Pressure's dispatch, a 3000-cycle spin behind the
//   platform's road (144 cycles from a handler's last statement at 144 MHz
//   on the CH32V203C8T6, docs/ch32vx03/platform.md - re-measured here at
//   this clock: every pass prints the TIM2 count where the spin begins
//   (I2cTraits::tick_spin_begin()), the cycles from the update event
//   through the vector and the road to the dispatch's spin, 176 to 262
//   cycles at its least), is shorter than the
//   shortest period's 9312 cycles, so a tick finds the last one done; at
//   97 us x 3000 it takes 32 % of the core. The exception is measured, not
//   placed: in letter h's 97 us passes, at the far end's release of the
//   line, that road has taken up to 7786 cycles, and 3000 more run past
//   the next update - that tick waits in Pressure's queue (eight deep)
//   behind the dispatch, none is dropped, and judge_pressure() judges
//   exactly that (one dispatch a tick, no overflow).
//   THE RULER. The kernel ticker's composed cycles() (the tick count and the
//   STK counter's position, ch32vx03/ticker.hpp), which the shared body's
//   passes and spans need monotone across a tick; spin() reads the STK
//   counter bare, one load, the restart at the compare folded in.
//   THE V4F'S FLOAT TAX (the CH32V303). Under ilp32f every vector that calls
//   out is a naked trampoline (qingke/pfic.hpp) and the stratum's vector
//   guard walks each body for an f-register instruction; no object of the
//   pack touches a float, and letter i reads mstatus.FS on that part.
//
// THE PEER, unchanged: twi_peer on the other board, kind 0x05, its I2C1 on
// PB6/PB7 a POLLED target at 96 MHz on its crystal that stretches by
// construction, one byte late (its header): ADDR holds SCL until its two
// reads, a receiver holds it at BTF with the next byte already in its
// shifter, and its commanded hold is spent at the address match and before
// every byte it takes or gives. It ends a counted serve with its count
// reached and the wire at rest for 40 us, never at the byte. A refused data
// byte is CTLR1.ACK set down IN ADVANCE, and on this block the same bit
// acknowledges the target's own address, so from the refused byte to the
// action's deadline the peer refuses its address too (its header: STOPF
// does not rise for a Stop after a refused byte, so the deadline's re-init
// is what puts ACK back) - I2cTraits::refusal_holds_address below.
//
// THE WIRES, fixed on the desk (docs/boards/weact-ch32v203c8.md,
// docs/boards/ch32v303-evt.md; test_vx03_i2c's pair): V203 PB6 to V303 PB6
// (SCL), V203 PB7 to V303 PB7 (SDA), 2.2 kOhm from each line to the
// CH32V203C8T6 board's 3.3 V, the grounds. THE PULL-UPS ARE ONE BOARD'S:
// a CH32V303VCT6 run needs the CH32V203C8T6's board powered, and every
// letter that drives the wire first reads both lines high on their pads
// (I2cTraits::bring_up()). The CH32V303's PA4 is not 5 V tolerant: that
// board stays powered whenever the CH32V203's runs. NOTHING LIT: no letter
// drives the LED's pad PB2.
//
// The letters (z: n o p q w h i; x outside it), each but i and x run as
// seven passes (the shared body's control and six pressures):
//   n  the command channel through the arbiter at 100 kHz: a ping, ident
//      (the sanity byte and the stratum's kind, 0x05), ten pings, every
//      tenure i2c_ok.
//   o  the tenure shapes at 400 kHz on both of the host's paths (I2cTraits
//      below names each shape's), one counted serve a pass: every status
//      i2c_ok, every read byte-exact, the peer's count, sum and address
//      matches exact.
//   p  100 and 400 kHz: a read of 16, a 1+4 and a write of 16 byte-exact,
//      the wire's time printed; the peer's commanded stretch, 1000 us a
//      byte at 100 kHz, priced from below by its holds (I2cTraits).
//   q  the vocabulary: a deaf client's i2c_nack_addr, the third of six
//      refused (i2c_nack_data, the write on the transmit engine), the last
//      written byte of a 2+4 refused (i2c_nack_data, the controller let go
//      through the void write), the peer's own word that it refused; inside
//      each refusing window the probe refused at the address (this peer),
//      and after the deadline the next tenure i2c_ok.
//   w  three requesters of three priorities at 400 kHz: the Client's
//      write-then-reads of 1+6 (the read half on the receive engine),
//      Minor's writes of two (the pump: both bytes in the ADDR entry),
//      Pressure's probes: in order, i2c_ok, nothing rejected, no timeout,
//      the peer's tallies exact.
//   h  the held wire (the peer holds SDA 60 ms) with Pressure's flood of
//      probes, paced by the Console's turns: answered in its place, every request once, the next
//      tenure i2c_ok and the arbiter idle; then the hold met at another
//      speed.
//   i  dark: every object idle, the arbiter's vote, no time event armed,
//      TIM2 stopped with its DMA enables clear, the host idle with its
//      enables at rest and both its channels disabled, BUSY clear and both
//      lines high; on the CH32V303, mstatus.FS not Dirty.
//   x  (outside z) 400 writes of 16 silent and 400 under 97 us x 3000:
//      the landings inside start() counted, and of them those with MSL set.
//
// build: boards = v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <iterator>
#include <variant>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/i2c.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/schedule.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
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

/// The PLL on the 8 MHz crystal at 96 MHz: PB1 = 48 MHz, inside 19.12.2's
/// FREQ window, static - the timing table is solved once.
using SysClock = brio::Clock<brio::ClockSource::pll, 96'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr bool v303 = device::device_class == DeviceClass::v30x_d8;

using P = Ch32vx03RubatoPlatform<>;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;
static_assert(SysClock::pclk1_hz == 48'000'000u, "I2C1's PB1 at 48 MHz: CCR 240 and 40, 100 and 400 kHz exactly");

/// The composed cycle count: monotone across the ticker's restart, a
/// difference exact under 2^32 cycles (44.7 s at 96 MHz).
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
};

/// The STK counter's period: it counts HCLK and restarts at the ticker's
/// compare every millisecond.
constexpr uint32_t stk_period = SysClock::hz / Ticker::ticks_per_second;

// ---- the pressure's clock ----------------------------------------------------------

using Pulse = Tim<2>;
static_assert(Pulse::clock_hz(SysClock{}) == SysClock::hz, "TIM2 counts HCLK at this clock");
constexpr uint32_t pulse_per_us = SysClock::hz / 1'000'000u;
static_assert(389u * pulse_per_us - 1u <= 0xFFFFu, "the longest period fits TIM2's 16 bits");
constexpr const char* pressure_name = "TIM2";
constexpr const char* peer_name = v303 ? "the CH32V203C8T6" : "the CH32V303VCT6";

/// A busy wait of `cycles` HCLK cycles: the STK counter read bare, one
/// load a turn, the restart folded in, for a span under half a period; the
/// composed count for a longer one. A preemption longer than a period only
/// lengthens a bare spin.
inline void spin(uint32_t cycles) {
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

/// USART1 on PA9/PA10, the probe's serial, engine-free.
using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
TestBench<Serial> bench;

// ---- the I2C host --------------------------------------------------------------------

using H = I2c<1>;
using TxEngine = DmaTxEngine<1, 6>;
using RxEngine = DmaRxEngine<1, 7>;
using Host = I2cHost<1, i2c_default_pins<1>, TxEngine, RxEngine>;
using TxChannel = DmaChannel<1, 6>;
using RxChannel = DmaChannel<1, 7>;
using SclPin = Pin<'B', 6>;
using SdaPin = Pin<'B', 7>;
static_assert(H::dma_tx_slot.controller == 1u && H::dma_tx_slot.channel == 6u && H::dma_rx_slot.channel == 7u,
              "I2C1's two rows: DMA1's channels 6 and 7 (table 11-5)");

constexpr uint8_t peer_kind = twilink::peer_ch32vx03;
/// The F1 event machine has Sm and Fm and no Fm+ (19.2): the two speeds
/// test_vx03_i2c's letter f judges against this peer.
constexpr I2cSpeed speeds[] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k};
constexpr uint32_t speed_hz(I2cSpeed s) { return i2c_speed_hz(s); }

/// What the shared body changes for this host and this peer.
struct Shape {
    uint8_t tx_len;
    uint8_t rx_len;
    const char* path;
};
struct I2cTraits {
    /// Letter o's table, each shape with the path this host takes: a write
    /// phase of three bytes or more on the transmit engine and of one or two
    /// in the entry that clears ADDR (the driver's dma_min_write, 3, a
    /// private constant: EVT8_1 takes two bytes with no interrupt for
    /// either), a read phase of two or more on the receive engine under
    /// LAST and of one on the pump. The 3+2 is the one shape whose write
    /// half rides the engine with a read half behind it: the transmit
    /// block's completion arms the buffer vector for the TxE that requests
    /// the repeated START. Ends on a plain write (the shared body's counted
    /// serve).
    static constexpr Shape shapes[] = {
        {0, 0, "the probe: SB and ADDR, two event entries"},
        {0, 1, "pump: ACK down before the address, STOP after ADDR's clear, the byte at RxNE"},
        {0, 2, "engines: the receive channel under LAST"},
        {0, 3, "engines"},
        {0, 8, "engines"},
        {0, 32, "engines"},
        {1, 1, "pump both halves: the byte and the repeated START in the ADDR entry"},
        {1, 2, "pump write (the ADDR entry), engine read"},
        {2, 8, "pump write (both bytes in the ADDR entry), engine read"},
        {3, 2, "engines both halves: the block's end arms TxE for the repeated START"},
        {1, 0, "pump: the byte in the ADDR entry, BTF the end"},
        {2, 0, "pump: both bytes in the ADDR entry, BTF the end"},
        {3, 0, "engines: the transmit channel, BTF with its count at zero"},
        {8, 0, "engines"},
        {32, 0, "engines"},
    };
    static_assert(Host::has_engines, "the table names engine paths");
    /// Minor's writes of two: the pump, both bytes taken in the entry that
    /// clears ADDR, so that letter w puts one requester on each path (the
    /// Client's read halves on the receive engine).
    static constexpr uint8_t w_minor_len = 2;
    /// The peer's holds inside the stretched tenures (its run_serve): one at
    /// the address match and one before each byte it takes or gives, a
    /// receiver one byte late. A read of 8: the address and the eight gives
    /// all fall inside the tenure (the controller reads the eighth before
    /// its completion; the ninth give, the byte the shifter asks beyond the
    /// closing NACK, overlaps the last byte and the STOP), nine holds. A
    /// write of 8: the controller's completion is BTF of the eighth byte,
    /// which the target lets through once it has taken the sixth (the
    /// seventh then in its data register, the eighth in its shifter), so
    /// the address and six, seven holds - docs/ch32vx03/i2c.md measured this
    /// peer's 2 ms holds so: "the seventh byte's hold falls after this
    /// host's completion". The wire adds its byte times on top, more than
    /// the two crystals' few ppm take off seven or nine milliseconds, so
    /// the bound is the holds alone.
    static constexpr uint8_t stretch_write_holds = 7;
    static constexpr uint8_t stretch_read_holds = 9;
    /// A refused LAST written byte of a write-then-read is closed by a void
    /// write from the repeated START already requested (the driver's
    /// close_refused()): one address more, which this peer, refusing its
    /// address after a refused byte, does not take.
    static constexpr uint8_t refused_close_hits = 1;
    /// From a refused data byte to the action's deadline this peer's ACK is
    /// down, and that bit acknowledges its own address too (the file
    /// header): the probe inside the window is refused at the address.
    static constexpr bool refusal_holds_address = true;
    /// The road instrument (below): TIM2's count where Pressure's spin
    /// begins.
    static void tick_spin_begin();
    /// Both lines high on their pads before the letter's first tenure: the
    /// pull-ups are the CH32V203C8T6 board's, so on the CH32V303VCT6 a line
    /// low at rest is that board unpowered (or a line held).
    static bool bring_up() { return SclPin::read() && SdaPin::read(); }
};

// THE TARGET-FREE HALF IS SHARED, NOT COPIED: every family's I2C suite
// compiles the same body around its own host.
#include "../../../avrxt/src/apps/rubato_i2c.hpp"

// =============================================================================
// What the shared body asks of this family
// =============================================================================

void pressure_arm(uint16_t period_us) {
    Pulse::enable(false);
    Pulse::set_count(0);
    (void)Pulse::set_period(static_cast<uint32_t>(period_us) * pulse_per_us - 1u);
    Pulse::clear_flags(Pulse::update_flag);
    Pulse::interrupts(Pulse::update_interrupt, true);
}
void pressure_go() { Pulse::enable(true); }
void pressure_halt() { Pulse::enable(false); }
void pressure_quiet() {
    Pulse::interrupts(Pulse::update_interrupt, false);
    Pulse::clear_flags(Pulse::update_flag);
    Pfic::clear_pending(Pulse::irq());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

/// The road instrument: TIM2 counts HCLK from zero at its update event, so
/// its count where Pressure's spin begins is the cycles from the event
/// through the interrupt's entry, the vector (its flag, the landing trace,
/// the post), the scheduler's road and the dispatch's head (a request
/// posted in letters w and h included) - the least of a pass that road
/// with nothing in its way, the most what the no-dropped-tick bound
/// spends.
volatile uint32_t road_least = 0xFFFF'FFFFu;
volatile uint32_t road_most = 0;
volatile uint32_t road_n = 0;
/// Where the interrupt found the kernel (the vector writes them before its
/// post) and, for the pass's longest road, where that one landed: the
/// ceiling, the kind of event the arbiter was dispatching, and TIM2's count
/// at the vector's first instruction - which parts the road into the time
/// before the vector and the time after it.
volatile uint8_t tick_ceiling = 0;
volatile uint8_t tick_arb = 0;
volatile uint8_t most_ceiling = 0;
volatile uint8_t most_arb = 0;
volatile uint32_t tick_entry = 0;
volatile uint32_t most_entry = 0;
void I2cTraits::tick_spin_begin() {
    const uint32_t count = Pulse::count();
    if (pressure_spin == 0u) {
        return;   // a tick outside a pressure pass (pass_end() clears the spin): no road kept
    }
    road_least = count < road_least ? count : road_least;
    if (count > road_most) {
        road_most = count;
        most_ceiling = tick_ceiling;
        most_arb = tick_arb;
        most_entry = tick_entry;
    }
    road_n = road_n + 1u;
}

/// STAR2.MSL: the controller is the bus's master, set when its START is on
/// the wire and cleared by its STOP (19.12.7). A STAR2 read alone clears
/// nothing an event vector waits for: ADDR's clear is STAR1 then STAR2,
/// and start() reads STAR1 for exactly that clear before its START.
bool host_on_wire() { return H::host(); }

uint32_t read_mstatus() {
    uint32_t v;
    __asm__ volatile("csrr %0, mstatus" : "=r"(v));
    return v;
}

const char* host_dark() {
    const uint16_t timer_dma = static_cast<uint16_t>(Pulse::update_dma | Pulse::trigger_dma | Pulse::compare_dma(0) |
                                                     Pulse::compare_dma(1) | Pulse::compare_dma(2) |
                                                     Pulse::compare_dma(3));
    if (Pulse::enabled() || (Pulse::interrupts() & Pulse::update_interrupt) != 0u || Pfic::pending(Pulse::irq())) {
        return "TIM2 running, its update interrupt on or its line pending";
    }
    if ((Pulse::interrupts() & timer_dma) != 0u) {
        return "a DMA request enable of TIM2 is set (channel 7 is ORed with its channels 2 and 4)";
    }
    if (Host::busy()) {
        return "the host's engine is not between tenures";
    }
    // finish() puts CTLR2 back to the error vector alone in one store, the
    // event and buffer vectors, DMAEN and LAST with it (the class comment):
    // no request can be raised at rest.
    const uint16_t ctlr2 = H::regs().CTLR2;
    if ((ctlr2 & (i2c_itevten | i2c_itbufen | i2c_dmaen | i2c_last)) != 0u || (ctlr2 & i2c_iterren) == 0u) {
        return "the host's enables are not at rest (the error vector alone, DMAEN and LAST down)";
    }
    if (TxEngine::busy() || TxChannel::enabled() || RxChannel::enabled()) {
        return "a DMA channel of the host is enabled or its transmit engine holds a block";
    }
    if (Rcc::enabled(Bus::pb1, usart_gate_for(2))) {
        return "USART2's gate is open (its transmit request is ORed on channel 7)";
    }
    if (H::busy()) {
        return "the bus monitor reads BUSY";
    }
    if (!SclPin::read() || !SdaPin::read()) {
        return "a line reads low on its pad";
    }
    if constexpr (device::has_fpu) {
        // FS, mstatus[14:13]: 3 Dirty. The crt leaves it Dirty (its fcsr
        // write), the scheduler's entry saves and cleans it at the first
        // preemption, and nothing of the pack touches a float after that.
        const uint32_t fs = (read_mstatus() >> 13) & 0x3u;
        print(serial, "  mstatus.FS = ", fs, " (0 off, 1 Initial, 2 Clean, 3 Dirty)", crlf);
        if (fs == 3u) {
            return "the float unit Dirty: a float written since the last preemption's save";
        }
    }
    return nullptr;
}

void print_host() {
    // TxEngine::faults() counts abandon(), which put_engines_away() calls
    // at every tenure an error or recover() ends, a block in flight or not.
    print(serial, "; unwedges ", Host::unwedges(), ", transmit engine abandons ", TxEngine::faults(),
          ", receive faults ", RxEngine::faults(), " (since boot)");
    if (road_n != 0u) {
        print(serial, "; the update to Pressure's spin ", road_least, "..", road_most, " cycles over ", road_n,
              " dispatches, the longest landing at ceiling ", most_ceiling, " with the arbiter's event ", most_arb,
              " (its vector entered at ", most_entry, ")");
    }
    road_least = 0xFFFF'FFFFu;
    road_most = 0;
    road_n = 0;
}

bool clock_ok = false;
bool tick_ok = false;
bool host_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "HSE8x12=96MHz" : "FAILED (no crystal)", " tick=",
          tick_ok ? "STK" : "FAILED", " ruler=STK i2c1=", host_ok ? "host" : "FAILED", crlf);
    print(serial, "test_vx03_rubato_i2c - the I2C stack under Rubato on the ", device::part_name, " against ",
          peer_name, " (twi_peer)", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// Every vector here posts, so each is BRIO_CH32_VECTOR: a naked trampoline
// under the CH32V303's ilp32f, the plain handler under the CH32V203's
// ilp32. The two parts bind the same lines.

BRIO_CH32_VECTOR(usart1_handler) {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return<K>(); }

/// The pressure: TIM2's update, where it landed, and the post.
BRIO_CH32_VECTOR(tim2_handler) {
    tick_entry = Pulse::count();
    Pulse::clear_flags(Pulse::update_flag);
    tick_ceiling = P::ceiling;
    tick_arb = arb_event;
    pressure_landed();
}

/// The event vector: SB, ADDR, the pump's TxE and RxNE, BTF - the tenure's
/// end for the pump, a plain write's on the engine and the void write's.
BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

/// The error vector: a NACK on the address or a byte, ARLO, BERR - each the
/// tenure's end with its code.
BRIO_CH32_VECTOR(i2c1_er_handler) {
    if (Host::error_isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

/// The two channels: the receive block's completion ends a read phase, the
/// transmit block's hands the end to the event vector, a transfer error
/// ends the tenure on either.
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    host_ok = Host::init(clock);
    Pulse::init();
    (void)Pulse::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFFu});
    brio::Pfic::enable(Pulse::irq());
    brio::scheduler_init();

    bench.letter('n', "the command channel through the arbiter", tn_command_channel);
    bench.letter('o', "the tenure shapes on both of the host's paths", to_shapes);
    bench.letter('p', "the speeds, and the peer's commanded stretch", tp_speeds);
    bench.letter('q', "the vocabulary under pressure: deaf, a refused byte, a refused last byte", tq_vocabulary);
    bench.letter('w', "the arbiter with three requesters of three priorities", tw_arbiter);
    bench.letter('h', "the held wire, with requests arriving while it is held", th_held_wire);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.letter('x', "the landings inside start() counted: 400 writes silent and pressed", tx_fill, false);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
