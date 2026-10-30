// test_stm32_rubato_i2c - the bench suite of the STM32G0 stratum's I2C
// stack under Rubato, the preemptive kernel (kernel/rubato.hpp):
// util/i2c_bus.hpp's arbiter over stm32g0/i2c.hpp's I2cHost and its DMA
// engines, run against the desk's Nucleo-F446RE and its unchanged
// instrument (stm32f4/src/apps/twi_peer.cpp) while an object of the highest
// priority preempts it at moments the traffic does not choose. Every
// verdict is the traffic judged byte-exact and every status the one the
// wire produced, as the Tenuto suite's peer letters (test_stm32_i2c, key w)
// judge them; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests: the program is one Rubato pack of
// five objects - Pressure, Client, I2cArb, Minor and Console, from the
// highest priority down - and everything of it that names no register, the
// pack, the passes, the command channel and the letters, is the I2C suites'
// shared body (avrxt/src/apps/rubato_i2c.hpp, whose header describes each
// object, each letter and where preemption can reach a tenure). What is
// this family's is here, each choice from RM0444 and the driver header:
//
//   THE HOST. I2cHost<1> on PB8 (SCL) and PB9 (SDA) at AF6 with the DMA
//   engines on DMA1 channels 1 (transmit) and 2 (receive) - test_stm32_i2c's
//   engined host - its kernel clock PCLK under the static PLL at 64 MHz (a
//   DynamicClock would move TIMINGR, which no tenure may see). Its engine,
//   from the chapter (stm32g0/i2c.hpp's class comment, docs/stm32g0/i2c.md):
//   the byte counter NBYTES with AUTOEND carries the tenure (32.4.9),
//   RELOAD never used (a Request's lengths are bytes); the first byte of a
//   pumped write PRELOADED into TXDR before the START; a read's last byte
//   taken at STOPF; a NACK read at STOPF with no NACKIE; one interrupt a
//   data byte on the pump (no FIFO: TXDR, RXDR and the shifter); the
//   engines carry halves of two bytes or more (the driver's dma_min_bytes,
//   a private constant: a half of one costs the pump no data interrupt),
//   armed for their errors alone, STOPF the edge; at rest ERRIE alone and
//   both DMA request enables clear; ES0548 2.10.2's spurious BERR counted
//   (spurious_bus_errors()) and never a status. start() writes the enables
//   and then the ONE CR2 store that carries START (transfer_now): nothing is
//   on the wire before its last store, so a preemption inside start() is a
//   later START - a gap between tenures - and inside a tenure the master
//   holds SCL while TXIS or RXNE waits for a late vector (32.4.8, 32.4.9).
//   THE PREDICTION: strict at every speed, path and pass; letter x's
//   landings inside start() find the wire idle.
//   THE HELD WIRE has two signatures on this host (docs/stm32g0/i2c.md, "THE
//   WEDGE HAS A SECOND SIGNATURE"): a START into a line already seen low
//   parks with BUSY standing and only the arbiter's 20 ms answers
//   (i2c_timeout), and a START put out at once into the held line loses its
//   first one bit (i2c_arb_lost). A speed change is a PE cycle before the
//   START (apply(): TIMINGR is PE-gated) and PE = 0 clears BUSY (32.9.7), so
//   letter h meets the hold at the speed in force in its passes and, once
//   after them, at another: the prediction is i2c_timeout for the first and
//   i2c_arb_lost for the second, the requirement judged either way;
//   recover() is 32.4.6's PE cycle.
//   THE PRESSURE. TIM14's update interrupt, 16 bits on TIMPCLK = HCLK =
//   64 MHz at prescaler 0: 97 / 193 / 389 us are 6208 / 12352 / 24896
//   counts; its dispatch, a 3000-cycle spin and a road-2 post of 517
//   (docs/stm32g0/platform.md), is shorter than the shortest period's 6208
//   cycles, so no tick finds the last one still queued; at 97 us x 3000 it
//   takes 48 % of the core. TIM14 has no DMA request in use here.
//   THE RULER. TIM2 free-running at HCLK, one load of CNT, a count a core
//   cycle (ARMv6-M has no DWT); the pressure's timer and the ruler count one
//   clock, so judge_pressure()'s arithmetic is exact.
//
// THE PEER, unchanged: twi_peer on the Nucleo-F446RE, kind 0x04, its I2C1 on
// PB8/PB9 a POLLED target that stretches by construction, one byte late
// (its header: it takes the next byte into its shifter while the last
// still sits in DR, so a hold shorter than a byte time never reaches the
// wire), whose commanded hold is spent at the address match and before
// every data byte it takes or gives. No Fm+ on that block (RM0390 27.3.3:
// I2C1 is the F1 event machine, Sm and Fm), so the speeds are 100 and
// 400 kHz, what test_stm32_i2c's letter q judges against it. It ends a
// counted serve at the byte that reaches the count, re-initing its block
// there (twi_peer.cpp's run_serve), which is why every counted serve of the
// shared body ends on a plain write.
//
// THE WIRES, fixed on the desk (test_stm32_i2c's peer letters and
// test_stm32f4_i2c's key t): this board's PB8 (SCL) to the F446's PB8,
// PB9 (SDA) to its PB9 - on both Nucleo-64s the Arduino header's D15 and
// D14, CN5 pins 10 and 9 (UM2324 Rev 5 table 12, UM1724 Rev 17) - and a
// ground. The same two nodes carry both boards' self-link taps (this
// board's PA11/PA12, the F446's PC6/PC7) with their 2.2 kOhm pull-ups to
// 3.3 V; neither image claims those pads, which stay at their reset state.
// NOTHING LIT: no letter drives the LED's pad.
//
// The letters (z: n o p q w h i; x outside it), each but i and x run as
// seven passes (the shared body's control and six pressures):
//   n  the command channel through the arbiter at 100 kHz: a ping, ident
//      (the sanity byte and the F4 port's kind, 0x04), ten pings, every
//      tenure i2c_ok.
//   o  the tenure shapes at 400 kHz on both of the host's paths (I2cTraits
//      below names each shape's), one counted serve a pass: every status
//      i2c_ok, every read byte-exact, the peer's count, sum and address
//      matches exact.
//   p  100 and 400 kHz: a read of 16, a 1+4 and a write of 16 byte-exact,
//      the wire's time printed; the peer's commanded stretch, 1000 us a
//      byte at 100 kHz, priced from below by its holds (I2cTraits).
//   q  the vocabulary: a deaf client's i2c_nack_addr, the third of six
//      refused (i2c_nack_data, the write on the engines), the last written
//      byte of a 2+4 refused (i2c_nack_data, the read half never begun),
//      the next tenure i2c_ok after each.
//   w  three requesters of three priorities at 400 kHz: the Client's
//      write-then-reads of 1+6 (the read half on the engines), Minor's
//      writes of one (the pump's preload), Pressure's probes: in order,
//      i2c_ok, nothing rejected, no timeout, the peer's tallies exact.
//   h  the held wire (the peer holds SDA 60 ms) with Pressure's probes at
//      every tick: answered in its place, every request once, the next
//      tenure i2c_ok and the arbiter idle; then the hold met at another
//      speed (above).
//   i  dark: every object idle, the arbiter's vote, no time event armed,
//      TIM14 stopped, the host idle with its enables at rest and both DMA
//      request enables clear, BUSY clear and both lines high.
//   x  (outside z) 400 writes of 16 silent and 400 under 97 us x 3000:
//      the landings inside start() counted, and of them those with BUSY
//      set.
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <iterator>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/i2c.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's instrument
// compiles twi_link.hpp too, and the controller's end of it is one reviewed
// file.
#include "../../../avrxt/src/apps/twi_link_client.hpp"

/// The PLL on the HSI16 at 64 MHz, static: TIMINGR is solved once against
/// one fixed PCLK = HCLK.
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32g0RubatoPlatform<>;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

/// TIM2 free-running at HCLK: one load of CNT, a count a cycle.
using Clock32 = Tim<2>;
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() { return Clock32::count(); }
};

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 3, PinFunction::af1}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;
TestBench<Serial> bench;

// ---- the I2C host --------------------------------------------------------------------

/// I2C1 on PB8/PB9 at AF6 (DS13560 tables 13 and 15), the pads the
/// Nucleo-64 brings out as D15 and D14.
constexpr I2cPins host_pins{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 9, PinFunction::af6},
};
using TxEngine = DmaTxEngine<1, 1>;
using RxEngine = DmaRxEngine<1, 2>;
using Host = I2cHost<1, host_pins, TxEngine, RxEngine>;
using H = I2c<1>;
using SclPin = Pin<'B', 8>;
using SdaPin = Pin<'B', 9>;

constexpr uint8_t peer_kind = twilink::peer_stm32f4;
/// The F446's I2C1 serves Sm and Fm (RM0390 27.3.3), what test_stm32_i2c's
/// letter q judges against it.
constexpr I2cSpeed speeds[] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k};
constexpr uint32_t speed_hz(I2cSpeed s) { return i2c_speed_hz(s); }

/// What the shared body changes for this host and this peer.
struct Shape {
    uint8_t tx_len;
    uint8_t rx_len;
    const char* path;
};
struct I2cTraits {
    /// Letter o's table, each shape with the path this host takes: a half
    /// of two bytes or more rides the engines, a half of one the pump
    /// (the driver's dma_min_bytes, 2: the preload or the read at the STOP
    /// costs the pump no data interrupt for it). Ends on a plain write (the
    /// shared body's counted serve).
    static constexpr Shape shapes[] = {
        {0, 0, "the probe: the address alone, STOPF its one interrupt"},
        {0, 1, "pump: the one byte read at STOPF"},
        {0, 2, "engines: the receive channel, STOPF the edge"},
        {0, 3, "engines"},
        {0, 8, "engines"},
        {0, 32, "engines"},
        {1, 1, "pump both halves: the preload, TC's repeated START, the read at STOPF"},
        {1, 2, "pump write (the preload), engine read"},
        {2, 8, "engines both halves"},
        {1, 0, "pump: the preload alone, STOPF the one interrupt"},
        {2, 0, "engines: the transmit channel, STOPF the edge"},
        {3, 0, "engines"},
        {8, 0, "engines"},
        {32, 0, "engines"},
    };
    static_assert(Host::has_engines, "the table names engine paths");
    /// Minor's writes of one: the pump's preload, so that letter w puts one
    /// requester on each path (the Client's read halves on the engines).
    static constexpr uint8_t w_minor_len = 1;
    /// The F446 peer's holds inside the stretched tenures (its run_serve):
    /// one at the address match and one before each byte it takes or
    /// gives. A read of 8: the address and all eight gives fall inside the
    /// tenure (the controller NACKs the eighth after it), nine holds. A
    /// write of 8: the tenure ends once the eighth byte can complete, which
    /// needs DR emptied of the seventh - the take after the seventh hold -
    /// so the address and seven, eight holds. The wire adds nine byte times
    /// (810 us at 100 kHz), more than this end's HSI16 can take off eight
    /// or nine milliseconds (1 %, docs/stm32g0/clock.md), so the bound is
    /// the holds alone.
    static constexpr uint8_t stretch_write_holds = 8;
    static constexpr uint8_t stretch_read_holds = 9;
};

// ---- the pressure's clock ----------------------------------------------------------

using Pulse = Tim<14>;
constexpr uint32_t pulse_per_us = SysClock::hz / 1'000'000u;   // TIMPCLK = HCLK
static_assert(389u * pulse_per_us - 1u <= 0xFFFFu, "the longest period fits TIM14's 16 bits");
constexpr const char* pressure_name = "TIM14";
constexpr const char* peer_name = "the STM32F446RE";

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
    Nvic::clear_pending(Pulse::irq());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

/// ISR.BUSY: the bus monitor saw a START and no STOP since (32.9.7).
bool host_on_wire() { return H::busy(); }

const char* host_dark() {
    if (Pulse::enabled() || (Pulse::interrupts() & Pulse::update_interrupt) != 0u || Nvic::pending(Pulse::irq())) {
        return "TIM14 running, its update interrupt on or its line pending";
    }
    if (!Host::idle()) {
        return "the host's engine is not between tenures";
    }
    // finish() puts the enables back to ERRIE alone in one store, the two
    // DMA request enables with them (the class comment): no request can
    // be raised at rest.
    if (H::interrupts() != I2cInterrupt::error || H::dma_transmit() || H::dma_receive()) {
        return "the host's enables are not at rest (ERRIE alone, both DMA requests off)";
    }
    if (TxEngine::busy()) {
        return "the transmit engine holds a block";
    }
    if (H::busy()) {
        return "the bus monitor reads BUSY";
    }
    if (!SclPin::read() || !SdaPin::read()) {
        return "a line reads low on its pad";
    }
    return nullptr;
}

void print_host() {
    print(serial, "; spurious BERR ", Host::spurious_bus_errors(), ", transmit tails abandoned ", TxEngine::faults(),
          ", receive faults ", RxEngine::faults(), " (since boot)");
}

bool clock_ok = false;
bool tick_ok = false;
bool host_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL64" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
          " ruler=TIM2 i2c1=", host_ok ? "host" : "FAILED", crlf);
    print(serial, "test_stm32_rubato_i2c - the I2C stack under Rubato against the STM32F446RE (twi_peer)", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void BRIO_STM32G0_USART2_HANDLER() {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }

/// The pressure: TIM14's update, where it landed, and the post.
extern "C" void TIM14_IRQHandler() {
    Pulse::clear_flags(Pulse::update_flag);
    pressure_landed();
}

/// Every tenure's edge, STOPF (and an ARLO): the host's one completion.
extern "C" void I2C1_IRQHandler() {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

/// Channel 1, the transmit engine, and channels 2 and 3's line (RM0444
/// table 61), the receive engine on channel 2 and nothing on 3: both armed
/// for their errors alone, so a completion here is a DMA fault ending its
/// tenure.
extern "C" void DMA1_Channel1_IRQHandler() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

extern "C" void DMA1_Channel2_3_IRQHandler() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    Clock32::init();
    (void)Clock32::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFF'FFFFu});
    Clock32::enable(true);
    host_ok = Host::init(clock, brio::I2cClock::pclk);
    Pulse::init();
    (void)Pulse::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFFu});
    brio::Nvic::enable(Pulse::irq());
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
