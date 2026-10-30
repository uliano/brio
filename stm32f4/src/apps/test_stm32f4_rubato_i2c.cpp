// test_stm32f4_rubato_i2c - the bench suite of the STM32F4 stratum's I2C
// stack under Rubato, the preemptive kernel (kernel/rubato.hpp):
// util/i2c_bus.hpp's arbiter over stm32f4/i2c.hpp's I2cHost on I2C1 and its
// DMA engines, run against the desk's Nucleo-G0B1RE and its unchanged
// instrument (stm32g0/src/apps/twi_peer.cpp) while an object of the highest
// priority preempts it at moments the traffic does not choose. Every
// verdict is the traffic judged byte-exact and every status the one the
// wire produced, as the Tenuto suite's partner letter (test_stm32f4_i2c,
// key t) judges them; nothing here changes a driver or a service. ITS TWIN,
// test_stm32f4_rubato_fmpi2c, is this source whole over the chip's second
// I2C design, FMPI2C1 (stm32f4/fmpi2c.hpp) - the host, its pads, its
// engines and its four vectors chosen by the one macro the twin defines
// before it includes this file, everything else the same.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests: the program is one Rubato pack of
// five objects - Pressure, Client, I2cArb, Minor and Console, from the
// highest priority down - and everything of it that names no register, the
// pack, the passes, the command channel and the letters, is the I2C suites'
// shared body (avrxt/src/apps/rubato_i2c.hpp, whose header describes each
// object, each letter and where preemption can reach a tenure). What is
// this family's is here, each choice from RM0390 and the driver headers:
//
//   I2C1 (this image). I2cHost<1> on PB8 (SCL) and PB9 (SDA) at AF4, its
//   DMA engines on DMA1 stream 6 (transmit) and stream 0 (receive), both on
//   channel 1 (RM0390 table 28) - test_stm32f4_i2c's engined host - its
//   reference clock APB1 at 45 MHz under the static 180 MHz. Its engine,
//   from the chapter (stm32f4/i2c.hpp's class comment, docs/stm32f4/i2c.md):
//   the F1 lineage's event machine (RM0390 24.3.3), SB, ADDR and BTF each holding
//   SCL until software runs the flag's sequence; the first two data bytes
//   of a write loaded in the entry that clears ADDR (EV8_1, no interrupt for
//   either); a write phase of three bytes or more on the transmit engine, a
//   read phase of two or more on the receive engine under CR2.LAST, the
//   one-byte read on the pump (the driver's dma_min_write, 3, a private
//   constant, so the shapes below name their paths in prose); the repeated
//   START requested on the last written byte's TxE behind a dummy store; a
//   refused LAST written byte of a write-then-read closed by a void write
//   from the Sr (24.6.1 forbids any CR1 access while START stands); the
//   event vector on from start() to the completion, the error vector
//   always; every BERR counted (spurious_bus_errors(): ES0298 2.11.1 says
//   a controller's can be spurious) and never a status. start()
//   WAITS in the caller: for the last tenure's STOP to leave the wire (CR1
//   STOP standing, at most 5 ms) and for BUSY (100 us), each a count of
//   delay_us(1) turns, then a PE cycle only if the speed moves, the event
//   vector's enable, and LAST the CR1 read-modify-write that sets START.
//   THE PREDICTION: strict at every speed, path and pass; a preemption
//   inside start() lands in a wait or before the START store and lengthens
//   the gap between tenures (a wait it lands in lasts longer, never
//   shorter), and a landing inside start() finds SR2.MSL clear but in
//   start()'s last instructions after the START store - or in the STOP
//   drain, where MSL stands until the last tenure's STOP is out. THE HELD
//   WIRE: the START waits for BUSY to fall (24.3.3) and parks; the
//   arbiter's 20 ms answers it i2c_timeout, recover() is SWRST
//   (docs/stm32f4/i2c.md).
//
//   FMPI2C1 (the twin). FmpI2cHost<1> on PC6 (SCL) and PC7 (SDA) at AF4
//   (DS10693 table 11), its engines on DMA1 stream 5 (transmit) and stream
//   2 (receive), both on channel 2 (RM0390 table 28) - test_stm32f4_fmpi2c's
//   engined host - its kernel clock APB1 at 45 MHz, the port's own pull-ups
//   asked for beside the bench's 2.2 kOhm pair as that suite asks. The
//   STM32G0's register file (23.4.9): NBYTES with AUTOEND carries a tenure,
//   a write-then-read's repeated START is the one CR2 store at TC, STOPF the
//   completion edge of every shape, a NACK recorded on the event vector and
//   closed by the STOP the block sends itself; WITH ENGINES NAMED EVERY
//   DATA BYTE RIDES THEM, whatever the length (23.4.16: the engines started
//   before the START; only the address phase and the end stay on the
//   vectors); ES0298 2.12.3's spurious BERR counted; transmit_stall_risk()
//   printed (ES0298 2.12.8's band on the APB-to-kernel ratio). start()
//   waits for nothing and its LAST act is the CR2 store that carries START
//   (transfer_now), after the engines are armed. THE PREDICTION: the
//   STM32G0's - strict, a preemption in start() a later START; three speeds,
//   100, 400 and 1000 kHz (the G0 peer is solved for Fm+). THE HELD WIRE:
//   the requirement judged; a START at the speed in force parks behind
//   BUSY, and one after a PE cycle (a speed change, recover(): 23.4.6, PE =
//   0 clears BUSY) goes out into the held line. ONE DEFECT FOUND: with the
//   engines named, a refused DATA byte is answered i2c_nack_addr - the event
//   vector tells the address from the data by the pump's byte position,
//   which the engines never advance (fmpi2c.hpp's isr()) - so this image
//   keeps letter q out of z, runnable by its name (main() below).
//
//   THE PRESSURE. TIM2's update interrupt, 32 bits on APB1's timer clock,
//   2 x PCLK1 = 90 MHz at prescaler 0: 97 / 193 / 389 us are 8730 / 17370 /
//   35010 counts. Its dispatch, a 3000-cycle spin and a road-2 post of 398
//   (docs/stm32f4/platform.md), is shorter than the shortest period's 17460
//   core cycles, so no tick finds the last one still queued; at 97 us x
//   3000 it takes 17 % of the core. TIM2's DMA requests (DMA1 channel 3 on
//   streams 1, 5, 6 and 7, RM0390 table 28 - streams 5 and 6 are the two
//   hosts' transmit streams on channels 2 and 1) stay off, and letter i
//   reads DIER to say so: a stream serves the one channel CHSEL names.
//   THE RULER. DWT's CYCCNT, a count a core cycle; TIM2 counts at exactly
//   half the core clock from the same crystal, so judge_pressure()'s
//   arithmetic is exact.
//
// THE PEER, unchanged: twi_peer on the Nucleo-G0B1RE, kind 0x03, its I2C1 on
// PB8/PB9: a polled target that stretches by construction (ADDR, RXNE and
// TXIS hold SCL until software answers, RM0444 32.4.8), whose commanded
// hold is spent at the address match and before every data byte it takes
// or gives (its header), its client delays solved for Fm+ so that one
// configuration serves every rung. It ends a counted serve at the top of
// its loop once the count is reached and a STOP has been seen in the
// serve - so at the byte that reaches the count when an earlier tenure
// already ended - and re-inits its block there (its run_serve), which is why
// every counted serve of the shared body ends on a plain write.
//
// THE WIRES, fixed on the desk (test_stm32f4_i2c's and test_stm32f4_fmpi2c's
// key t): this board's PB8 (SCL) to the G0's PB8, PB9 (SDA) to its PB9 - on
// both Nucleo-64s the Arduino header's D15 and D14 (UM1724 Rev 17, UM2324
// Rev 5 table 12) - and a ground; this board's PC6 and PC7 (FMPI2C1) and the
// G0's PA11 and PA12 (its I2C2) tap the same two nodes, with the self-links'
// 2.2 kOhm pull-ups to 3.3 V. Each image claims the pads of its own host
// alone and leaves the other host's at their reset state. NOTHING LIT: no
// letter drives the LED's pad.
//
// The letters (z: n o p q w h i; x outside it), each but i and x run as
// seven passes (the shared body's control and six pressures):
//   n  the command channel through the arbiter at 100 kHz: a ping, ident
//      (the sanity byte and the G0 port's kind, 0x03), ten pings, every
//      tenure i2c_ok.
//   o  the tenure shapes at 400 kHz on every path of the host (I2cTraits
//      below names each shape's), one counted serve a pass: every status
//      i2c_ok, every read byte-exact, the peer's count, sum and address
//      matches exact.
//   p  every speed of the image: a read of 16, a 1+4 and a write of 16
//      byte-exact, the wire's time printed; the peer's commanded stretch,
//      1000 us a byte at 100 kHz, priced from below by its holds.
//   q  the vocabulary: a deaf client's i2c_nack_addr, the third of six
//      refused (i2c_nack_data, the write on the engines), the last written
//      byte of a 2+4 refused (i2c_nack_data, the read half never begun),
//      the next tenure i2c_ok after each. On I2C1 the refused last byte is
//      closed by the F1 lineage's void write, which the peer acknowledges
//      (refused_close_hits below). On FMPI2C1 the letter stands OUTSIDE z
//      (the defect above).
//   w  three requesters of three priorities at 400 kHz: the Client's
//      write-then-reads of 1+6 (the read half on the engines), Minor's
//      writes of two, Pressure's probes: in order, i2c_ok, nothing
//      rejected, no timeout, the peer's tallies exact.
//   h  the held wire (the peer holds SDA 60 ms) with Pressure's flood of
//      probes, paced by the Console's turns: answered in its place, every
//      request once, the next tenure i2c_ok and the arbiter idle; then the
//      hold met at 400 kHz, another speed than the one in force.
//   i  dark: every object idle, the arbiter's vote, no time event armed,
//      TIM2 stopped with its DMA requests off, the host between tenures
//      with its enables at rest and its two streams idle, BUSY clear and
//      both lines high.
//   x  (outside z) 400 writes of 16 silent and 400 under 97 us x 3000:
//      the landings inside start() counted, and of them those with the
//      host on the wire (I2C1's SR2.MSL, FMPI2C1's ISR.BUSY).
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <iterator>
#include <type_traits>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/fmpi2c.hpp"
#include "stm32f4/i2c.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's instrument
// compiles twi_link.hpp too, and the controller's end of it is one reviewed
// file.
#include "../../../avrxt/src/apps/twi_link_client.hpp"

// THE HOST UNDER TEST, chosen once. A vector's NAME is the one thing a
// template cannot choose, so the preprocessor names the four the image binds
// and the constant beside them selects every type below; the static_assert
// after the bindings keeps the two one answer.
#if defined(BRIO_SUITE_FMPI2C)
#define BRIO_SUITE_I2C_EV_HANDLER FMPI2C1_EV_IRQHandler
#define BRIO_SUITE_I2C_ER_HANDLER FMPI2C1_ER_IRQHandler
#define BRIO_SUITE_TX_DMA_HANDLER DMA1_Stream5_IRQHandler
#define BRIO_SUITE_RX_DMA_HANDLER DMA1_Stream2_IRQHandler
constexpr bool suite_on_fmpi2c = true;
#else
#define BRIO_SUITE_I2C_EV_HANDLER I2C1_EV_IRQHandler
#define BRIO_SUITE_I2C_ER_HANDLER I2C1_ER_IRQHandler
#define BRIO_SUITE_TX_DMA_HANDLER DMA1_Stream6_IRQHandler
#define BRIO_SUITE_RX_DMA_HANDLER DMA1_Stream0_IRQHandler
constexpr bool suite_on_fmpi2c = false;
#endif

/// The Nucleo's HSE is the ST-LINK's 8 MHz clock, taken in bypass; 180 MHz
/// puts APB1 at 45 MHz (both hosts' reference) and TIM2 at 90.
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4RubatoPlatform<>;
using Ruler = CycleCounter;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;
TestBench<Serial> bench;

// ---- the pressure's clock ----------------------------------------------------------

/// TIM2: 32 bits on APB1, its timer clock 2 x PCLK1 = 90 MHz - a timer
/// neither host touches.
using Pulse = Tim<2>;
constexpr uint32_t pulse_per_us = Pulse::clock_hz(SysClock{}) / 1'000'000u;
static_assert(pulse_per_us == 90u, "TIM2 counts at 90 MHz at this clock");
constexpr const char* pressure_name = "TIM2";
constexpr const char* peer_name = "the STM32G0B1RE";
/// Every DMA request enable of DIER (RM0390 17.4.4): update, the four
/// compares and the trigger.
constexpr uint32_t pulse_dma_enables = Pulse::update_dma | Pulse::trigger_dma | Pulse::compare_dma(0) |
                                       Pulse::compare_dma(1) | Pulse::compare_dma(2) | Pulse::compare_dma(3);

/// TIM2 stopped, its update interrupt off, its line quiet and its DMA
/// requests off: letter i's first half on either host.
const char* pressure_dark() {
    if (Pulse::enabled() || (Pulse::interrupts() & Pulse::update_interrupt) != 0u || Nvic::pending(Pulse::irq())) {
        return "TIM2 running, its update interrupt on or its line pending";
    }
    if ((Pulse::interrupts() & pulse_dma_enables) != 0u) {
        return "a TIM2 DMA request enabled (DMA1 channel 3 on streams 1, 5, 6 and 7)";
    }
    return nullptr;
}

// ---- the two hosts, and the one this image runs --------------------------------------

/// Letter o's rows, each with the path the host takes for it.
struct Shape {
    uint8_t tx_len;
    uint8_t rx_len;
    const char* path;
};

/// What the PEER changes for the shared body, whichever host drives it.
struct PeerTraits {
    /// The G0 peer's holds inside the stretched tenures (its run_serve):
    /// one at the address match and one before each byte it takes or gives.
    /// A read of 8: the address and all eight gives fall inside the tenure
    /// (the controller NACKs the eighth after it), nine holds. A write of
    /// 8: its RXDR and shifter take two bytes before SCL stretches, so the
    /// hold before taking byte k delays byte k + 1's acknowledge, and the
    /// tenure ends once the eighth is acknowledged - after the seventh
    /// hold, so the address and seven, eight holds (test_stm32f4_i2c's
    /// letter n measures the same eight at 2 ms: 16 ms). The wire adds
    /// nine byte times (810 us at 100 kHz), and this end's crystal-derived
    /// ruler takes nothing off, so the bound is the holds alone. A write
    /// that follows a read at once meets most of one hold more: the peer
    /// spends a hold before the byte it is asked for beyond the
    /// controller's last, which starts as that last byte goes out and runs
    /// past the read's STOP - a lower bound still.
    static constexpr uint8_t stretch_write_holds = 8;
    static constexpr uint8_t stretch_read_holds = 9;
};

/// I2C1, the F1 lineage's block.
struct OnI2c1 {
    /// I2C1 on PB8/PB9 at AF4 (DS10693 table 11), the pads the Nucleo-64
    /// brings out as D15 and D14.
    static constexpr I2cPins pins{.scl = {'B', 8, PinFunction::af4}, .sda = {'B', 9, PinFunction::af4}};
    /// I2C1_TX on DMA1 stream 6 and I2C1_RX on stream 0, both channel 1
    /// (RM0390 table 28); the reserve checks the two cells.
    using TxEngine = DmaTxEngine<1, 6, 1>;
    using RxEngine = DmaRxEngine<1, 0, 1>;
    using Host = I2cHost<1, pins, TxEngine, RxEngine>;
    using Res = I2c<1>;
    using SclPin = Pin<'B', 8>;
    using SdaPin = Pin<'B', 9>;
    using TxStream = DmaStream<1, 6>;
    using RxStream = DmaStream<1, 0>;
    static constexpr const char* name = "I2C1";
    static constexpr const char* image = "test_stm32f4_rubato_i2c";
    /// The F1 block serves Sm and Fm (RM0390 24.3.3), and the G0 peer both.
    static constexpr I2cSpeed speeds[] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k};
    static constexpr uint32_t hz(I2cSpeed s) { return i2c_speed_hz(s); }

    struct Traits : PeerTraits {
        /// Letter o's table: a write phase of three bytes or more rides the
        /// transmit engine and a read phase of two or more the receive
        /// engine; one or two written bytes are EV8_1's, loaded in the
        /// entry that clears ADDR, and the one-byte read is the pump's.
        /// Ends on a plain write (the shared body's counted serve).
        static constexpr Shape shapes[] = {
            {0, 0, "the probe: the address alone, the STOP requested in ADDR's entry"},
            {0, 1, "pump: ACK down before the address, STOP right after ADDR's clear"},
            {0, 2, "engines: the receive stream under LAST"},
            {0, 3, "engines"},
            {0, 8, "engines"},
            {0, 32, "engines"},
            {1, 1, "pump both halves: the byte in ADDR's entry, the Sr on its TxE, the one-byte read"},
            {1, 2, "pump write, engine read"},
            {2, 8, "pump write (both bytes in ADDR's entry), engine read"},
            {1, 0, "pump: the byte in ADDR's entry, BTF the end"},
            {2, 0, "pump: both bytes in ADDR's entry (EV8_1), BTF the end"},
            {3, 0, "engines: the transmit stream, BTF the end"},
            {8, 0, "engines"},
            {32, 0, "engines"},
        };
        static_assert(Host::has_engines, "the table names engine paths");
        /// A refused LAST written byte of a write-then-read is closed by a
        /// VOID WRITE from the repeated START already requested (RM0390
        /// 24.6.1 forbids any CR1 access while START stands: close_refused(),
        /// the address with the write bit and a STOP behind it), and the
        /// peer acknowledges that address - one address match more.
        static constexpr uint8_t refused_close_hits = 1;
    };

    static bool init() { return Host::init(clock); }

    /// SR2.MSL: this side is the bus controller in the tenure under way -
    /// set as its START goes out. A plain read of SR2; inside start(),
    /// which is the only place letter x asks, no ADDR can stand (the
    /// address is written by the event vector, which a landing interrupt
    /// holds off).
    static bool on_wire() { return Res::host(); }

    static const char* dark() {
        // finish(), recover() and init() put CR2's enables back to ITERREN
        // alone in one store, the event vector, the buffer vector, DMAEN and
        // LAST with them (the class comment): between tenures, no request
        // can be raised.
        if ((Res::regs().CR2 & (I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_DMAEN |
                                I2C_CR2_LAST)) != I2C_CR2_ITERREN) {
            return "the host's enables are not at rest (ITERREN alone: no event vector, no DMA request)";
        }
        if (Res::starting() || Res::stopping()) {
            return "a START or a STOP stands in CR1";
        }
        if (TxEngine::busy() || TxStream::enabled() || RxStream::enabled()) {
            return "a DMA stream of the host holds a block";
        }
        if (Res::busy() || Res::host()) {
            return "the bus monitor reads BUSY, or MSL stands";
        }
        if (!SclPin::read() || !SdaPin::read()) {
            return "a line reads low on its pad";
        }
        return nullptr;
    }

    static void print_counters() {
        print(serial, "; spurious BERR ", Host::spurious_bus_errors(), ", transmit tails abandoned ", TxEngine::faults(),
              ", receive faults ", RxEngine::faults(), " (since boot)");
    }
};

/// FMPI2C1, the STM32G0's register file.
struct OnFmpi2c1 {
    /// FMPI2C1 on PC6/PC7 at AF4 (DS10693 table 11), the only pair of the
    /// block's pads a 64-pin package bonds.
    static constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4}, .sda = {'C', 7, PinFunction::af4}};
    /// FMPI2C1_TX on DMA1 stream 5 and FMPI2C1_RX on stream 2, both channel
    /// 2 (RM0390 table 28): the one cell each, which the reserve checks.
    using TxEngine = DmaTxEngine<1, 5, 2>;
    using RxEngine = DmaRxEngine<1, 2, 2>;
    using Host = FmpI2cHost<1, pins, TxEngine, RxEngine>;
    using Res = FmpI2c<1>;
    using SclPin = Pin<'C', 6>;
    using SdaPin = Pin<'C', 7>;
    using TxStream = DmaStream<1, 5>;
    using RxStream = DmaStream<1, 2>;
    static constexpr const char* name = "FMPI2C1";
    static constexpr const char* image = "test_stm32f4_rubato_fmpi2c";
    /// Sm, Fm and Fm+, the three rows the kernel clock's 45 MHz produces
    /// (init() says which it could, speed_ok()); the G0 peer is solved for
    /// Fm+.
    static constexpr FmpI2cSpeed speeds[] = {FmpI2cSpeed::standard_100k, FmpI2cSpeed::fast_400k,
                                             FmpI2cSpeed::fast_plus_1m};
    static constexpr uint32_t hz(FmpI2cSpeed s) { return fmpi2c_speed_hz(s); }

    struct Traits : PeerTraits {
        /// Letter o's table: with the engines named every data half rides
        /// them, started before the START (23.4.16) - the host keeps no
        /// pump for data and no crossover; the address phase and the end
        /// stay on the vectors. Ends on a plain write.
        static constexpr Shape shapes[] = {
            {0, 0, "the probe: the address alone, STOPF the edge"},
            {0, 1, "engines: the receive stream, one byte"},
            {0, 2, "engines"},
            {0, 3, "engines"},
            {0, 8, "engines"},
            {0, 32, "engines"},
            {1, 1, "engines both halves, TC's repeated START in one CR2 store"},
            {1, 2, "engines both halves"},
            {2, 8, "engines both halves"},
            {1, 0, "engines: the transmit stream, one byte, STOPF the edge"},
            {2, 0, "engines"},
            {3, 0, "engines"},
            {8, 0, "engines"},
            {32, 0, "engines"},
        };
        static_assert(Host::has_engines, "the table names engine paths");
    };

    /// test_stm32f4_fmpi2c's configuration: the APB1 kernel clock, no
    /// filters, the port's pull-ups beside the bench's 2.2 kOhm pair; and
    /// the 20 mA pad drive a Fast-mode Plus bus wants, which nothing turns
    /// on by itself (docs/stm32f4/fmpi2c.md) - this image runs the 1 MHz
    /// row.
    static bool init() {
        const bool ok = Host::init(clock, FmpI2cHostConfig{.kernel = FmpI2cClock::pclk, .filters = {},
                                                           .internal_pull_up = true, .bus = {}});
        Host::fast_plus_drive(true);
        return ok && Host::fast_plus_drive();
    }

    /// ISR.BUSY: the bus monitor saw a START and no STOP since (23.7.7).
    static bool on_wire() { return Res::busy(); }

    static const char* dark() {
        if (!Host::idle()) {
            return "the host's engine is not between tenures";
        }
        // At rest the event and error enables the engine arms at init()
        // (RXIE, TXIE, NACKIE, STOPIE, ERRIE), TCIE taken away by finish(),
        // and both DMA request enables clear (put_engines_away()).
        constexpr uint32_t at_rest = FmpI2cInterrupt::rx | FmpI2cInterrupt::tx | FmpI2cInterrupt::nack |
                                     FmpI2cInterrupt::stop | FmpI2cInterrupt::error;
        if (Res::interrupts() != at_rest || Res::dma_transmit() || Res::dma_receive()) {
            return "the host's enables are not at rest (TCIE off, both DMA requests off)";
        }
        if (Res::starting()) {
            return "a START stands in CR2";
        }
        if (TxEngine::busy() || TxStream::enabled() || RxStream::enabled()) {
            return "a DMA stream of the host holds a block";
        }
        if (Res::busy()) {
            return "the bus monitor reads BUSY";
        }
        if (!SclPin::read() || !SdaPin::read()) {
            return "a line reads low on its pad";
        }
        return nullptr;
    }

    static void print_counters() {
        print(serial, "; spurious BERR ", Host::spurious_bus_errors(), ", transmit tails abandoned ", TxEngine::faults(),
              ", receive faults ", RxEngine::faults(), " (since boot); ES0298 2.12.8's stall band ",
              Host::transmit_stall_risk() ? "IN FORCE" : "clear");
    }
};

using On = std::conditional_t<suite_on_fmpi2c, OnFmpi2c1, OnI2c1>;
static_assert(On::TxEngine::stream == (suite_on_fmpi2c ? 5u : 6u) &&
                  On::RxEngine::stream == (suite_on_fmpi2c ? 2u : 0u),
              "the DMA vectors the preprocessor names are the engines' own streams");

using Host = On::Host;
using TxEngine = On::TxEngine;
using RxEngine = On::RxEngine;

constexpr uint8_t peer_kind = twilink::peer_stm32g0;
constexpr const auto& speeds = On::speeds;
constexpr uint32_t speed_hz(decltype(Host::Request::speed) s) { return On::hz(s); }

/// What the shared body changes for this host and this peer.
struct I2cTraits : On::Traits {};

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

bool host_on_wire() { return On::on_wire(); }

const char* host_dark() {
    const char* why = pressure_dark();
    return why != nullptr ? why : On::dark();
}

void print_host() { On::print_counters(); }

bool clock_ok = false;
bool tick_ok = false;
bool ruler_ok = false;
bool host_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
          " ruler=", ruler_ok ? "DWT" : "FAILED", " ", On::name, "=", host_ok ? "host" : "FAILED", crlf);
    print(serial, On::image, " - the I2C stack under Rubato on ", On::name,
          " against the STM32G0B1RE (twi_peer)", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void USART2_IRQHandler() {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }

/// The pressure: TIM2's update, where it landed, and the post.
extern "C" void TIM2_IRQHandler() {
    Pulse::clear_flags(Pulse::update_flag);
    pressure_landed();
}

/// The host's event vector: every byte decision of a tenure, and its end.
extern "C" void BRIO_SUITE_I2C_EV_HANDLER() {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

/// The host's error vector: a NACK, a lost arbitration, a counted BERR.
extern "C" void BRIO_SUITE_I2C_ER_HANDLER() {
    if (Host::error_isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

/// The two streams' vectors, one a stream and shared with nothing: on I2C1
/// the receive block's completion ends a read phase, on FMPI2C1 only a
/// transfer error ends a tenure here.
extern "C" void BRIO_SUITE_TX_DMA_HANDLER() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}
extern "C" void BRIO_SUITE_RX_DMA_HANDLER() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    ruler_ok = Ruler::init();
    brio::Dma<1>::init();
    host_ok = On::init();
    Pulse::init();
    (void)Pulse::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFF'FFFFu});
    brio::Nvic::enable(Pulse::irq());
    brio::scheduler_init();

    bench.letter('n', "the command channel through the arbiter", tn_command_channel);
    bench.letter('o', "the tenure shapes on every path of the host", to_shapes);
    bench.letter('p', "the speeds, and the peer's commanded stretch", tp_speeds);
    if constexpr (suite_on_fmpi2c) {
        // STOPPED, OUTSIDE z: FmpI2cHost with its engines named answers a
        // refused data byte i2c_nack_addr (the header says why) - measured
        // on the third byte of six and the last of a 2+4's write, every
        // pass. A driver defect this lane reports; the letter runs by its
        // name and fails those two verdicts until the driver is mended.
        bench.letter('q', "the vocabulary under pressure: deaf, a refused byte, a refused last byte", tq_vocabulary,
                     false);
    } else {
        bench.letter('q', "the vocabulary under pressure: deaf, a refused byte, a refused last byte", tq_vocabulary);
    }
    bench.letter('w', "the arbiter with three requesters of three priorities", tw_arbiter);
    bench.letter('h', "the held wire, with requests arriving while it is held", th_held_wire);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.letter('x', "the landings inside start() counted: 400 writes silent and pressed", tx_fill, false);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
