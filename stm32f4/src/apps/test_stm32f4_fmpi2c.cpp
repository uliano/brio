// test_stm32f4_fmpi2c - the reference bench suite for the STM32F4's
// FAST-MODE PLUS I2C: the resource over the whole of RM0390 ch. 23, the
// FmpI2cHost engine under util/i2c_bus.hpp's arbiter, the DMA engines and
// the recovery verbs - stm32f4/fmpi2c.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE BUS IS THE BENCH'S SELF-LINK, AND MOSTLY NOBODY ANSWERS ON IT. The
// block exists on one board of this stratum's (the FMPI2C is the F410's,
// F412's, F413/F423's and F446's), and the bench wires its two pads to the
// same chip's I2C1. Every letter but m keeps I2C1 silent, so the suite
// measures WHAT A BUS WITH NOBODY ANSWERING SHOWS, which is more than it
// sounds; letter m makes I2C1 a target and moves bytes:
//
//   FMPI2C1   SCL PC6, SDA PC7 (AF4), the port's pull-ups asked for as
//             well; the bench's 2.2 kOhm resistors on each line hold the
//             wire, and the port alone is enough to let the block drive,
//             release and read its own wire on a board without them
//   I2C1      SCL PB8, SDA PB9 (AF4), wired to PC6 and PC7: letter m's
//             target at 0x3A, served from its own two vectors
//
//   - the register file at reset and every gate the chapter puts on a
//     write;
//   - the timing arithmetic against the manual's own tables 134 and 135,
//     at each of the three kernel clocks the multiplexer offers (this
//     instance's APB1, SYSCLK and the HSI) and for each of the three
//     speeds - which is what an INDEPENDENT CLOCK is for and what the
//     other I2C of this family has not got;
//   - every one of the 112 addresses answered with a NACK, which is a
//     complete transaction: START, address, no acknowledge, and the STOP
//     the peripheral sends by itself;
//   - SCL counted on the clock pad's OWN INPUT BUFFER while such a
//     transaction runs - an I2C pad is an open-drain alternate function,
//     so its IDR is the wire;
//   - the SMBus time-out unit, whose IDLE mode needs no peer at all: it
//     watches both lines standing high, which is what an empty bus does;
//   - the DMA engines armed and torn down around a tenure that ends in a
//     NACK;
//   - and the arbiter over all of it.
//
// and, in letter m, the whole data path against a real target.
//
// What is exercised, letter by letter:
//   a  the block: presence, the gate, the reset values, the two vectors
//   b  the timing arithmetic and what it puts in TIMINGR
//   c  the kernel-clock multiplexer, and what each source can produce
//   d  what the driver refuses, and that a refusal writes nothing
//   e  the address scan: 112 addresses, 112 NACKs, and the STOP after one
//   f  the speeds, with SCL counted on the pad: no rate above the speed
//      asked, a measured wire stated to init() run at the speed itself,
//      and the classic I2C1's own arithmetic on the same wire beside it
//   g  the SMBus time-out unit on an idle bus
//   h  the noise filters and the Fm+ pad drive
//   i  THE KERNEL: I2cBus over FmpI2cHost
//   j  the DMA engines around a tenure that NACKs
//   k  the recovery verbs
//   l  the client, configured and never addressed
//   m  THE SELF-LINK: FMPI2C1 the host, I2C1 the target - the probe, an
//      absent address, a write, reads of one to four and eight bytes and
//      a write-then-read at 100 and 400 kHz, the same through the DMA
//      engines, the engine each Request names judged on the streams (a
//      write of one with no stream, of two with the stream on its second
//      byte and a read of one under `dma`, `interrupt` on the engined host
//      touching no stream, `dma` refused on the engineless one), the last
//      written byte of a write-then-read refused on the pump and under `dma`,
//      and the next write byte for byte after any refusal; declined by name
//      where nothing answers 0x3A
//
// and against the desk partner's `twi_peer` (the second set, `t`):
//   n  Fast-mode Plus acknowledged, a held SDA and a held clock, the rebase
//   o  ES0298 2.12.8: writes in and out of the clock-ratio band, on the
//      interrupt and on the DMA engines, the partner's count and sum judging
//      every byte, no stall on the host's own path, and the erratum's stall
//      staged by the letter beside it as the control
//   p  a refused write, position by position: on the address, on the first,
//      second and a later data byte, a write of one and of two, under both
//      engines and both speeds and in and out of the band, the partner's tally
//      judging each, the next request whole
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/fmpi2c.hpp"
#include "stm32f4/i2c.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/syscfg.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#include "../../../avrxt/src/apps/twi_link_client.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000,
                             brio::HseMode::bypass>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the console ---------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

// TWO SETS: `z` the board's own letters - most of them measure a bus
// NOBODY answers, so the desk partner is on its dark console - and `t`,
// whose instrument is the partner's `twi_peer` on the same wire and which
// FAILS without it.
TwoKeyTestBench<Serial, 't'> bench;

// ---- the bus -----------------------------------------------------------------------------

/// DS10693 table 11: FMPI2C1_SCL is on PC6 and FMPI2C1_SDA on PC7 at AF4 -
/// the only pair of this peripheral's eight pads a 64-pin package bonds
/// (the others are PD11..PD15 and PF13..PF15, ports this package has not).
constexpr FmpI2cPins fmp_pins{.scl = {'C', 6, PinFunction::af4},
                              .sda = {'C', 7, PinFunction::af4}};

using S = FmpI2c<1>;
using Host = FmpI2cHost<1, fmp_pins>;

/// RM0390 table 28: one cell per direction, both on DMA1 channel 2 - the
/// transmit request on stream 5 and the receive one on stream 2. The
/// reserve checks these two cells at compile time.
using TxEngine = DmaTxEngine<1, 5, 2>;
using RxEngine = DmaRxEngine<1, 2, 2>;
using DmaHost = FmpI2cHost<1, fmp_pins, TxEngine, RxEngine>;

using Peer = FmpI2cClient<1, fmp_pins>;

/// I2C1 on PB8/PB9 (AF4), joined to PC6/PC7 by the bench's two wires: letter
/// m's target, and letter f's second host.
constexpr I2cPins classic_pins{.scl = {'B', 8, PinFunction::af4},
                               .sda = {'B', 9, PinFunction::af4}};

/// The two pads, read as inputs: what the wire is doing.
using SclPad = Pin<'C', 6>;
using SdaPad = Pin<'C', 7>;

/// The port's own pull-ups are asked for besides the bench's 2.2 kOhm pair:
/// they let the block drive, release and read its own wire on a board
/// without them.
constexpr FmpI2cHostConfig base_config{.kernel = FmpI2cClock::pclk,
                                       .filters = {},
                                       .internal_pull_up = true,
                                       .bus = {}};

volatile bool bus_ao_live = false;
volatile bool dma_host_live = false;
volatile bool peer_live = false;
volatile bool xfer_done = false;
/// Entries of the two DMA streams' vectors, counted whoever owns them.
volatile uint32_t tx_dma_entries = 0;
volatile uint32_t rx_dma_entries = 0;

/// What the EVENT vector saw, in order, for the tenure being watched: the
/// ISR's own reading of ISR before the engine touches it. Four entries is
/// more than a probe can raise.
volatile uint32_t ev_log[4] = {};
volatile uint8_t ev_n = 0;
volatile uint8_t er_n = 0;
/// Every ISR value the ERROR vector met, ORed, and the tick of its first
/// entry: letter n's TIMEOUT is swept by the engine's error body, and this
/// is how the letter sees that it came and when.
volatile uint32_t er_seen = 0;
volatile uint32_t er_first_tick = 0;

void arm_log() {
    ev_n = 0;
    er_n = 0;
    er_seen = 0;
    er_first_tick = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        ev_log[i] = 0;
    }
}

// ---- the rulers ------------------------------------------------------------------------

/// SysTick's reload period in core cycles - what a VAL delta wraps by. VAL
/// is the one ruler fine enough for a bit period, and reading it has no
/// side effect (cortexm/delay.hpp says why).
uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// Core cycles since `from`, VAL counting DOWN and wrapping at the reload.
uint32_t since(uint32_t from, uint32_t now, uint32_t period) {
    return (from >= now) ? (from - now) : (from + period - now);
}

// ---- the bus, in the state the letters expect ----------------------------------------------

bool host_ready(FmpI2cClock kernel = FmpI2cClock::pclk) {
    bus_ao_live = false;
    dma_host_live = false;
    peer_live = false;
    FmpI2cHostConfig c = base_config;
    c.kernel = kernel;
    return Host::init(clock, c);
}

/// What this suite reports for a tenure that never answered - the code an
/// arbiter with a timeout would replace with i2c_timeout.
constexpr uint8_t parked = 0xFF;

/// One tenure, driven by hand (the arbiter skipped, as i2c-bus.md allows a
/// device that owns a bus alone). Returns the status the engine ended with,
/// or `parked`; the wait is bounded.
/// THE ENGINE: `dma` on the engined host, which is what DmaHost is in this
/// suite for, and `interrupt` on the plain one, unless the caller names it.
template <typename Bus>
constexpr I2cEngine engine_of() {
    return Bus::has_dma ? I2cEngine::dma : I2cEngine::interrupt;
}

template <typename Bus>
uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
               FmpI2cSpeed speed = FmpI2cSpeed::standard_100k,
               I2cEngine engine = engine_of<Bus>()) {
    typename Bus::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine;
    xfer_done = false;
    if (Bus::start(r)) {
        return Bus::status();   // the synchronous refusal
    }
    for (uint32_t spins = 8'000'000u; spins != 0u; --spins) {
        if (xfer_done) {
            return Bus::status();
        }
    }
    return parked;
}

/// The empty Request: START, the address, STOP - the acknowledge is the
/// answer, and on this bus there is never one.
uint8_t probe(uint8_t addr, FmpI2cSpeed s = FmpI2cSpeed::standard_100k) {
    return tenure<Host>(addr, nullptr, 0, nullptr, 0, s);
}

// ---- what the registers held before this program touched them --------------------------------

struct BootState {
    bool gate = false;
    uint32_t cr1 = 0, cr2 = 0, oar1 = 0, oar2 = 0, timingr = 0, timeoutr = 0, isr = 0, pecr = 0;
    uint8_t kernel_code = 0;
    bool scl = false, sda = false;   ///< the two lines, read as plain inputs
    uint32_t isr_ready = 0;          ///< ISR once the pads are the peripheral's
    bool smbus = false;              ///< the silicon's own answer, through TIMEOUTR
    uint32_t wupen_readback = 0;     ///< CR1 after a write of the G0's WUPEN bit
};
BootState boot;

// =============================================================================
// a - the block
// =============================================================================

void ta_block() {
    print(serial, "  FMPI2C1 gate at reset: ", boot.gate ? "OPEN" : "closed", "; CR1 ",
          hex(boot.cr1), " CR2 ", hex(boot.cr2), " OAR1 ", hex(boot.oar1), " OAR2 ",
          hex(boot.oar2), " TIMINGR ", hex(boot.timingr), " TIMEOUTR ", hex(boot.timeoutr),
          " ISR ", hex(boot.isr), " PECR ", hex(boot.pecr), crlf);
    bench.verdict("every peripheral clock is off at reset, this one included", !boot.gate);
    // 23.7: every register resets to zero but ISR, whose reset value is
    // 0x0001 - TXE stands, because an empty transmit register is what an
    // idle transmitter has.
    bench.verdict("the reset values are the chapter's: every register zero but ISR, which is "
                  "0x0001 because TXE stands out of reset",
                  boot.cr1 == 0u && boot.cr2 == 0u && boot.oar1 == 0u && boot.oar2 == 0u &&
                      boot.timingr == 0u && boot.timeoutr == 0u && boot.pecr == 0u &&
                      boot.isr == 0x1u);
    print(serial, "  the wire at boot, read as plain inputs with the port's pull-ups on: SCL ",
          boot.scl ? "high" : "LOW", ", SDA ", boot.sda ? "high" : "LOW",
          "; ISR.BUSY with the pads NOT yet handed over: ",
          (boot.isr & FmpI2cFlag::busy) != 0u ? "SET" : "clear", "; after init(): ",
          (boot.isr_ready & FmpI2cFlag::busy) != 0u ? "SET" : "clear", crlf);
    bench.verdict("both pads read high: nothing on this bus but the port's own pull-ups",
                  boot.scl && boot.sda);
    // BUSY is set by a START and cleared by a STOP or by PE = 0 (23.7.7),
    // and the PE cycle 23.4.6 prescribes is part of every init() here - so
    // unlike the other I2C of this family, this block cannot come up busy
    // over an idle wire.
    bench.verdict("... and the peripheral agrees: BUSY is clear once init() has handed the pads "
                  "over and cycled PE",
                  (boot.isr_ready & FmpI2cFlag::busy) == 0u);

    print(serial, "  vectors: EV ", static_cast<int32_t>(S::event_irq), ", ER ",
          static_cast<int32_t>(S::error_irq), "; the other I2C's I2C1_EV is ",
          static_cast<int32_t>(i2c_event_irq(1)), crlf);
    bench.verdict("this block has TWO vectors of its own, the events on one and the errors on "
                  "the other, and neither is any I2C's",
                  S::event_irq == FMPI2C1_EV_IRQn && S::error_irq == FMPI2C1_ER_IRQn &&
                      S::event_irq != i2c_event_irq(1) && S::error_irq != i2c_error_irq(1));

    print(serial, "  table 127 as this driver carries it: 10-bit ", S::has_ten_bit ? "yes" : "no",
          ", Fm+ ", S::has_fast_plus ? "yes" : "no", ", SMBus claimed ",
          S::smbus_claimed ? "yes" : "no", " and PROBED ", boot.smbus ? "yes" : "no",
          ", wake from Stop ", S::wakes_from_stop ? "yes" : "no", crlf);
    // 23.7.6: without the SMBus half TIMEOUTR is "reserved, and its bits
    // are forced by hardware to 0" - so a value that reads back is the
    // silicon's own answer to the implementation table.
    bench.verdict("the SMBus half is there, and the peripheral says so itself: a value written "
                  "into TIMEOUTR reads back",
                  S::smbus_claimed && boot.smbus);
    // Table 127's dash, and the header's silence: CR1 bit 18 is where the
    // STM32G0's identical register file keeps WUPEN.
    print(serial, "  CR1 after writing bit 18 (the STM32G0's WUPEN): ", hex(boot.wupen_readback),
          "; the verb answers ", S::wake_from_stop(true) ? "true" : "false", crlf);
    bench.verdict("this block has NO wake from Stop: the bit does not stick and the verb refuses",
                  !S::wakes_from_stop && (boot.wupen_readback & (1u << 18)) == 0u &&
                      !S::wake_from_stop(true) && !S::wake_from_stop());

    print(serial, "  APB1 = ", Host::reference_hz() / 1000u, " kHz, SYSCLK = ",
          SysClock::hz / 1000u, " kHz, the kernel in force = ", Host::kernel_hz() / 1000u,
          " kHz (FMPI2C1SEL was ", boot.kernel_code, " at reset)", crlf);
    bench.verdict("the multiplexer comes up on the APB clock (code 0), and init() left it there",
                  boot.kernel_code == 0u && Host::kernel_hz() == SysClock::pclk1_hz &&
                      S::kernel_clock() == FmpI2cClock::pclk);
}

// =============================================================================
// b - the timing arithmetic
// =============================================================================

void tb_timing() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // The manual's own tables, priced by this file's arithmetic. They are
    // static_asserts in the header too; here they are printed, because a
    // reader of a bench log should see the chapter reproduced.
    const uint32_t sm8 = fmpi2c_scl_hz(8'000'000UL, FmpI2cTiming{0x1, 0x13, 0xF, 2, 4}, 1000u);
    const uint32_t fm16 = fmpi2c_scl_hz(16'000'000UL, FmpI2cTiming{0x1, 0x9, 0x3, 2, 3}, 750u);
    const uint32_t fp16 = fmpi2c_scl_hz(16'000'000UL, FmpI2cTiming{0x0, 0x4, 0x2, 0, 2}, 500u);
    print(serial, "  tables 134/135 priced: Sm at 8 MHz ", sm8, " Hz, Fm at 16 MHz ", fm16,
          " Hz, Fm+ at 16 MHz ", fp16, " Hz", crlf);
    bench.verdict("the manual's own example values price out at the manual's own frequencies",
                  sm8 == 100'000UL && fm16 == 400'000UL && fp16 == 1'000'000UL);

    const uint32_t ker = Host::kernel_hz();
    bool never_faster = true;
    for (uint8_t i = 0; i < fmpi2c_speed_count; ++i) {
        const FmpI2cSpeed s = static_cast<FmpI2cSpeed>(i);
        const FmpI2cTiming t = Host::timing_of(s);
        const uint32_t made = Host::scl_hz(s);
        print(serial, "  ",
              i == 0 ? "Sm  100k" : (i == 1 ? "Fm  400k" : "Fm+ 1000k"),
              ": ", Host::speed_ok(s) ? "reachable" : "REFUSED  ", " PRESC ", t.presc, " SCLL ",
              t.scll, " SCLH ", t.sclh, " SDADEL ", t.sdadel, " SCLDEL ", t.scldel, " -> ", made,
              " Hz; tSCLL ", fmpi2c_scll_ns(ker, t), " ns tSCLH ", fmpi2c_sclh_ns(ker, t),
              " ns tSDADEL ", fmpi2c_sdadel_ns(ker, t), " ns tSCLDEL ",
              fmpi2c_scldel_ns(ker, t), " ns", crlf);
        if (Host::speed_ok(s) && made > fmpi2c_speed_hz(s)) {
            never_faster = false;
        }
    }
    bench.verdict("all three speeds are reachable at 45 MHz of kernel clock",
                  Host::speed_ok(FmpI2cSpeed::standard_100k) &&
                      Host::speed_ok(FmpI2cSpeed::fast_400k) &&
                      Host::speed_ok(FmpI2cSpeed::fast_plus_1m));
    bench.verdict("... and not one of them runs FASTER than the speed asked for, at a kernel "
                  "rate that divides it (Fm+) or one that does not (Fm: 45 MHz is not a "
                  "multiple of 400 kHz)",
                  never_faster);

    // What the registers really hold once a speed is applied.
    const FmpI2cTiming want = Host::timing_of(FmpI2cSpeed::standard_100k);
    const FmpI2cTiming got = S::timing();
    print(serial, "  TIMINGR in force ", hex(S::timing_reg()), ": PRESC ", got.presc, " SCLL ",
          got.scll, " SCLH ", got.sclh, " SDADEL ", got.sdadel, " SCLDEL ", got.scldel, crlf);
    bench.verdict("the register in force is exactly the row the arithmetic solved for the speed "
                  "in force",
                  got.presc == want.presc && got.scll == want.scll && got.sclh == want.sclh &&
                      got.sdadel == want.sdadel && got.scldel == want.scldel);
    bench.verdict("... and it reads back through the unpacking as it was packed",
                  fmpi2c_timingr(got) == S::timing_reg());

    // 23.4.5's two conditions, on the values in force.
    const FmpI2cBusTiming bus = fmpi2c_bus_timing(FmpI2cSpeed::standard_100k);
    bench.verdict("the chosen value meets 23.4.5's setup and hold conditions at this kernel "
                  "clock",
                  fmpi2c_setup_ok(ker, got, bus) && fmpi2c_hold_ok(ker, got, FmpI2cFilters{}, bus));
    print(serial, "  the minimum SCL stretch this value costs every falling edge: ",
          fmpi2c_min_stretch_cycles(got), " kernel periods = ",
          fmpi2c_cycles_ns(ker, fmpi2c_min_stretch_cycles(got)), " ns", crlf);
}

// =============================================================================
// c - the kernel-clock multiplexer
// =============================================================================

void tc_kernel_clock() {
    struct Row {
        FmpI2cClock code;
        const char* name;
    };
    const Row rows[3] = {{FmpI2cClock::pclk, "APB1  "},
                         {FmpI2cClock::sysclk, "SYSCLK"},
                         {FmpI2cClock::hsi, "HSI   "}};
    uint8_t up = 0;
    uint32_t rates[3] = {};
    bool fast_plus[3] = {};
    bool risk[3] = {};
    for (uint8_t i = 0; i < 3u; ++i) {
        if (!host_ready(rows[i].code)) {
            print(serial, "  ", rows[i].name, ": init REFUSED", crlf);
            continue;
        }
        ++up;
        rates[i] = Host::kernel_hz();
        fast_plus[i] = Host::speed_ok(FmpI2cSpeed::fast_plus_1m);
        risk[i] = Host::transmit_stall_risk();
        // A speed the kernel clock cannot make has no row in the table, so
        // its produced rate is not a number: it is printed as a dash and
        // not as the arithmetic's answer for an empty value.
        print(serial, "  ", rows[i].name, ": readback ",
              static_cast<uint8_t>(S::kernel_clock()), ", kernel ", rates[i] / 1000u,
              " kHz; Sm ", Host::speed_ok(FmpI2cSpeed::standard_100k) ? "yes" : "NO ", " (",
              Host::scl_hz(FmpI2cSpeed::standard_100k), " Hz), Fm ",
              Host::speed_ok(FmpI2cSpeed::fast_400k) ? "yes" : "NO ", " (",
              Host::scl_hz(FmpI2cSpeed::fast_400k), " Hz), Fm+ ",
              fast_plus[i] ? "yes" : "NO ", " (");
        if (fast_plus[i]) {
            print(serial, Host::scl_hz(FmpI2cSpeed::fast_plus_1m), " Hz)");
        } else {
            print(serial, "no row)");
        }
        print(serial, "; ES0298 2.12.8 ratio APB/kernel ",
              risk[i] ? "IN the 1.5..3 band" : "outside", crlf);
    }
    bench.verdict("the multiplexer takes all three of its sources and reads each one back",
                  up == 3u);
    bench.verdict("each source really moves the kernel: 45 MHz on the APB, 180 on SYSCLK, 16 on "
                  "the HSI - which is the whole point of an INDEPENDENT clock, and what the "
                  "other I2C of this family has not got",
                  rates[0] == SysClock::pclk1_hz && rates[1] == SysClock::hz &&
                      rates[2] == 16'000'000UL);
    bench.verdict("ES0298 2.12.2's floor drops Fm+ on the HSI and keeps it on the other two: a "
                  "20 MHz kernel is the price of a 1 MHz bus",
                  fast_plus[0] && fast_plus[1] && !fast_plus[2]);
    bench.verdict("ES0298 2.12.8's band is where the arithmetic puts it: the HSI against a "
                  "45 MHz APB is a ratio of 2.8 and sits inside it, the other two do not - and "
                  "the driver publishes that instead of promising a transmission",
                  !risk[0] && !risk[1] && risk[2]);
    (void)host_ready();
}

// =============================================================================
// d - what the driver refuses
// =============================================================================

void td_refusals() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // TIMINGR, the filters and NOSTRETCH are all PE-gated (23.7.1, 23.7.5).
    const uint32_t before = S::timing_reg();
    bench.verdict("TIMINGR is refused while the peripheral is enabled",
                  S::enabled() && !S::timing(FmpI2cTiming{1, 2, 3, 4, 5}));
    bench.verdict("... and a refused write leaves the register as it was",
                  S::timing_reg() == before);
    bench.verdict("the noise filters are refused there too",
                  !S::filters(FmpI2cFilters{.analog = false, .digital = 4}));
    bench.verdict("and so is NOSTRETCH", !S::no_stretch(true));
    bench.verdict("a whole configuration is refused there as well",
                  !S::configure(FmpI2cConfig{}));

    (void)S::disable();
    bench.verdict("with PE down the same writes are accepted",
                  S::timing(FmpI2cTiming{1, 2, 3, 4, 5}) &&
                      S::filters(FmpI2cFilters{.analog = false, .digital = 4}) &&
                      S::no_stretch(true));
    bench.verdict("a digital filter past the four bits of DNF is refused",
                  !S::filters(FmpI2cFilters{.digital = 16}));
    bench.verdict("23.4.8's incompatible pair - byte control with the clock stretch given up - "
                  "is refused as a configuration",
                  !S::configure(FmpI2cConfig{.no_stretch = true, .byte_control = true}) &&
                      fmpi2c_config_valid(FmpI2cConfig{.byte_control = true}));
    (void)S::no_stretch(false);

    // The addresses.
    (void)S::addresses(FmpI2cAddressConfig{.own = 0x41});
    const uint32_t oar = S::oar1();
    bench.verdict("an own address past seven bits is refused",
                  !S::addresses(FmpI2cAddressConfig{.own = 0x80}));
    bench.verdict("... and a refused address configuration writes nothing", S::oar1() == oar);
    bench.verdict("the seven-bit address reads back where OA1 keeps it, shifted up one",
                  S::own_address() == 0x41u);
    bench.verdict("a ten-bit address of the same value is a different register word",
                  S::addresses(FmpI2cAddressConfig{.own = 0x123,
                                                   .mode = FmpI2cAddressMode::ten_bit}) &&
                      S::own_address10() == 0x123u);
    // OAR2's mask, whose codes are how many low bits go uncompared.
    bench.verdict("the second address takes its mask, and the mask says how many of the seven "
                  "bits are still compared",
                  S::addresses(FmpI2cAddressConfig{.own = 0x41,
                                                   .second = 0x42,
                                                   .second_mask = FmpI2cOa2Mask::low_2,
                                                   .second_enable = true}) &&
                      fmpi2c_oa2_compared_bits(FmpI2cOa2Mask::low_2) == 5u &&
                      (S::oar2() & (7u << 8)) == (2u << 8));
    bench.verdict("a mask code past the three bits is refused",
                  !fmpi2c_oa2_mask_valid(static_cast<FmpI2cOa2Mask>(8)));

    // CR2's own gate: nothing of it may move while START stands.
    S::enable();
    S::start();
    const bool refused_under_start = !S::transfer(0x41, false, 1, true);
    S::clear(FmpI2cClear::addr);   // 23.7.8: ADDRCF also clears START
    (void)S::cycle();
    bench.verdict("no field of CR2 may move while START stands, and the verb refuses instead of "
                  "storing",
                  refused_under_start);
    bench.verdict("AUTOEND with RELOAD is refused: the chapter says AUTOEND has no effect there, "
                  "so the pair is a caller's mistake and not a state",
                  !S::transfer(0x41, false, 4, true, true));
    bench.verdict("an address past its mode is refused", !S::transfer(0x80, false, 1, true) &&
                                                             !S::transfer(0x400, false, 1, true,
                                                                          false,
                                                                          FmpI2cAddressMode::ten_bit));
    bench.verdict("a RELOAD chunk of zero bytes is refused: writing zero clears nothing and "
                  "releases nothing",
                  !S::reload(0, true, false));

    // The engine's one synchronous refusal, on the kernel clock that cannot
    // make the speed.
    (void)host_ready(FmpI2cClock::hsi);
    const uint8_t st = probe(0x42, FmpI2cSpeed::fast_plus_1m);
    print(serial, "  a Fm+ request on the HSI's 16 MHz kernel came back ", st, " (", i2c_rejected,
          " is i2c_rejected)", crlf);
    bench.verdict("a speed this kernel clock cannot produce is answered i2c_rejected inside "
                  "start(), no byte moved - refused, never silently slowed",
                  st == i2c_rejected);
    (void)host_ready();

    // The other: an address no Request may name (util/i2c_bus.hpp) - the
    // general call, the codes below 0x08, the four 10-bit prefixes and the
    // four above them - answered true and i2c_rejected by start() with
    // nothing written: CR2 as it was (no START, no address) and no
    // interrupt enabled.
    const uint32_t cr2 = S::cr2();
    const uint32_t ints = S::interrupts();
    const uint8_t refused[] = {0x00, 0x07, 0x78, 0x7A, 0x7B, 0x7C, 0x7F};
    uint8_t refusals = 0;
    for (const uint8_t a : refused) {
        Host::Request r{};
        r.addr = a;
        if (Host::start(r) && Host::status() == i2c_rejected && S::cr2() == cr2 && S::interrupts() == ints) {
            ++refusals;
        }
    }
    print(serial, "  addresses outside 0x08..0x77 refused inside start(), nothing written: ", refusals,
          " of ", static_cast<uint8_t>(sizeof refused), crlf);
    bench.verdict("an address outside 0x08..0x77 is answered i2c_rejected inside start(), CR2 and the "
                  "interrupt enables untouched",
                  refusals == sizeof refused);
}

// =============================================================================
// e - the address scan
// =============================================================================

void te_scan() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    uint16_t nacked = 0;
    uint16_t answered = 0;
    uint16_t other = 0;
    for (uint8_t a = 0x08; a <= 0x77u; ++a) {
        const uint8_t st = probe(a);
        if (st == i2c_nack_addr) {
            ++nacked;
        } else if (st == i2c_ok) {
            ++answered;
            print(serial, "  an address ANSWERED: ", hex(a), crlf);
        } else {
            ++other;
        }
    }
    print(serial, "  ", nacked, " of 112 addresses NACKed, ", answered, " answered, ", other,
          " ended some other way", crlf);
    bench.verdict("every one of the 112 addresses comes back i2c_nack_addr: there is no device "
                  "on this bus, and the engine says so in the arbiter's own vocabulary",
                  nacked == 112u && answered == 0u && other == 0u);

    // The order of the two flags, read by the ISR before the engine touched
    // them. A NACK is not the end: 23.4.9 makes the peripheral send the
    // STOP by itself, and the completion is that STOP.
    arm_log();
    const uint8_t st = probe(0x42);
    const uint8_t seen = ev_n;
    print(serial, "  one probe raised ", seen, " event interrupts and ", er_n,
          " error ones; ISR at each: ", hex(ev_log[0]), " ", hex(ev_log[1]), " ", hex(ev_log[2]),
          crlf);
    bench.verdict("the tenure ends i2c_nack_addr", st == i2c_nack_addr);
    bench.verdict("NACKF comes FIRST and alone - the address went unanswered and no data byte "
                  "ever moved",
                  seen >= 1u && (ev_log[0] & FmpI2cFlag::nack) != 0u &&
                      (ev_log[0] & FmpI2cFlag::stop) == 0u);
    bench.verdict("... and the STOP the peripheral sends by itself arrives as its OWN interrupt, "
                  "which is what completes the tenure (23.4.9)",
                  seen >= 2u && (ev_log[1] & FmpI2cFlag::stop) != 0u);
    bench.verdict("no error interrupt was raised: a NACK is not an error on this block, it is an "
                  "event",
                  er_n == 0u);

    // And the wire after it.
    const bool busy = S::busy();
    const bool scl = SclPad::read();
    const bool sda = SdaPad::read();
    print(serial, "  after the STOP: BUSY ", busy ? "SET" : "clear", ", the pads read SCL ",
          scl ? "high" : "LOW", " SDA ", sda ? "high" : "LOW", crlf);
    bench.verdict("the STOP really landed on the wire: BUSY is clear and both lines are released",
                  !busy && scl && sda);
    bench.verdict("the bus is fit for the next tenure", probe(0x43) == i2c_nack_addr);
}

// =============================================================================
// f - the speeds, with SCL counted on the pad
// =============================================================================

/// Count SCL's rising edges while a tenure runs, and keep the SHORTEST
/// period between two of them - which is the bit period, every software
/// sequence of the chapter only ever lengthening one. The pad is an
/// open-drain alternate function, so its input buffer reads the wire.
///
/// THE PERIOD IS READ ON THE FIRST EIGHT RISING EDGES AND NO LATER ONES: the
/// address and direction bits of a tenure that has raised no event yet. The
/// ninth rise is where the block samples the acknowledge, and its NACK entry
/// (some 230 cycles) lands right there: it dated that rise late, or at 400 kHz
/// covered SCL's whole high phase so that the rise went unseen, one run of the
/// letter in ten, a period read 12 to 16 per cent long on a wire that had
/// nothing wrong. Every rising edge is still counted in `edges`.
struct SclReading {
    uint32_t edges = 0;
    uint32_t shortest = 0;   ///< the shortest gap between two rising edges, in core cycles
    uint32_t average = 0;    ///< the mean gap over the whole tenure, in core cycles
    /// One turn of the counting loop, in core cycles: the cycles from the
    /// first rising edge to the last over the turns between them. Each
    /// edge is dated to within one turn, so the mean of the gaps is good
    /// to a turn over their number - the instrument's own resolution.
    uint32_t poll = 0;
    /// The gaps `average` is the mean of: the edges the loop dated, less one.
    uint32_t gaps = 1;
    /// A tick's entry landed between the first dated edge and the last.
    bool ticked = false;
};

/// The rising edges a period is dated over, SclReading's reason beside it.
constexpr uint32_t scl_dated_edges = 8u;

template <typename Pad = SclPad>
SclReading count_scl() {
    SclReading r{};
    const uint32_t period = systick_period();
    uint32_t last_val = SysTick->VAL;
    uint32_t shortest = 0xFFFFFFFFu;
    uint32_t total = 0;
    uint32_t turns = 0;
    uint32_t turns_at_last = 0;
    uint32_t dated = 0;   ///< the edges the loop dated, at most scl_dated_edges
    uint32_t tick_at_first = 0;
    // IDR read here and not through Pad::read(), which gcc takes out of line
    // once this file calls it often enough: a call in the loop takes a turn from
    // 26 cycles to 46, past SCL's high phase at Fm+ on this wire, and an edge
    // goes uncounted.
    GPIO_TypeDef& pads = Pad::port();
    bool prev = (pads.IDR & Pad::mask) != 0u;
    for (uint32_t spins = 40'000'000u; spins != 0u; --spins) {
        const bool done = xfer_done;
        const uint32_t v = SysTick->VAL;
        const bool now = (pads.IDR & Pad::mask) != 0u;
        // THE PASS THAT SAW THE COMPLETION READS THE PAD TOO. A completion
        // landing between a rise and this read would otherwise end the count
        // with that rise unread. It is counted and left out of the mean: it is
        // dated only to the completion's handler, which ran over it.
        if (done) {
            if (now && !prev) {
                ++r.edges;
            }
            break;
        }
        if (r.edges != 0u) {
            ++turns;
        }
        if (now && !prev) {
            if (dated < scl_dated_edges) {
                const uint32_t d = since(last_val, v, period);
                if (r.edges != 0u) {
                    total += d;
                    if (d < shortest) {
                        shortest = d;
                    }
                } else {
                    tick_at_first = Ticker::ticks();
                }
                last_val = v;
                ++dated;
                turns_at_last = turns;
                if (dated == scl_dated_edges) {
                    r.ticked = Ticker::ticks() != tick_at_first;
                }
            }
            ++r.edges;
        }
        prev = now;
    }
    r.shortest = shortest == 0xFFFFFFFFu ? 0u : shortest;
    // Rounded to the nearest cycle, as the turn below.
    r.gaps = dated > 1u ? dated - 1u : 1u;
    r.average = dated > 1u ? (total + r.gaps / 2u) / r.gaps : 0u;
    r.poll = turns_at_last == 0u ? 0u : (total + turns_at_last / 2u) / turns_at_last;
    return r;
}

/// The console's last line out before a stretch the pad is timed over: its
/// transmit interrupt, one every 87 us at 115200 and some hundred cycles an
/// entry, lands inside a clock count and dates its last edge late (one run of
/// the Fm+ row in 43 read 210 cycles of mean gap for 179). The longest line
/// here takes 30 ms; the bound is its double and more. The tick stays on: its
/// entry inside a count is reported by `ticked`, and the told row takes such a
/// count again.
void console_quiet() {
    const uint32_t t0 = Ticker::ticks();
    while (!Serial::tx_idle() && Ticker::ticks() - t0 < 100u) {
    }
}

SclReading timed_probe(uint8_t addr, FmpI2cSpeed s) {
    console_quiet();
    Host::Request r{};
    r.addr = addr;
    r.speed = s;
    xfer_done = false;
    if (Host::start(r)) {
        return SclReading{};
    }
    return count_scl();
}

/// A mean gap in core cycles as nanoseconds.
uint32_t gap_ns(uint32_t cycles) {
    return static_cast<uint32_t>((static_cast<uint64_t>(cycles) * 1000ULL) /
                                 (SysClock::hz / 1'000'000u));
}

const char* speed_name(uint8_t i) { return i == 0 ? "Sm  " : (i == 1 ? "Fm  " : "Fm+ "); }

/// I2C1, the OTHER design of this family, on the same wire as a HOST: its
/// own arithmetic (CCR, 27.6.8) beside this block's, the probe's clocks
/// counted on PB8. Live only inside letter f.
using ClassicHost = I2cHost<1, classic_pins>;
using ClassicScl = Pin<'B', 8>;
volatile bool classic_host_live = false;

void tf_speeds() {
    struct Row {
        FmpI2cClock kernel;
        const char* name;
    };
    const Row rows[3] = {{FmpI2cClock::pclk, "APB1  "},
                         {FmpI2cClock::sysclk, "SYSCLK"},
                         {FmpI2cClock::hsi, "HSI   "}};
    bool all_nacked = true;
    bool never_shorter = true;
    bool never_faster = true;
    uint32_t sm_pclk = 0;
    uint32_t fm_pclk = 0;
    uint32_t sync_pclk[fmpi2c_speed_count] = {};
    for (uint8_t k = 0; k < 3u; ++k) {
        if (!host_ready(rows[k].kernel)) {
            all_nacked = false;
            continue;
        }
        Host::fast_plus_drive(true);
        const uint32_t ker = Host::kernel_hz();
        for (uint8_t i = 0; i < fmpi2c_speed_count; ++i) {
            const FmpI2cSpeed s = static_cast<FmpI2cSpeed>(i);
            if (!Host::speed_ok(s)) {
                print(serial, "  ", rows[k].name, " ", speed_name(i), ": refused at this kernel "
                      "clock (ES0298 2.12.2)", crlf);
                continue;
            }
            const SclReading r = timed_probe(0x42, s);
            const uint8_t st = Host::status();
            const uint32_t high = r.shortest == 0u ? 0u : SysClock::hz / r.shortest;
            const uint32_t mean_hz = r.average == 0u ? 0u : SysClock::hz / r.average;
            // WHAT THE WIRE SAYS THE DETECTION DELAY IS. 23.4.9 makes
            // tSCL = tSYNC1 + tSYNC2 + tSCLL + tSCLH, and the two
            // programmed halves are known exactly - so the measured period
            // less them is the SCL detection delay this board really pays,
            // against the FLOOR the chooser charges when nothing was
            // measured (two kernel periods an edge and the filters, no
            // slope). The MEAN gap is the period: a probe's nine clocks
            // are the hardware's alone, nothing stretches them, while the
            // shortest gap moves with the polling loop's phase - by up to
            // 100 core cycles with a SYSCLK kernel and this board's 2.2
            // kOhm edges, more than the delay being measured.
            const FmpI2cTiming t = Host::timing_of(s);
            const uint32_t programmed_ns = fmpi2c_scll_ns(ker, t) + fmpi2c_sclh_ns(ker, t);
            const uint32_t measured_ns = r.average == 0u ? 0u : gap_ns(r.average);
            const bool longer = measured_ns >= programmed_ns;
            const uint32_t sync_ns = longer ? measured_ns - programmed_ns : 0u;
            const uint32_t asked_ns = 1'000'000'000UL / fmpi2c_speed_hz(s);
            print(serial, "  ", rows[k].name, " ", speed_name(i), ": the ceiling ",
                  Host::scl_hz(s), " Hz; the pad gave ", r.edges,
                  " rising edges, the shortest gap ", r.shortest, " core cycles (", high,
                  " Hz) and the mean ", r.average, " (", mean_hz, " Hz); tSCLL + tSCLH is ",
                  programmed_ns, " ns of that ", measured_ns, " ns, so the detection delay is ",
                  sync_ns, " ns against the floor's ",
                  fmpi2c_cycles_ns(ker, fmpi2c_sync_floor_cycles(ker, FmpI2cFilters{})),
                  " ns; status ", st, crlf);
            if (st != i2c_nack_addr) {
                all_nacked = false;
            }
            if (!longer) {
                never_shorter = false;
            }
            // THE CEILING: the mean period is at least the one asked for,
            // to the poll's resolution (a turn of the counting loop over
            // the gaps, and the cycle the mean is rounded to).
            if (measured_ns + gap_ns(r.poll / r.gaps + 1u) < asked_ns) {
                never_faster = false;
            }
            if (rows[k].kernel == FmpI2cClock::pclk) {
                sync_pclk[i] = sync_ns;
                if (i == 0u) {
                    sm_pclk = high;
                } else if (i == 1u) {
                    fm_pclk = high;
                }
            }
        }
        Host::fast_plus_drive(false);
    }
    bench.verdict("every one of the eight tenures reached the wire and came back with the "
                  "address unanswered", all_nacked);
    bench.verdict("SCL really moves, and faster in fast mode than in standard: the pad's own "
                  "input buffer counted both",
                  sm_pclk != 0u && fm_pclk > sm_pclk);
    bench.verdict("every period on the wire is LONGER than the two halves TIMINGR programs, "
                  "which is 23.4.9's formula with a detection delay that cannot be negative",
                  never_shorter);
    bench.verdict("AND NOT ONE OF THE EIGHT RUNS FASTER THAN THE SPEED ASKED: with no measured "
                  "tSYNC the chooser charges the floor, and this wire's slopes only add to it",
                  never_faster);

    // ---- a MEASURED wire, stated to init() ----
    // The detection delay the APB1 rows just measured, handed back: the
    // chooser then subtracts the wire's own tSYNC instead of the floor, and
    // the bus runs at the speed asked, to the prescaler's resolution.
    bool exact = true;
    for (uint8_t i = 0; i < fmpi2c_speed_count; ++i) {
        const FmpI2cSpeed s = static_cast<FmpI2cSpeed>(i);
        FmpI2cBusTiming wire = fmpi2c_bus_timing(s);
        wire.sync_ns = static_cast<uint16_t>(sync_pclk[i]);
        FmpI2cHostConfig c = base_config;
        c.bus = wire;
        if (sync_pclk[i] == 0u || !Host::init(clock, c) || !Host::speed_ok(s)) {
            exact = false;
            continue;
        }
        Host::fast_plus_drive(true);
        // The tick's window is seven gaps in a tick of 180000 cycles: 7 per cent
        // of the counts at 100 kHz, under 1 at 1 MHz. Three tries, a lengthened
        // count judged only after the third, one count in some three thousand
        // at 100 kHz, where the bound's four per cent holds a tick's entry.
        SclReading r = timed_probe(0x42, s);
        for (uint8_t again = 0; r.ticked && again != 2u; ++again) {
            r = timed_probe(0x42, s);
        }
        Host::fast_plus_drive(false);
        const uint32_t mean_hz = r.average == 0u ? 0u : SysClock::hz / r.average;
        const uint32_t want = fmpi2c_speed_hz(s);
        // Within the prescaler's resolution below the speed, and above it
        // no further than the poll resolves: a turn of the counting loop
        // over the gaps, and the cycle the mean is rounded to.
        const uint32_t slack = r.poll / r.gaps + 1u;
        const uint32_t want_cycles = SysClock::hz / want;
        print(serial, "  APB1   ", speed_name(i), " told its wire (tSYNC ", sync_pclk[i],
              " ns): the arithmetic states ", Host::scl_hz(s), " Hz, the pad's mean ", mean_hz,
              " Hz = ", r.average, " core cycles against ", want_cycles, ", the poll's turn ",
              r.poll, " over ", r.gaps, " gaps", crlf);
        if (mean_hz < want - want / 25u || r.average + slack < want_cycles) {
            exact = false;
        }
    }
    bench.verdict("... and a wire whose measured tSYNC is stated runs at the speed asked, "
                  "within four per cent under it and above it no further than the poll "
                  "resolves",
                  exact);

    // ---- the other I2C of this family, on the same wire ----
    // I2C1's arithmetic is CCR's (27.6.8), not this chapter's, and it is
    // reported here beside it: the same probe, nobody answering, the
    // clocks counted on PB8's own input buffer.
    Host::release();
    bool classic_ran = true;
    if (ClassicHost::init(clock)) {
        classic_host_live = true;
        for (const I2cSpeed s : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
            ClassicHost::Request rq{};
            rq.addr = 0x42;
            rq.speed = s;
            console_quiet();
            xfer_done = false;
            if (ClassicHost::start(rq)) {
                classic_ran = false;
                continue;
            }
            const SclReading r = count_scl<ClassicScl>();
            const uint32_t mean_hz = r.average == 0u ? 0u : SysClock::hz / r.average;
            const uint32_t stated = ClassicHost::scl_hz(s);
            print(serial, "  I2C1 (CCR) ", s == I2cSpeed::fast_400k ? "Fm  " : "Sm  ",
                  ": CCR states ", stated, " Hz, the pad's mean ", mean_hz, " Hz (",
                  r.edges, " rising edges), status ", ClassicHost::status(), crlf);
            // THE NINTH RISE. This engine's completion is the AF that rise
            // raises, and count_scl() reads the pad once more behind it. At
            // 100 kHz the high phase is 4 us, 720 core cycles, against the
            // error vector's longest entry of about 200: the read still finds
            // the line high, and nine is the bound. At 400 kHz it is 0.6 us,
            // 108 cycles: the handler outlasts it, the line is low again, and
            // whether the loop's own last read caught the rise is the image's
            // layout (eight every run on this one), so eight is the bound.
            const uint8_t least = s == I2cSpeed::fast_400k ? 8u : 9u;
            if (r.edges < least || ClassicHost::status() != i2c_nack_addr) {
                classic_ran = false;
            }
        }
        classic_host_live = false;
        ClassicHost::release();
    } else {
        classic_ran = false;
    }
    bench.verdict("the classic I2C1 ran the same probe on the same wire at both its speeds",
                  classic_ran);
    print(serial, "  (nine of the rising edges are the address phase - eight bits and the "
                  "acknowledge slot nobody filled - and the tenth, where the loop is still "
                  "watching, is SCL going up for the STOP. The bus is the bench's two 2.2 kOhm "
                  "pull-ups with the port's own beside them; one board is one specimen. What "
                  "the detection-delay column shows is why the default is the floor: the "
                  "manual's 1000 / 750 / 500 ns and ST's own timing utility both charge more "
                  "than a short wire takes, so a bus solved against them runs FASTER than "
                  "asked, while one solved against the floor can only run slower)",
          crlf);
    (void)host_ready();
}

// =============================================================================
// g - the SMBus time-out unit, on an idle bus
// =============================================================================

void tg_timeouts() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint32_t ker = Host::kernel_hz();
    // The arithmetic against tables 138, 139 and 140 - the manual's own
    // cells, at the manual's own kernel clocks.
    bench.verdict("table 138's two cells: 25 ms of clock-low time-out is code 0x61 at 8 MHz and "
                  "0xC3 at 16",
                  *fmpi2c_timeout_code_for(8'000'000UL, 25'000u, false) == 0x61 &&
                      *fmpi2c_timeout_code_for(16'000'000UL, 25'000u, false) == 0xC3);
    bench.verdict("table 140's two: 50 us of idle detection is 0x63 at 8 MHz and 0xC7 at 16",
                  *fmpi2c_timeout_code_for(8'000'000UL, 50u, true) == 0x63 &&
                      *fmpi2c_timeout_code_for(16'000'000UL, 50u, true) == 0xC7);
    bench.verdict("twelve bits do not reach a second at this kernel clock: refused, not "
                  "truncated",
                  !fmpi2c_timeout_code_for(ker, 1'000'000u, false).has_value() &&
                      fmpi2c_timeout_code_for(ker, 150'000u, false).has_value());

    // The write gates: TIMEOUTA and TIDLE need TIMOUTEN clear, TIMEOUTB
    // needs TEXTEN clear - and the verb clears both first, which is the only
    // order in which every field lands.
    const uint16_t a = *fmpi2c_timeout_code_for(ker, 50u, true);
    const uint16_t b = *fmpi2c_timeout_code_for(ker, 8'000u, false);
    const bool wrote = S::timeouts(a, true, true, b, true);
    const uint32_t back = S::timeouts();
    print(serial, "  TIMEOUTR ", hex(back), ": TIMEOUTA ", back & 0xFFFu, " (asked ", a,
          ", which is ", fmpi2c_timeout_us(ker, a, true), " us of idle), TIMEOUTB ",
          (back >> 16) & 0xFFFu, " (asked ", b, ", ", fmpi2c_timeout_us(ker, b, false) / 1000u,
          " ms of own stretch), TIDLE ", (back & (1u << 12)) != 0u ? 1 : 0, " TIMOUTEN ",
          (back & (1u << 15)) != 0u ? 1 : 0, " TEXTEN ", (back >> 31) != 0u ? 1 : 0, crlf);
    bench.verdict("both halves of TIMEOUTR land with their enables, past two different write "
                  "gates",
                  wrote && (back & 0xFFFu) == a && ((back >> 16) & 0xFFFu) == b &&
                      (back & (1u << 12)) != 0u && (back & (1u << 15)) != 0u &&
                      (back >> 31) != 0u);
    bench.verdict("a code past twelve bits is refused, and nothing is written",
                  !S::timeouts(4096, false, true, 0, false) && S::timeouts() == back);

    // THE IDLE MODE NEEDS NO PEER: it watches both lines standing high,
    // which is exactly what this bus does. Disarmed, cleared, then ARMED
    // with the clock running - so what is timed is the unit's own span and
    // not the interval between two software writes.
    //
    // AND THE ERROR INTERRUPT IS TAKEN AWAY FIRST, because TIMEOUT is one
    // of the six conditions behind ERRIE: with the vector live the flag is
    // swept by the engine's error body (which is what it is there for)
    // before a polling loop can see it, and the loop then measures the NEXT
    // period, or the one after that. Measured while chasing exactly that:
    // the span came out at fourteen periods when the sweep won the race.
    const uint32_t cycles_per_us = SysClock::hz / 1'000'000u;
    S::interrupt(FmpI2cInterrupt::error, false);
    (void)S::timeouts(a, true, false, b, false);
    S::clear(FmpI2cClear::timeout);
    const uint32_t period = systick_period();
    const uint32_t t0 = SysTick->VAL;
    (void)S::timeouts(a, true, true, b, true);
    uint32_t waited = 0;
    bool fired = false;
    for (uint32_t spins = 2'000'000u; spins != 0u; --spins) {
        if (S::flag(FmpI2cFlag::timeout)) {
            waited = since(t0, SysTick->VAL, period);
            fired = true;
            break;
        }
    }
    // And once the span has elapsed the condition STANDS: the flag is a
    // level dressed as an event, so clearing it on a bus that is still idle
    // brings it straight back.
    S::clear(FmpI2cClear::timeout);
    const uint32_t t1 = SysTick->VAL;
    uint32_t again = 0;
    for (uint32_t spins = 2'000'000u; spins != 0u; --spins) {
        if (S::flag(FmpI2cFlag::timeout)) {
            again = since(t1, SysTick->VAL, period);
            break;
        }
    }
    print(serial, "  TIDLE armed at ", fmpi2c_timeout_us(ker, a, true), " us: the flag ",
          fired ? "came" : "DID NOT COME", " after ", waited, " core cycles = ",
          waited / cycles_per_us, ".", (waited % cycles_per_us) * 10u / cycles_per_us,
          " us; cleared on a bus that is still idle it came back in ", again, " core cycles",
          crlf);
    bench.verdict("the idle time-out fires on a bus nobody is using: both lines high for longer "
                  "than tIDLE is the condition, and an empty bus meets it",
                  fired);
    bench.verdict("... and the span it measures is the one the code asked for, to within the "
                  "microsecond a polling loop can see",
                  fired && waited >= 45u * cycles_per_us && waited <= 60u * cycles_per_us);
    bench.verdict("... and it stands as a LEVEL: cleared on a bus that is still idle it is back "
                  "within another span",
                  again != 0u && again <= 60u * cycles_per_us);

    // Now the same flag WITH the error vector live, which is how a program
    // that arms a time-out and runs a bus really meets it.
    arm_log();
    S::clear(FmpI2cClear::timeout);
    S::interrupt(FmpI2cInterrupt::error, true);
    for (uint32_t spins = 400'000u; spins != 0u && er_n == 0u; --spins) {
    }
    const uint8_t errors = er_n;
    const bool swept = !S::flag(FmpI2cFlag::timeout) || er_n > errors;
    print(serial, "  with ERRIE armed the same time-out reached the ERROR vector ", errors,
          " time(s) in the same wait, and the engine's error body swept it", crlf);
    bench.verdict("an armed SMBus time-out is an ERRIE condition and reaches the error vector, "
                  "where this engine sweeps it rather than ending a tenure with it - the "
                  "arbiter's own timeout is what answers a bus that stops answering",
                  errors != 0u && swept);

    // And the OTHER mode of the same field watches SCL held LOW, which an
    // idle bus never is - so the same code with TIDLE clear must stay quiet.
    (void)S::timeouts(a, false, true, b, true);
    S::clear(FmpI2cClear::timeout);
    bool quiet = true;
    for (uint32_t spins = 2'000'000u; spins != 0u && quiet; --spins) {
        if (S::flag(FmpI2cFlag::timeout)) {
            quiet = false;
        }
    }
    print(serial, "  the same code with TIDLE clear - the SCL-low time-out - stayed ",
          quiet ? "quiet" : "SET", " over the same wait", crlf);
    bench.verdict("the two modes of TIMEOUTA are two different watchers: one sees an idle bus, "
                  "the other sees a held clock, and this bus is the first and not the second",
                  quiet);
    (void)S::timeouts(0, false, false, 0, false);
    S::clear(FmpI2cClear::timeout);
    bench.verdict("... and a tenure still runs with the unit disarmed",
                  probe(0x42) == i2c_nack_addr);
}

// =============================================================================
// h - the filters and the Fm+ pad drive
// =============================================================================

void th_filters() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const FmpI2cFilters at_reset = S::filters();
    print(serial, "  the filters init() left: analog ", at_reset.analog ? "on" : "OFF",
          ", digital ", at_reset.digital, crlf);
    bench.verdict("the analog filter is ON out of reset (ANFOFF is the OFF switch) and the "
                  "digital one is not",
                  at_reset.analog && at_reset.digital == 0u);

    (void)S::disable();
    (void)S::filters(FmpI2cFilters{.analog = false, .digital = 9});
    const FmpI2cFilters back = S::filters();
    S::enable();
    print(serial, "  written analog off with DNF 9, reads analog ", back.analog ? "on" : "OFF",
          " digital ", back.digital, crlf);
    bench.verdict("both filters read back as they were written", !back.analog &&
                                                                     back.digital == 9u);

    // A DEEP DIGITAL FILTER COSTS THE FAST END OF THE VOCABULARY, and not
    // by an arbitrary bound: 23.4.3's own condition is tI2CCLK <
    // (tLOW - tfilters)/4, and the digital filter's delay is DNF kernel
    // periods.
    FmpI2cHostConfig deep = base_config;
    deep.filters = FmpI2cFilters{.analog = true, .digital = 15};
    const bool up = Host::init(clock, deep);
    print(serial, "  with DNF 15 at 45 MHz: Sm ",
          Host::speed_ok(FmpI2cSpeed::standard_100k) ? "reachable" : "REFUSED", ", Fm ",
          Host::speed_ok(FmpI2cSpeed::fast_400k) ? "reachable" : "REFUSED", ", Fm+ ",
          Host::speed_ok(FmpI2cSpeed::fast_plus_1m) ? "reachable" : "REFUSED", crlf);
    bench.verdict("a filter that eats too much of the low period is refused by 23.4.3's own "
                  "condition, speed by speed, and the slow end survives",
                  up && Host::speed_ok(FmpI2cSpeed::standard_100k));
    bench.verdict("... and a tenure still runs at the speed that is left",
                  probe(0x42) == i2c_nack_addr);
    (void)host_ready();

    // The Fm+ pad drive: two bits in SYSCFG, one per SIGNAL.
    Host::fast_plus_drive(true);
    const bool both_on = S::fast_plus_scl() && S::fast_plus_sda();
    Syscfg::fast_mode_plus(true, false);
    const bool split = S::fast_plus_scl() && !S::fast_plus_sda();
    Host::fast_plus_drive(false);
    const bool both_off = !S::fast_plus_scl() && !S::fast_plus_sda();
    print(serial, "  SYSCFG_CFGR's two drive bits: both on ", both_on ? "yes" : "NO",
          ", SCL alone ", split ? "yes" : "NO", ", both off ", both_off ? "yes" : "NO", crlf);
    bench.verdict("the 20 mA drive is TWO BITS, one per signal and not one per pad - and this "
                  "part has the register at all, which is how the reserve knows it has the "
                  "peripheral",
                  Syscfg::has_fast_mode_plus() && both_on && split && both_off);
}

// =============================================================================
// i - THE KERNEL: I2cBus over FmpI2cHost
// =============================================================================

namespace kl {

/// The per-bus timeout is 20 ms: far above any tenure letter i queues (a
/// probe is about 0.1 ms at 100 kHz), and the limit letter n's peer
/// stretches past.
constexpr uint32_t bus_timeout_ticks = ticks_from_ms<P>(20);
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, bus_timeout_ticks>;

struct Probe {
    using Event = std::variant<I2cDone, SleepVote>;
    static inline EventQueue<Event, 8, P> queue{};
    static inline uint8_t n = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;
    static inline uint8_t replies[8] = {};

    static void clear_tally() {
        n = 0;
        rejected = 0;
        votes = 0;
        for (uint8_t i = 0; i < 8u; ++i) {
            replies[i] = 0;
        }
    }

    static void init() {}

    static void dispatch(const Event& e) {
        brio::match(
            e,
            [](const I2cDone& d) {
                if (n < 8u) {
                    replies[n] = d.status;
                }
                ++n;
                if (d.status == i2c_rejected) {
                    ++rejected;
                }
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
            });
    }
};

using BusKernel = Tenuto<P, Probe, I2cArb>;

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

Host::Request request(uint8_t addr) {
    Host::Request r{};
    r.addr = addr;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

}   // namespace kl

void ti_kernel() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    kl::BusKernel::init_all();
    kl::Probe::clear_tally();
    bus_ao_live = true;

    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::I2cArb>(kl::request(static_cast<uint8_t>(0x20u + i)));
    }
    kl::pump_until(4, 500);
    print(serial, "  four queued probes: replies ", kl::Probe::n, " [", kl::Probe::replies[0],
          " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3],
          "]", crlf);
    bench.verdict("four tenures through I2cBus, four replies - util/i2c_bus.hpp and "
                  "util/bus_master.hpp unchanged over this second block of the same family",
                  kl::Probe::n == 4u);
    bench.verdict("and the wire's own outcome reaches the requester untouched: i2c_nack_addr, "
                  "four times",
                  kl::Probe::replies[0] == i2c_nack_addr &&
                      kl::Probe::replies[1] == i2c_nack_addr &&
                      kl::Probe::replies[2] == i2c_nack_addr &&
                      kl::Probe::replies[3] == i2c_nack_addr);

    // The synchronous refusal travels the same way: the arbiter replies
    // with status() for a request start() answered without moving a byte.
    kl::Probe::clear_tally();
    Host::Request too_fast = kl::request(0x21);
    too_fast.speed = FmpI2cSpeed::fast_plus_1m;
    (void)host_ready(FmpI2cClock::hsi);
    bus_ao_live = true;
    post<kl::I2cArb>(too_fast);
    kl::pump_until(1, 200);
    print(serial, "  a Fm+ request on a kernel clock that cannot make it came back ",
          kl::Probe::replies[0], " (", i2c_rejected, " is i2c_rejected)", crlf);
    bench.verdict("the one synchronous refusal is answered by the arbiter with status(), and it "
                  "is i2c_rejected",
                  kl::Probe::n == 1u && kl::Probe::rejected == 1u);
    (void)host_ready();
    bus_ao_live = true;

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(0x22));
    }
    kl::pump_until(6, 500);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately",
                  kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);

    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep",
                  kl::Probe::votes == 1u && kl::Probe::last_vote);

    bus_ao_live = false;
    (void)host_ready();
}

// =============================================================================
// j - the DMA engines
// =============================================================================

void tj_engines() {
    Dma<1>::init();
    bus_ao_live = false;
    if (!DmaHost::init(clock, base_config)) {
        bench.verdict("the engined host came up", false);
        return;
    }
    dma_host_live = true;
    bench.verdict("the two engines sit on the only cells RM0390 table 28 gives this block - "
                  "DMA1 stream 5 channel 2 out, stream 2 channel 2 in - and the reserve checked "
                  "that at compile time",
                  DmaHost::has_engines && fmpi2c_dma_placement_valid(true, 1, 5, 2) &&
                      fmpi2c_dma_placement_valid(false, 1, 2, 2));

    // A WRITE tenure whose data would ride the transmit engine: the engine
    // is started before the START bit (23.4.16's own order; its first
    // byte waits for the first TXIS, after the address), and the address
    // is NACKed before a byte reaches TXDR.
    static const uint8_t out[4] = {0xA5, 0x5A, 0x00, 0xFF};
    arm_log();
    const uint8_t st = tenure<DmaHost>(0x42, out, 4, nullptr, 0);
    const bool tx_armed = S::dma_transmit();
    print(serial, "  a four-byte write through the transmit engine to an address nobody "
                  "answers: status ", st, ", TXDMAEN after the tenure ",
          tx_armed ? "STILL SET" : "cleared", ", event interrupts ", ev_n, crlf);
    bench.verdict("the tenure ends i2c_nack_addr, as it does on the byte pump: the address phase "
                  "is the interrupt's whatever carries the data",
                  st == i2c_nack_addr);
    bench.verdict("... and the engines are torn down with it - the DMA enables are clear again, "
                  "so the next tenure starts from a clean slate",
                  !tx_armed && !S::dma_receive());

    // A READ tenure, whose receive engine is armed and drains nothing.
    static uint8_t in[8] = {};
    const uint8_t rst = tenure<DmaHost>(0x43, nullptr, 0, in, 8);
    print(serial, "  an eight-byte read through the receive engine: status ", rst, crlf);
    bench.verdict("a read tenure ends the same way, and the receive engine comes down with it",
                  rst == i2c_nack_addr && !S::dma_receive());

    // And the engined host recovers and releases like the plain one.
    bench.verdict("recover() puts an engined host back where start() is legal",
                  DmaHost::recover());
    bench.verdict("... and the bus still reaches the wire after it",
                  tenure<DmaHost>(0x44, nullptr, 0, nullptr, 0) == i2c_nack_addr);
    dma_host_live = false;
    DmaHost::release();
    (void)host_ready();
    bench.verdict("the plain host takes the peripheral back", probe(0x45) == i2c_nack_addr);
}

// =============================================================================
// k - the recovery verbs
// =============================================================================

void tk_recovery() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // 23.4.6's PE cycle KEEPS the configuration, which is the whole
    // difference between it and the RCC's reset line - and the reason
    // recover() reconfigures nothing.
    (void)S::addresses(FmpI2cAddressConfig{.own = 0x33});
    const uint32_t timing_before = S::timing_reg();
    const uint32_t oar_before = S::oar1();
    const uint32_t cr1_before = S::interrupts();
    const bool cycled = S::cycle();
    print(serial, "  after the PE cycle: TIMINGR ", hex(S::timing_reg()), " (was ",
          hex(timing_before), "), OAR1 ", hex(S::oar1()), " (was ", hex(oar_before),
          "), the interrupt enables ", hex(S::interrupts()), " (were ", hex(cr1_before),
          "), ISR ", hex(S::flags()), crlf);
    bench.verdict("PE = 0 is a SOFTWARE RESET of the state machines and the flags, and it keeps "
                  "every configuration register - the timing, the addresses and the interrupt "
                  "enables all stand",
                  cycled && S::timing_reg() == timing_before && S::oar1() == oar_before &&
                      S::interrupts() == cr1_before);
    bench.verdict("... and the status register is back to its reset value, TXE standing",
                  (S::flags() & ~FmpI2cFlag::txe) == 0u);
    // The RCC's reset line, by contrast, takes everything.
    S::reset();
    print(serial, "  after the RCC reset line: TIMINGR ", hex(S::timing_reg()), " OAR1 ",
          hex(S::oar1()), crlf);
    bench.verdict("the RCC's reset line is the other verb and takes the configuration with it",
                  S::timing_reg() == 0u && S::oar1() == 0u);
    (void)host_ready();

    bench.verdict("recover() is that PE cycle with the vectors quiesced around it",
                  Host::recover());
    bench.verdict("... and the bus works after it", probe(0x42) == i2c_nack_addr);

    // The unstick: nine clocks and a STOP by hand, only when SDA is really
    // held low. A healthy wire is left alone and says 0.
    const uint8_t pulses = Host::unstick();
    print(serial, "  unstick() on a free wire: ", pulses,
          " pulses (0 = SDA was already high and nothing was driven)", crlf);
    bench.verdict("unstick() finds this bus free and drives nothing", pulses == 0u);
    bench.verdict("... and the pads come back to the peripheral",
                  probe(0x42) == i2c_nack_addr);

    print(serial, "  bus errors seen and ignored since init(): ", Host::spurious_bus_errors(),
          " (ES0298 2.12.3: in controller mode a BERR is spurious or harmless, and the tenure "
          "runs on)", crlf);
    bench.verdict("every tenure above completed whatever BERR did - the count is printed and "
                  "not judged, one board being one specimen",
                  true);
}

// =============================================================================
// l - the client, configured and never addressed
// =============================================================================

void tl_client() {
    Host::release();
    peer_live = true;
    const bool up = Peer::init(clock,
                               FmpI2cAddressConfig{.own = 0x2A,
                                                   .second = 0x2B,
                                                   .second_mask = FmpI2cOa2Mask::low_1,
                                                   .second_enable = true},
                               FmpI2cSpeed::fast_400k, base_config);
    print(serial, "  the client came up ", up ? "yes" : "NO", ": OA1 ", hex(S::oar1()), " OA2 ",
          hex(S::oar2()), ", kernel ", Peer::kernel_hz() / 1000u, " kHz, TIMINGR ",
          hex(S::timing_reg()), crlf);
    bench.verdict("the target role configures both own addresses past their two different write "
                  "gates",
                  up && S::own_address() == 0x2Au && (S::oar2() & 0xFEu) == (0x2Bu << 1) &&
                      (S::oar2() & (1u << 15)) != 0u);
    bench.verdict("A CLIENT NEEDS TIMINGR TOO: SDADEL and SCLDEL are the hold and setup delays "
                  "a target applies, solved here against the FASTEST bus it expects to sit on",
                  S::timing_reg() != 0u);
    bench.verdict("byte control is refused with the clock stretch given up and accepted with it",
                  Peer::byte_control(true));

    // Nothing addresses this target - there is no controller on the wire -
    // so the two bodies must report `none` and change nothing.
    const FmpI2cClientEvent ev = Peer::service();
    const FmpI2cClientEvent er = Peer::error_service();
    print(serial, "  with no controller on the wire the two ISR bodies report ",
          static_cast<uint8_t>(ev), " and ", static_cast<uint8_t>(er), " (0 = none); ADDR ",
          Peer::addressed() ? "SET" : "clear", ", STOPF ", Peer::stop_seen() ? "SET" : "clear",
          ", OVR ", Peer::overrun() ? "SET" : "clear", crlf);
    bench.verdict("both bodies report nothing on a bus nobody is driving, and neither invents an "
                  "event",
                  ev == FmpI2cClientEvent::none && er == FmpI2cClientEvent::none &&
                      !Peer::addressed() && !Peer::stop_seen());
    Peer::release();
    peer_live = false;
    (void)host_ready();
    bench.verdict("the host takes the peripheral back after the client had it",
                  probe(0x42) == i2c_nack_addr);
    print(serial, "  (a target is a PROTOCOL and this one was never addressed: no controller "
                  "exists on this desk to address it. docs/stm32f4/fmpi2c.md says what would "
                  "measure it)", crlf);
}

// =============================================================================
// m - the self-link: FMPI2C1 the host, I2C1 the target
// =============================================================================

/// THE OTHER END, on this board: I2C1 on PB8/PB9 (AF4), joined to PC6/PC7
/// by the bench's two wires with a 2.2 kOhm pull-up on each line, a target
/// at 0x3A SERVED FROM ITS OWN TWO VECTORS - a byte sink, and a pattern
/// source that restarts at every address match. `refuse_at` makes it NACK
/// one written byte, counted from one: 27.6.1's ACK governs the byte being
/// received, so it goes down as the byte BEFORE the refused one is taken
/// (at the address match for the first) and back up with the refused one.
using Classic = I2c<1>;
using ClassicTarget = I2cClient<1, classic_pins>;
constexpr uint8_t self_addr = 0x3A;

struct Far {
    static inline volatile bool live = false;
    static inline volatile bool serving = false;
    static inline volatile uint8_t pos = 0;
    static inline volatile uint8_t took = 0;
    static inline volatile uint8_t refuse_at = 0;
    static inline volatile uint8_t matches = 0;   ///< address matches, whoever reads them
    static inline uint8_t sink[32];

    static uint8_t value(uint8_t i) { return static_cast<uint8_t>(0x47u + 0x1Du * i); }

    [[gnu::always_inline]] static void event() {
        switch (ClassicTarget::service()) {
            case I2cClientEvent::addressed:
                matches = static_cast<uint8_t>(matches + 1u);
                pos = 0;
                serving = ClassicTarget::host_reads();
                Classic::buffer_interrupt(true);
                if (!serving) {
                    took = 0;
                    if (refuse_at == 1u) {
                        ClassicTarget::acknowledge(false);
                    }
                }
                break;
            case I2cClientEvent::byte_received: {
                const uint8_t v = ClassicTarget::take();
                if (took < sizeof sink) {
                    sink[took] = v;
                }
                took = static_cast<uint8_t>(took + 1u);
                if (took == refuse_at) {
                    ClassicTarget::acknowledge(true);
                } else if (refuse_at != 0u && took + 1u == refuse_at) {
                    ClassicTarget::acknowledge(false);
                }
                break;
            }
            case I2cClientEvent::byte_wanted:
                // TxE stands between tenures too, and a byte stored outside
                // a read wedges the next one (docs/stm32f4/i2c.md): only
                // inside a read is DR written; otherwise the buffer vector
                // goes quiet.
                if (serving) {
                    ClassicTarget::give(value(pos));
                    pos = static_cast<uint8_t>(pos + 1u);
                } else {
                    Classic::buffer_interrupt(false);
                }
                break;
            case I2cClientEvent::stop:
                serving = false;
                ClassicTarget::acknowledge(true);
                break;
            default:
                break;
        }
    }
    [[gnu::always_inline]] static void error() {
        if (ClassicTarget::error_service() == I2cClientEvent::nacked) {
            // The host's closing NACK: the byte the shifter asked for ahead
            // of it is dropped by a PE cycle (27.6.1: the address and the
            // timing survive it), held down a few bus cycles.
            serving = false;
            Classic::disable();
            (void)Classic::enabled();
            (void)Classic::enabled();
            (void)Classic::enabled();
            Classic::enable();
        }
        ClassicTarget::acknowledge(true);
    }
};

/// The tenure shapes through one host at one speed against the far end:
/// the probe, an absent address, a write of eight, reads of one, two,
/// three, four and eight, a write of two then a read of four. Returns how
/// many of the eight came back exact.
template <typename Bus>
uint8_t self_shapes(FmpI2cSpeed speed) {
    uint8_t exact = 0;
    uint8_t out[8];
    uint8_t in[8];
    for (uint8_t i = 0; i < 8u; ++i) {
        out[i] = static_cast<uint8_t>(0x11u * (i + 1u));
    }
    exact = static_cast<uint8_t>(exact + (tenure<Bus>(self_addr, nullptr, 0, nullptr, 0, speed) ==
                                                  i2c_ok ? 1u : 0u));
    exact = static_cast<uint8_t>(exact + (tenure<Bus>(0x42, nullptr, 0, nullptr, 0, speed) ==
                                                  i2c_nack_addr ? 1u : 0u));
    Far::took = 0;
    bool w = tenure<Bus>(self_addr, out, 8, nullptr, 0, speed) == i2c_ok && Far::took == 8u;
    for (uint8_t i = 0; i < 8u && w; ++i) {
        w = Far::sink[i] == out[i];
    }
    exact = static_cast<uint8_t>(exact + (w ? 1u : 0u));
    const uint8_t counts[5] = {1, 2, 3, 4, 8};
    for (const uint8_t c : counts) {
        for (uint8_t i = 0; i < 8u; ++i) {
            in[i] = 0xEE;
        }
        bool r = tenure<Bus>(self_addr, nullptr, 0, in, c, speed) == i2c_ok;
        for (uint8_t i = 0; i < c && r; ++i) {
            r = in[i] == Far::value(i);
        }
        exact = static_cast<uint8_t>(exact + (r ? 1u : 0u));
    }
    Far::took = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        in[i] = 0xEE;
    }
    bool c = tenure<Bus>(self_addr, out, 2, in, 4, speed) == i2c_ok && Far::took == 2u &&
             Far::sink[0] == out[0] && Far::sink[1] == out[1];
    for (uint8_t i = 0; i < 4u && c; ++i) {
        c = in[i] == Far::value(i);
    }
    return static_cast<uint8_t>(exact + (c ? 1u : 0u));
}

/// One to four bytes written and two read, the LAST written byte refused,
/// four times a count: how many came back i2c_nack_data, how many left
/// the bus let go (START and STOP down, BUSY clear, 2 ms on) and how many
/// were followed by a probe that ran.
template <typename Bus>
void self_refusals(FmpI2cSpeed speed, uint8_t& nacked, uint8_t& gone, uint8_t& next) {
    const uint8_t out[4] = {0x21, 0x32, 0x43, 0x54};
    uint8_t in[2];
    for (uint8_t n = 1; n <= 4u; ++n) {
        for (uint8_t trial = 0; trial < 4u; ++trial) {
            Far::refuse_at = n;
            const uint8_t st = tenure<Bus>(self_addr, out, n, in, 2, speed);
            const uint32_t t0 = Ticker::ticks();
            while (Ticker::ticks() - t0 < 2u) {
            }
            const bool free = (S::cr2() & (FMPI2C_CR2_START | FMPI2C_CR2_STOP)) == 0u && !S::busy();
            Far::refuse_at = 0;
            ClassicTarget::acknowledge(true);
            const uint8_t after = tenure<Bus>(self_addr, nullptr, 0, nullptr, 0, speed);
            nacked = static_cast<uint8_t>(nacked + (st == i2c_nack_data ? 1u : 0u));
            gone = static_cast<uint8_t>(gone + (free ? 1u : 0u));
            next = static_cast<uint8_t>(next + (after == i2c_ok ? 1u : 0u));
            if (!free || after != i2c_ok) {
                (void)Bus::recover();
                (void)Bus::unstick();
            }
        }
    }
}

/// THE WRITE AFTER A FAILED ONE, byte for byte on the far end. Each refused
/// position of writes of one, two, three and eight, and an absent address, on
/// engine `e`, and straight after each - no init(), no PE cycle, at one speed -
/// a write of eight on engine `next`, compared with what the far end kept. A
/// byte the failed tenure left in TXDR would reach the wire ahead of this one
/// (RM0390 23.4.7: TXE clear, its content goes to the shifter at the address's
/// ninth pulse) and NBYTES would drop this one's last. Returns the failures.
template <typename Bus>
uint8_t self_next_write(FmpI2cSpeed speed, I2cEngine e, I2cEngine next) {
    const uint8_t out[8] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x18};
    const uint8_t after[8] = {0x3C, 0x4D, 0x5E, 0x6F, 0x70, 0x81, 0x92, 0xA3};
    const uint8_t lengths[4] = {1, 2, 3, 8};
    uint8_t wrong = 0;
    auto judge = [&](bool failed_right) {
        Far::refuse_at = 0;
        ClassicTarget::acknowledge(true);
        Far::took = 0;
        bool whole = failed_right &&
                     tenure<Bus>(self_addr, after, 8, nullptr, 0, speed, next) == i2c_ok &&
                     Far::took == 8u;
        for (uint8_t i = 0; i < 8u && whole; ++i) {
            whole = Far::sink[i] == after[i];
        }
        if (!whole) {
            wrong = static_cast<uint8_t>(wrong + 1u);
        }
    };
    for (const uint8_t n : lengths) {
        judge(tenure<Bus>(0x42, out, n, nullptr, 0, speed, e) == i2c_nack_addr);
        for (uint8_t k = 1; k <= n; ++k) {
            Far::refuse_at = k;
            judge(tenure<Bus>(self_addr, out, n, nullptr, 0, speed, e) == i2c_nack_data);
        }
    }
    return wrong;
}

void tm_self_link() {
    // The far end first, so its pads are the peripheral's before a START.
    Far::live = true;
    Far::refuse_at = 0;
    const bool far_up = ClassicTarget::init(clock, I2cAddressConfig{.own = self_addr});
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t found = probe(self_addr);
    if (!far_up || found != i2c_ok) {
        // A board without the two wires: nothing answers, and the letter
        // says so instead of failing.
        print(serial, "  nothing answers ", hex(self_addr), " (status ", found,
              "): this board does not wire I2C1 to FMPI2C1 - the letter declines by name", crlf);
        ClassicTarget::release();
        Far::live = false;
        (void)host_ready();
        return;
    }
    print(serial, "  I2C1 answers at ", hex(self_addr), " on PB8/PB9, wired to PC6/PC7", crlf);
    uint8_t exact = 0;
    for (const FmpI2cSpeed speed : {FmpI2cSpeed::standard_100k, FmpI2cSpeed::fast_400k}) {
        const uint8_t e = self_shapes<Host>(speed);
        print(serial, "  ", speed == FmpI2cSpeed::fast_400k ? "400" : "100",
              " kHz, the pump: ", e, " of 9 shapes exact", crlf);
        exact = static_cast<uint8_t>(exact + e);
    }
    bench.verdict("THE FMPI2C1'S FIRST ACKNOWLEDGED BYTES: the probe, an absent address, a write "
                  "of eight, reads of one, two, three, four and eight and a write-then-read, "
                  "byte-exact against I2C1 at 100 and 400 kHz",
                  exact == 18u);

    // The engines: the same shapes through DMA1 streams 5 and 2.
    Dma<1>::init();
    dma_host_live = true;
    uint8_t dma_exact = 0;
    if (DmaHost::init(clock, base_config)) {
        dma_exact = self_shapes<DmaHost>(FmpI2cSpeed::fast_400k);
    }
    dma_host_live = false;
    DmaHost::release();
    print(serial, "  400 kHz, the engines: ", dma_exact, " of 9 shapes exact", crlf);
    bench.verdict("... and through the DMA engines", dma_exact == 9u);

    // THE ENGINE NAMED, judged on the streams: which mechanism carried a
    // half is read from the stream's memory address (zeroed before the
    // tenure with the stream stopped, written by nothing but a block
    // start, stm32f4/dma.hpp's launch()) and from its vector, entered once
    // by the block's completion; identities, no tolerance.
    dma_host_live = true;
    (void)DmaHost::init(clock, base_config);
    using TxRegs = DmaStream<TxEngine::controller, TxEngine::stream>;
    using RxRegs = DmaStream<RxEngine::controller, RxEngine::stream>;
    struct Trace {
        uint8_t status;
        uint32_t tx_at;   ///< the transmit stream's memory address, 0 when no block started
        bool rx;
        uint32_t tx_n;
        uint32_t rx_n;
    };
    auto traced = [](const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                     I2cEngine e) {
        // SxM0AR takes a write only with EN down (RM0390 9.5.8); between two
        // tenures both streams have stopped.
        TxRegs::regs().M0AR = 0;
        RxRegs::regs().M0AR = 0;
        tx_dma_entries = 0;
        rx_dma_entries = 0;
        const uint8_t st = tenure<DmaHost>(self_addr, tx, tx_len, rx, rx_len,
                                           FmpI2cSpeed::fast_400k, e);
        return Trace{st, TxRegs::regs().M0AR,
                     rx != nullptr && RxRegs::regs().M0AR == reinterpret_cast<uint32_t>(rx),
                     tx_dma_entries, rx_dma_entries};
    };
    static const uint8_t one_out[2] = {0x6C, 0x7D};
    uint8_t one_in = 0xEE;
    Far::took = 0;
    Trace t = traced(one_out, 1, nullptr, 0, I2cEngine::dma);
    const bool w1 = t.status == i2c_ok && t.tx_at == 0u && t.tx_n == 0u && Far::took == 1u &&
                    Far::sink[0] == one_out[0];
    Far::took = 0;
    t = traced(one_out, 2, nullptr, 0, I2cEngine::dma);
    const bool w2 = t.status == i2c_ok && t.tx_at == reinterpret_cast<uint32_t>(one_out + 1) &&
                    t.tx_n == 1u && Far::took == 2u && Far::sink[0] == one_out[0] &&
                    Far::sink[1] == one_out[1];
    t = traced(nullptr, 0, &one_in, 1, I2cEngine::dma);
    const bool r1 = t.status == i2c_ok && t.rx && t.rx_n == 1u && one_in == Far::value(0);
    print(serial, "  dma, a write of one: ", w1 ? "no stream, exact" : "WRONG",
          "; a write of two: ", w2 ? "the stream on the second byte, exact" : "WRONG",
          "; a read of one: ", r1 ? "on the receive stream, exact" : "WRONG", crlf);
    bench.verdict("dma with the first byte in TXDR before the START: a write of one starts no "
                  "stream and enters no stream vector, a write of two starts the stream on its "
                  "second byte, completing once, both byte-exact",
                  w1 && w2);
    bench.verdict("... and a read of one is on the receive stream, completing once, byte-exact", r1);
    uint8_t eight_in[8] = {};
    static const uint8_t eight_out[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    Far::took = 0;
    t = traced(eight_out, 8, eight_in, 8, I2cEngine::interrupt);
    bool i8 = t.status == i2c_ok && t.tx_at == 0u && !t.rx && t.tx_n == 0u && t.rx_n == 0u &&
              Far::took == 8u;
    for (uint8_t i = 0; i < 8u && i8; ++i) {
        i8 = Far::sink[i] == eight_out[i] && eight_in[i] == Far::value(i);
    }
    print(serial, "  interrupt on the engined host, eight written and eight read: status ",
          t.status, ", streams started tx ", t.tx_at != 0u, " rx ", t.rx, ", stream vector entries ",
          t.tx_n + t.rx_n, crlf);
    bench.verdict("interrupt on the engined host touches no stream, and the tenure is exact", i8);
    uint8_t d_nacked = 0;
    uint8_t d_gone = 0;
    uint8_t d_next = 0;
    self_refusals<DmaHost>(FmpI2cSpeed::fast_400k, d_nacked, d_gone, d_next);
    print(serial, "  dma, the last written byte refused, one to four written: 16 refusals, "
                  "i2c_nack_data ", d_nacked, ", the bus let go ", d_gone,
          ", the next tenure ok ", d_next, crlf);
    bench.verdict("the LAST WRITTEN byte of a write-then-read refused under dma: i2c_nack_data, "
                  "the bus let go, the next tenure running",
                  d_nacked == 16u && d_gone == 16u && d_next == 16u);
    dma_host_live = false;
    DmaHost::release();

    // The write after a failed one, byte for byte, across the two engines.
    {
        dma_host_live = true;
        (void)DmaHost::init(clock, base_config);
        const FmpI2cSpeed v = FmpI2cSpeed::fast_400k;
        const uint8_t dd = self_next_write<DmaHost>(v, I2cEngine::dma, I2cEngine::dma);
        const uint8_t di = self_next_write<DmaHost>(v, I2cEngine::dma, I2cEngine::interrupt);
        const uint8_t id = self_next_write<DmaHost>(v, I2cEngine::interrupt, I2cEngine::dma);
        const uint8_t ii = self_next_write<DmaHost>(v, I2cEngine::interrupt, I2cEngine::interrupt);
        dma_host_live = false;
        DmaHost::release();
        (void)host_ready();
        const uint8_t pp = self_next_write<Host>(v, I2cEngine::interrupt, I2cEngine::interrupt);
        print(serial, "  the write after a refusal, 18 of each, wrong on the far end: dma then dma ",
              dd, ", dma then interrupt ", di, ", interrupt then dma ", id,
              ", interrupt then interrupt ", ii, ", the engineless pump ", pp, crlf);
        bench.verdict("after a refused address or any refused byte under dma, the next write is "
                      "whole on the far end byte for byte, on either engine",
                      dd == 0u && di == 0u);
        bench.verdict("... and the same after one refused on the event vector",
                      id == 0u && ii == 0u && pp == 0u);
    }

    // `dma` on the host with no engines bound: refused inside start(), CR2
    // untouched (no START), BUSY low and no address match at the far end;
    // then the same host's next tenure exact.
    (void)host_ready();
    {
        Far::matches = 0;
        const uint32_t cr2 = S::cr2();
        Host::Request q{};
        q.addr = self_addr;
        q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(eight_out));
        q.tx_len = 4;
        q.engine = I2cEngine::dma;
        const bool sync = Host::start(q);
        const uint8_t st = Host::status();
        (void)delay_us(clock, 200u);   // an address would be at the far end in 25 us
        const uint8_t matched = Far::matches;
        const bool quiet = sync && st == i2c_rejected && S::cr2() == cr2 && !S::busy() &&
                           matched == 0u;
        Far::took = 0;
        const uint8_t after = tenure<Host>(self_addr, eight_out, 4, nullptr, 0);
        print(serial, "  dma on the engineless host: start() ", sync, ", status ", st,
              ", address matches ", matched, "; the next write: status ", after,
              ", the far end took ", Far::took, crlf);
        bench.verdict("dma on a host with no engines bound: i2c_rejected inside start(), nothing "
                      "on the wire",
                      quiet);
        bench.verdict("... and the next tenure on that host is exact",
                      after == i2c_ok && Far::took == 4u && Far::sink[0] == eight_out[0] &&
                          Far::sink[1] == eight_out[1] && Far::sink[2] == eight_out[2] &&
                          Far::sink[3] == eight_out[3]);
    }

    // The refusal of the last written byte, as this host sees it: its NACK
    // stops the tenure by hardware (23.4.9), whatever comes after.
    (void)host_ready();
    Far::live = true;
    uint8_t nacked = 0;
    uint8_t gone = 0;
    uint8_t next = 0;
    self_refusals<Host>(FmpI2cSpeed::standard_100k, nacked, gone, next);
    self_refusals<Host>(FmpI2cSpeed::fast_400k, nacked, gone, next);
    print(serial, "  the last written byte refused, one to four written, both speeds: 32 "
                  "refusals, i2c_nack_data ", nacked, ", the bus let go ", gone,
          ", the next tenure ok ", next, crlf);
    bench.verdict("the LAST WRITTEN byte of a write-then-read refused: i2c_nack_data, the bus "
                  "let go, the next tenure running",
                  nacked == 32u && gone == 32u && next == 32u);
    ClassicTarget::release();
    Far::live = false;
    (void)host_ready();
}

// =============================================================================
// n - THE DESK PARTNER: the STM32G0's twi_peer on the same wire (set `t`)
// =============================================================================
//
// The partner's I2C1 taps PC6/PC7's two nodes; its command client answers
// 0x6B alone, so letter e's 112 NACKs want it dark and this set wants it
// running `twi_peer`. What it gives this controller that the self-link's
// classic I2C cannot: a TARGET SOLVED FOR FAST-MODE PLUS, a clock held low
// on command, SDA held from its own GPIO until a count of SCL edges.

/// THE CORES the rebase leg walks: the boot rate, the PLL at 48 MHz and the
/// HSI alone at 16 MHz - a DynamicClock over the three, the console, the
/// tick and this host its users (the DMA host left out: two sets of statics
/// over one instance would each re-apply their own cached speed).
using Core48 = Clock<ClockSource::pll_hse, 48'000'000, 8'000'000, HseMode::bypass>;
using Core16 = Clock<ClockSource::hsi, 16'000'000>;
using Cores = DynamicClock<Rates<SysClock, Core48, Core16>, Ticker, Serial, Host>;

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
    }
}

struct LinkPort {
    static uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                          uint8_t rx_len) {
        return ::tenure<Host>(addr, tx, tx_len, rx, rx_len);
    }
    static void ready() { (void)host_ready(); }
    static void wait_ms(uint32_t ms) { ::wait_ms(ms); }
};
using Link = TwiLinkClient<LinkPort>;
using twilink::Op;

bool need_twi_peer() {
    (void)host_ready();
    if (Link::command(Op::ping)) {
        return true;
    }
    bench.verdict("the desk partner answers on the command address (the STM32G0 running "
                  "twi_peer)",
                  false);
    return false;
}

void tn_partner() {
    if (!need_twi_peer()) {
        return;
    }
    twilink::Ident id{};
    bench.verdict("the partner is twi_peer, an STM32G0 (fw 0x03xx), and the channel is two "
                  "tenures of this engine a command",
                  Link::ident(id) && id.sanity == twilink::ident_sanity &&
                      twilink::peer_kind(id.version) == twilink::peer_stm32g0);

    // ---- Fast-mode Plus with every byte acknowledged ----
    const bool fmp_ok = Host::speed_ok(FmpI2cSpeed::fast_plus_1m);
    twilink::Params a{};
    a.count = 64;   // never reached: the deadline ends the serve
    a.ms = 250;
    a.addr = twilink::dut_addr;
    a.seed = 0x90;
    a.pattern = twilink::pattern_counting;
    if (!Link::act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve", false);
        return;
    }
    static const uint8_t w8[8] = {0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7};
    uint8_t r8[8] = {};
    const uint32_t period = systick_period();
    const uint32_t t0 = SysTick->VAL;
    const uint8_t ws = tenure<Host>(twilink::dut_addr, w8, 8, nullptr, 0,
                                    FmpI2cSpeed::fast_plus_1m);
    const uint32_t w_cycles = since(t0, SysTick->VAL, period);
    const uint8_t rs = tenure<Host>(twilink::dut_addr, nullptr, 0, r8, 8,
                                    FmpI2cSpeed::fast_plus_1m);
    uint8_t mism = 0;
    for (uint8_t i = 0; i < 8u; ++i) {
        if (r8[i] != twilink::pattern_value(a.pattern, a.seed, i)) {
            ++mism;
        }
    }
    wait_ms(300);
    twilink::Report r{};
    const bool rep = Link::report(r);
    // 81 SCL periods (the address and eight bytes, nine bits each) in the
    // write's stopwatch, which also holds the START, the STOP, the
    // request's own cost and every stretch the peer's polled pump adds - so
    // the figure is a floor on the period and the rate a ceiling.
    const uint32_t per_mhz = SysClock::hz / 1'000'000u;
    const uint32_t ns = static_cast<uint32_t>(static_cast<uint64_t>(w_cycles) * 1000u /
                                              per_mhz / 81u);
    const uint32_t khz = ns != 0u ? 1'000'000u / ns : 0u;
    print(serial, "  Fm+ (reachable here: ", fmp_ok ? "yes" : "NO", ", kernel ",
          Host::kernel_hz() / 1000u, " kHz): write ", ws, ", read ", rs, ", ", mism,
          " byte(s) wrong; the write at ", khz, " kHz of SCL on average; the peer took ", r.aux0,
          " and served ", r.aux1, crlf);
    bench.verdict("at 1 MHz every byte is ACKNOWLEDGED by a real Fm+ target: a write and a read "
                  "of eight, byte-exact, the peer accounting both",
                  fmp_ok && ws == i2c_ok && rs == i2c_ok && mism == 0u && rep && r.aux0 == 8u &&
                      r.aux1 == 8u);
    bench.verdict("... and never above the rate asked (1 MHz plus 2 %)", khz <= 1020u);

    // ---- unstick() against a client that really holds SDA ----
    twilink::Params h{};
    h.ms = 300;
    h.aux8 = 4;
    if (!Link::act(Op::hold_sda, h)) {
        bench.verdict("the peer accepted the SDA hold", false);
        return;
    }
    wait_ms(5);
    const bool held = !SdaPad::read();
    const uint8_t pulses = Host::unstick();
    wait_ms(350);
    twilink::Report hr{};
    const bool hrep = Link::report(hr);
    print(serial, "  SDA held by the peer: ", held ? "low" : "NOT low", "; unstick() answered ",
          pulses, "; the peer counted ", hr.aux0, " falling edge(s)", crlf);
    bench.verdict("a client holding SDA until four SCL edges is freed by unstick() at the "
                  "fourth pulse, and unstick() says so",
                  held && pulses == 4u && hrep && hr.aux0 == 4u);
    twilink::Params h0{};
    h0.ms = 100;
    if (!Link::act(Op::hold_sda, h0)) {
        bench.verdict("the peer accepted the SDA hold to its deadline", false);
        return;
    }
    wait_ms(5);
    const uint8_t stuck = Host::unstick();
    wait_ms(150);
    const uint8_t after = Host::unstick();
    print(serial, "  SDA held to the deadline: unstick() ", stuck, "; afterwards ", after, crlf);
    bench.verdict("nine pulses and a STOP cannot free a pad nothing releases (0xFF), and the "
                  "free wire is then left alone (0)",
                  stuck == 0xFFu && after == 0u);
    bench.verdict("the command channel is back after the holds", need_twi_peer());

    // ---- the arbiter's per-bus timeout over this engine ----
    twilink::Params w{};
    w.count = 64;
    w.ms = 150;
    w.addr = twilink::dut_addr;
    w.hold_us = 40'000;
    if (!Link::act(Op::serve, w)) {
        bench.verdict("the peer accepted the wedging serve", false);
        return;
    }
    kl::BusKernel::init_all();
    bus_ao_live = true;
    kl::Probe::clear_tally();
    static uint8_t two[2] = {0x5A, 0xA5};
    Host::Request rq{};
    rq.addr = twilink::dut_addr;
    rq.tx = lend<Lease::reply>(static_cast<const uint8_t*>(two));
    rq.tx_len = 2;
    rq.reply = reply_to<kl::Probe, I2cDone>();
    const uint32_t k0 = Ticker::ticks();
    post<kl::I2cArb>(rq);
    uint32_t replied_at = 0;
    while (Ticker::ticks() - k0 < 200u) {
        kl::pump();
        if (kl::Probe::n != 0u && replied_at == 0u) {
            replied_at = Ticker::ticks() - k0;
        }
    }
    bus_ao_live = false;
    wait_ms(150);
    (void)Host::unstick();
    print(serial, "  a tenure stretched 40 ms at its address: reply ", kl::Probe::replies[0],
          " after ", replied_at, " ms", crlf);
    // The band: a time event of 20 ticks matured by a pump polling every
    // tick, and the stopwatch's own tick either side - 19..22.
    bench.verdict("the arbiter answers it with i2c_timeout, at its 20 ms limit (19..22 ms)",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_timeout &&
                      replied_at >= 19u && replied_at <= 22u);
    bench.verdict("and the same host carries the command channel again", need_twi_peer());

    // ---- TIMEOUTA, the SMBus clock-low watcher, against a held clock ----
    // Letter g showed it quiet on an idle bus; here the peer holds SCL low
    // for 40 ms at the address, past the 25 ms SMBus limit.
    const uint32_t ker = Host::kernel_hz();
    const std::optional<uint16_t> code = fmpi2c_timeout_code_for(ker, 25'000u, false);
    twilink::Params lo{};
    lo.count = 64;
    lo.ms = 150;
    lo.addr = twilink::dut_addr;
    lo.hold_us = 40'000;
    if (!code || !Link::act(Op::serve, lo)) {
        bench.verdict("a 25 ms code exists here and the peer accepted the holding serve", false);
        return;
    }
    (void)S::timeouts(*code, false, true, 0, false);
    S::clear(FmpI2cClear::timeout);
    arm_log();
    const uint32_t a0 = Ticker::ticks();
    const uint8_t ts = tenure<Host>(twilink::dut_addr, two, 2, nullptr, 0);
    const uint32_t done_ms = Ticker::ticks() - a0;
    const uint32_t first_er = er_n != 0u ? er_first_tick - a0 : 0u;
    const uint32_t seen = er_seen;
    (void)S::timeouts(0, false, false, 0, false);
    S::clear(FmpI2cClear::timeout);
    wait_ms(150);
    (void)Host::unstick();
    print(serial, "  TIMEOUTA at ", fmpi2c_timeout_us(ker, *code, false) / 1000u,
          " ms against a 40 ms hold: the error vector first at ", first_er, " ms with ISR bits ",
          hex(seen), " (TIMEOUT is ", hex(FmpI2cFlag::timeout), "); the tenure ended ", ts,
          " after ", done_ms, " ms", crlf);
    // THE BAND: SCL goes low at the address's ninth clock (0.1 ms in), the
    // watcher counts 25 ms of it in steps of 2048 kernel clocks (45 us at
    // 45 MHz), and the tick reads +-1: 24..27 ms.
    bench.verdict("TIMEOUTA fires on a clock held low by ANOTHER CHIP, about 25 ms into the "
                  "hold (24..27 ms on the tick) - the half letter g could not stage",
                  (seen & FmpI2cFlag::timeout) != 0u && first_er >= 24u && first_er <= 27u);
    bench.verdict("the command channel is back after it", need_twi_peer());

    // ---- rebase() across the core's rate ----
    // The host is a user of the DynamicClock below, so every switch runs
    // its rebase(): the kernel clock (APB1 here) re-read and TIMINGR
    // re-derived. A wrong derivation still moves bytes - at a wrong rate -
    // so each rung TIMES its write against the rate asked.
    uint8_t rungs_ok = 0;
    for (uint8_t i = 0; i < 3u; ++i) {
        const bool up = i == 0u   ? Cores::template set_index<0>()
                        : i == 1u ? Cores::template set_index<1>()
                                  : Cores::template set_index<2>();
        twilink::Params sv{};
        sv.count = 64;
        sv.ms = 250;
        sv.addr = twilink::dut_addr;
        sv.seed = static_cast<uint8_t>(0x30u + i);
        const bool cmd = up && Link::act(Op::serve, sv);
        uint8_t rb[8] = {};
        // The kernel ticker's composed cycle count, not a bare VAL delta:
        // at 16 MHz the write outlasts a tick's period.
        const uint32_t c0 = Ticker::cycles();
        const uint8_t ws2 = tenure<Host>(twilink::dut_addr, w8, 8, nullptr, 0);
        const uint32_t cyc = Ticker::cycles() - c0;
        const uint8_t rs2 = tenure<Host>(twilink::dut_addr, nullptr, 0, rb, 8);
        uint8_t bad = 0;
        for (uint8_t k = 0; k < 8u; ++k) {
            if (rb[k] != twilink::pattern_value(twilink::pattern_counting, sv.seed, k)) {
                ++bad;
            }
        }
        wait_ms(300);
        const uint32_t mhz = Cores::hz() / 1'000'000u;
        const uint32_t per_ns = mhz != 0u ? static_cast<uint32_t>(
                                                static_cast<uint64_t>(cyc) * 1000u / mhz / 81u)
                                          : 0u;
        const uint32_t pm = per_ns != 0u ? 10'000'000u / per_ns : 0u;   // thousandths of 100 kHz
        print(serial, "  core ", mhz, " MHz, kernel ", Host::kernel_hz() / 1000u,
              " kHz, TIMINGR ", hex(S::regs().TIMINGR), ": serve ", cmd, ", write ", ws2, ", read ", rs2, ", ", bad,
              " wrong, the write's SCL at ", pm, " thousandths of 100 kHz", crlf);
        // Never above the rate by more than 2 %, and within a quarter below
        // it: a short tenure carries the request's own cost, and the
        // partner's polled target stretches each byte a little.
        if (cmd && ws2 == i2c_ok && rs2 == i2c_ok && bad == 0u && pm <= 1020u && pm >= 750u) {
            ++rungs_ok;
        }
    }
    (void)Cores::template set_index<0>();
    (void)host_ready();
    bench.verdict("at 180, 48 and 16 MHz of core the host's rebase() keeps the bus at the rate "
                  "asked (within 2 % above, a quarter below) and byte-exact against the partner",
                  rungs_ok == 3u);
    bench.verdict("... and the channel is back at the boot rate", need_twi_peer());
}

// =============================================================================
// o - ES0298 2.12.8: writes inside the erratum's clock-ratio band
// =============================================================================
//
// The erratum: with the first byte not already in TXDR, two bytes are
// requested in a row, and if the first is written "in less than two FMPI2C
// kernel clock cycles after the TXIS/DMA request" while the APB-to-kernel
// ratio is between 1.5 and 3, the second "is not internally detected" and the
// block stalls until a PE cycle. The host under `dma` has its first byte in
// TXDR before the START, its one path in and out of the band, so these writes
// judge that path. The HSI as the kernel against the 45 MHz APB1 is a ratio of
// 2.81; two kernel cycles there are 125 ns, 22 core cycles at 180 MHz. THE
// CONTROL is the sequence the erratum describes, staged by the letter through
// the resource's verbs: it must stall, or the letter proves nothing.

/// The longest write the Request's eight-bit length can name.
uint8_t trial_out[255];

struct TrialRow {
    uint16_t tenures = 0;
    uint16_t completed = 0;   ///< answered i2c_ok
    uint16_t parked_n = 0;    ///< no completion inside the bound: stalled, and recovered
    uint16_t other = 0;       ///< answered anything else
    uint16_t sent = 0;        ///< the bytes that should have reached the partner
    uint16_t sum = 0;         ///< and their sum, as the partner keeps it
    uint32_t stall_isr = 0;   ///< at the first stall: ISR
    uint32_t stall_ndtr = 0;  ///< the transmit stream's count
    bool stall_scl = true;    ///< and the clock line
    bool stall_sda = true;    ///< and the data line
    twilink::Report peer{};
    bool reported = false;
};

/// A byte of tenure `k`: every tenure and every position different, so that a
/// lost or a repeated byte moves the peer's sum as well as its count.
constexpr uint8_t trial_value(uint16_t k, uint8_t i) {
    return static_cast<uint8_t>(0x35u + 0x4Bu * k + 0x1Du * i + ((i * i) >> 3));
}

/// THE BOUND ON ONE TENURE, in ticks of 1 ms. On the wire: nine clocks for the
/// address and for each byte, at the speed's period - a rate that letter f
/// measures within 6 per cent of the speed at 100 kHz and 400 kHz on this
/// kernel clock - plus the START and the STOP, under a period each. The
/// partner's polled target adds what the preemptive suite measures, at most
/// 66 us of a 382 us write of sixteen bytes, 17 per cent. Twice the wire holds
/// all of that, and 3 ticks cover the tick's own reading on either side and a
/// tenure far shorter than a tick.
uint32_t trial_limit_ms(uint8_t len, FmpI2cSpeed s) {
    const uint32_t clocks = 9u * (static_cast<uint32_t>(len) + 1u) + 2u;
    return 2u * clocks * 1000u / fmpi2c_speed_hz(s) + 3u;
}

/// One write of the buffer below, waited for on the tick: its status, or
/// `parked`.
template <typename Bus>
uint8_t trial_write(uint8_t len, FmpI2cSpeed s, I2cEngine e, uint32_t limit_ms) {
    typename Bus::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(trial_out));
    r.tx_len = len;
    r.speed = s;
    r.engine = e;
    xfer_done = false;
    if (Bus::start(r)) {
        return Bus::status();
    }
    const uint32_t t0 = Ticker::ticks();
    while (!xfer_done && Ticker::ticks() - t0 < limit_ms) {
    }
    return xfer_done ? Bus::status() : parked;
}

/// THE CONTROL, staged by the letter and not by the host: TXDR left empty, the
/// transmit stream started on the WHOLE run and its request enabled before the
/// START, through the resource's verbs with the host's vectors masked off and
/// STOPF polled. The erratum's own conditions, inside the band; a run of two
/// bytes or more, as the letter asks it.
uint8_t trial_unloaded(uint8_t len, FmpI2cSpeed s, uint32_t limit_ms) {
    S::interrupt(FmpI2cInterrupt::rx | FmpI2cInterrupt::tx | FmpI2cInterrupt::nack |
                     FmpI2cInterrupt::stop | FmpI2cInterrupt::error |
                     FmpI2cInterrupt::transfer_complete,
                 false);
    S::clear(FmpI2cClear::all);
    (void)S::disable();
    (void)S::timing(DmaHost::timing_of(s));
    S::enable();
    (void)TxEngine::start(std::span<const uint8_t>(trial_out, len));
    S::dma_transmit(true);
    (void)S::transfer_now(twilink::dut_addr, false, len, true);
    uint8_t st = parked;
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < limit_ms) {
        const uint32_t f = S::flags();
        if ((f & FmpI2cFlag::stop) != 0u) {
            st = (f & FmpI2cFlag::nack) != 0u ? i2c_nack_data : i2c_ok;
            break;
        }
    }
    S::dma_transmit(false);
    TxEngine::stop();
    S::clear(FmpI2cClear::all);
    return st;
}

/// A batch: the partner told to take bytes for as long as all of it could
/// last, the host brought up on `kernel`, `tenures` writes of `len`, a stall
/// recovered by the PE cycle the erratum names, then the partner's count, sum
/// and address matches read back on the APB1 kernel, outside the band.
/// `control` writes through trial_unloaded() in place of the host.
template <typename Bus>
TrialRow trial_batch(FmpI2cClock kernel, FmpI2cSpeed s, uint8_t len, uint16_t tenures,
                     I2cEngine e, bool control = false) {
    TrialRow row{};
    row.tenures = tenures;
    const uint32_t limit = trial_limit_ms(len, s);
    twilink::Params a{};
    a.count = 0;   // the deadline alone ends the serve
    a.ms = static_cast<uint16_t>(tenures * limit + 20u);
    a.addr = twilink::dut_addr;
    (void)host_ready();
    if (!Link::act(Op::serve, a)) {
        return row;
    }
    const uint32_t t0 = Ticker::ticks();
    FmpI2cHostConfig c = base_config;
    c.kernel = kernel;
    if constexpr (Bus::has_dma) {
        dma_host_live = true;
    }
    if (Bus::init(clock, c)) {
        for (uint16_t k = 0; k < tenures; ++k) {
            uint16_t sum = 0;
            for (uint16_t i = 0; i < len; ++i) {
                trial_out[i] = trial_value(k, static_cast<uint8_t>(i));
                sum = static_cast<uint16_t>(sum + trial_out[i]);
            }
            const uint8_t st = control ? trial_unloaded(len, s, limit)
                                       : trial_write<Bus>(len, s, e, limit);
            if (st == i2c_ok) {
                row.completed = static_cast<uint16_t>(row.completed + 1u);
                row.sent = static_cast<uint16_t>(row.sent + len);
                row.sum = static_cast<uint16_t>(row.sum + sum);
            } else if (st == parked) {
                // "Stalled after first byte": that byte is on the wire, and
                // the partner's tally is expected to hold it and nothing more.
                row.sent = static_cast<uint16_t>(row.sent + 1u);
                row.sum = static_cast<uint16_t>(row.sum + trial_out[0]);
                if (row.parked_n == 0u) {
                    row.stall_isr = S::flags();
                    row.stall_ndtr = DmaStream<TxEngine::controller, TxEngine::stream>::regs().NDTR;
                    row.stall_scl = SclPad::read();
                    row.stall_sda = SdaPad::read();
                }
                row.parked_n = static_cast<uint16_t>(row.parked_n + 1u);
                (void)Bus::recover();
            } else {
                row.other = static_cast<uint16_t>(row.other + 1u);
            }
        }
    }
    dma_host_live = false;
    while (Ticker::ticks() - t0 < a.ms + 5u) {
    }
    (void)host_ready();
    row.reported = Link::report(row.peer);
    return row;
}

/// The partner's tally is the host's, byte for byte: every tenure whole, or
/// stalled with its first byte delivered and nothing after it.
bool trial_accounted(const TrialRow& r) {
    return r.reported && r.other == 0u && r.completed + r.parked_n == r.tenures &&
           r.peer.count == r.sent && r.peer.sum == r.sum && r.peer.addr_hits == r.tenures;
}

/// ... and no tenure stalled.
bool trial_exact(const TrialRow& r) { return trial_accounted(r) && r.parked_n == 0u; }

void trial_print(const char* kernel, const char* engine, FmpI2cSpeed s, uint8_t len,
                 const TrialRow& r) {
    print(serial, "  ", kernel, " ", engine, " ", s == FmpI2cSpeed::fast_400k ? "400" : "100",
          " kHz, write of ", len, ": ", r.tenures, " tenures, ", r.completed, " ok, ", r.parked_n,
          " STALLED, ", r.other, " other; the partner took ", r.peer.count, " bytes of ", r.sent,
          " accounted, sum ", r.peer.sum, " of ", r.sum, ", ", r.peer.addr_hits,
          " address matches", crlf);
    if (r.parked_n != 0u) {
        print(serial, "    the first stall: ISR ", hex(r.stall_isr), ", the transmit stream's "
              "count ", r.stall_ndtr, ", SCL ", r.stall_scl ? "high" : "LOW", ", SDA ",
              r.stall_sda ? "high" : "LOW", crlf);
    }
}

void to_erratum() {
    if (!need_twi_peer()) {
        return;
    }
    Dma<1>::init();

    // ---- what a stall and a wrong byte look like to this letter ----
    // A tenure whose transmit interrupt the letter takes away after start() is a
    // stall made by hand: the address goes out and nobody ever writes TXDR. And
    // a byte changed after its sum was taken is what the partner's sum exists to
    // see. Each on the APB1 kernel, outside the band.
    {
        twilink::Params a{};
        a.ms = 80;
        a.addr = twilink::dut_addr;
        (void)host_ready();
        const bool cmd = Link::act(Op::serve, a);
        const uint32_t t0 = Ticker::ticks();
        (void)host_ready();
        for (uint8_t i = 0; i < 16u; ++i) {
            trial_out[i] = trial_value(0, i);
        }
        Host::Request q{};
        q.addr = twilink::dut_addr;
        q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(trial_out));
        q.tx_len = 16;
        q.speed = FmpI2cSpeed::fast_400k;
        xfer_done = false;
        const bool sync = Host::start(q);
        S::interrupt(FmpI2cInterrupt::tx, false);
        const uint32_t limit = trial_limit_ms(16, FmpI2cSpeed::fast_400k);
        const uint32_t w0 = Ticker::ticks();
        while (!xfer_done && Ticker::ticks() - w0 < limit) {
        }
        const bool staged = !sync && !xfer_done && !SclPad::read();
        (void)Host::recover();
        S::interrupt(FmpI2cInterrupt::tx, true);
        uint16_t sum = 0;
        for (uint8_t i = 0; i < 16u; ++i) {
            sum = static_cast<uint16_t>(sum + trial_out[i]);
        }
        trial_out[3] = static_cast<uint8_t>(trial_out[3] ^ 0x01u);
        const uint8_t st = trial_write<Host>(16, FmpI2cSpeed::fast_400k, I2cEngine::interrupt,
                                             limit);
        while (Ticker::ticks() - t0 < a.ms + 5u) {
        }
        (void)host_ready();
        twilink::Report r{};
        const bool rep = Link::report(r);
        print(serial, "  the control: a stall by hand parked ", staged, " with SCL low; the next "
              "write ", st, ", the partner took ", r.count, " with sum ", r.sum, " against ", sum,
              " for the bytes before one was changed", crlf);
        bench.verdict("A STALL IS SEEN: a transmit interrupt taken away parks a tenure with "
                      "SCL low, inside the bound",
                      cmd && staged);
        bench.verdict("... and one byte changed after its sum was taken moves the partner's sum",
                      rep && st == i2c_ok && r.count == 16u && r.sum != sum);
    }

    // ---- outside the band: the host's one path holds there too ----
    const FmpI2cSpeed speeds[2] = {FmpI2cSpeed::standard_100k, FmpI2cSpeed::fast_400k};
    struct Shape {
        uint8_t len;
        uint16_t tenures;
    };
    const Shape shapes[3] = {{2, 100}, {16, 100}, {255, 20}};
    print(serial, "  APB1 / kernel: ", Host::reference_hz(), " / ", Host::kernel_hz(), " Hz", crlf);
    bool out_exact = true;
    {
        TrialRow r = trial_batch<Host>(FmpI2cClock::pclk, FmpI2cSpeed::fast_400k, 16, 100,
                                       I2cEngine::interrupt);
        trial_print("APB1", "interrupt", FmpI2cSpeed::fast_400k, 16, r);
        out_exact = out_exact && trial_exact(r);
        r = trial_batch<DmaHost>(FmpI2cClock::pclk, FmpI2cSpeed::fast_400k, 1, 100, I2cEngine::dma);
        trial_print("APB1", "dma", FmpI2cSpeed::fast_400k, 1, r);
        out_exact = out_exact && trial_exact(r);
        for (const Shape& sh : shapes) {
            r = trial_batch<DmaHost>(FmpI2cClock::pclk, FmpI2cSpeed::fast_400k, sh.len, sh.tenures,
                                     I2cEngine::dma);
            trial_print("APB1", "dma", FmpI2cSpeed::fast_400k, sh.len, r);
            out_exact = out_exact && trial_exact(r);
            r = trial_batch<DmaHost>(FmpI2cClock::sysclk, FmpI2cSpeed::fast_400k, sh.len, sh.tenures,
                                     I2cEngine::dma);
            trial_print("SYSCLK", "dma", FmpI2cSpeed::fast_400k, sh.len, r);
            out_exact = out_exact && trial_exact(r);
        }
    }
    bench.verdict("outside the band, ratios of 1.0 and 0.25, the host's writes of one, two, "
                  "sixteen and 255 on the DMA engines: no stall, every byte judged by the "
                  "partner's count and sum, and a hundred of sixteen on the interrupt",
                  out_exact);

    // ---- inside the band ----
    (void)host_ready(FmpI2cClock::hsi);
    const bool in_band = Host::transmit_stall_risk();
    print(serial, "  APB1 / kernel: ", Host::reference_hz(), " / ", Host::kernel_hz(),
          " Hz, transmit_stall_risk() ", in_band, crlf);
    bench.verdict("the HSI against the 45 MHz APB1 is inside the erratum's band, and the host "
                  "says so",
                  in_band);
    bool pump_exact = true;
    bool dma_exact = true;
    bool control_accounted = true;
    uint32_t dma_tenures = 0;
    uint32_t dma_stalls = 0;
    uint32_t control_tenures = 0;
    uint32_t control_stalls = 0;
    for (const FmpI2cSpeed s : speeds) {
        for (const Shape& sh : shapes) {
            const TrialRow p = trial_batch<Host>(FmpI2cClock::hsi, s, sh.len, sh.tenures,
                                                 I2cEngine::interrupt);
            trial_print("HSI", "interrupt", s, sh.len, p);
            pump_exact = pump_exact && trial_exact(p);
            const TrialRow d = trial_batch<DmaHost>(FmpI2cClock::hsi, s, sh.len, sh.tenures,
                                                    I2cEngine::dma);
            trial_print("HSI", "dma", s, sh.len, d);
            dma_exact = dma_exact && trial_exact(d);
            dma_tenures += d.tenures;
            dma_stalls += d.parked_n;
            const TrialRow c = trial_batch<DmaHost>(FmpI2cClock::hsi, s, sh.len, sh.tenures,
                                                    I2cEngine::dma, true);
            trial_print("HSI", "the control, TXDR left empty,", s, sh.len, c);
            control_accounted = control_accounted && trial_accounted(c);
            control_tenures += c.tenures;
            control_stalls += c.parked_n;
        }
    }
    {
        const TrialRow one = trial_batch<DmaHost>(FmpI2cClock::hsi, FmpI2cSpeed::fast_400k, 1, 100,
                                                  I2cEngine::dma);
        trial_print("HSI", "dma", FmpI2cSpeed::fast_400k, 1, one);
        dma_exact = dma_exact && trial_exact(one);
    }
    print(serial, "  inside the band: the host on the DMA engines ", dma_stalls, " stalls in ",
          dma_tenures, " tenures; the control ", control_stalls, " stalls in ", control_tenures,
          crlf);
    bench.verdict("inside the band on the interrupt: no stall and every byte of every write - the "
                  "vector writes TXDR outside the erratum's two kernel cycles",
                  pump_exact);
    bench.verdict("inside the band on the DMA engines, the host's own path, a write of one "
                  "included: no stall and every byte of every write",
                  dma_exact && dma_stalls == 0u);
    bench.verdict("the control, TXDR left empty, every tenure accounted byte for byte: whole, or "
                  "stalled with its first byte on the wire and nothing after it, and the PE "
                  "cycle recovers it",
                  control_accounted);
    // THE BOUND: 2798 stalls in 20240 tenures of this control measured, 13.8 per
    // cent, so 440 tenures with none would come up (1 - 0.138)^440, less than once
    // in 10^28 runs. A control with no stall is a letter that cannot fail.
    bench.verdict("... and THE LETTER CAN FAIL: the erratum is alive on this die, at least one "
                  "stall in the control's tenures",
                  control_stalls != 0u);
    (void)host_ready();
    bench.verdict("... and the channel is back at the boot rate", need_twi_peer());
}

// =============================================================================
// p - A REFUSED WRITE, position by position, against the desk partner (set `t`)
// =============================================================================
//
// The partner refuses by its own target byte control: `nack_at` names the
// received byte, counted from one, and every byte after it; `flag_deaf` parks
// its client at twi_link's deaf_addr, which is a NACK on the address at
// dut_addr. Its tally counts the refused byte too: the byte reaches RXDR before
// the ninth pulse is answered.
//
// THE RUN HAS NO init() AND NO PE CYCLE BETWEEN A FAILED TENURE AND THE NEXT
// REQUEST, on purpose: either would flush TXDR (RM0390 23.7.7, TXE set when
// PE = 0) and hide a byte the failed tenure left there. So the command channel
// runs over the engined host itself, at the row's own speed (a change of speed
// costs a PE cycle), and on the engine the row names.

/// The command channel over the host under test, at the row's speed.
FmpI2cSpeed link_speed = FmpI2cSpeed::standard_100k;

template <I2cEngine e>
struct EnginePort {
    static uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                          uint8_t rx_len) {
        return ::tenure<DmaHost>(addr, tx, tx_len, rx, rx_len, link_speed, e);
    }
    static void ready() { (void)DmaHost::recover(); }
    static void wait_ms(uint32_t ms) { ::wait_ms(ms); }
};
using LinkOnPump = TwiLinkClient<EnginePort<I2cEngine::interrupt>>;
using LinkOnDma = TwiLinkClient<EnginePort<I2cEngine::dma>>;

/// A write of `tx` then a read of `rx`, refused at data byte `at` of the write,
/// counted from one, or on the address at zero.
struct Refusal {
    uint8_t tx;
    uint8_t rx;
    uint8_t at;
};

constexpr Refusal refusal_rows[] = {
    {1, 0, 0},    {2, 0, 0},  {16, 0, 0}, {1, 4, 0},  {2, 4, 0},     // the address
    {1, 0, 1},    {1, 4, 1},                                         // a write of one
    {2, 0, 1},    {2, 4, 1},  {16, 0, 1},                            // the first byte
    {2, 0, 2},    {2, 4, 2},  {16, 0, 2},                            // the second
    {3, 0, 3},    {16, 0, 3}, {16, 0, 7}, {16, 0, 16},               // a later byte
    {255, 0, 200}, {255, 0, 255},
};

/// One position's rows: how many landed there and how many of those were right.
struct Position {
    uint16_t seen = 0;
    uint16_t right = 0;
    constexpr void add(const Position& o) {
        seen = static_cast<uint16_t>(seen + o.seen);
        right = static_cast<uint16_t>(right + o.right);
    }
    constexpr bool whole() const { return seen != 0u && right == seen; }
};

/// What the rows of a pass came to, by where the partner's count says each landed.
struct RefusalPass {
    uint16_t rows = 0;
    uint16_t status_right = 0;   ///< start() asynchronous and status() the right NACK
    uint16_t tally_right = 0;    ///< the partner's count, sum, first byte, matches and flag
    uint16_t next_whole = 0;     ///< the next request whole on the partner
    uint16_t slipped = 0;        ///< rows the partner refused one byte sooner than asked
    Position address;            ///< refused on the address
    Position first;              ///< the first byte, of a write of two or more
    Position second;
    Position later;
    Position one;                ///< a write of one
    Position two;                ///< a write of two, either byte
};

/// The bytes after a NACK on the address: a second tenure in the same serve, to
/// the address the partner is parked at.
constexpr uint8_t refusal_after(uint8_t i) { return static_cast<uint8_t>(0xC7u + 0x35u * i); }

/// One row. `starve` takes the transmit request away just after start(), so the
/// stream never answers and the count stands still under a byte already gone:
/// the case the test of TXE exists for.
template <typename Link>
void refusal_row(const Refusal& row, I2cEngine e, FmpI2cSpeed s, bool starve, bool print_row,
                 RefusalPass& pass) {
    const uint8_t next_len = 16;
    const uint32_t limit = trial_limit_ms(row.tx, s);
    twilink::Params a{};
    a.ms = static_cast<uint16_t>(limit + trial_limit_ms(next_len, s) + 10u);
    a.addr = twilink::dut_addr;
    if (row.at == 0u) {
        a.flags = twilink::flag_deaf;
    } else {
        a.nack_at = row.at;
    }
    link_speed = s;
    const bool served = Link::act(Op::serve, a);
    const uint32_t t0 = Ticker::ticks();

    uint16_t sum = 0;
    for (uint16_t i = 0; i < row.tx; ++i) {
        trial_out[i] = trial_value(static_cast<uint16_t>(row.tx + row.at), static_cast<uint8_t>(i));
    }
    uint8_t in[4] = {0xEE, 0xEE, 0xEE, 0xEE};
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(trial_out));
    r.tx_len = row.tx;
    r.rx = lend<Lease::reply>(row.rx != 0u ? in : nullptr);
    r.rx_len = row.rx;
    r.speed = s;
    r.engine = e;
    xfer_done = false;
    const bool synchronous = DmaHost::start(r);
    if (starve) {
        S::dma_transmit(false);
    }
    const uint32_t w0 = Ticker::ticks();
    while (!xfer_done && Ticker::ticks() - w0 < limit) {
    }
    const uint8_t status = xfer_done ? DmaHost::status() : parked;
    if (status == parked) {
        (void)DmaHost::recover();
    }
    bool untouched = true;
    for (const uint8_t v : in) {
        untouched = untouched && v == 0xEEu;
    }

    // THE NEXT REQUEST. After a refused address, a write in the same serve, the
    // partner's tally judging it byte for byte; after a refused byte the partner
    // keeps refusing until its deadline, so the next request is the report's
    // frame: a stale byte ahead of it costs the frame its checksum.
    uint8_t after[next_len];
    uint16_t after_sum = 0;
    for (uint8_t i = 0; i < next_len; ++i) {
        after[i] = refusal_after(i);
        after_sum = static_cast<uint16_t>(after_sum + after[i]);
    }
    bool next_ok = true;
    if (row.at == 0u) {
        next_ok = tenure<DmaHost>(twilink::deaf_addr, after, next_len, nullptr, 0, s, e) == i2c_ok;
    }
    while (Ticker::ticks() - t0 < a.ms + 5u) {
    }
    twilink::Report rep{};
    bool first_try = false;
    {
        twilink::Frame f;
        first_try = Link::command_once(Op::report, after, 0);
        if (first_try) {
            wait_ms(2);
            first_try = Link::recv_frame(f) && f.op == Op::report_data &&
                        f.len == twilink::report_size;
            if (first_try) {
                rep = twilink::get_report(f.data);
            }
        }
    }
    const bool reported = first_try || Link::report(rep);
    if (row.at != 0u) {
        next_ok = first_try;
    }

    const uint8_t want = row.at == 0u ? i2c_nack_addr : i2c_nack_data;
    const bool status_right = !synchronous && status == want && untouched;
    const bool nacked = (rep.flags & twilink::report_nacked) != 0u;
    // WHERE THE PARTNER REALLY REFUSED IS ITS OWN COUNT, `at` or one sooner. Its
    // target polls RXNE and TCR on separate passes and decides at TCR on what it
    // has counted so far, so a pass that sees RXNE first counts the byte before
    // the decision and refuses that one. The sum is taken over the count, which
    // judges every byte the partner kept wherever the refusal landed.
    uint16_t landed = 0;
    if (row.at != 0u && reported && rep.count <= row.tx) {
        landed = rep.count;
        for (uint16_t i = 0; i < landed; ++i) {
            sum = static_cast<uint16_t>(sum + trial_out[i]);
        }
    }
    const bool tally_right =
        served && reported && rep.addr_hits == 1u &&
        (row.at == 0u ? rep.count == next_len && rep.sum == after_sum &&
                            rep.first == after[0] && !nacked
                      : landed != 0u && landed <= row.at && landed + 1u >= row.at &&
                            rep.sum == sum && rep.first == trial_out[0] && nacked);
    const bool whole = next_ok && (row.at != 0u || tally_right);
    const bool right = status_right && tally_right && whole;
    if (print_row || !right) {
        print(serial, "    ", row.tx, "+", row.rx, " refused at ", row.at, ": start ", synchronous,
              ", status ", status, " of ", want, "; the partner took ", rep.count, " sum ", rep.sum,
              " first ", hex(rep.first), " matches ", rep.addr_hits, " flags ", hex(rep.flags),
              "; the next request ", next_ok ? "whole" : "WRONG", right ? "" : " <- WRONG", crlf);
    }
    pass.rows = static_cast<uint16_t>(pass.rows + 1u);
    pass.status_right = static_cast<uint16_t>(pass.status_right + (status_right ? 1u : 0u));
    pass.tally_right = static_cast<uint16_t>(pass.tally_right + (tally_right ? 1u : 0u));
    pass.next_whole = static_cast<uint16_t>(pass.next_whole + (whole ? 1u : 0u));
    pass.slipped = static_cast<uint16_t>(pass.slipped + (landed != 0u && landed != row.at ? 1u : 0u));
    auto count = [right](Position& p) {
        p.seen = static_cast<uint16_t>(p.seen + 1u);
        p.right = static_cast<uint16_t>(p.right + (right ? 1u : 0u));
    };
    if (row.at == 0u) {
        count(pass.address);
    } else if (row.tx == 1u) {
        count(pass.one);
    } else if (landed == 1u) {
        count(pass.first);
    } else if (landed == 2u) {
        count(pass.second);
    } else {
        count(pass.later);
    }
    if (row.tx == 2u) {
        count(pass.two);
    }
}

/// A pass of every row: the host brought up once on `kernel`, nothing between
/// the rows but the requests.
template <typename Link>
RefusalPass refusal_pass(FmpI2cClock kernel, FmpI2cSpeed s, I2cEngine e, const char* name) {
    RefusalPass pass{};
    FmpI2cHostConfig c = base_config;
    c.kernel = kernel;
    dma_host_live = true;
    if (DmaHost::init(clock, c)) {
        for (const Refusal& row : refusal_rows) {
            refusal_row<Link>(row, e, s, false, false, pass);
        }
    }
    dma_host_live = false;
    print(serial, "  ", name, " ", s == FmpI2cSpeed::fast_400k ? "400" : "100", " kHz: ",
          pass.rows, " rows, status right ", pass.status_right, ", the partner's tally right ",
          pass.tally_right, ", the next request whole ", pass.next_whole, ", refused sooner ",
          pass.slipped, crlf);
    return pass;
}

void tp_refusals() {
    if (!need_twi_peer()) {
        return;
    }
    Dma<1>::init();
    const FmpI2cSpeed speeds[2] = {FmpI2cSpeed::standard_100k, FmpI2cSpeed::fast_400k};
    RefusalPass dma{};
    bool pump_right = true;
    uint32_t pump_rows = 0;
    auto fold = [&](const RefusalPass& p) {
        dma.rows = static_cast<uint16_t>(dma.rows + p.rows);
        dma.status_right = static_cast<uint16_t>(dma.status_right + p.status_right);
        dma.tally_right = static_cast<uint16_t>(dma.tally_right + p.tally_right);
        dma.next_whole = static_cast<uint16_t>(dma.next_whole + p.next_whole);
        dma.slipped = static_cast<uint16_t>(dma.slipped + p.slipped);
        dma.address.add(p.address);
        dma.first.add(p.first);
        dma.second.add(p.second);
        dma.later.add(p.later);
        dma.one.add(p.one);
        dma.two.add(p.two);
    };
    auto fold_pump = [&](const RefusalPass& p) {
        pump_rows += p.rows;
        pump_right = pump_right && p.rows != 0u && p.status_right == p.rows &&
                     p.tally_right == p.rows && p.next_whole == p.rows;
    };
    for (const FmpI2cClock kernel : {FmpI2cClock::pclk, FmpI2cClock::hsi}) {
        const bool hsi = kernel == FmpI2cClock::hsi;
        for (const FmpI2cSpeed s : speeds) {
            fold(refusal_pass<LinkOnPump>(kernel, s, I2cEngine::dma,
                                          hsi ? "HSI dma, the next on interrupt"
                                              : "APB1 dma, the next on interrupt"));
            fold(refusal_pass<LinkOnDma>(kernel, s, I2cEngine::dma,
                                         hsi ? "HSI dma, the next on dma" : "APB1 dma, the next on dma"));
            fold_pump(refusal_pass<LinkOnPump>(kernel, s, I2cEngine::interrupt,
                                               hsi ? "HSI interrupt, the next on interrupt"
                                                   : "APB1 interrupt, the next on interrupt"));
            fold_pump(refusal_pass<LinkOnDma>(kernel, s, I2cEngine::interrupt,
                                              hsi ? "HSI interrupt, the next on dma"
                                                  : "APB1 interrupt, the next on dma"));
        }
    }
    print(serial, "  under dma, right of landed: the address ", dma.address.right, "/", dma.address.seen,
          ", a write of one ", dma.one.right, "/", dma.one.seen, ", a first byte ", dma.first.right,
          "/", dma.first.seen, ", a second ", dma.second.right, "/", dma.second.seen, ", a later ",
          dma.later.right, "/", dma.later.seen, ", a write of two ", dma.two.right, "/", dma.two.seen,
          "; refused sooner than asked ", dma.slipped, " of ", dma.rows, "; the event vector's rows ",
          pump_rows, crlf);
    bench.verdict("under dma a NACK on the ADDRESS answers i2c_nack_addr, nothing of the write on "
                  "the partner, in a write of one, of two, of sixteen and with a read half behind",
                  dma.address.whole());
    bench.verdict("... on the FIRST data byte of two or more, i2c_nack_data, the partner's count, "
                  "sum and first byte exact",
                  dma.first.whole());
    bench.verdict("... on the SECOND data byte, the same", dma.second.whole());
    bench.verdict("... on a LATER byte, the last of sixteen and of 255 among the rows, the same",
                  dma.later.whole());
    bench.verdict("... on a WRITE OF ONE, i2c_nack_data, with no stream under it", dma.one.whole());
    bench.verdict("... on either byte of a WRITE OF TWO", dma.two.whole());
    bench.verdict("every one of them: start() asynchronous, status() the right NACK, a read half "
                  "behind it never run",
                  dma.status_right == dma.rows);
    bench.verdict("... the partner's tally exact and the next request whole, on either engine",
                  dma.tally_right == dma.rows && dma.next_whole == dma.rows);
    bench.verdict("the same rows on the event vector", pump_right);

    // THE STREAM THAT NEVER ANSWERED. A first byte gone to the shifter and the
    // stream's request taken away: TXE stands and the count has not moved, so a
    // test of the count alone would read this data NACK as the address's.
    {
        RefusalPass starved{};
        dma_host_live = true;
        if (DmaHost::init(clock, base_config)) {
            const Refusal row{16, 0, 1};
            refusal_row<LinkOnPump>(row, I2cEngine::dma, FmpI2cSpeed::fast_400k, true, true, starved);
        }
        dma_host_live = false;
        bench.verdict("the first byte refused with the stream starved: TXE tells it from the "
                      "address, i2c_nack_data, and the next request whole",
                      starved.rows == 1u && starved.status_right == 1u && starved.tally_right == 1u &&
                          starved.next_whole == 1u);
    }
    // A START LOST INTO A HELD LINE, the first byte already in TXDR and the stream
    // armed. The partner's hold falling under a high SCL reads as a START, so BUSY
    // would park the tenure behind it: the PE cycle takes BUSY down first, as the
    // arbiter's recover() does, and the START then loses its address to the low
    // line. The next request is the report's frame, with no PE cycle in between.
    {
        dma_host_live = true;
        link_speed = FmpI2cSpeed::fast_400k;
        const bool up = DmaHost::init(clock, base_config);
        twilink::Params h{};
        h.ms = 20;
        const bool held = up && LinkOnPump::act(Op::hold_sda, h);
        const uint32_t t0 = Ticker::ticks();
        (void)DmaHost::recover();
        for (uint8_t i = 0; i < 4u; ++i) {
            trial_out[i] = trial_value(0x55u, i);
        }
        const uint8_t st = tenure<DmaHost>(twilink::dut_addr, trial_out, 4, nullptr, 0,
                                           FmpI2cSpeed::fast_400k, I2cEngine::dma);
        const bool stream_off = (DmaStream<TxEngine::controller, TxEngine::stream>::regs().CR &
                                 DMA_SxCR_EN) == 0u;
        while (Ticker::ticks() - t0 < h.ms + 5u) {
        }
        const bool next = LinkOnPump::command_once(Op::ping, trial_out, 0);
        print(serial, "  a START lost into a held line, under dma: status ", st, ", the stream off ",
              stream_off, "; the next request ", next ? "whole" : "WRONG", crlf);
        bench.verdict("a START lost under dma: i2c_arb_lost, the stream taken back, the first byte "
                      "gone with it and the next request whole",
                      held && st == i2c_arb_lost && stream_off && next);
        dma_host_live = false;
    }
    (void)host_ready();
    bench.verdict("... and the channel is back at the boot rate", need_twi_peer());
}

// =============================================================================
// the menu
// =============================================================================

void banner() {
    print(serial, crlf, "test_stm32f4_fmpi2c - FMPI2C1 on PC6/PC7, a bus with no device on it",
          crlf);
    bench.menu();
}

}   // namespace

// ---- the vectors ---------------------------------------------------------------------------

extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }

// I2C1's two vectors: letter m's far end, and letter f's classic host.
extern "C" void I2C1_EV_IRQHandler() {
    if (Far::live) {
        Far::event();
    } else if (classic_host_live && ClassicHost::isr()) {
        xfer_done = true;
    }
}
extern "C" void I2C1_ER_IRQHandler() {
    if (Far::live) {
        Far::error();
    } else if (classic_host_live && ClassicHost::error_isr()) {
        xfer_done = true;
    }
}

extern "C" void FMPI2C1_EV_IRQHandler() {
    if (ev_n < 4u) {
        ev_log[ev_n] = brio::FmpI2c<1>::flags();
        ev_n = static_cast<uint8_t>(ev_n + 1u);
    }
    if (peer_live) {
        (void)Peer::service();
        return;
    }
    if (dma_host_live) {
        if (DmaHost::isr()) {
            xfer_done = true;
        }
        return;
    }
    if (Host::isr()) {
        xfer_done = true;
        if (bus_ao_live) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        }
    }
}

extern "C" void FMPI2C1_ER_IRQHandler() {
    if (er_n == 0u) {
        er_first_tick = brio::Ticker::ticks();
    }
    er_seen = er_seen | brio::FmpI2c<1>::flags();
    er_n = static_cast<uint8_t>(er_n + 1u);
    if (peer_live) {
        (void)Peer::error_service();
        return;
    }
    if (dma_host_live) {
        if (DmaHost::error_isr()) {
            xfer_done = true;
        }
        return;
    }
    if (Host::error_isr()) {
        xfer_done = true;
        if (bus_ao_live) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        }
    }
}

extern "C" void DMA1_Stream5_IRQHandler() {
    tx_dma_entries = tx_dma_entries + 1u;
    if (dma_host_live && DmaHost::dma_isr()) {
        xfer_done = true;
    }
}
extern "C" void DMA1_Stream2_IRQHandler() {
    rx_dma_entries = rx_dma_entries + 1u;
    if (dma_host_live && DmaHost::dma_isr()) {
        xfer_done = true;
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    // What the silicon held before a line of this program ran: the gate is
    // read out of the RCC (reading it does not open it), and the registers
    // behind it need the clock, so it is opened for the reading.
    boot.gate = brio::FmpI2c<1>::bus_clock();
    brio::FmpI2c<1>::bus_clock(true);
    boot.cr1 = brio::FmpI2c<1>::regs().CR1;
    boot.cr2 = brio::FmpI2c<1>::regs().CR2;
    boot.oar1 = brio::FmpI2c<1>::regs().OAR1;
    boot.oar2 = brio::FmpI2c<1>::regs().OAR2;
    boot.timingr = brio::FmpI2c<1>::regs().TIMINGR;
    boot.timeoutr = brio::FmpI2c<1>::regs().TIMEOUTR;
    boot.isr = brio::FmpI2c<1>::regs().ISR;
    boot.pecr = brio::FmpI2c<1>::regs().PECR;
    boot.kernel_code = static_cast<uint8_t>(brio::FmpI2c<1>::kernel_clock());
    boot.smbus = brio::FmpI2c<1>::smbus_probe();
    // Where the STM32G0's identical register file keeps WUPEN: bit 18 of
    // CR1, which 23.7.1 makes reserved here.
    brio::FmpI2c<1>::regs().CR1 = 1u << 18;
    boot.wupen_readback = brio::FmpI2c<1>::regs().CR1;
    brio::FmpI2c<1>::regs().CR1 = 0;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // The two lines as plain inputs with the port's pull-ups, before the
    // peripheral is given them: what this board leaves on the wire.
    SclPad::input(brio::PinPull::up);
    SdaPad::input(brio::PinPull::up);
    (void)brio::delay_us(brio::delay_rate(SysClock::hz), 100);
    boot.scl = SclPad::read();
    boot.sda = SdaPad::read();

    const bool host_ok = host_ready();
    boot.isr_ready = brio::FmpI2c<1>::regs().ISR;

    bench.letter('a', "the block", ta_block);
    bench.letter('b', "the timing arithmetic", tb_timing);
    bench.letter('c', "the kernel-clock multiplexer", tc_kernel_clock);
    bench.letter('d', "what the driver refuses", td_refusals);
    bench.letter('e', "the address scan: 112 addresses, 112 NACKs", te_scan);
    bench.letter('f', "the speeds, with SCL counted on the pad", tf_speeds);
    bench.letter('g', "the SMBus time-out unit on an idle bus", tg_timeouts);
    bench.letter('h', "the filters and the Fm+ pad drive", th_filters);
    bench.letter('i', "THE KERNEL: I2cBus over FmpI2cHost", ti_kernel);
    bench.letter('j', "the DMA engines around a tenure that NACKs", tj_engines);
    bench.letter('k', "the recovery verbs", tk_recovery);
    bench.letter('l', "the client, configured and never addressed", tl_client);
    bench.letter('m', "THE SELF-LINK: FMPI2C1 the host, I2C1 the target", tm_self_link);
    bench.second_letter('n', "THE DESK PARTNER: Fm+ acknowledged, a held SDA, a held clock",
                        tn_partner);
    bench.second_letter('o', "ES0298 2.12.8: writes in and out of the clock-ratio band",
                        to_erratum);
    bench.second_letter('p', "A REFUSED WRITE, position by position, against the partner",
                        tp_refusals);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED",
                    " fmpi2c1=", host_ok ? "host" : "FAILED", " wire=",
                    boot.scl && boot.sda ? "idle high" : "PULLED LOW", brio::crlf);
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
        brio::print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
