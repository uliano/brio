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
// THERE IS NO DEVICE ON THIS BUS, AND NO BYTE WAS EVER ACKNOWLEDGED HERE.
// The block exists on one board of this stratum's three (the FMPI2C is the
// F410's, F412's, F413/F423's and F446's), its two pads there are free
// pins with nothing on them and no pull-ups of their own, and there is no
// second device to wire. So this suite measures WHAT A BUS WITH NOBODY ON
// IT SHOWS, which is more than it sounds:
//
//   FMPI2C1   SCL PC6, SDA PC7 (AF4), pulled up by the PORT - some tens
//             of kiloohms, which is far too weak for a real I2C edge and
//             exactly enough to let the block drive, release and read its
//             own wire
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
// What is NOT here is the whole data path, and docs/stm32f4/fmpi2c.md says
// so: a device on the bus is what would measure it.
//
// What is exercised, letter by letter:
//   a  the block: presence, the gate, the reset values, the two vectors
//   b  the timing arithmetic and what it puts in TIMINGR
//   c  the kernel-clock multiplexer, and what each source can produce
//   d  what the driver refuses, and that a refusal writes nothing
//   e  the address scan: 112 addresses, 112 NACKs, and the STOP after one
//   f  the speeds, with SCL counted on the pad
//   g  the SMBus time-out unit on an idle bus
//   h  the noise filters and the Fm+ pad drive
//   i  THE KERNEL: I2cBus over FmpI2cHost
//   j  the DMA engines around a tenure that NACKs
//   k  the recovery verbs
//   l  the client, configured and never addressed
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/fmpi2c.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/syscfg.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

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

TestBench<Serial> bench;

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

/// The two pads, read as inputs: what the wire is doing.
using SclPad = Pin<'C', 6>;
using SdaPad = Pin<'C', 7>;

/// The board's own bus has no pull-ups, so the port's are asked for - and
/// they are the reason the numbers below are what they are.
constexpr FmpI2cHostConfig base_config{.kernel = FmpI2cClock::pclk,
                                       .filters = {},
                                       .internal_pull_up = true,
                                       .bus = {}};

volatile bool bus_ao_live = false;
volatile bool dma_host_live = false;
volatile bool peer_live = false;
volatile bool xfer_done = false;

/// What the EVENT vector saw, in order, for the tenure being watched: the
/// ISR's own reading of ISR before the engine touches it. Four entries is
/// more than a probe can raise.
volatile uint32_t ev_log[4] = {};
volatile uint8_t ev_n = 0;
volatile uint8_t er_n = 0;

void arm_log() {
    ev_n = 0;
    er_n = 0;
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
template <typename Bus>
uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
               FmpI2cSpeed speed = FmpI2cSpeed::standard_100k) {
    typename Bus::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
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
struct SclReading {
    uint32_t edges = 0;
    uint32_t shortest = 0;   ///< the shortest gap between two rising edges, in core cycles
    uint32_t average = 0;    ///< the mean gap over the whole tenure, in core cycles
};

SclReading count_scl() {
    SclReading r{};
    const uint32_t period = systick_period();
    uint32_t last_val = SysTick->VAL;
    uint32_t shortest = 0xFFFFFFFFu;
    uint32_t total = 0;
    bool prev = SclPad::read();
    for (uint32_t spins = 40'000'000u; spins != 0u && !xfer_done; --spins) {
        const uint32_t v = SysTick->VAL;
        const bool now = SclPad::read();
        if (now && !prev) {
            const uint32_t d = since(last_val, v, period);
            if (r.edges != 0u) {
                total += d;
                if (d < shortest) {
                    shortest = d;
                }
            }
            last_val = v;
            ++r.edges;
        }
        prev = now;
    }
    r.shortest = shortest == 0xFFFFFFFFu ? 0u : shortest;
    r.average = r.edges > 1u ? total / (r.edges - 1u) : 0u;
    return r;
}

SclReading timed_probe(uint8_t addr, FmpI2cSpeed s) {
    Host::Request r{};
    r.addr = addr;
    r.speed = s;
    xfer_done = false;
    if (Host::start(r)) {
        return SclReading{};
    }
    return count_scl();
}

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
    uint32_t sm_pclk = 0;
    uint32_t fm_pclk = 0;
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
                print(serial, "  ", rows[k].name, " ",
                      i == 0 ? "Sm  " : (i == 1 ? "Fm  " : "Fm+ "), ": refused at this kernel "
                      "clock (ES0298 2.12.2)", crlf);
                continue;
            }
            const SclReading r = timed_probe(0x42, s);
            const uint8_t st = Host::status();
            const uint32_t high = r.shortest == 0u ? 0u : SysClock::hz / r.shortest;
            const uint32_t low = r.average == 0u ? 0u : SysClock::hz / r.average;
            // WHAT THE WIRE SAYS THE DETECTION DELAY IS. 23.4.9 makes
            // tSCL = tSYNC1 + tSYNC2 + tSCLL + tSCLH, and the two
            // programmed halves are known exactly - so the measured period
            // less them is the SCL detection delay this board really pays,
            // against the 1000 / 750 / 500 ns the manual's tables assume.
            const FmpI2cTiming t = Host::timing_of(s);
            const uint32_t programmed_ns =
                fmpi2c_scll_ns(ker, t) + fmpi2c_sclh_ns(ker, t);
            const uint32_t measured_ns =
                r.shortest == 0u ? 0u
                                 : static_cast<uint32_t>((static_cast<uint64_t>(r.shortest) *
                                                          1000ULL) /
                                                         (SysClock::hz / 1'000'000u));
            const bool longer = measured_ns >= programmed_ns;
            print(serial, "  ", rows[k].name, " ", i == 0 ? "Sm  " : (i == 1 ? "Fm  " : "Fm+ "),
                  ": the arithmetic states ", Host::scl_hz(s), " Hz; the pad gave ", r.edges,
                  " rising edges, the shortest gap ", r.shortest, " core cycles (", high,
                  " Hz) and the mean ", r.average, " (", low, " Hz); tSCLL + tSCLH is ",
                  programmed_ns, " ns of that ", measured_ns, " ns, so the detection delay is ",
                  longer ? measured_ns - programmed_ns : 0u, " ns against the ",
                  fmpi2c_bus_timing(s).sync_ns, " ns the table assumes; status ", st, crlf);
            if (st != i2c_nack_addr) {
                all_nacked = false;
            }
            if (!longer) {
                never_shorter = false;
            }
            if (rows[k].kernel == FmpI2cClock::pclk && i == 0u) {
                sm_pclk = high;
            }
            if (rows[k].kernel == FmpI2cClock::pclk && i == 1u) {
                fm_pclk = high;
            }
        }
        Host::fast_plus_drive(false);
    }
    bench.verdict("every one of the eight tenures reached the wire and came back with the "
                  "address unanswered", all_nacked);
    bench.verdict("SCL really moves, and faster in fast mode than in standard: the pad's own "
                  "input buffer counted both",
                  sm_pclk != 0u && fm_pclk > sm_pclk);
    bench.verdict("and every period on the wire is LONGER than the two halves TIMINGR programs, "
                  "which is 23.4.9's formula with a detection delay that cannot be negative",
                  never_shorter);
    print(serial, "  (nine of those rising edges are the address phase - eight bits and the "
                  "acknowledge slot nobody filled - and the tenth, where the loop is still "
                  "watching, is SCL going up for the STOP. The rates are PRINTED and not judged: "
                  "this bus is pulled up by the port alone, some tens of kiloohms, so the rises "
                  "are slow exponentials and one board is one specimen. What the detection delay "
                  "column shows is the honest part - the manual's assumed tSYNC is generous "
                  "here, so a bus solved against it runs a few per cent FASTER than the speed "
                  "asked for, and an application that must not pass the standard's ceiling "
                  "states its own measured sync_ns to init())",
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

using I2cArb = I2cBus<Host, P, 4>;

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
    // is started before the START bit (23.4.16's own order, and ES0298
    // 2.12.8's first workaround), and the address is NACKed before a byte
    // leaves TXDR.
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
    if (dma_host_live && DmaHost::dma_isr()) {
        xfer_done = true;
    }
}
extern "C" void DMA1_Stream2_IRQHandler() {
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
