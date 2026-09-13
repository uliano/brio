// test_stm32f4_i2c - the reference bench suite for the STM32F4's I2C: the
// resource over the whole of the chapter, the I2cHost engine under
// util/i2c_bus.hpp's arbiter, the DMA engines and the recovery verbs -
// stm32f4/i2c.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS A DEVICE SOLDERED TO THE BOARD, which is why this
// suite builds for the STM32F429I-DISC1 alone: a bus driver measured
// against nothing is a register read-back, and this board carries its
// touch-screen controller on I2C3 -
//
//   I2C3   SCL PA8, SDA PC9 (AF4), the pull-ups the board's own
//   the touch controller   an STMPE811 at the 7-bit address 0x41, whose
//                          CHIP_ID at register 0x00 reads 0x0811 as two
//                          bytes MSB first and whose ID_VER at 0x02 reads
//                          a revision byte; SYS_CTRL2 at 0x04 is four
//                          clock gates, written and read back here, and
//                          SYS_CTRL1's soft reset puts every one of its
//                          registers back
//
// and the parts of the chapter that need no device are measured anyway:
// the timing arithmetic against the registers in force, the address scan
// against every address nobody answers, and the SCL frequency counted on
// the clock pad's OWN INPUT BUFFER while a DMA-carried read runs (an I2C
// pad is an open-drain alternate function, so its IDR is the wire).
//
// The touch controller is left in ITS RESET STATE (the soft reset of
// letter h is the last thing written to it).
//
// What is exercised, letter by letter:
//   a  the instances: presence, bus, the two vectors, the gate closed at
//      reset and the reset values behind it
//   b  the timing arithmetic and what it puts in FREQ, CCR and TRISE
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the address scan: who answers, and that everybody else NACKs
//   e  the device's identity through a write-then-read tenure
//   f  the three receive choreographies - one, two and N bytes
//   g  the speeds and both fast-mode duties, with SCL counted on the pad
//   h  a register written and read back, then the device's own reset
//   i  THE KERNEL: I2cBus over I2cHost, the rejection, the sleep votes
//   j  the same tenures through the two DMA engines
//   k  the recovery verbs: SWRST, recover() and unstick()
//
// build: boards = f429zi
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/i2c.hpp"
#include "stm32f4/nvic.hpp"
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

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the console ---------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
using Serial = Uart<1, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the bus and the device -------------------------------------------------------------

constexpr I2cPins i2c3_pins{.scl = {'A', 8, PinFunction::af4}, .sda = {'C', 9, PinFunction::af4}};

using S = I2c<3>;
using Host = I2cHost<3, i2c3_pins>;

/// I2C3's transmit request is DMA1 stream 4 on channel 3 and its receive
/// DMA1 stream 2 on channel 3 (RM0090 table 43) - the reserve checks these
/// two cells at compile time.
using TxEngine = DmaTxEngine<1, 4, 3>;
using RxEngine = DmaRxEngine<1, 2, 3>;
using DmaHost = I2cHost<3, i2c3_pins, TxEngine, RxEngine>;

/// The two pads, read as inputs: what the wire is doing. An I2C pad is an
/// open-drain alternate function, so its input buffer reads the LINE even
/// while the peripheral owns it.
using SclPad = Pin<'A', 8>;
using SdaPad = Pin<'C', 9>;

/// The touch controller's address and the registers this suite touches.
constexpr uint8_t touch_addr = 0x41;
constexpr uint8_t reg_chip_id = 0x00;
constexpr uint8_t reg_id_ver = 0x02;
constexpr uint8_t reg_sys_ctrl1 = 0x03;
constexpr uint8_t reg_sys_ctrl2 = 0x04;
constexpr uint16_t stmpe811_chip_id = 0x0811;
constexpr uint8_t stmpe811_soft_reset = 0x02;

/// What the boot-time scan found: which address answered, and whether the
/// part behind it is the STMPE811 this board's schematic names.
uint8_t peer_addr = 0;
bool peer_is_stmpe = false;
uint8_t peer_sys_ctrl2_boot = 0;

volatile bool bus_ao_live = false;
volatile bool dma_host_live = false;
volatile bool xfer_done = false;

// ---- the rulers ------------------------------------------------------------------------

/// SysTick's reload period in core cycles - what a VAL delta wraps by.
/// VAL is the one ruler fine enough for a bit period, and reading it has
/// no side effect (cortexm/delay.hpp says why).
uint32_t systick_period() { return SysTick->LOAD + 1u; }

bool same(const uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// ---- the bus, in the state the letters expect ----------------------------------------------

/// Bring I2C3 back up as the host every letter starts from.
bool host_ready(I2cDuty duty = I2cDuty::ratio_2) {
    bus_ao_live = false;
    dma_host_live = false;
    return Host::init(clock, I2cHostConfig{.duty = duty});
}

/// What this suite reports for a tenure that never answered - the code an
/// arbiter with a timeout would replace with i2c_timeout. It must never be
/// mistaken for the i2c_ok start() left standing.
constexpr uint8_t parked = 0xFF;

/// One tenure against `addr`, driven by hand (the arbiter skipped, as
/// i2c-bus.md allows a device that owns a bus alone). Returns the status
/// the engine ended with, or `parked`; the wait is bounded.
template <typename Bus>
uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
               I2cSpeed speed = I2cSpeed::standard_100k) {
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

/// A register read as one write-then-read tenure: the index out, a
/// repeated START, the value back.
template <typename Bus = Host>
uint8_t read_regs(uint8_t reg, uint8_t* into, uint8_t n,
                  I2cSpeed speed = I2cSpeed::standard_100k) {
    const uint8_t cmd[1] = {reg};
    return tenure<Bus>(peer_addr, cmd, 1, into, n, speed);
}

uint8_t read_reg(uint8_t reg, I2cSpeed speed = I2cSpeed::standard_100k) {
    uint8_t v = 0;
    (void)read_regs(reg, &v, 1, speed);
    return v;
}

uint8_t write_reg(uint8_t reg, uint8_t value) {
    const uint8_t cmd[2] = {reg, value};
    return tenure<Host>(peer_addr, cmd, 2, nullptr, 0);
}

/// The empty Request: START, the address, STOP - the ACK is the answer.
uint8_t probe(uint8_t addr) { return tenure<Host>(addr, nullptr, 0, nullptr, 0); }

// ---- what the registers held before this program touched them --------------------------------

struct BootState {
    bool gate = false;
    uint32_t cr1 = 0, cr2 = 0, oar1 = 0, oar2 = 0, sr1 = 0, sr2 = 0, ccr = 0, trise = 0;
    bool scl = false, sda = false;   ///< the two lines, read as plain inputs
    uint32_t sr2_ready = 0;          ///< SR2 once the pads are the peripheral's
};
BootState boot;

// =============================================================================
// a - the instances and the block
// =============================================================================

void ta_block() {
    print(serial, "  I2C3 gate at reset: ", boot.gate ? "OPEN" : "closed", "; CR1 ", hex(boot.cr1),
          " CR2 ", hex(boot.cr2), " OAR1 ", hex(boot.oar1), " OAR2 ", hex(boot.oar2), " SR1 ",
          hex(boot.sr1), " SR2 ", hex(boot.sr2), " CCR ", hex(boot.ccr), " TRISE ",
          hex(boot.trise), crlf);
    bench.verdict("every peripheral clock is off at reset, this one included", !boot.gate);
    // Table 125: every register zero but TRISE, which resets to 0x0002.
    // SR2's BUSY is not a stored bit at all - "set by hardware on detection
    // of SDA or SCL low", and "still updated when the interface is
    // disabled" (27.6.7) - so it is read as the WIRE and judged below.
    bench.verdict("the reset values are the chapter's: every register zero but TRISE, which is "
                  "0x0002 (SR2's BUSY excluded: it is a live reading of the wire and not a "
                  "stored bit)",
                  boot.cr1 == 0u && boot.cr2 == 0u && boot.oar1 == 0u && boot.oar2 == 0u &&
                      boot.sr1 == 0u && (boot.sr2 & ~I2C_SR2_BUSY) == 0u && boot.ccr == 0u &&
                      boot.trise == 2u);
    print(serial, "  the wire at boot, read as plain inputs: SCL ", boot.scl ? "high" : "LOW",
          ", SDA ", boot.sda ? "high" : "LOW", "; SR2.BUSY with the pads NOT yet handed over: ",
          (boot.sr2 & I2C_SR2_BUSY) != 0u ? "SET" : "clear", "; after init(): ",
          (boot.sr2_ready & I2C_SR2_BUSY) != 0u ? "SET" : "clear", crlf);
    // The trap this chapter sets for a bring-up, measured: BUSY watches the
    // PERIPHERAL'S line inputs, and those read low while the pad is not in
    // its alternate function - so a block configured before its pads comes
    // up BUSY over an idle wire and its first START never leaves, BUSY
    // being cleared only by a STOP that will never come. init() therefore
    // hands the pads over BEFORE the software reset that takes BUSY with
    // them.
    bench.verdict("with the pads not yet in their alternate function the peripheral reads its own "
                  "inputs LOW and BUSY stands, over a wire both pads read HIGH",
                  boot.scl && boot.sda && (boot.sr2 & I2C_SR2_BUSY) != 0u);
    bench.verdict("... and once init() has handed the pads over and pulsed SWRST behind them, "
                  "BUSY reads the wire and is clear",
                  (boot.sr2_ready & I2C_SR2_BUSY) == 0u);

    print(serial, "  instances on this part:");
    uint8_t count = 0;
    for (uint8_t i = 1; i <= 3u; ++i) {
        if (i2c_present(i)) {
            ++count;
            print(serial, " I2C", i, "(", i2c_on_apb2(i) ? "APB2" : "APB1", ",ev",
                  static_cast<int32_t>(i2c_event_irq(i)), ",er",
                  static_cast<int32_t>(i2c_error_irq(i)), ")");
        }
    }
    print(serial, crlf);
    bench.verdict("this part carries three I2C instances, all on APB1", count == 3u &&
                                                                            !i2c_on_apb2(1) &&
                                                                            !i2c_on_apb2(3));
    bench.verdict("each has TWO vectors of its own, the events on one and the errors on the other",
                  S::event_irq == I2C3_EV_IRQn && S::error_irq == I2C3_ER_IRQn &&
                      i2c_event_irq(1) != i2c_error_irq(1));
    print(serial, "  the noise filter register: ", S::has_filter ? "present" : "absent",
          "; an FMPI2C1 on this part: ", fmpi2c_present() ? "yes" : "no", crlf);
    bench.verdict("this part class has I2C_FLTR - the analog filter's off switch and the digital "
                  "filter - and no FMPI2C1, which is another block and another chapter",
                  S::has_filter && !fmpi2c_present());

    print(serial, "  APB1 = ", Host::reference_hz() / 1000u, " kHz, PCLK2 = ",
          SysClock::pclk2_hz / 1000u, " kHz, HCLK = ", SysClock::hz / 1000u, " kHz", crlf);
    bench.verdict("the rate FREQ states and CCR divides is ITS OWN bus clock, PCLK1, and not "
                  "SYSCLK",
                  Host::reference_hz() == SysClock::pclk1_hz &&
                      Host::reference_hz() != SysClock::hz);
}

// =============================================================================
// b - the timing arithmetic
// =============================================================================

void tb_timing() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint32_t pclk = Host::reference_hz();
    bool all_stated = true;
    // The duty shape is fast mode's alone, so the standard row is printed
    // once and the fast one under each of its two shapes.
    for (uint8_t duty = 0; duty < 2u; ++duty) {
        if (!host_ready(static_cast<I2cDuty>(duty))) {
            all_stated = false;
            continue;
        }
        for (uint8_t i = 0; i < i2c_speed_count; ++i) {
            const I2cSpeed sp = static_cast<I2cSpeed>(i);
            if (sp == I2cSpeed::standard_100k && duty != 0u) {
                continue;
            }
            const I2cTiming t = Host::timing_of(sp);
            print(serial, "  ",
                  sp == I2cSpeed::standard_100k ? "Sm 100k       "
                                                : (duty == 0u ? "Fm 400k duty 2" : "Fm 400k 16/9  "),
                  ": FREQ ", t.freq_mhz, " CCR ", hex(t.ccr), " TRISE ", t.trise, " -> ",
                  Host::scl_hz(sp), " Hz", crlf);
            if (t.freq_mhz != pclk / 1'000'000u) {
                all_stated = false;
            }
        }
    }
    bench.verdict("FREQ carries the APB1 clock in whole megahertz on every row of the table",
                  all_stated);
    (void)host_ready();

    // What the registers really hold once a speed is applied: the driver
    // writes the row and nothing else.
    const I2cTiming want = Host::timing_of(I2cSpeed::standard_100k);
    const I2cTiming got = S::timing();
    print(serial, "  applied: FREQ ", got.freq_mhz, " CCR ", hex(got.ccr), " TRISE ", got.trise,
          "; the table says FREQ ", want.freq_mhz, " CCR ", hex(want.ccr), " TRISE ", want.trise,
          crlf);
    bench.verdict("the three timing registers in force are exactly the row the arithmetic solved",
                  got.freq_mhz == want.freq_mhz && got.ccr == want.ccr && got.trise == want.trise);
    bench.verdict("... and at 45 MHz of APB1 standard mode is EXACT: CCR 225 gives 100000 Hz",
                  want.ccr == 225u && Host::scl_hz(I2cSpeed::standard_100k) == 100'000UL);
    bench.verdict("TRISE is the specification's 1000 ns in whole APB periods plus one - 46 here",
                  want.trise == 46u);

    // Fast mode at this clock is NOT exact, and the rounding is downwards
    // in frequency: 45 MHz / 1.2 MHz is 37.5, so CCR is 38 and the bus
    // runs slower than asked, never faster.
    const uint32_t fast = Host::scl_hz(I2cSpeed::fast_400k);
    print(serial, "  fast mode at 45 MHz: CCR ", Host::timing_of(I2cSpeed::fast_400k).ccr & 0xFFFu,
          " -> ", fast, " Hz (400000 would want a multiple of 1.2 MHz on APB1)", crlf);
    bench.verdict("CCR is rounded UP, so a bus never runs faster than the speed asked for",
                  fast < 400'000UL && fast > 390'000UL);

    // The window FREQ imposes, and the refusals outside it.
    bench.verdict("below 2 MHz of APB1 no speed is legal (27.6.2)",
                  !i2c_timing_for(1'999'999UL, I2cSpeed::standard_100k).has_value());
    bench.verdict("... and fast mode wants 4 MHz, where standard mode is already legal",
                  !i2c_timing_for(2'000'000UL, I2cSpeed::fast_400k).has_value() &&
                      i2c_timing_for(2'000'000UL, I2cSpeed::standard_100k).has_value());
    bench.verdict("above 50 MHz the FREQ field has no encoding, and the row is refused",
                  !i2c_timing_for(50'000'001UL, I2cSpeed::standard_100k).has_value());
    bench.verdict("a rise time TRISE's six bits cannot hold is refused rather than truncated",
                  !i2c_timing_for(45'000'000UL, I2cSpeed::standard_100k, I2cDuty::ratio_2, 1500)
                       .has_value());

    print(serial, "  table 122's ceiling on the digital filter at this clock: ",
          Host::digital_filter_max(I2cSpeed::standard_100k), " in Sm, ",
          Host::digital_filter_max(I2cSpeed::fast_400k), " in Fm; ES0206 2.10.4 puts a repeated "
          "START at ", Host::scl_hz(I2cSpeed::standard_100k), " Hz ",
          Host::repeated_start_setup_at_risk(I2cSpeed::standard_100k) ? "IN" : "outside",
          " the window where its setup time can be violated", crlf);
    bench.verdict("a 100 kHz standard-mode bus sits inside that errata window, and the driver "
                  "says so instead of promising the timing",
                  Host::repeated_start_setup_at_risk(I2cSpeed::standard_100k) &&
                      !Host::repeated_start_setup_at_risk(I2cSpeed::fast_400k));
}

// =============================================================================
// c - the refusals
// =============================================================================

void tc_refusals() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // An own address outside its field: refused whole, nothing written.
    (void)S::addresses(I2cAddressConfig{.own = 0x41});
    const uint32_t before = S::regs().OAR1;
    bench.verdict("an own address past seven bits is refused",
                  !S::addresses(I2cAddressConfig{.own = 0x80}));
    bench.verdict("... and a refused address configuration writes nothing",
                  S::regs().OAR1 == before);
    bench.verdict("a ten-bit own address WITH a second one is refused - the dual arrangement is "
                  "seven-bit only (27.6.4)",
                  !S::addresses(I2cAddressConfig{.own = 0x123, .ten_bit = true, .second = 0x10}));
    bench.verdict("the same ten-bit address alone is accepted",
                  S::addresses(I2cAddressConfig{.own = 0x123, .ten_bit = true}));
    // 27.6.3: bit 14 of OAR1 "should always be kept at 1 by software".
    print(serial, "  OAR1 after a ten-bit address: ", hex(S::regs().OAR1), crlf);
    bench.verdict("bit 14 of OAR1 is kept at one by every write, as 27.6.3 asks",
                  (S::regs().OAR1 & (1u << 14)) != 0u);

    // The filter is enable-protected and its length is four bits.
    S::enable();
    bench.verdict("the noise filters are refused while the peripheral is enabled",
                  !S::filter(I2cFilter{.analog = false, .digital = 1}));
    S::disable();
    bench.verdict("with PE down they are accepted", S::filter(I2cFilter{.analog = false,
                                                                       .digital = 4}));
    const I2cFilter back = S::filter();
    print(serial, "  FLTR reads analog ", back.analog ? "on" : "OFF", ", digital ", back.digital,
          crlf);
    bench.verdict("... and read back as they were written", !back.analog && back.digital == 4u);
    bench.verdict("a digital filter past the four bits of DNF is refused",
                  !S::filter(I2cFilter{.digital = 16}));

    // The engine's one synchronous refusal: a speed the clock cannot make.
    (void)host_ready();
    bench.verdict("both speeds are reachable at 45 MHz of APB1",
                  Host::speed_ok(I2cSpeed::standard_100k) && Host::speed_ok(I2cSpeed::fast_400k));
    print(serial, "  (a speed the clock in force cannot produce is answered i2c_rejected inside "
                  "start(), no byte moved - and at this clock there is no such speed to show)",
          crlf);
}

// =============================================================================
// d - the address scan
// =============================================================================

void td_scan() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    uint8_t answered = 0;
    uint8_t nacked = 0;
    uint8_t other = 0;
    uint8_t first = 0;
    print(serial, "  answering addresses:");
    for (uint8_t a = 0x08; a <= 0x77u; ++a) {
        const uint8_t st = probe(a);
        if (st == i2c_ok) {
            ++answered;
            if (first == 0u) {
                first = a;
            }
            print(serial, " ", hex(a));
        } else if (st == i2c_nack_addr) {
            ++nacked;
        } else {
            ++other;
        }
    }
    print(serial, answered == 0u ? " (none)" : "", crlf);
    print(serial, "  ", answered, " answered, ", nacked, " NACKed, ", other,
          " ended some other way; the STMPE811 this board's schematic names is at ",
          hex(touch_addr), crlf);
    bench.verdict("the probe - a Request with both spans empty - reaches the wire: at least one "
                  "device on this board answers its address",
                  answered != 0u);
    bench.verdict("every address nobody answers comes back i2c_nack_addr, and nothing comes back "
                  "any other way",
                  other == 0u && nacked == (0x77u - 0x08u + 1u) - answered);
    bench.verdict("the touch controller is one of them", peer_addr != 0u);
    print(serial, "  a second scan repeats it: ", probe(peer_addr) == i2c_ok ? "yes" : "NO",
          "; an address next to it still NACKs: ",
          probe(static_cast<uint8_t>(peer_addr + 1u)) == i2c_nack_addr ? "yes" : "NO", crlf);
    bench.verdict("the scan is repeatable, and a NACK on the address leaves the bus fit for the "
                  "next tenure",
                  probe(peer_addr) == i2c_ok);
}

// =============================================================================
// e - the identity
// =============================================================================

void te_identity() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    uint8_t id[2] = {};
    const uint8_t st = read_regs(reg_chip_id, id, 2);
    const uint16_t chip = static_cast<uint16_t>((static_cast<uint16_t>(id[0]) << 8) | id[1]);
    const uint8_t ver = read_reg(reg_id_ver);
    print(serial, "  address ", hex(peer_addr), ": CHIP_ID (0x00, two bytes MSB first) reads ",
          hex(chip), ", ID_VER (0x02) reads ", hex(ver), "; status ", st, crlf);
    bench.verdict("a write-then-read tenure - the index out, a repeated START, the value back - "
                  "answers i2c_ok",
                  st == i2c_ok);
    bench.verdict("the device on this bus is the STMPE811 the board's schematic names, and it "
                  "says so in its own identity register",
                  chip == stmpe811_chip_id);

    // The same read eight times running: a bus that is right is right
    // every time.
    bool stable = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        uint8_t again[2] = {};
        (void)read_regs(reg_chip_id, again, 2);
        if (!same(id, again, 2)) {
            stable = false;
        }
    }
    bench.verdict("eight write-then-read tenures in a row give the same two bytes", stable);

    // A register that is not the identity reads as something else, which
    // is what says the bytes above are the device's and not the wire's.
    const uint8_t ctrl = read_reg(reg_sys_ctrl2);
    print(serial, "  SYS_CTRL2 (0x04) reads ", hex(ctrl), " - a different register, a different "
          "byte", crlf);
    bench.verdict("a different register answers differently: the bytes come from the device and "
                  "not from a pull-up",
                  ctrl != id[0] || ctrl != id[1]);
}

// =============================================================================
// f - the three receive choreographies
// =============================================================================

void tf_receive_shapes() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    // 27.3.3 gives the controller receiver THREE different procedures by
    // count, and this is where they are proved to agree: the same
    // registers read one byte at a time, two at a time and eight at a
    // time must come back identical.
    uint8_t one[8] = {};
    for (uint8_t i = 0; i < 8u; ++i) {
        one[i] = read_reg(static_cast<uint8_t>(reg_chip_id + i));
    }
    uint8_t two[8] = {};
    for (uint8_t i = 0; i < 8u; i += 2u) {
        (void)read_regs(static_cast<uint8_t>(reg_chip_id + i), &two[i], 2);
    }
    uint8_t eight[8] = {};
    const uint8_t st = read_regs(reg_chip_id, eight, 8);

    print(serial, "  one at a time:", crlf, "   ");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", hex(one[i]));
    }
    print(serial, crlf, "  two at a time (POS and the single BTF):", crlf, "   ");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", hex(two[i]));
    }
    print(serial, crlf, "  eight in one tenure (the BTF pair at the tail):", crlf, "   ");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", hex(eight[i]));
    }
    print(serial, crlf);

    bench.verdict("the ONE-byte procedure - ACK cleared before ADDR, STOP right after - reads the "
                  "device",
                  one[0] == 0x08u && one[1] == 0x11u);
    bench.verdict("the TWO-byte procedure - POS with ACK, ACK off after ADDR, both bytes off one "
                  "BTF - agrees with it byte for byte",
                  same(one, two, 8));
    bench.verdict("the N-byte procedure - RxNE until three remain, then two BTFs - agrees with "
                  "both",
                  st == i2c_ok && same(one, eight, 8));

    // The device auto-increments its own register pointer, so a long read
    // is one tenure and not eight: the count is what changes, and the
    // three answers above are what says the choreography is right.
    uint8_t run[16] = {};
    const uint8_t long_st = read_regs(reg_chip_id, run, 16);
    bench.verdict("a sixteen-byte read is one tenure and ends i2c_ok",
                  long_st == i2c_ok && run[0] == 0x08u && run[1] == 0x11u);
}

// =============================================================================
// g - the speeds, with SCL counted on the pad
// =============================================================================

/// Count SCL's rising edges while a tenure runs, and keep the SHORTEST
/// period between two of them - which is the bit period, every software
/// sequence of the chapter only ever lengthening one. The pad is an
/// open-drain alternate function, so its input buffer reads the wire.
struct SclReading {
    uint32_t edges = 0;
    uint32_t shortest = 0;   ///< the shortest gap between two edges, in core cycles
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
            const uint32_t d = (last_val >= v) ? (last_val - v) : (last_val + period - v);
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

/// A DMA-carried read of `n` bytes, with SCL watched from the loop: the
/// engines keep the CPU out of the byte pump, so what the pad shows is the
/// wire and not the software.
SclReading timed_read(uint8_t n, I2cSpeed speed, uint8_t* into) {
    const uint8_t cmd[1] = {reg_chip_id};
    DmaHost::Request r{};
    r.addr = peer_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(into);
    r.rx_len = n;
    r.speed = speed;
    xfer_done = false;
    if (DmaHost::start(r)) {
        return SclReading{};
    }
    return count_scl();
}

void tg_speeds() {
    uint8_t got[32] = {};
    uint8_t reference[32] = {};
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    (void)read_regs(reg_chip_id, reference, 32);

    Dma<1>::init();
    struct Row {
        I2cSpeed speed;
        I2cDuty duty;
        const char* name;
    };
    const Row rows[3] = {{I2cSpeed::standard_100k, I2cDuty::ratio_2, "Sm 100k       "},
                         {I2cSpeed::fast_400k, I2cDuty::ratio_2, "Fm 400k duty 2"},
                         {I2cSpeed::fast_400k, I2cDuty::ratio_16_9, "Fm 400k 16/9  "}};
    uint8_t exact = 0;
    bool all_ok = true;
    uint32_t fastest[3] = {};
    for (uint8_t i = 0; i < 3u; ++i) {
        bus_ao_live = false;
        if (!DmaHost::init(clock, I2cHostConfig{.duty = rows[i].duty})) {
            all_ok = false;
            continue;
        }
        dma_host_live = true;
        for (uint8_t k = 0; k < 32u; ++k) {
            got[k] = 0;
        }
        const SclReading r = timed_read(32, rows[i].speed, got);
        const uint8_t st = DmaHost::status();
        dma_host_live = false;
        const uint32_t stated = DmaHost::scl_hz(rows[i].speed);
        const uint32_t high = r.shortest == 0u ? 0u : SysClock::hz / r.shortest;
        const uint32_t low = r.average == 0u ? 0u : SysClock::hz / r.average;
        fastest[i] = high;
        // The address with the write bit, the index, the address again
        // with the read bit and the 32 bytes: nine clocks a byte,
        // thirty-five bytes.
        const uint32_t expected_edges = 9u * (3u + 32u);
        if (st == i2c_ok && same(reference, got, 32)) {
            ++exact;
        }
        print(serial, "  ", rows[i].name, ": CCR ", DmaHost::timing_of(rows[i].speed).ccr & 0xFFFu,
              " states ", stated, " Hz; the pad gave ", r.edges, " rising edges of ",
              expected_edges, ", the shortest gap ", r.shortest, " core cycles (", high,
              " Hz) and the mean ", r.average, " (", low, " Hz); status ", st, crlf);
        if (st != i2c_ok) {
            all_ok = false;
        }
    }
    DmaHost::release();
    (void)host_ready();

    bench.verdict("the device answers at both speeds and under both fast-mode duty shapes, and "
                  "the bytes are the same at each",
                  exact == 3u);
    bench.verdict("every one of the three tenures ended i2c_ok", all_ok);
    bench.verdict("the three rows are three different rates ON THE WIRE, in the order the CCR "
                  "arithmetic puts them: fast mode at DUTY 2 above fast mode at 16/9, and both "
                  "well above standard mode",
                  fastest[1] > fastest[2] && fastest[2] > fastest[0]);
    print(serial, "  (the count is nine clocks for each of the tenure's thirty-five bytes, plus "
                  "the STOP's own rise where the loop is still watching when it comes. The two "
                  "rates BRACKET the bit rate: the mean gap counts the two address phases, whose "
                  "software sequences stretch SCL low, so it reads slow; the shortest gap is one "
                  "bit period sampled by a polling loop that timestamps a little before the edge "
                  "it sees, so it reads fast. What the CCR arithmetic states sits between them, "
                  "and the numbers are printed rather than judged - one board, one set of "
                  "pull-ups)",
          crlf);
}

// =============================================================================
// h - a register written and read back, then the device's own reset
// =============================================================================

void th_write_back() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    // SYS_CTRL2 is four clock gates and nothing that moves a pin.
    const uint8_t start = read_reg(reg_sys_ctrl2);
    const uint8_t patterns[3] = {0x00u, 0x0Fu, 0x01u};
    bool wrote = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        const uint8_t st = write_reg(reg_sys_ctrl2, patterns[i]);
        const uint8_t back = read_reg(reg_sys_ctrl2);
        print(serial, "  SYS_CTRL2 <- ", hex(patterns[i]), " (status ", st, "), reads ", hex(back),
              crlf);
        if (st != i2c_ok || back != patterns[i]) {
            wrote = false;
        }
    }
    bench.verdict("a register written over this bus reads back exactly - the write path and the "
                  "read path are the same tenure with a repeated START between them",
                  wrote);
    // And one pattern the DEVICE masks: with its ADC gate open this part
    // holds the temperature-sensor gate clear whatever is written. A
    // device rule and not a bus one, so it is printed and not judged -
    // what the bus owes is that the byte written is the byte that
    // arrived, which the three above say.
    const uint8_t masked_st = write_reg(reg_sys_ctrl2, 0x0Cu);
    print(serial, "  SYS_CTRL2 <- 0xC (status ", masked_st, "), reads ", hex(read_reg(reg_sys_ctrl2)),
          " - the device masks a bit of its own, and that is the device's business", crlf);

    // A two-byte write in one tenure: the index and the value are one
    // span, so this is already what every write above did; what it proves
    // here is that the tenure ends on BTF and not on TxE.
    const uint8_t st = write_reg(reg_sys_ctrl2, start);
    print(serial, "  SYS_CTRL2 back to the value found at boot, ", hex(start), " (status ", st,
          ")", crlf);
    bench.verdict("the boot value is restored", st == i2c_ok && read_reg(reg_sys_ctrl2) == start);

    // And the device's own soft reset, which is how this suite leaves it:
    // every register of its back where its power-on put them.
    const uint8_t reset_st = write_reg(reg_sys_ctrl1, stmpe811_soft_reset);
    for (uint32_t spins = 400'000u; spins != 0u; --spins) {
    }
    uint8_t id[2] = {};
    (void)read_regs(reg_chip_id, id, 2);
    const uint8_t after = read_reg(reg_sys_ctrl2);
    print(serial, "  after SYS_CTRL1's soft reset: CHIP_ID ", hex(id[0]), " ", hex(id[1]),
          ", SYS_CTRL2 ", hex(after), " (its reset value; boot found ", hex(peer_sys_ctrl2_boot),
          ")", crlf);
    bench.verdict("the device's own soft reset goes out over this bus and the device comes back "
                  "on it, its registers where its reset leaves them",
                  reset_st == i2c_ok && id[0] == 0x08u && id[1] == 0x11u);
}

// =============================================================================
// i - the kernel
// =============================================================================

namespace kl {

using I2cArb = I2cBus<Host, P, 4>;

uint8_t rx_a[2];
const uint8_t index_chip_id[1] = {reg_chip_id};

class Probe {
public:
    using Event = std::variant<I2cDone, SleepVote>;
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

Host::Request request(uint8_t addr, uint8_t* rx, uint8_t rx_len) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(index_chip_id));
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

}  // namespace kl

void ti_kernel() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    kl::BusKernel::init_all();
    bus_ao_live = true;

    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::I2cArb>(kl::request(peer_addr, (i == 3u) ? kl::rx_a : nullptr, (i == 3u) ? 2 : 1));
    }
    kl::pump_until(4, 500);
    print(serial, "  four queued tenures: replies ", kl::Probe::n, " [", kl::Probe::replies[0], " ",
          kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3],
          "], the last one read ", hex(kl::rx_a[0]), " ", hex(kl::rx_a[1]), crlf);
    bench.verdict("four tenures through I2cBus, four replies - util/i2c_bus.hpp and "
                  "util/bus_master.hpp unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... every one i2c_ok",
                  kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      kl::Probe::replies[2] == i2c_ok && kl::Probe::replies[3] == i2c_ok);
    bench.verdict("and what the arbiter handed back is the device's own identity",
                  kl::rx_a[0] == 0x08u && kl::rx_a[1] == 0x11u);

    // The address NACK travels through the arbiter untouched: a probe of
    // an address nobody answers is the scanner's own result.
    kl::Probe::clear_tally();
    Host::Request nobody{};
    nobody.addr = 0x7Au;
    nobody.reply = reply_to<kl::Probe, I2cDone>();
    post<kl::I2cArb>(nobody);
    kl::pump_until(1, 200);
    print(serial, "  a probe of an address nobody answers came back ", kl::Probe::replies[0], " (",
          i2c_nack_addr, " is i2c_nack_addr)", crlf);
    bench.verdict("the wire's own outcome reaches the requester untouched: i2c_nack_addr",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_nack_addr);

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(peer_addr, nullptr, 1));
    }
    kl::pump_until(6, 500);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately", kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);

    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);

    bus_ao_live = false;
    (void)host_ready();
}

// =============================================================================
// j - the DMA engines
// =============================================================================

void tj_engines() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    uint8_t reference[8] = {};
    (void)read_regs(reg_chip_id, reference, 8);
    // Read out of the device what the engined write below will disturb,
    // while the plain host still owns the vectors.
    const uint8_t start = read_reg(reg_sys_ctrl2);

    Dma<1>::init();
    bus_ao_live = false;
    if (!DmaHost::init(clock)) {
        bench.verdict("the engined host came up", false);
        return;
    }
    dma_host_live = true;

    // A write phase on the transmit engine, a read phase of eight on the
    // receive engine under CR2.LAST.
    uint8_t got[8] = {};
    const uint8_t st = read_regs<DmaHost>(reg_chip_id, got, 8);
    print(serial, "  eight bytes through the engines: ", hex(got[0]), " ", hex(got[1]), " ",
          hex(got[2]), " ", hex(got[3]), " ", hex(got[4]), " ", hex(got[5]), " ", hex(got[6]), " ",
          hex(got[7]), ", status ", st, crlf);
    bench.verdict("a write-then-read tenure whose phases both ride the DMA ends i2c_ok",
                  st == i2c_ok);
    bench.verdict("... and the block is byte-exact against the same read on the pump",
                  same(reference, got, 8));

    // A ONE-BYTE read stays on the pump even with the engines named: the
    // ACK-before-ADDR sequence has no DMA expression.
    uint8_t one = 0;
    const uint8_t one_st = read_regs<DmaHost>(reg_chip_id, &one, 1);
    print(serial, "  a one-byte read with the engines named: ", hex(one), ", status ", one_st,
          " (the pump served it - the ACK-before-ADDR sequence cannot be expressed as a block)",
          crlf);
    bench.verdict("the one-byte read is served by the pump under an engined host, and it is the "
                  "same byte",
                  one_st == i2c_ok && one == reference[0]);

    // A write of two bytes through the transmit engine, read back on the
    // pump: the engined write path really moves the bytes.
    const uint8_t cmd[2] = {reg_sys_ctrl2, 0x00u};
    const uint8_t wr = tenure<DmaHost>(peer_addr, cmd, 2, nullptr, 0);
    dma_host_live = false;
    DmaHost::release();
    (void)host_ready();
    const uint8_t back = read_reg(reg_sys_ctrl2);
    print(serial, "  a two-byte write through the transmit engine: status ", wr,
          ", the register reads ", hex(back), " (it held ", hex(start), " before)", crlf);
    bench.verdict("the engined write phase reaches the device", wr == i2c_ok && back == 0x00u);
    (void)write_reg(reg_sys_ctrl2, start);
    (void)write_reg(reg_sys_ctrl1, stmpe811_soft_reset);
}

// =============================================================================
// k - the recovery verbs
// =============================================================================

void tk_recovery() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    // SWRST puts the state machines and the flags back and keeps nothing:
    // 27.6.1's own verb, and ES0206 2.10.3's workaround for a START that
    // will not come out. The registers behind it are the caller's to
    // rewrite, which is what recover() does.
    (void)S::addresses(I2cAddressConfig{.own = 0x33});
    const uint32_t before_ccr = S::regs().CCR;
    const uint32_t before_oar = S::regs().OAR1;
    S::software_reset();
    const uint32_t after_ccr = S::regs().CCR;
    const uint32_t after_oar = S::regs().OAR1;
    const uint32_t after_cr1 = S::regs().CR1;
    print(serial, "  SWRST pulsed: CCR ", hex(before_ccr), " -> ", hex(after_ccr), ", OAR1 ",
          hex(before_oar), " -> ", hex(after_oar), ", CR1 ", hex(after_cr1), crlf);
    bench.verdict("the software reset puts the whole register file back, the configuration "
                  "included - so a driver that uses it must rewrite the timing",
                  after_ccr == 0u && after_oar == 0u && (after_cr1 & I2C_CR1_PE) == 0u);

    const bool recovered = Host::recover();
    const I2cTiming t = S::timing();
    print(serial, "  after recover(): FREQ ", t.freq_mhz, " CCR ", hex(t.ccr), " TRISE ", t.trise,
          ", PE ", S::enabled() ? "up" : "DOWN", crlf);
    bench.verdict("recover() is the SWRST with the timing written again and PE back up",
                  recovered && S::enabled() && t.ccr == Host::timing_of(I2cSpeed::standard_100k).ccr);
    uint8_t id[2] = {};
    const uint8_t st = read_regs(reg_chip_id, id, 2);
    bench.verdict("... and the bus works after it", st == i2c_ok && id[0] == 0x08u);

    // The unstick: nine clocks and a STOP by hand, only when SDA is
    // actually held low. A healthy wire is left alone and says 0.
    const uint8_t pulses = Host::unstick();
    print(serial, "  unstick() on a healthy wire: ", pulses,
          " pulses (0 = SDA was free and nothing was driven)", crlf);
    bench.verdict("unstick() finds this bus free and drives nothing", pulses == 0u);
    const uint8_t after = read_regs(reg_chip_id, id, 2);
    bench.verdict("... and the pads come back to the peripheral", after == i2c_ok && id[1] == 0x11u);

    // The errata's own counter: how many BUS ERRORS were seen and ignored
    // while this side was the controller. RM0090 27.3.4 says a controller's
    // transfer is unaffected by one and ES0206 2.10.1 says one can be
    // raised where there was none, so the driver counts instead of aborting.
    print(serial, "  bus errors seen and ignored since boot: ", Host::spurious_bus_errors(),
          " (ES0206 2.10.1: in controller mode a BERR is spurious or harmless, and the tenure "
          "runs on)", crlf);
    bench.verdict("the tenures above all completed, whatever BERR did - the count is printed and "
                  "not judged, one board being one specimen",
                  true);
}

// =============================================================================
// the menu
// =============================================================================

void banner() {
    print(serial, crlf, "test_stm32f4_i2c - I2C3 with the board's own touch controller", crlf);
    bench.menu();
}

/// The boot scan: who is on this bus, and is it the part the schematic
/// names. Every device letter reads `peer_addr` from here.
void find_peer() {
    if (!host_ready()) {
        return;
    }
    for (uint8_t a = 0x08; a <= 0x77u && peer_addr == 0u; ++a) {
        if (probe(a) == i2c_ok) {
            peer_addr = a;
        }
    }
    if (peer_addr == 0u) {
        return;
    }
    uint8_t id[2] = {};
    (void)read_regs(reg_chip_id, id, 2);
    peer_is_stmpe = peer_addr == touch_addr &&
                    static_cast<uint16_t>((static_cast<uint16_t>(id[0]) << 8) | id[1]) ==
                        stmpe811_chip_id;
    peer_sys_ctrl2_boot = read_reg(reg_sys_ctrl2);
}

}  // namespace

// ---- the vectors ---------------------------------------------------------------------------

extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }

extern "C" void I2C3_EV_IRQHandler() {
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

extern "C" void I2C3_ER_IRQHandler() {
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

extern "C" void DMA1_Stream4_IRQHandler() {
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
    boot.gate = brio::I2c<3>::bus_clock();
    brio::I2c<3>::bus_clock(true);
    boot.cr1 = brio::I2c<3>::regs().CR1;
    boot.cr2 = brio::I2c<3>::regs().CR2;
    boot.oar1 = brio::I2c<3>::regs().OAR1;
    boot.oar2 = brio::I2c<3>::regs().OAR2;
    boot.sr1 = brio::I2c<3>::regs().SR1;
    boot.sr2 = brio::I2c<3>::regs().SR2;
    boot.ccr = brio::I2c<3>::regs().CCR;
    boot.trise = brio::I2c<3>::regs().TRISE;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // The two lines as plain inputs, before the peripheral is given them:
    // what the board leaves on the wire at reset.
    SclPad::input();
    SdaPad::input();
    boot.scl = SclPad::read();
    boot.sda = SdaPad::read();

    const bool host_ok = host_ready();
    boot.sr2_ready = brio::I2c<3>::regs().SR2;
    find_peer();

    bench.letter('a', "the instances and the block", ta_block);
    bench.letter('b', "the timing arithmetic", tb_timing);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the address scan", td_scan);
    bench.letter('e', "the device's identity", te_identity);
    bench.letter('f', "the three receive choreographies", tf_receive_shapes);
    bench.letter('g', "the speeds, with SCL counted on the pad", tg_speeds);
    bench.letter('h', "a register written and read back", th_write_back);
    bench.letter('i', "THE KERNEL: I2cBus over I2cHost", ti_kernel);
    bench.letter('j', "the same tenures through the DMA engines", tj_engines);
    bench.letter('k', "the recovery verbs", tk_recovery);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", " i2c3=", host_ok ? "host" : "FAILED",
                    " peer=", brio::hex(peer_addr), peer_is_stmpe ? " (STMPE811)" : " (unknown)",
                    brio::crlf);
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
