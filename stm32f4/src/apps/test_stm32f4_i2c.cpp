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
// suite builds for the two Discovery boards alone: a bus driver measured
// against nothing is a register read-back, and each of them carries its
// touch-screen controller on a bus of its own. The letters know a PEER -
// an address, an identity register with the bytes its datasheet gives it,
// one register they may write and read back with nothing moving on the
// board, and the way the device is put back where its reset leaves it -
// and nothing else of the device:
//
//   STM32F429I-DISC1
//   I2C3   SCL PA8, SDA PC9 (AF4), the pull-ups the board's own
//   the touch controller   an STMPE811 at the 7-bit address 0x41, whose
//                          CHIP_ID at register 0x00 reads 0x0811 as two
//                          bytes MSB first and whose ID_VER at 0x02 reads
//                          a revision byte; SYS_CTRL2 at 0x04 is four
//                          clock gates, written and read back here, and
//                          SYS_CTRL1's soft reset puts every one of its
//                          registers back
//
//   32F469IDISCOVERY
//   I2C1   SCL PB8, SDA PB9 (AF4), the board's 1.5 k pull-ups (MB1189
//          R140/R142); the panel and its touch controller share the
//          reset line PH7 (UM1932 4.14), which this suite pulses before
//          the scan and again to leave the device where its reset puts it
//   the touch controller   the FocalTech capacitive controller of the
//                          MB1166 display board, found by the scan at
//                          the 7-bit address 0x38, tearing its INT line
//                          on PJ5: its register map is FocalTech's
//                          "Application Note for FT6x06 CTPM" (v1.0,
//                          bound into the FT6236/FT6336/FT6436 series
//                          datasheet v0.3), its pins and timings the
//                          FT6x06 datasheet v0.1 and that series
//                          datasheet - FOCALTECH_ID at 0xA8 (0x11 by the
//                          note), CIPHER at 0xA3, FIRMID at 0xA6, LIB_VER
//                          at 0xA1/0xA2, RELEASE_CODE_ID at 0xAF; TH_GROUP
//                          at 0x80 is the touch threshold, written and
//                          read back here; the reset is the shared line's
//                          pulse, 300 ms before the first report
//
// and the parts of the chapter that need no device are measured anyway:
// the timing arithmetic against the registers in force, the address scan
// against every address nobody answers, and the SCL frequency counted on
// the clock pad's OWN INPUT BUFFER while a DMA-carried read runs (an I2C
// pad is an open-drain alternate function, so its IDR is the wire).
//
// The touch controller is left in ITS RESET STATE (the device's own
// reset - the STMPE811's soft reset, the other board's reset line - is
// the last thing the device letters do to it).
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
//   l  (the 32F469IDISCOVERY, by name only) A FINGER ON THE GLASS: the
//      touch controller in interrupt trigger mode, its INT counted on
//      EXTI line 5 and its touch data polled for eight seconds while a
//      human touches the panel - coordinates, event flags, touch ids
//
// build: boards = f429zi,f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/exti.hpp"
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

// ---- the board: the console, the bus, the peer -------------------------------------------

/// What the letters know of the device beyond the bus: the address the
/// board's schematic names, where its identity starts and what the
/// datasheet says it reads (MSB first), a second register printed beside
/// it, and one register that may be written and read back with nothing
/// moving on the board. `id_len == 0` says the datasheet is NOT on the
/// desk: the letters that need the identity or the scratch register then
/// decline by name, and the bus is still measured on whatever the device
/// answers at `id_reg`.
struct PeerFacts {
    const char* name;
    uint8_t addr;
    uint8_t id_reg;
    uint8_t id_len;
    uint8_t id_bytes[2];
    const char* id_name;
    uint8_t ver_reg;
    const char* ver_name;
    uint8_t scratch_reg;
    const char* scratch_name;
    uint8_t scratch_patterns[3];
};

#if defined(STM32F469xx)
constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
constexpr uint8_t console_instance = 3;
constexpr uint8_t bus_instance = 1;
constexpr I2cPins bus_pins{.scl = {'B', 8, PinFunction::af4}, .sda = {'B', 9, PinFunction::af4}};
using SclPad = Pin<'B', 8>;
using SdaPad = Pin<'B', 9>;
/// I2C1's transmit request is DMA1 stream 6 on channel 1 and its receive
/// DMA1 stream 0 on channel 1 (RM0386 table 29) - the reserve checks these
/// two cells at compile time.
using TxEngine = DmaTxEngine<1, 6, 1>;
using RxEngine = DmaRxEngine<1, 0, 1>;
/// The line the panel and the touch controller are reset by (UM1932 4.14).
using ResetLine = Pin<'H', 7>;
/// FocalTech's application note for the FT6x06 CTPM: FOCALTECH_ID at
/// 0xA8 (0x11), FIRMID at 0xA6, TH_GROUP at 0x80 - the threshold for
/// touch detection, a plain read/write register that moves no pin.
constexpr PeerFacts peer{.name = "the MB1166's touch controller",
                         .addr = 0,   // found by the scan
                         .id_reg = 0xA8,
                         .id_len = 1,
                         .id_bytes = {0x11, 0},
                         .id_name = "FOCALTECH_ID (0xA8)",
                         .ver_reg = 0xA6,
                         .ver_name = "FIRMID (0xA6)",
                         .scratch_reg = 0x80,
                         .scratch_name = "TH_GROUP (0x80)",
                         .scratch_patterns = {0x14, 0x40, 0x22}};
/// The rest of the note's map the finger letter reads: the touch data
/// block (TD_STATUS then the first point's XH/XL/YH/YL), the interrupt
/// mode, the versions.
constexpr uint8_t reg_td_status = 0x02;
constexpr uint8_t reg_g_mode = 0xA4;
constexpr uint8_t reg_lib_ver_h = 0xA1;
constexpr uint8_t reg_cipher = 0xA3;
constexpr uint8_t reg_release_code = 0xAF;
constexpr uint8_t reg_ctrl = 0x86;
constexpr uint8_t reg_period_active = 0x88;
/// LCD_INT on PJ5 (UM1932 4.14): the controller's INT line, EXTI line 5.
using TouchInt = Pin<'J', 5>;
using TouchIrq = ExtInt<TouchInt>;
volatile uint32_t touch_int_edges = 0;
#else
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
constexpr uint8_t bus_instance = 3;
constexpr I2cPins bus_pins{.scl = {'A', 8, PinFunction::af4}, .sda = {'C', 9, PinFunction::af4}};
using SclPad = Pin<'A', 8>;
using SdaPad = Pin<'C', 9>;
/// I2C3's transmit request is DMA1 stream 4 on channel 3 and its receive
/// DMA1 stream 2 on channel 3 (RM0090 table 43) - the reserve checks these
/// two cells at compile time.
using TxEngine = DmaTxEngine<1, 4, 3>;
using RxEngine = DmaRxEngine<1, 2, 3>;
/// The STMPE811: CHIP_ID at 0x00 (0x0811, two bytes MSB first), ID_VER at
/// 0x02, SYS_CTRL2 at 0x04 - four clock gates and nothing that moves a
/// pin - and SYS_CTRL1's soft reset.
constexpr uint8_t reg_sys_ctrl1 = 0x03;
constexpr uint8_t stmpe811_soft_reset = 0x02;
constexpr PeerFacts peer{.name = "the STMPE811",
                         .addr = 0x41,
                         .id_reg = 0x00,
                         .id_len = 2,
                         .id_bytes = {0x08, 0x11},
                         .id_name = "CHIP_ID (0x00, two bytes MSB first)",
                         .ver_reg = 0x02,
                         .ver_name = "ID_VER (0x02)",
                         .scratch_reg = 0x04,
                         .scratch_name = "SYS_CTRL2 (0x04)",
                         .scratch_patterns = {0x00, 0x0F, 0x01}};
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

using S = I2c<bus_instance>;
using Host = I2cHost<bus_instance, bus_pins>;
using DmaHost = I2cHost<bus_instance, bus_pins, TxEngine, RxEngine>;

/// The registers the letters read and write, by the peer's names.
constexpr uint8_t reg_chip_id = peer.id_reg;
constexpr uint8_t reg_id_ver = peer.ver_reg;
constexpr uint8_t reg_sys_ctrl2 = peer.scratch_reg;
constexpr bool peer_known = peer.id_len != 0u;

/// What the boot-time scan found: which address answered, and whether the
/// part behind it is the one this board's schematic names.
uint8_t peer_addr = 0;
bool peer_identified = false;
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

/// Whether `bytes` are the identity the datasheet gives the peer.
bool is_peer_id(const uint8_t* bytes) { return peer_known && same(bytes, peer.id_bytes, peer.id_len); }

#if defined(STM32F469xx)
void spin_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
    }
}
#endif

/// The device put back where its reset leaves it, and the time it takes
/// to answer again: the STMPE811's soft reset over the bus, the other
/// board's reset LINE pulsed low - the panel behind it resets too, which
/// is nothing this suite drew on it.
uint8_t peer_reset() {
#if defined(STM32F469xx)
    ResetLine::output();
    ResetLine::clear();
    spin_ms(20);
    ResetLine::set();
    spin_ms(300);
    return i2c_ok;
#else
    const uint8_t cmd[2] = {reg_sys_ctrl1, stmpe811_soft_reset};
    const uint8_t st = tenure<Host>(peer_addr, cmd, 2, nullptr, 0);
    for (uint32_t spins = 400'000u; spins != 0u; --spins) {
    }
    return st;
#endif
}

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
    print(serial, "  I2C", bus_instance, " gate at reset: ", boot.gate ? "OPEN" : "closed", "; CR1 ", hex(boot.cr1),
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
                  S::event_irq == i2c_event_irq(bus_instance) &&
                      S::error_irq == i2c_error_irq(bus_instance) &&
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
          " ended some other way; ", peer.name, " is ", crlf, "  ");
    if (peer.addr != 0u) {
        print(serial, "at ", hex(peer.addr), " by the board's schematic", crlf);
    } else {
        print(serial, "at the first address that answered, ", hex(peer_addr), crlf);
    }
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
    const uint8_t ver = read_reg(reg_id_ver);
    print(serial, "  address ", hex(peer_addr), ": ", peer.id_name, " reads ", hex(id[0]), " ",
          hex(id[1]), ", ", peer.ver_name, " reads ", hex(ver), "; status ", st, crlf);
    bench.verdict("a write-then-read tenure - the index out, a repeated START, the value back - "
                  "answers i2c_ok",
                  st == i2c_ok);
    if constexpr (peer_known) {
        bench.verdict("the device on this bus is the one the board's schematic names, and it says "
                      "so in its own identity register",
                      is_peer_id(id));
    } else {
        print(serial, "  the device's datasheet is not on the desk: its identity is printed and "
                      "not judged (declined by name)", crlf);
    }

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
    // is what says the bytes above are the device's and not the wire's -
    // a judgement that needs the datasheet to name a register that
    // differs, so without it the line is printed and not judged.
    if constexpr (peer_known) {
        const uint8_t ctrl = read_reg(reg_sys_ctrl2);
        print(serial, "  ", peer.scratch_name, " reads ", hex(ctrl),
              " - a different register, a different byte", crlf);
        bench.verdict("a different register answers differently: the bytes come from the device "
                      "and not from a pull-up",
                      ctrl != id[0] || ctrl != id[1]);
    } else {
        print(serial, "  which register would answer differently is the datasheet's to say: "
                      "not judged here (declined by name)", crlf);
    }
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

    // With the datasheet on the desk the bytes are the identity; without
    // it the same one-byte read done again is what a right procedure owes.
    uint8_t one_again = 0;
    (void)read_regs(reg_chip_id, &one_again, 1);
    bench.verdict("the ONE-byte procedure - ACK cleared before ADDR, STOP right after - reads the "
                  "device",
                  peer_known ? is_peer_id(one) : one_again == one[0]);
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
                  long_st == i2c_ok && same(run, one, 8));
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
    if constexpr (!peer_known) {
        print(serial, "  the device's datasheet is not on the desk, so no register of its is "
                      "written: the letter declines by name", crlf);
        return;
    } else {
        // The scratch register is one the datasheet says moves no pin: the
        // STMPE811's SYS_CTRL2 is four clock gates.
        const uint8_t start = read_reg(reg_sys_ctrl2);
        bool wrote = true;
        for (uint8_t i = 0; i < 3u; ++i) {
            const uint8_t st = write_reg(reg_sys_ctrl2, peer.scratch_patterns[i]);
            const uint8_t back = read_reg(reg_sys_ctrl2);
            print(serial, "  ", peer.scratch_name, " <- ", hex(peer.scratch_patterns[i]),
                  " (status ", st, "), reads ", hex(back), crlf);
            if (st != i2c_ok || back != peer.scratch_patterns[i]) {
                wrote = false;
            }
        }
        bench.verdict("a register written over this bus reads back exactly - the write path and "
                      "the read path are the same tenure with a repeated START between them",
                      wrote);
#if !defined(STM32F469xx)
        // And one pattern the DEVICE masks: with its ADC gate open this
        // part holds the temperature-sensor gate clear whatever is
        // written. A device rule and not a bus one, so it is printed and
        // not judged - what the bus owes is that the byte written is the
        // byte that arrived, which the three above say.
        const uint8_t masked_st = write_reg(reg_sys_ctrl2, 0x0Cu);
        print(serial, "  SYS_CTRL2 <- 0xC (status ", masked_st, "), reads ",
              hex(read_reg(reg_sys_ctrl2)),
              " - the device masks a bit of its own, and that is the device's business", crlf);
#endif

        // A two-byte write in one tenure: the index and the value are one
        // span, so this is already what every write above did; what it
        // proves here is that the tenure ends on BTF and not on TxE.
        const uint8_t st = write_reg(reg_sys_ctrl2, start);
        print(serial, "  ", peer.scratch_name, " back to the value found at boot, ", hex(start),
              " (status ", st, ")", crlf);
        bench.verdict("the boot value is restored",
                      st == i2c_ok && read_reg(reg_sys_ctrl2) == start);

        // And the device's own reset, which is how this suite leaves it:
        // every register of its back where its power-on put them.
        const uint8_t reset_st = peer_reset();
        uint8_t id[2] = {};
        (void)read_regs(reg_chip_id, id, 2);
        const uint8_t after = read_reg(reg_sys_ctrl2);
        print(serial, "  after the device's reset: ", peer.id_name, " ", hex(id[0]), " ",
              hex(id[1]), ", ", peer.scratch_name, " ", hex(after),
              " (its reset value; boot found ", hex(peer_sys_ctrl2_boot), ")", crlf);
        bench.verdict("the device's own reset goes out and the device comes back on the bus, its "
                      "registers where its reset leaves them",
                      reset_st == i2c_ok && is_peer_id(id));
    }
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
                  (peer_known ? is_peer_id(kl::rx_a) : true));

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
    // pump: the engined write path really moves the bytes. Without the
    // device's datasheet nothing of its is written, and the write phase
    // is proved on the index byte every read above sent through the same
    // engine.
    if constexpr (peer_known) {
        const uint8_t cmd[2] = {reg_sys_ctrl2, peer.scratch_patterns[0]};
        const uint8_t wr = tenure<DmaHost>(peer_addr, cmd, 2, nullptr, 0);
        dma_host_live = false;
        DmaHost::release();
        (void)host_ready();
        const uint8_t back = read_reg(reg_sys_ctrl2);
        print(serial, "  a two-byte write through the transmit engine: status ", wr,
              ", the register reads ", hex(back), " (it held ", hex(start), " before)", crlf);
        bench.verdict("the engined write phase reaches the device",
                      wr == i2c_ok && back == peer.scratch_patterns[0]);
        (void)write_reg(reg_sys_ctrl2, start);
        (void)peer_reset();
    } else {
        dma_host_live = false;
        DmaHost::release();
        (void)host_ready();
        (void)start;
        print(serial, "  no register of the device is written (its datasheet is not on the desk): "
                      "the engined write phase is the index byte of the reads above (declined by "
                      "name)", crlf);
    }
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
    bench.verdict("... and the bus works after it",
                  st == i2c_ok && (peer_known ? id[0] == peer.id_bytes[0] : true));

    // The unstick: nine clocks and a STOP by hand, only when SDA is
    // actually held low. A healthy wire is left alone and says 0.
    const uint8_t pulses = Host::unstick();
    print(serial, "  unstick() on a healthy wire: ", pulses,
          " pulses (0 = SDA was free and nothing was driven)", crlf);
    bench.verdict("unstick() finds this bus free and drives nothing", pulses == 0u);
    const uint8_t after = read_regs(reg_chip_id, id, 2);
    bench.verdict("... and the pads come back to the peripheral",
                  after == i2c_ok && (peer_known ? is_peer_id(id) : true));

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

#if defined(STM32F469xx)
// =============================================================================
// l - a finger on the glass (by name only)
// =============================================================================

void tl_finger() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    // The versions, for the record.
    uint8_t lib[2] = {0, 0};
    (void)read_regs(reg_lib_ver_h, lib, 2);
    print(serial, "  LIB_VER ", hex(lib[0]), " ", hex(lib[1]), ", CIPHER ", hex(read_reg(reg_cipher)), ", FIRMID ",
          hex(read_reg(peer.ver_reg)), ", RELEASE_CODE_ID ", hex(read_reg(reg_release_code)), ", CTRL ",
          hex(read_reg(reg_ctrl)), ", PERIODACTIVE ", hex(read_reg(reg_period_active)), ", G_MODE ",
          hex(read_reg(reg_g_mode)), crlf);

    // Interrupt trigger mode: a pulse on INT per report while a finger is
    // down (the note's 1.2), counted on EXTI line 5.
    const uint8_t g_before = read_reg(reg_g_mode);
    (void)write_reg(reg_g_mode, 0x01);
    const uint8_t g_trigger = read_reg(reg_g_mode);
    bench.verdict("G_MODE takes the interrupt trigger mode and reads it back", g_trigger == 0x01u);
    (void)TouchIrq::claim(PinPull::up);
    (void)TouchIrq::configure(ExtiSense::falling);
    (void)TouchIrq::clear();
    touch_int_edges = 0;
    (void)TouchIrq::arm(true);
    Nvic::enable(TouchIrq::irq());
    const bool int_idle_high = TouchInt::read();

    // Up to twenty-five seconds for a finger to arrive, then eight
    // seconds of it, the touch data polled every 10 ms.
    print(serial, "  TOUCH THE GLASS - waiting up to 25 s for a finger, then eight seconds of it", crlf);
    uint32_t samples = 0, touched = 0, twos = 0, errors = 0;
    uint16_t x_min = 0xFFFF, x_max = 0, y_min = 0xFFFF, y_max = 0;
    uint8_t flags_seen = 0, ids_seen = 0;
    bool first_printed = false;
    uint32_t start = Ticker::ticks();
    uint32_t window = 25000u;
    bool arrived = false;
    while (Ticker::ticks() - start < window) {
        if (!arrived && touched != 0u) {
            arrived = true;
            start = Ticker::ticks();
            window = 8000u;
            print(serial, "  a finger: eight seconds from now", crlf);
        }
        uint8_t td[5] = {0, 0, 0, 0, 0};
        if (read_regs(reg_td_status, td, 5) != i2c_ok) {
            ++errors;
        } else {
            ++samples;
            const uint8_t points = td[0] & 0x0Fu;
            if (points == 1u || points == 2u) {
                ++touched;
                if (points == 2u) {
                    ++twos;
                }
                const uint8_t flag = static_cast<uint8_t>(td[1] >> 6);
                const uint16_t x = static_cast<uint16_t>(((td[1] & 0x0Fu) << 8) | td[2]);
                const uint16_t y = static_cast<uint16_t>(((td[3] & 0x0Fu) << 8) | td[4]);
                const uint8_t id = static_cast<uint8_t>(td[3] >> 4);
                flags_seen = static_cast<uint8_t>(flags_seen | (1u << flag));
                ids_seen = static_cast<uint8_t>(ids_seen | (id < 8u ? (1u << id) : 0x80u));
                x_min = x < x_min ? x : x_min;
                x_max = x > x_max ? x : x_max;
                y_min = y < y_min ? y : y_min;
                y_max = y > y_max ? y : y_max;
                if (!first_printed) {
                    print(serial, "  first report: TD_STATUS ", hex(td[0]), ", event flag ", flag, ", id ", id, ", x ", x,
                          ", y ", y, crlf);
                    first_printed = true;
                }
            }
        }
        const uint32_t t = Ticker::ticks();
        while (Ticker::ticks() - t < 10u) {
        }
    }
    Nvic::disable(TouchIrq::irq());
    (void)TouchIrq::arm(false);
    const uint32_t edges = touch_int_edges;
    (void)write_reg(reg_g_mode, g_before);
    print(serial, "  ", samples, " samples, ", touched, " with a finger (", twos, " with two), ", errors, " bus errors; x ",
          x_min, "..", x_max, ", y ", y_min, "..", y_max, "; event flags seen ", hex(flags_seen), " (bit 0 press, 1 lift, 2 contact, 3 none), ids ",
          hex(ids_seen), "; INT idle ", int_idle_high ? "high" : "LOW", ", ", edges, " falling edges", crlf);
    bench.verdict("a finger was reported: TD_STATUS counted one or two points", touched != 0u);
    bench.verdict("the coordinates stayed inside a 800x800 frame (12 bits each, the panel 800 by 480)",
                  touched != 0u && x_max < 800u && y_max < 800u);
    bench.verdict("the contact event (10b) was among the flags seen", (flags_seen & 0x04u) != 0u);
    bench.verdict("INT idles high with no finger and pulsed on the reports in trigger mode",
                  int_idle_high && edges != 0u);
    bench.verdict("no bus error over the polling", errors == 0u);
    bench.verdict("G_MODE put back to what it was", read_reg(reg_g_mode) == g_before);
}
#endif

void banner() {
    print(serial, crlf, "test_stm32f4_i2c - I2C", bus_instance,
          " with the board's own touch controller", crlf);
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
    peer_identified = (peer.addr == 0u || peer_addr == peer.addr) && is_peer_id(id);
    peer_sys_ctrl2_boot = peer_known ? read_reg(reg_sys_ctrl2) : 0u;
}

/// What the board wants before the first tenure: the other Discovery's
/// touch controller sits behind the panel's reset line, left floating by
/// the boot, so it is pulsed once here - the pull-ups then read an idle
/// bus with a device on it.
void board_prepare() {
#if defined(STM32F469xx)
    (void)peer_reset();
#endif
}

}  // namespace

// ---- the vectors ---------------------------------------------------------------------------

#if defined(STM32F469xx)
extern "C" void USART3_IRQHandler() { (void)Serial::isr(); }
extern "C" void EXTI9_5_IRQHandler() {
    const uint32_t fired = brio::Exti::isr(TouchIrq::mask);
    if (TouchIrq::served(fired)) {
        touch_int_edges = touch_int_edges + 1u;
    }
}
#define BRIO_I2C_EV_HANDLER I2C1_EV_IRQHandler
#define BRIO_I2C_ER_HANDLER I2C1_ER_IRQHandler
#define BRIO_I2C_TX_DMA_HANDLER DMA1_Stream6_IRQHandler
#define BRIO_I2C_RX_DMA_HANDLER DMA1_Stream0_IRQHandler
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#define BRIO_I2C_EV_HANDLER I2C3_EV_IRQHandler
#define BRIO_I2C_ER_HANDLER I2C3_ER_IRQHandler
#define BRIO_I2C_TX_DMA_HANDLER DMA1_Stream4_IRQHandler
#define BRIO_I2C_RX_DMA_HANDLER DMA1_Stream2_IRQHandler
#endif

extern "C" void BRIO_I2C_EV_HANDLER() {
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

extern "C" void BRIO_I2C_ER_HANDLER() {
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

extern "C" void BRIO_I2C_TX_DMA_HANDLER() {
    if (dma_host_live && DmaHost::dma_isr()) {
        xfer_done = true;
    }
}
extern "C" void BRIO_I2C_RX_DMA_HANDLER() {
    if (dma_host_live && DmaHost::dma_isr()) {
        xfer_done = true;
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    // What the silicon held before a line of this program ran: the gate is
    // read out of the RCC (reading it does not open it), and the registers
    // behind it need the clock, so it is opened for the reading.
    boot.gate = S::bus_clock();
    S::bus_clock(true);
    boot.cr1 = S::regs().CR1;
    boot.cr2 = S::regs().CR2;
    boot.oar1 = S::regs().OAR1;
    boot.oar2 = S::regs().OAR2;
    boot.sr1 = S::regs().SR1;
    boot.sr2 = S::regs().SR2;
    boot.ccr = S::regs().CCR;
    boot.trise = S::regs().TRISE;

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

    board_prepare();
    const bool host_ok = host_ready();
    boot.sr2_ready = S::regs().SR2;
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
#if defined(STM32F469xx)
    bench.letter('l', "a finger on the glass: the touch controller reporting", tl_finger, false);
#endif

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", " i2c", bus_instance, "=",
                    host_ok ? "host" : "FAILED", " peer=", brio::hex(peer_addr),
                    peer_identified ? " (identified)" : " (unknown)", brio::crlf);
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
