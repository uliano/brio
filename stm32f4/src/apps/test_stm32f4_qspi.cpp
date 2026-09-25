// test_stm32f4_qspi - the reference bench suite for the STM32F4's Quad-SPI
// memory interface: the block in its three functional modes - indirect,
// automatic status polling, memory-mapped - against the serial flash the
// board carries, stm32f4/quadspi.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS A FLASH SOLDERED TO THE BOARD, which is why this suite
// builds for the 32F469IDISCOVERY alone: a command engine measured against
// nothing is a register read-back, and this board wires a Micron
// MT25QL128ABA (128 Mbit, 16 MB, 64 KB sectors with 4 KB and 32 KB
// subsectors, 133 MHz in single transfer rate) to the block's bank 1 -
//
//   the pads   CLK PF10 (AF9), NCS PB6 (AF10, the board's 10 k pull-up
//              R55), IO0 PF8 (AF10), IO1 PF9 (AF10), IO2 PF7 (AF9), IO3 PF6
//              (AF9) - MB1189's QSPI sheet, DS11189 table 12
//   the clock  HCLK/2 = 90 MHz for every command but READ 03h, whose own
//              ceiling is 54 MHz (MT25QL128ABA Rev. K table 44) and which
//              runs at HCLK/4 = 45 MHz
//   the device's own words   every opcode, dummy count and register below
//              is table 18 and tables 3..11 of that datasheet, spelled as
//              QspiCommand values here - the driver knows phases and lines
//              and nothing of any device
//
// WHAT IT COSTS. Letter e ERASES the flash's LAST 4 KB subsector (address
// 0xFFF000) once and programs a pattern into it; every other letter reads
// that subsector back. One subsector erase a run, on a device rated for
// 100 000 of them, and the rest of the array is never touched - the chip
// was found blank and stays blank but for those 4 KB. The device's
// volatile configuration is written by letter g and put back by it; a
// software reset at boot puts it back too, should a run die in between.
//
// What is exercised, letter by letter:
//   a  the block: the gate closed at reset and the registers behind it,
//      the configuration read back, the vector, the window, the DMA cell
//   b  what the driver refuses, and that a refusal writes nothing
//   c  the device's identity: READ ID's manufacturer, type and capacity,
//      its unique id, and the SFDP signature
//   d  the device's registers: status, flag status, the three
//      configuration registers against the delivery state
//   e  the last subsector: erased through a command and the block's
//      status-polling mode, blank-checked, programmed a page at a time,
//      read back byte-exact - with the erase and the page times
//   f  the read commands agree: READ, FAST READ, dual output, quad output
//      and quad I/O, each byte-exact against the pattern, each timed
//   g  the dummy cycles against the clock: the device's table 9 measured
//      through its volatile configuration register
//   h  the FIFO and the flags: the threshold, a full FIFO stalling the
//      clock, an abort mid-read, a transfer error past the flash's size
//   i  the interrupt: the vector entered once for a completed read
//   j  memory-mapped mode: the subsector read through the window in
//      bytes, halfwords and words, a copy timed, BUSY standing on the
//      prefetch, the window closed again
//   k  deep power-down: entered and released
//
// build: boards = f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/quadspi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the console --------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
using Serial = Uart<3, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the pads -----------------------------------------------------------------------

using Clk = Pin<'F', 10>;
using Ncs = Pin<'B', 6>;
using Io0 = Pin<'F', 8>;
using Io1 = Pin<'F', 9>;
using Io2 = Pin<'F', 7>;
using Io3 = Pin<'F', 6>;

void claim_pads() {
    constexpr PinConfig fast{.speed = PinSpeed::very_high};
    Clk::function(PinFunction::af9, fast);
    Ncs::function(PinFunction::af10, fast);
    Io0::function(PinFunction::af10, fast);
    Io1::function(PinFunction::af10, fast);
    Io2::function(PinFunction::af9, fast);
    Io3::function(PinFunction::af9, fast);
}

// ---- the device: the MT25QL128ABA in the chapter's words -------------------------------

constexpr uint32_t flash_bytes = 16u * 1024u * 1024u;
constexpr uint32_t page_bytes = 256;
constexpr uint32_t subsector_bytes = 4096;
constexpr uint32_t test_subsector = flash_bytes - subsector_bytes;   // the last 4 KB

/// The block at HCLK/2: 90 MHz, under the device's 133 MHz for every
/// command but READ; NCS high for five cycles (55.6 ns) between commands,
/// which covers the 50 ns a non-read command wants (tSHSL2).
constexpr QspiConfig fast_cfg{.prescaler = qspi_prescaler_for(SysClock::hz, 133'000'000u),
                              .sample_shift = false,
                              .fifo_threshold = 4,
                              .address_bits = qspi_address_bits(flash_bytes),
                              .cs_high_cycles = qspi_cs_high_cycles_for(90'000'000u, 50u)};
/// READ 03h's own ceiling is 54 MHz: HCLK/4.
constexpr QspiConfig slow_cfg{.prescaler = qspi_prescaler_for(SysClock::hz, 54'000'000u),
                              .sample_shift = false,
                              .fifo_threshold = 4,
                              .address_bits = qspi_address_bits(flash_bytes),
                              .cs_high_cycles = qspi_cs_high_cycles_for(45'000'000u, 50u)};
static_assert(fast_cfg.prescaler == 1 && slow_cfg.prescaler == 3);
static_assert(fast_cfg.address_bits == 24 && fast_cfg.cs_high_cycles == 5 && slow_cfg.cs_high_cycles == 3);

// Table 18 of the datasheet, one command a row: the protocol columns are
// the lines of each phase, the dummy column its cycles.
constexpr QspiCommand reset_enable{.instruction = 0x66};
constexpr QspiCommand reset_memory{.instruction = 0x99};
constexpr QspiCommand read_id{.instruction = 0x9E, .data_lines = QspiLines::one};
constexpr QspiCommand read_sfdp{.instruction = 0x5A, .address_lines = QspiLines::one,
                                .dummy_cycles = 8, .data_lines = QspiLines::one};
constexpr QspiCommand read_slow{.instruction = 0x03, .address_lines = QspiLines::one,
                                .data_lines = QspiLines::one};
constexpr QspiCommand fast_read{.instruction = 0x0B, .address_lines = QspiLines::one,
                                .dummy_cycles = 8, .data_lines = QspiLines::one};
constexpr QspiCommand dual_out_read{.instruction = 0x3B, .address_lines = QspiLines::one,
                                    .dummy_cycles = 8, .data_lines = QspiLines::two};
constexpr QspiCommand quad_out_read{.instruction = 0x6B, .address_lines = QspiLines::one,
                                    .dummy_cycles = 8, .data_lines = QspiLines::four};
constexpr QspiCommand quad_io_read{.instruction = 0xEB, .address_lines = QspiLines::four,
                                   .dummy_cycles = 10, .data_lines = QspiLines::four};
constexpr QspiCommand write_enable{.instruction = 0x06};
constexpr QspiCommand read_status{.instruction = 0x05, .data_lines = QspiLines::one};
constexpr QspiCommand read_flag_status{.instruction = 0x70, .data_lines = QspiLines::one};
constexpr QspiCommand read_nvcr{.instruction = 0xB5, .data_lines = QspiLines::one};
constexpr QspiCommand read_vcr{.instruction = 0x85, .data_lines = QspiLines::one};
constexpr QspiCommand read_evcr{.instruction = 0x65, .data_lines = QspiLines::one};
constexpr QspiCommand write_vcr{.instruction = 0x81, .data_lines = QspiLines::one};
constexpr QspiCommand clear_flag_status{.instruction = 0x50};
constexpr QspiCommand page_program{.instruction = 0x02, .address_lines = QspiLines::one,
                                   .data_lines = QspiLines::one};
constexpr QspiCommand subsector_erase{.instruction = 0x20, .address_lines = QspiLines::one};
constexpr QspiCommand enter_deep_power_down{.instruction = 0xB9};
constexpr QspiCommand release_deep_power_down{.instruction = 0xAB};

/// Status register bits (table 3) and flag status register bits (table 5).
constexpr uint8_t sr_write_in_progress = 0x01;
constexpr uint8_t sr_write_enable_latch = 0x02;
constexpr uint8_t fsr_ready = 0x80;
constexpr uint8_t fsr_erase_error = 0x20;
constexpr uint8_t fsr_program_error = 0x10;
constexpr uint8_t fsr_protection_error = 0x02;
/// The volatile configuration register as delivered (table 7): dummy
/// cycles at their default (1111), XIP disabled (1), the reserved 0,
/// continuous wrap (11) - and the same byte with a dummy count of n.
constexpr uint8_t vcr_default = 0xFB;
constexpr uint8_t vcr_with_dummies(uint8_t n) { return static_cast<uint8_t>((n << 4) | 0x0B); }

/// The pattern the subsector carries: a different byte at every offset, a
/// swapped address line a wrong byte and not a repeated one.
constexpr uint8_t pattern(uint32_t i) {
    return static_cast<uint8_t>(((i * 0x9E37u) >> 8) ^ (i * 31u));
}

// ---- the rulers ---------------------------------------------------------------------

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// Microseconds of kernel time from the tick and SysTick's own counter,
/// read coherently: the ruler for a page program, which outlives a
/// millisecond only on a tired device.
uint32_t micros() {
    uint32_t t1, t2, v;
    do {
        t1 = Ticker::ticks();
        v = SysTick->VAL;
        t2 = Ticker::ticks();
    } while (t1 != t2);
    const uint32_t period = systick_period();
    return t1 * 1000u + ((period - 1u - v) * 1000u) / period;
}

void wait_us(uint32_t us) {
    const uint32_t t0 = micros();
    while (micros() - t0 < us) {
    }
}

bool same(const uint8_t* a, const uint8_t* b, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// ---- the device, driven ----------------------------------------------------------------

uint8_t buffer[subsector_bytes];

uint8_t read_byte_register(const QspiCommand& c) {
    uint8_t v = 0xFF;
    (void)Quadspi::read(c, 0, &v, 1);
    return v;
}

/// WIP down, through the block's own status-polling mode: RDSR every 16
/// clocks, AND-matched on bit 0 against 0, stopping on the match. What
/// the last poll read is handed back beside the status.
QspiStatus wait_write_done(uint32_t& last_status, uint32_t spins = Quadspi::default_spins) {
    return Quadspi::poll(read_status, 0, 1, sr_write_in_progress, 0x00u, 16, false, last_status,
                         spins);
}

bool write_enabled() { return (read_byte_register(read_status) & sr_write_enable_latch) != 0u; }

/// The software reset pair (66h then 99h): every volatile setting of the
/// device back to what its nonvolatile configuration says.
bool device_reset() {
    if (Quadspi::command(reset_enable) != QspiStatus::ok) {
        return false;
    }
    if (Quadspi::command(reset_memory) != QspiStatus::ok) {
        return false;
    }
    wait_us(100);   // the datasheet's reset recovery is tens of microseconds; measured below
    return true;
}

bool pattern_matches(const uint8_t* got, uint32_t n, uint32_t* first_bad = nullptr) {
    for (uint32_t i = 0; i < n; ++i) {
        if (got[i] != pattern(i)) {
            if (first_bad != nullptr) {
                *first_bad = i;
            }
            return false;
        }
    }
    return true;
}

/// One read of the whole test subsector with `c`, timed in microseconds.
QspiStatus read_subsector(const QspiCommand& c, uint32_t& us) {
    for (uint32_t i = 0; i < subsector_bytes; ++i) {
        buffer[i] = 0;
    }
    const uint32_t t0 = micros();
    const QspiStatus st = Quadspi::read(c, test_subsector, buffer, subsector_bytes);
    us = micros() - t0;
    return st;
}

// ---- what the registers held before this program touched them ---------------------------

struct BootState {
    bool gate = false;
    uint32_t cr = 0, dcr = 0, sr = 0, dlr = 0, ccr = 0, ar = 0, abr = 0, psmkr = 0, psmar = 0,
             pir = 0, lptr = 0;
};
BootState boot;

volatile uint32_t irq_entries = 0;
volatile bool irq_saw_complete = false;

// =============================================================================
// a - the block
// =============================================================================

void ta_block() {
    print(serial, "  AHB3 gate at reset: ", boot.gate ? "OPEN" : "closed", "; CR ", hex(boot.cr),
          " DCR ", hex(boot.dcr), " SR ", hex(boot.sr), " DLR ", hex(boot.dlr), " CCR ",
          hex(boot.ccr), " AR ", hex(boot.ar), " ABR ", hex(boot.abr), " PSMKR ", hex(boot.psmkr),
          " PSMAR ", hex(boot.psmar), " PIR ", hex(boot.pir), " LPTR ", hex(boot.lptr), crlf);
    bench.verdict("the gate is closed at reset, like every peripheral clock of this family",
                  !boot.gate);
    bench.verdict("table 94: every register reads zero at reset",
                  boot.cr == 0u && boot.dcr == 0u && boot.sr == 0u && boot.dlr == 0u &&
                      boot.ccr == 0u && boot.ar == 0u && boot.abr == 0u && boot.psmkr == 0u &&
                      boot.psmar == 0u && boot.pir == 0u && boot.lptr == 0u);
    print(serial, "  after init: prescaler ", Quadspi::prescaler(), " (CLK ",
          qspi_clock_hz(SysClock::hz, Quadspi::prescaler()) / 1000u, " kHz), FIFO threshold ",
          Quadspi::fifo_threshold(), ", flash ", Quadspi::address_bits(), " address bits, NCS high ",
          Quadspi::cs_high_cycles(), " cycles, enabled ", Quadspi::enabled() ? "yes" : "no", crlf);
    bench.verdict("CR and DCR read back the configuration: HCLK/2, a 16 MB device, five cycles of "
                  "NCS between commands",
                  Quadspi::enabled() && Quadspi::prescaler() == 1u && Quadspi::fifo_threshold() == 4u &&
                      Quadspi::address_bits() == 24u && Quadspi::cs_high_cycles() == 5u);
    bench.verdict("the block is idle: BUSY clear, the FIFO empty, no flag standing",
                  !Quadspi::busy() && Quadspi::fifo_level() == 0u &&
                      (Quadspi::regs().SR & (QUADSPI_SR_TCF | QUADSPI_SR_TEF | QUADSPI_SR_SMF |
                                             QUADSPI_SR_TOF)) == 0u);
    print(serial, "  vector ", static_cast<uint32_t>(Quadspi::irq), "; the window at ",
          hex(Quadspi::window_facts.base), ", ", Quadspi::window_facts.bytes / (1024u * 1024u),
          " MB; the DMA cell DMA", quadspi_dma_placements().at[0].controller, " stream ",
          quadspi_dma_placements().at[0].stream, " channel ",
          quadspi_dma_placements().at[0].channel, crlf);
    bench.verdict("this part's facts: the vector is 91, the window 0x9000 0000 of 256 MB, the "
                  "request on DMA2 stream 7 channel 3",
                  static_cast<uint32_t>(Quadspi::irq) == 91u && Quadspi::window_facts.known &&
                      Quadspi::window_facts.base == 0x9000'0000u &&
                      quadspi_dma_placement_valid(2, 7, 3));
}

// =============================================================================
// b - what the driver refuses
// =============================================================================

void tb_refusals() {
    const uint32_t cr = Quadspi::regs().CR;
    const uint32_t dcr = Quadspi::regs().DCR;
    const uint32_t ccr = Quadspi::regs().CCR;
    bool refused = true;
    refused = refused && !Quadspi::init(QspiConfig{.fifo_threshold = 0});
    refused = refused && !Quadspi::init(QspiConfig{.fifo_threshold = 33});
    refused = refused && !Quadspi::init(QspiConfig{.address_bits = 0});
    refused = refused && !Quadspi::init(QspiConfig{.address_bits = 33});
    refused = refused && !Quadspi::init(QspiConfig{.cs_high_cycles = 0});
    refused = refused && !Quadspi::init(QspiConfig{.cs_high_cycles = 9});
    bench.verdict("a configuration outside the fields - the threshold, the size, the chip-select "
                  "time - is refused",
                  refused);
    bench.verdict("... and CR and DCR are as they were",
                  Quadspi::regs().CR == cr && Quadspi::regs().DCR == dcr);

    uint8_t b[4] = {};
    uint32_t st = 0;
    bench.verdict("a command with no phase at all is refused",
                  Quadspi::command(QspiCommand{.instruction_lines = QspiLines::none}) ==
                      QspiStatus::refused);
    bench.verdict("command() refuses a command that has a data phase (that is read()'s or "
                  "write()'s)",
                  Quadspi::command(read_status) == QspiStatus::refused);
    bench.verdict("ES0321 2.4.2 AS CODE: an indirect write with dummy cycles - the first nibble "
                  "would be lost - is refused",
                  Quadspi::write(fast_read, 0, b, 4) == QspiStatus::refused);
    bench.verdict("a read on four lines with no dummy cycle - no turn-around, 13.3.3 - is refused",
                  Quadspi::read(QspiCommand{.instruction = 0xEB, .address_lines = QspiLines::four,
                                            .data_lines = QspiLines::four},
                                0, b, 4) == QspiStatus::refused);
    bench.verdict("a read of nothing, and a poll of more than four bytes, are refused",
                  Quadspi::read(fast_read, 0, b, 0) == QspiStatus::refused &&
                      Quadspi::poll(read_status, 0, 5, 1, 0, 16, false, st) == QspiStatus::refused);
    bench.verdict("map() refuses a command with no data phase",
                  !Quadspi::map(write_enable));
    bench.verdict("... and none of the refusals wrote CCR or started anything",
                  Quadspi::regs().CCR == ccr && !Quadspi::busy());
}

// =============================================================================
// c - the device's identity
// =============================================================================

void tc_identity() {
    uint8_t id[20] = {};
    const QspiStatus st = Quadspi::read(read_id, 0, id, sizeof id);
    print(serial, "  READ ID (9Eh):");
    for (uint8_t i = 0; i < sizeof id; ++i) {
        print(serial, " ", hex(id[i]));
    }
    print(serial, crlf, "  manufacturer ", hex(id[0]), " type ", hex(id[1]), " capacity ", hex(id[2]),
          ", ", id[3], " more bytes: extended id ", hex(id[4]), ", configuration ", hex(id[5]),
          crlf);
    bench.verdict("READ ID ends i2c-style clean: the command completed and its twenty bytes came",
                  st == QspiStatus::ok);
    bench.verdict("the JEDEC signature is Micron's 20h, the 3 V type BAh, the 128 Mbit capacity 18h "
                  "(datasheet table 16)",
                  id[0] == 0x20u && id[1] == 0xBAu && id[2] == 0x18u);
    bench.verdict("seventeen bytes follow (10h), and the extended id says a 64 KB uniform sector "
                  "device (table 17's bits 1:0 = 00)",
                  id[3] == 0x10u && (id[4] & 0x03u) == 0u);
    bool same_twice = true;
    for (uint8_t round = 0; round < 4u; ++round) {
        uint8_t again[20] = {};
        (void)Quadspi::read(read_id, 0, again, sizeof again);
        same_twice = same_twice && same(id, again, sizeof id);
    }
    bench.verdict("four more reads give the same twenty bytes", same_twice);

    uint8_t sfdp[8] = {};
    const QspiStatus sst = Quadspi::read(read_sfdp, 0, sfdp, sizeof sfdp);
    print(serial, "  SFDP (5Ah at 0): ", hex(sfdp[0]), " ", hex(sfdp[1]), " ", hex(sfdp[2]), " ",
          hex(sfdp[3]), " revision ", sfdp[5], ".", sfdp[4], ", ", sfdp[6] + 1u, " parameter headers",
          crlf);
    bench.verdict("the serial flash discovery table opens with 'SFDP' - a command with an address "
                  "and eight dummy cycles, the first of them here",
                  sst == QspiStatus::ok && sfdp[0] == 'S' && sfdp[1] == 'F' && sfdp[2] == 'D' &&
                      sfdp[3] == 'P');
}

// =============================================================================
// d - the device's registers
// =============================================================================

void td_registers() {
    const uint8_t sr = read_byte_register(read_status);
    const uint8_t fsr = read_byte_register(read_flag_status);
    uint8_t nvcr[2] = {};
    (void)Quadspi::read(read_nvcr, 0, nvcr, 2);
    const uint16_t nv = static_cast<uint16_t>(nvcr[0] | (nvcr[1] << 8));
    const uint8_t vcr = read_byte_register(read_vcr);
    const uint8_t evcr = read_byte_register(read_evcr);
    print(serial, "  status ", hex(sr), ", flag status ", hex(fsr), ", NVCR ", hex(nv), ", VCR ",
          hex(vcr), ", EVCR ", hex(evcr), crlf);
    bench.verdict("the status register: no write in progress, the write enable latch clear, no "
                  "block protected (table 3, as delivered: 00h)",
                  sr == 0x00u);
    bench.verdict("the flag status register: the controller ready and no error bit standing "
                  "(table 5)",
                  (fsr & fsr_ready) != 0u &&
                      (fsr & (fsr_erase_error | fsr_program_error | fsr_protection_error)) == 0u);
    bench.verdict("the nonvolatile configuration register reads FFFFh - the delivery state, never "
                  "written (and never written here)",
                  nv == 0xFFFFu);
    bench.verdict("the volatile configuration register: dummy cycles at the default, XIP disabled, "
                  "continuous wrap (table 7)",
                  vcr == vcr_default);
    bench.verdict("the enhanced volatile configuration register: quad and dual command input "
                  "disabled, single transfer rate, HOLD#/RESET# enabled, 30 ohm drive (table 11)",
                  evcr == 0xFFu);
}

// =============================================================================
// e - the last subsector: erase, program, read back
// =============================================================================

void te_subsector() {
    uint32_t last = 0;
    // WRITE ENABLE, and the latch it sets.
    bench.verdict("WRITE ENABLE sets the write enable latch (a command of one phase)",
                  Quadspi::command(write_enable) == QspiStatus::ok && write_enabled());

    // The erase, timed through the status-polling mode.
    const uint32_t t0 = micros();
    const QspiStatus est = Quadspi::command(subsector_erase, test_subsector);
    const QspiStatus pst = wait_write_done(last);
    const uint32_t erase_us = micros() - t0;
    const uint8_t fsr = read_byte_register(read_flag_status);
    print(serial, "  4 KB SUBSECTOR ERASE at ", hex(test_subsector), ": ", erase_us,
          " us until WIP fell (datasheet tSSE 50 ms typical, 400 max); the last poll read ",
          hex(last & 0xFFu), ", flag status ", hex(fsr), crlf);
    bench.verdict("the erase command goes out and the block's AUTOMATIC STATUS-POLLING MODE sees "
                  "WIP fall - the match stops the polling and BUSY comes down (13.3.6, APMS)",
                  est == QspiStatus::ok && pst == QspiStatus::ok && (last & sr_write_in_progress) == 0u);
    bench.verdict("the flag status register says ready with no erase error",
                  (fsr & fsr_ready) != 0u && (fsr & fsr_erase_error) == 0u);
    bench.verdict("the erase took between 1 ms and the datasheet's 400 ms maximum",
                  erase_us > 1000u && erase_us <= 400'000u);

    // Blank.
    uint32_t us = 0;
    const QspiStatus bst = read_subsector(fast_read, us);
    bool blank = bst == QspiStatus::ok;
    for (uint32_t i = 0; i < subsector_bytes; ++i) {
        blank = blank && buffer[i] == 0xFFu;
    }
    bench.verdict("the whole subsector reads FFh through FAST READ", blank);

    // The pages.
    uint32_t page_us_max = 0;
    uint32_t page_us_total = 0;
    bool programmed = true;
    for (uint32_t page = 0; page < subsector_bytes / page_bytes; ++page) {
        uint8_t data[page_bytes];
        for (uint32_t i = 0; i < page_bytes; ++i) {
            data[i] = pattern(page * page_bytes + i);
        }
        if (Quadspi::command(write_enable) != QspiStatus::ok) {
            programmed = false;
            break;
        }
        const uint32_t p0 = micros();
        const QspiStatus wst =
            Quadspi::write(page_program, test_subsector + page * page_bytes, data, page_bytes);
        const QspiStatus ws = wait_write_done(last);
        const uint32_t pu = micros() - p0;
        page_us_total += pu;
        if (pu > page_us_max) {
            page_us_max = pu;
        }
        if (wst != QspiStatus::ok || ws != QspiStatus::ok) {
            programmed = false;
            break;
        }
    }
    print(serial, "  16 PAGE PROGRAMs of 256 bytes: ", page_us_total / 16u, " us a page on average, ",
          page_us_max, " at most (tPP 120 us typical, 1800 max), flag status ",
          hex(read_byte_register(read_flag_status)), crlf);
    bench.verdict("sixteen pages program through the indirect write path, each with its own WRITE "
                  "ENABLE and each waited for",
                  programmed);
    bench.verdict("a page takes under the datasheet's 1800 us maximum", page_us_max <= 1800u);
    bench.verdict("the write enable latch is clear again after a program (table 3: cleared by the "
                  "device at the end of the cycle)",
                  !write_enabled());

    const QspiStatus rst = read_subsector(fast_read, us);
    uint32_t bad = 0;
    const bool exact = pattern_matches(buffer, subsector_bytes, &bad);
    print(serial, "  read back through FAST READ in ", us, " us: ", exact ? "byte-exact" : "WRONG",
          crlf);
    if (!exact) {
        print(serial, "  first wrong byte at offset ", bad, ": ", hex(buffer[bad]), " for ",
              hex(pattern(bad)), crlf);
    }
    bench.verdict("the 4 KB read back byte-exact", rst == QspiStatus::ok && exact);
}

// =============================================================================
// f - the read commands agree
// =============================================================================

void tf_reads() {
    struct Row {
        const char* name;
        const QspiCommand* cmd;
        bool slow;
    };
    const Row rows[5] = {{"READ 03h at 45 MHz (1-1-1, no dummy)", &read_slow, true},
                         {"FAST READ 0Bh (1-1-1, 8 dummies)", &fast_read, false},
                         {"DUAL OUTPUT FAST READ 3Bh (1-1-2, 8)", &dual_out_read, false},
                         {"QUAD OUTPUT FAST READ 6Bh (1-1-4, 8)", &quad_out_read, false},
                         {"QUAD I/O FAST READ EBh (1-4-4, 10)", &quad_io_read, false}};
    uint32_t us[5] = {};
    bool all_exact = true;
    bool all_ok = true;
    for (uint8_t i = 0; i < 5u; ++i) {
        if (!Quadspi::init(rows[i].slow ? slow_cfg : fast_cfg)) {
            all_ok = false;
            continue;
        }
        const QspiStatus st = read_subsector(*rows[i].cmd, us[i]);
        uint32_t bad = 0;
        const bool exact = pattern_matches(buffer, subsector_bytes, &bad);
        const uint32_t kbps = us[i] != 0u ? (subsector_bytes * 1000u) / us[i] : 0u;
        print(serial, "  ", rows[i].name, ": ", us[i], " us for 4 KB (", kbps, " KB/s), ",
              exact ? "byte-exact" : "WRONG", crlf);
        all_ok = all_ok && st == QspiStatus::ok;
        all_exact = all_exact && exact;
    }
    (void)Quadspi::init(fast_cfg);
    bench.verdict("every read command completes", all_ok);
    bench.verdict("READ, FAST READ, dual output, quad output and quad I/O all give the same 4 KB, "
                  "byte for byte",
                  all_exact);
    bench.verdict("the one-line FAST READ is the slowest of the four fast reads, the two-line one "
                  "sits between, and the two four-line ones are the fastest",
                  us[1] > us[2] && us[2] > us[3] && us[2] > us[4]);
}

// =============================================================================
// g - the dummy cycles against the clock
// =============================================================================

bool set_vcr(uint8_t value) {
    if (Quadspi::command(write_enable) != QspiStatus::ok) {
        return false;
    }
    if (Quadspi::write(write_vcr, 0, &value, 1) != QspiStatus::ok) {
        return false;
    }
    uint32_t last = 0;
    return wait_write_done(last) == QspiStatus::ok;
}

void tg_dummies() {
    // Table 9: quad I/O fast read at 90 MHz wants 7 dummy cycles (97 MHz
    // is the seventh row's ceiling); 6 cycles are good to 86 MHz. The
    // volatile configuration register sets the count for every FAST READ
    // command, the QspiCommand has to say the same, and the register is
    // put back at the end.
    struct Row {
        uint8_t dummies;
        uint32_t table_ceiling_mhz;
    };
    const Row rows[3] = {{10, 125}, {7, 97}, {6, 86}};
    bool exact[3] = {false, false, false};
    bool set_ok = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        set_ok = set_ok && set_vcr(vcr_with_dummies(rows[i].dummies));
        QspiCommand c = quad_io_read;
        c.dummy_cycles = rows[i].dummies;
        uint32_t us = 0;
        const QspiStatus st = read_subsector(c, us);
        uint32_t bad = 0;
        exact[i] = st == QspiStatus::ok && pattern_matches(buffer, subsector_bytes, &bad);
        print(serial, "  VCR dummies = ", rows[i].dummies, " (table 9: good to ",
              rows[i].table_ceiling_mhz, " MHz), quad I/O at 90 MHz: ",
              exact[i] ? "byte-exact" : "WRONG", crlf);
    }
    const bool restored = set_vcr(vcr_default) && read_byte_register(read_vcr) == vcr_default;
    uint32_t us = 0;
    const bool after = read_subsector(quad_io_read, us) == QspiStatus::ok &&
                       pattern_matches(buffer, subsector_bytes);
    bench.verdict("the volatile configuration register takes a dummy count and reads it back "
                  "(the 81h write behind a WRITE ENABLE)",
                  set_ok);
    bench.verdict("with the datasheet's default and with the seven cycles table 9 allows at 90 MHz "
                  "the quad I/O read is byte-exact",
                  exact[0] && exact[1]);
    print(serial, "  six cycles at 90 MHz are outside table 9 (86 MHz): the read came back ",
          exact[2] ? "right anyway - margin the table does not promise" : "wrong, as the table says",
          crlf);
    bench.verdict("the register is back at its default and the default read is exact again",
                  restored && after);
}

// =============================================================================
// h - the FIFO and the flags
// =============================================================================

void th_fifo() {
    // A read started by hand - the driver's own steps, without its pump -
    // so that the FIFO can be watched filling: the block reads until the
    // 32 bytes are full, then STOPS THE CLOCK until the program reads.
    Quadspi::clear_flags();
    Quadspi::regs().DLR = 64u - 1u;
    Quadspi::regs().CCR = Quadspi::ccr_word(fast_read, 1);
    Quadspi::regs().AR = test_subsector;
    uint32_t spins = 4'000'000u;
    while (Quadspi::fifo_level() < 32u && --spins != 0u) {
    }
    const uint8_t level = Quadspi::fifo_level();
    const bool ftf = Quadspi::fifo_threshold_reached();
    const bool busy = Quadspi::busy();
    print(serial, "  a 64-byte read left alone: FLEVEL ", level, ", FTF ", ftf ? "set" : "clear",
          ", BUSY ", busy ? "set" : "clear", ", TCF ",
          Quadspi::transfer_complete() ? "set" : "clear", crlf);
    bench.verdict("the FIFO fills to 32 and the block waits - BUSY stands, TCF does not, the "
                  "threshold flag is set above four bytes",
                  level == 32u && ftf && busy && !Quadspi::transfer_complete());
    // Drain the first half by hand, in words.
    uint32_t words[8];
    for (uint8_t i = 0; i < 8u; ++i) {
        words[i] = Quadspi::regs().DR;
    }
    spins = 4'000'000u;
    while (Quadspi::fifo_level() < 32u && --spins != 0u) {
    }
    bench.verdict("reading 32 bytes lets the other 32 in", Quadspi::fifo_level() == 32u);
    volatile uint8_t& dr8 = *reinterpret_cast<volatile uint8_t*>(&Quadspi::regs().DR);
    for (uint8_t i = 0; i < 32u; ++i) {
        [[maybe_unused]] const uint8_t b = dr8;
    }
    spins = 4'000'000u;
    while (!Quadspi::transfer_complete() && --spins != 0u) {
    }
    const bool done = Quadspi::transfer_complete() && !Quadspi::busy() && Quadspi::fifo_level() == 0u;
    bench.verdict("with the 64th byte out TCF is set, BUSY falls and the FIFO is empty", done);
    Quadspi::clear_flags();
    bench.verdict("CTCF clears TCF", !Quadspi::transfer_complete());
    bool first_words_right = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        const uint32_t want = static_cast<uint32_t>(pattern(i * 4u)) |
                              (static_cast<uint32_t>(pattern(i * 4u + 1u)) << 8) |
                              (static_cast<uint32_t>(pattern(i * 4u + 2u)) << 16) |
                              (static_cast<uint32_t>(pattern(i * 4u + 3u)) << 24);
        first_words_right = first_words_right && words[i] == want;
    }
    bench.verdict("a word read of DR is four bytes, the first at the bottom (13.5.9)",
                  first_words_right);

    // An abort mid-read.
    Quadspi::regs().DLR = subsector_bytes - 1u;
    Quadspi::regs().CCR = Quadspi::ccr_word(fast_read, 1);
    Quadspi::regs().AR = test_subsector;
    spins = 4'000'000u;
    while (Quadspi::fifo_level() < 8u && --spins != 0u) {
    }
    const bool was_busy = Quadspi::busy();
    const bool aborted = Quadspi::abort();
    print(serial, "  a 4 KB read aborted eight bytes in: BUSY ", was_busy ? "was up" : "was DOWN",
          ", after the abort BUSY ", Quadspi::busy() ? "SET" : "clear", ", FLEVEL ",
          Quadspi::fifo_level(), crlf);
    bench.verdict("ABORT stops a running read: BUSY and ABORT fall, the FIFO is flushed (13.3.14)",
                  was_busy && aborted && !Quadspi::busy() && Quadspi::fifo_level() == 0u);
    uint32_t us = 0;
    bench.verdict("and the next read is whole and exact",
                  read_subsector(fast_read, us) == QspiStatus::ok &&
                      pattern_matches(buffer, subsector_bytes));

    // The transfer error: past the end of the flash.
    uint8_t b[16] = {};
    const QspiStatus past = Quadspi::read(fast_read, flash_bytes, b, 16);
    const QspiStatus across = Quadspi::read(fast_read, flash_bytes - 8u, b, 16);
    const QspiStatus at_end = Quadspi::read(fast_read, flash_bytes - 16u, b, 16);
    print(serial, "  a read at the size: ", static_cast<uint32_t>(past), ", one crossing it: ",
          static_cast<uint32_t>(across), ", the last 16 bytes: ", static_cast<uint32_t>(at_end),
          " (0 ok, 2 error)", crlf);
    bench.verdict("an address at or across the flash's size raises TEF and the read reports the "
                  "error (13.3.13); the last sixteen bytes read fine",
                  past == QspiStatus::error && across == QspiStatus::error &&
                      at_end == QspiStatus::ok);
    bench.verdict("... and after the error the block is idle and the flag cleared",
                  !Quadspi::busy() && !Quadspi::transfer_error());
}

// =============================================================================
// i - the interrupt
// =============================================================================

void ti_interrupt() {
    irq_entries = 0;
    irq_saw_complete = false;
    // A flag left standing by the letter before would be an entry of its
    // own the moment the enable is written: cleared first, and said.
    const uint32_t sr_before = Quadspi::regs().SR;
    Quadspi::clear_flags();
    Nvic::clear_pending(Quadspi::irq);
    Nvic::enable(Quadspi::irq);
    Quadspi::interrupts({.transfer_complete = true});
    print(serial, "  SR before arming: ", hex(sr_before), " (cleared before the enable)", crlf);
    uint8_t b[16] = {};
    const QspiStatus st = Quadspi::read(fast_read, test_subsector, b, 16);
    wait_us(100);
    Quadspi::interrupts({});
    Nvic::disable(Quadspi::irq);
    print(serial, "  TCIE armed over a 16-byte read: ", irq_entries, " entries, the body saw TCF: ",
          irq_saw_complete ? "yes" : "no", crlf);
    bench.verdict("the vector is entered once for the completed read and the body reports the "
                  "transfer complete",
                  st == QspiStatus::ok && irq_entries == 1u && irq_saw_complete);
    bench.verdict("the bytes are the pattern", same(b, buffer, 0) && pattern_matches(b, 16));
}

// =============================================================================
// j - memory-mapped mode
// =============================================================================

void tj_mapped() {
    Quadspi::regs().AR = 3u;   // 2.4.3's condition, staged on purpose: map() must clear it
    const bool mapped = Quadspi::map(quad_io_read);
    const volatile uint8_t* w = Quadspi::window();
    const uint32_t ar_mapped = Quadspi::regs().AR;
    bench.verdict("map() opens the window, and the AR of 3 staged before it reads 0 once mapped: "
                  "ES0321 2.4.3's clear landed (an abort before it, since a write to AR is ignored "
                  "while BUSY stands)",
                  mapped && Quadspi::mapped() && w != nullptr && ar_mapped == 0u);
    if (!mapped || w == nullptr) {
        return;
    }
    const volatile uint8_t* sub = w + test_subsector;
    const uint8_t first = sub[0];
    print(serial, "  AR reads ", hex(ar_mapped), " once mapped and ", hex(Quadspi::regs().AR),
          " after the first access (of ", hex(test_subsector), "): in memory-mapped mode the "
          "register follows the bus, it is not the program's", crlf);
    bench.verdict("the first byte through the window is the pattern's", first == pattern(0));
    bool bytes_exact = true;
    for (uint32_t i = 0; i < subsector_bytes; ++i) {
        if (sub[i] != pattern(i)) {
            bytes_exact = false;
            break;
        }
    }
    bench.verdict("every byte of the subsector reads through the window as the pattern (quad I/O, "
                  "ten dummies, an EBh per access)",
                  bytes_exact);
    const bool busy_after = Quadspi::busy();
    bench.verdict("BUSY stands after a memory-mapped access: the prefetch keeps NCS low (13.3.7)",
                  busy_after);
    bool halfwords_exact = true;
    const volatile uint16_t* h = reinterpret_cast<const volatile uint16_t*>(sub);
    for (uint32_t i = 0; i < subsector_bytes / 2u; ++i) {
        const uint16_t want = static_cast<uint16_t>(pattern(2u * i) | (pattern(2u * i + 1u) << 8));
        if (h[i] != want) {
            halfwords_exact = false;
            break;
        }
    }
    bool words_exact = true;
    const volatile uint32_t* v = reinterpret_cast<const volatile uint32_t*>(sub);
    for (uint32_t i = 0; i < subsector_bytes / 4u; ++i) {
        const uint32_t want = static_cast<uint32_t>(pattern(4u * i)) |
                              (static_cast<uint32_t>(pattern(4u * i + 1u)) << 8) |
                              (static_cast<uint32_t>(pattern(4u * i + 2u)) << 16) |
                              (static_cast<uint32_t>(pattern(4u * i + 3u)) << 24);
        if (v[i] != want) {
            words_exact = false;
            break;
        }
    }
    bench.verdict("halfword and word loads through the window agree with the byte ones",
                  halfwords_exact && words_exact);
    const uint32_t t0 = micros();
    for (uint32_t i = 0; i < subsector_bytes / 4u; ++i) {
        reinterpret_cast<uint32_t*>(buffer)[i] = v[i];
    }
    const uint32_t us = micros() - t0;
    print(serial, "  4 KB copied out of the window in words: ", us, " us (",
          us != 0u ? (subsector_bytes * 1000u) / us : 0u, " KB/s)", crlf);
    bench.verdict("the copy is exact", pattern_matches(buffer, subsector_bytes));
    const bool closed = Quadspi::unmap();
    bench.verdict("unmap() aborts the prefetch: BUSY falls and NCS rises with no timeout counter "
                  "(2.4.4's way)",
                  closed && !Quadspi::busy());
    uint32_t after_us = 0;
    bench.verdict("an indirect read works again after the window is closed",
                  read_subsector(fast_read, after_us) == QspiStatus::ok &&
                      pattern_matches(buffer, subsector_bytes));
}

// =============================================================================
// k - deep power-down
// =============================================================================

void tk_deep_power_down() {
    const QspiStatus in = Quadspi::command(enter_deep_power_down);
    wait_us(10);   // tDP: 3 us from NCS high to deep power-down
    uint8_t id[3] = {};
    (void)Quadspi::read(read_id, 0, id, 3);
    print(serial, "  in deep power-down READ ID gives ", hex(id[0]), " ", hex(id[1]), " ", hex(id[2]),
          " (the device ignores every command but the release)", crlf);
    bench.verdict("ENTER DEEP POWER-DOWN goes out and the device stops answering its identity",
                  in == QspiStatus::ok && !(id[0] == 0x20u && id[1] == 0xBAu && id[2] == 0x18u));
    const QspiStatus out = Quadspi::command(release_deep_power_down);
    wait_us(40);   // tRDP: 30 us to standby
    (void)Quadspi::read(read_id, 0, id, 3);
    bench.verdict("RELEASE FROM DEEP POWER-DOWN and 30 us later the identity answers again",
                  out == QspiStatus::ok && id[0] == 0x20u && id[1] == 0xBAu && id[2] == 0x18u);
}

// =============================================================================
// the menu
// =============================================================================

void banner() {
    print(serial, crlf, "test_stm32f4_qspi - the Quad-SPI interface with the board's MT25QL128", crlf);
    bench.menu();
}

}  // namespace

// ---- the vectors ---------------------------------------------------------------------------

extern "C" void USART3_IRQHandler() { (void)Serial::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void QUADSPI_IRQHandler() {
    const brio::QspiEvents e = brio::Quadspi::isr();
    irq_entries = irq_entries + 1u;
    if (e.transfer_complete) {
        irq_saw_complete = true;
    }
}

int main() {
    boot.gate = brio::Quadspi::clock();
    brio::Quadspi::clock(true);
    boot.cr = brio::Quadspi::regs().CR;
    boot.dcr = brio::Quadspi::regs().DCR;
    boot.sr = brio::Quadspi::regs().SR;
    boot.dlr = brio::Quadspi::regs().DLR;
    boot.ccr = brio::Quadspi::regs().CCR;
    boot.ar = brio::Quadspi::regs().AR;
    boot.abr = brio::Quadspi::regs().ABR;
    boot.psmkr = brio::Quadspi::regs().PSMKR;
    boot.psmar = brio::Quadspi::regs().PSMAR;
    boot.pir = brio::Quadspi::regs().PIR;
    boot.lptr = brio::Quadspi::regs().LPTR;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    claim_pads();
    const bool qspi_ok = brio::Quadspi::init<fast_cfg>();
    const bool reset_ok = device_reset();

    bench.letter('a', "the block", ta_block);
    bench.letter('b', "what the driver refuses", tb_refusals);
    bench.letter('c', "the device's identity", tc_identity);
    bench.letter('d', "the device's registers", td_registers);
    bench.letter('e', "the last subsector: erase, program, read back", te_subsector);
    bench.letter('f', "the read commands agree", tf_reads);
    bench.letter('g', "the dummy cycles against the clock", tg_dummies);
    bench.letter('h', "the FIFO and the flags", th_fifo);
    bench.letter('i', "the interrupt", ti_interrupt);
    bench.letter('j', "memory-mapped mode", tj_mapped);
    bench.letter('k', "deep power-down", tk_deep_power_down);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", " qspi=", qspi_ok ? "up" : "FAILED",
                    " device=", reset_ok ? "reset" : "SILENT", brio::crlf);
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
