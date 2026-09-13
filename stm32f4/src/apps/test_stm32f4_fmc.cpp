// test_stm32f4_fmc - the reference bench suite for the STM32F4's
// flexible memory controller: the block, the SDRAM controller of banks 5
// and 6 against the device this board carries, and the NOR/PSRAM half
// exercised at the register level - stm32f4/fmc.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS EIGHT MEGABYTES OF DRAM SOLDERED TO THE BOARD, which
// is why this suite builds for the STM32F429I-DISC1 alone: everything a
// memory controller does is either byte-exact or it is nothing, and the
// only honest measurement of it is a march over the whole array.
//
//   the device   an ISSI IS42S16400J - 64 Mbit, 4 internal banks of
//                4096 rows x 256 columns x 16 bits, so 8 MB, on FMC
//                SDRAM bank 2 (FMC_SDNE1 / FMC_SDCKE1) at 0xD0000000
//   the clock    HCLK/2 = 90 MHz with the core at 180 MHz
//   the pads     AF12 throughout, from the board's schematic:
//                  D0..D15   PD14 PD15 PD0 PD1 PE7 PE8 PE9 PE10 PE11
//                            PE12 PE13 PE14 PE15 PD8 PD9 PD10
//                  A0..A11   PF0..PF5 PF12..PF15 PG0 PG1
//                  BA0 PG4   BA1 PG5   SDCLK PG8
//                  SDNRAS PF11   SDNCAS PG15   SDNWE PC0
//                  SDCKE1 PB5    SDNE1 PB6
//                  NBL0 PE0      NBL1 PE1
//
// None of those pads is the display's, the gyroscope's or the touch
// controller's, so this suite and the SPI and I2C ones share the board
// without touching each other's wires. The LCD is left as the boot
// found it: its controller is another chapter.
//
// THE NOR/PSRAM HALF HAS NO DEVICE HERE. Letter n configures and reads
// back its registers and never touches a window: a bank enabled over
// nothing is an address the AHB answers at with whatever the floating
// data bus holds, and there is no measurement in that. It puts every
// register it touched back where reset left it.
//
// What is exercised, letter by letter:
//   a  the block: what this part has, the gate closed at reset, and the
//      register values behind it
//   b  what the driver refuses, and that a refusal writes nothing
//   c  the initialization sequence of 37.7.3, command by command, timed
//   d  the address and data lines, one bit at a time
//   e  the whole 8 MB in three access widths, byte-exact
//   f  checkerboard and moving inversion over the whole array
//   g  the array held over ten seconds with the refresh running
//   h  the refresh error flag at the counter's floor, and its interrupt
//   i  the refresh withheld: the memory clock stopped, and how long
//      the array holds without one (by name only - the sweep doubles
//      its dark span to 102 s)
//   j  the throughput ladder against internal SRAM
//   k  a DMA memory-to-memory block through the window
//   l  self-refresh and power-down, entered and left with the data
//   m  the timing floor: each SDTR field tightened until the march fails
//   n  the NOR/PSRAM controller, registers only
//
// build: boards = f429zi
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/fmc.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the console ---------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7},
                                .rx = {'A', 10, PinFunction::af7}};
using Serial = Uart<1, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the board's memory ----------------------------------------------------------------

using Sdram = FmcSdram<2>;

/// 4096 rows (12 bits) x 256 columns (8 bits) x 4 internal banks x 16
/// bits: the IS42S16400J's shape, and 8 MB of it.
constexpr SdramConfig geometry{.columns = SdramColumns::eight,
                               .rows = SdramRows::twelve,
                               .width = SdramWidth::bits16,
                               .internal_banks = SdramInternalBanks::four,
                               .cas = SdramCas::three,
                               .write_protect = false,
                               .clock = SdramClock::hclk_div2,
                               .read_burst = true,
                               .read_pipe = SdramPipe::none};

/// The device's nanoseconds at 90 MHz (11.2 ns a cycle), rounded up:
/// tRCD 15 ns and tRP 15 ns -> 2, tRAS 42 ns -> 4, tRC 60 ns -> 6,
/// tXSR 70 ns -> 7, tMRD 2 cycles, tWR 2 cycles (which is also the
/// least the chapter's two inequalities allow beside these). Letter m
/// is what says how much of this is margin.
constexpr SdramTiming timing{.load_mode_to_active = 2,
                             .exit_self_refresh = 7,
                             .self_refresh = 4,
                             .row_cycle = 6,
                             .write_recovery = 2,
                             .row_precharge = 2,
                             .row_to_column = 2};

constexpr uint32_t sdram_hz = sdram_clock_hz(SysClock::hz, geometry.clock);
constexpr uint32_t rows = 4096;
constexpr uint32_t refresh_period_us = 64'000;
constexpr uint32_t refresh_count = sdram_refresh_count(sdram_hz, refresh_period_us, rows);
constexpr uint16_t mode_register = sdram_mode_register(SdramCas::three);

constexpr SdramInit boot_sequence{.power_up_us = 100,
                                  .refresh_cycles = 8,
                                  .mode_register = mode_register,
                                  .refresh_count = static_cast<uint16_t>(refresh_count),
                                  .both_banks = false};

constexpr uint32_t sdram_bytes = sdram_capacity_bytes(geometry);
constexpr uint32_t sdram_words = sdram_bytes / 4u;
constexpr uint32_t sdram_halfwords = sdram_bytes / 2u;

/// A slice small enough to march in a few milliseconds, for the letters
/// that march many times; and the bigger one the decay letter rewrites
/// between spans.
constexpr uint32_t slice_words = 16u * 1024u;    // 64 KB
constexpr uint32_t decay_words = 256u * 1024u;   // 1 MB

/// The pads, per port, exactly as the schematic wires them.
constexpr uint32_t pb_mask = (1u << 5) | (1u << 6);
constexpr uint32_t pc_mask = (1u << 0);
constexpr uint32_t pd_mask =
    (1u << 0) | (1u << 1) | (1u << 8) | (1u << 9) | (1u << 10) | (1u << 14) | (1u << 15);
constexpr uint32_t pe_mask = (1u << 0) | (1u << 1) | 0xFF80u;
constexpr uint32_t pf_mask = 0x003Fu | (1u << 11) | 0xF000u;
constexpr uint32_t pg_mask =
    (1u << 0) | (1u << 1) | (1u << 4) | (1u << 5) | (1u << 8) | (1u << 15);

/// A parallel bus clocked at 90 MHz wants the fastest output stage the
/// pad has; the pull-up is what the board's own reference software uses
/// and costs nothing on a line that is driven at both ends only during
/// a read.
void claim_pads() {
    constexpr PinConfig cfg{.pull = PinPull::up, .open_drain = false,
                            .speed = PinSpeed::very_high};
    Port<'B'>::configure_mask(pb_mask, PinMode::alternate, cfg, PinFunction::af12);
    Port<'C'>::configure_mask(pc_mask, PinMode::alternate, cfg, PinFunction::af12);
    Port<'D'>::configure_mask(pd_mask, PinMode::alternate, cfg, PinFunction::af12);
    Port<'E'>::configure_mask(pe_mask, PinMode::alternate, cfg, PinFunction::af12);
    Port<'F'>::configure_mask(pf_mask, PinMode::alternate, cfg, PinFunction::af12);
    Port<'G'>::configure_mask(pg_mask, PinMode::alternate, cfg, PinFunction::af12);
}

// ---- what the registers held before this program touched them ------------------------

struct BootState {
    bool gate = false;
    uint32_t sdcr1 = 0, sdcr2 = 0, sdtr1 = 0, sdtr2 = 0, sdrtr = 0, sdsr = 0;
    uint32_t bcr[4] = {0, 0, 0, 0};
    uint32_t btr[4] = {0, 0, 0, 0};
    uint32_t bwtr[4] = {0, 0, 0, 0};
    bool initialized = false;
};
BootState boot;

// ---- the rulers ------------------------------------------------------------------------

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// SysTick counts DOWN, so a later sample is a smaller number - unless
/// the period wrapped between the two, which at 1000 Hz means the
/// measurement was longer than a millisecond and is not to be trusted.
uint32_t val_delta(uint32_t first, uint32_t second) {
    return first >= second ? first - second : first + systick_period() - second;
}

/// Wall time for the spans a march takes: the kernel timebase, 1 kHz.
uint32_t millis() { return P::now(); }

void wait_ms(uint32_t ms) {
    const uint32_t start = millis();
    while (millis() - start < ms) {
    }
}

// ---- the patterns ------------------------------------------------------------------

/// A multiplicative mixer: every index gets a different word, cheaply,
/// and a swapped address line shows as a wrong word rather than as a
/// repeated one.
constexpr uint32_t golden = 0x9E37'79B9u;

inline uint32_t word_of(uint32_t index, uint32_t seed) { return index * golden + seed; }
inline uint16_t half_of(uint32_t index, uint32_t seed) {
    return static_cast<uint16_t>(index * 0x9E37u + seed);
}
inline uint8_t byte_of(uint32_t index, uint32_t seed) {
    return static_cast<uint8_t>(index * 31u + seed);
}

struct MarchResult {
    uint32_t bad = 0;
    uint32_t first_index = 0;
    uint32_t first_got = 0;
    uint32_t first_want = 0;
};

volatile uint32_t* words() { return Sdram::at<uint32_t>(0); }
volatile uint16_t* halfwords() { return Sdram::at<uint16_t>(0); }
volatile uint8_t* bytes() { return Sdram::at<uint8_t>(0); }

void fill_words(uint32_t count, uint32_t seed) {
    volatile uint32_t* m = words();
    for (uint32_t i = 0; i < count; ++i) {
        m[i] = word_of(i, seed);
    }
}

MarchResult check_words(uint32_t count, uint32_t seed) {
    MarchResult r{};
    volatile uint32_t* m = words();
    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t want = word_of(i, seed);
        const uint32_t got = m[i];
        if (got != want) {
            if (r.bad == 0u) {
                r.first_index = i;
                r.first_got = got;
                r.first_want = want;
            }
            ++r.bad;
        }
    }
    return r;
}

/// The same read, in chunks with interrupts masked - ES0206 2.3.5's
/// workaround for the FMC holding variable data. A chunk is under a
/// millisecond, so at most one SysTick is delayed and none is lost.
MarchResult check_words_masked(uint32_t count, uint32_t seed) {
    constexpr uint32_t chunk = 4096;
    MarchResult r{};
    volatile uint32_t* m = words();
    for (uint32_t base = 0; base < count; base += chunk) {
        const uint32_t end = (base + chunk < count) ? base + chunk : count;
        P::CriticalSection guard;
        for (uint32_t i = base; i < end; ++i) {
            const uint32_t want = word_of(i, seed);
            const uint32_t got = m[i];
            if (got != want) {
                if (r.bad == 0u) {
                    r.first_index = i;
                    r.first_got = got;
                    r.first_want = want;
                }
                ++r.bad;
            }
        }
    }
    return r;
}

void report(const char* what, const MarchResult& r, uint32_t stride) {
    if (r.bad == 0u) {
        print(serial, "  ", what, ": clean", crlf);
        return;
    }
    print(serial, "  ", what, ": ", r.bad, " bad, first at ",
          hex(Sdram::base + r.first_index * stride), " got ", hex(r.first_got), " want ",
          hex(r.first_want), crlf);
}

// ---- a  the block ---------------------------------------------------------------------

void ta_block() {
    bench.verdict("the AHB3 gate is closed at reset", !boot.gate);
    print(serial, "  this part: ", Fmc::static_banks, " NOR/PSRAM sub-banks, ",
          Fmc::sdram_banks, " SDRAM banks, ", Fmc::nand_banks, " NAND banks, PC Card ",
          Fmc::has_pc_card ? "yes" : "no", ", vector ",
          static_cast<uint32_t>(Fmc::irq_line), crlf);
    bench.verdict("the F429 carries the whole controller",
                  Fmc::static_banks == 4u && Fmc::sdram_banks == 2u && Fmc::nand_banks == 2u &&
                      Fmc::has_pc_card);
    print(serial, "  windows: NOR/PSRAM ", hex(Fmc::static_window(1)), "..",
          hex(Fmc::static_window(4)), " NAND ", hex(Fmc::nand_window(2)), "/",
          hex(Fmc::nand_window(3)), " PC Card ", hex(Fmc::pc_card_window()), " SDRAM ",
          hex(Fmc::sdram_window(1)), "/", hex(Fmc::sdram_window(2)), crlf);

    print(serial, "  reset: SDCR1=", hex(boot.sdcr1), " SDCR2=", hex(boot.sdcr2),
          " SDTR1=", hex(boot.sdtr1), " SDTR2=", hex(boot.sdtr2), crlf);
    bench.verdict("both SDCR reset to 0x000002D0 - write protected, CAS 1, no clock",
                  boot.sdcr1 == 0x0000'02D0u && boot.sdcr2 == 0x0000'02D0u);
    bench.verdict("both SDTR reset to 0x0FFFFFFF - every delay at its slowest",
                  boot.sdtr1 == 0x0FFF'FFFFu && boot.sdtr2 == 0x0FFF'FFFFu);
    print(serial, "  reset: SDRTR=", hex(boot.sdrtr), " SDSR=", hex(boot.sdsr), crlf);
    bench.verdict("the refresh timer is stopped and no bank is anywhere but normal",
                  boot.sdrtr == 0u && boot.sdsr == 0u);

    print(serial, "  reset: BCR1=", hex(boot.bcr[0]), " BCR2=", hex(boot.bcr[1]),
          " BCR3=", hex(boot.bcr[2]), " BCR4=", hex(boot.bcr[3]), crlf);
    bench.verdict("BCR1 comes up ENABLED as a 16-bit multiplexed NOR, BCR2..4 disabled SRAM",
                  boot.bcr[0] == 0x0000'30DBu && boot.bcr[1] == 0x0000'30D2u &&
                      boot.bcr[2] == 0x0000'30D2u && boot.bcr[3] == 0x0000'30D2u);
    bench.verdict("every BTR and BWTR resets to 0x0FFFFFFF",
                  boot.btr[0] == 0x0FFF'FFFFu && boot.btr[3] == 0x0FFF'FFFFu &&
                      boot.bwtr[0] == 0x0FFF'FFFFu && boot.bwtr[3] == 0x0FFF'FFFFu);

    print(serial, "  the device: ", sdram_bytes / (1024u * 1024u), " MB at ",
          hex(Sdram::base), ", SDCLK ", sdram_hz / 1'000'000u, " MHz, refresh COUNT ",
          refresh_count, " (", refresh_period_us / 1000u, " ms over ", rows, " rows)", crlf);
    bench.verdict("the boot sequence brought the bank up", boot.initialized);
    bench.verdict("and it is in normal mode", Sdram::state() == SdramState::normal);
}

// ---- b  what the driver refuses ---------------------------------------------------------

void tb_refusals() {
    const uint32_t cr = Sdram::control_word();
    const uint32_t tr = Sdram::timing_word();
    const uint32_t rt = Sdram::refresh_rate();

    SdramConfig bad = geometry;
    bad.rows = static_cast<SdramRows>(3);
    bench.verdict("a reserved row count is refused", !Sdram::configure(bad));
    bad = geometry;
    bad.width = static_cast<SdramWidth>(3);
    bench.verdict("a reserved bus width is refused", !Sdram::configure(bad));
    bad = geometry;
    bad.cas = static_cast<SdramCas>(0);
    bench.verdict("CAS latency 0 is refused", !Sdram::configure(bad));
    bad = geometry;
    bad.clock = static_cast<SdramClock>(1);
    bench.verdict("the reserved SDCLK code is refused", !Sdram::configure(bad));
    bad = geometry;
    bad.read_pipe = static_cast<SdramPipe>(3);
    bench.verdict("a reserved read pipe is refused", !Sdram::configure(bad));

    SdramTiming bt = timing;
    bt.load_mode_to_active = 0;
    bench.verdict("a zero-cycle delay is refused", !Sdram::timing(bt));
    bt = timing;
    bt.exit_self_refresh = 17;
    bench.verdict("a seventeen-cycle delay is refused", !Sdram::timing(bt));
    bt = timing;
    bt.row_cycle = 7;
    bench.verdict("TRC beyond TWR + TRCD + TRP is refused", !Sdram::timing(bt));
    bt = timing;
    bt.self_refresh = 6;
    bench.verdict("TRAS beyond TWR + TRCD is refused", !Sdram::timing(bt));

    bench.verdict("the reserved command code is refused",
                  !Sdram::command(static_cast<SdramCommand>(7)));
    bench.verdict("sixteen auto-refresh cycles are refused",
                  !Sdram::command(SdramCommand::auto_refresh, 16));
    bench.verdict("a mode register past thirteen bits is refused",
                  !Sdram::command(SdramCommand::load_mode, 1, 0x2000));

    bench.verdict("a refresh count below the floor of 41 is refused",
                  !Sdram::refresh_rate(40u));
    bench.verdict("a refresh count past the thirteen-bit field is refused",
                  !Sdram::refresh_rate(0x2000u));
    bench.verdict("zero is refused as a rate - the silicon will not take it either",
                  !Sdram::refresh_rate(0u));
    print(serial, "  the one COUNT these timings forbid: ", sdram_forbidden_count(timing),
          crlf);
    bench.verdict("and it is refused when the timings are given",
                  !Sdram::refresh_rate(sdram_forbidden_count(timing), timing));

    bench.verdict("nothing of that reached a register",
                  Sdram::control_word() == cr && Sdram::timing_word() == tr &&
                      Sdram::refresh_rate() == rt);

    // The static half's refusals, on a bank with no device: nothing here
    // reaches a window either.
    using B2 = FmcNorPsram<2>;
    const uint32_t b2 = B2::control_word();
    bench.verdict("the continuous clock is refused on a bank that is not bank 1",
                  !B2::configure({.burst_read = true, .continuous_clock = true}));
    bench.verdict("a multiplexed SRAM is refused",
                  !B2::configure({.memory = StaticMemory::sram, .multiplexed = true}));
    bench.verdict("a CRAM page size on something that is not a PSRAM is refused",
                  !B2::configure({.memory = StaticMemory::nor,
                                  .page_size = CramPageSize::bytes256}));
    bench.verdict("a zero data phase is refused",
                  !B2::timing({.data_phase = 0}, StaticConfig{}));
    bench.verdict("a clock divider of one is refused",
                  !B2::timing({.clock_divide = 1}, StaticConfig{}));
    bench.verdict("and the static bank's control register did not move",
                  B2::control_word() == b2);
}

// ---- c  the initialization sequence -------------------------------------------------------

void tc_sequence() {
    // Run it again from where it stands. Re-initializing a live device
    // costs it nothing: the sequence is a precharge, a refresh burst and
    // a mode-register write, and the rows keep their charge throughout.
    const uint32_t v0 = SysTick->VAL;
    const bool ok = Sdram::initialize(clock, geometry, timing, boot_sequence);
    const uint32_t whole = val_delta(v0, SysTick->VAL);
    bench.verdict("the whole of 37.7.3 runs and answers true", ok);
    print(serial, "  initialize(): ", whole, " cycles (", whole / cycles_per_us, " us), of ",
          boot_sequence.power_up_us, " us asked for the power-up delay alone", crlf);
    bench.verdict("and it costs at least the power-up delay it promises",
                  whole >= boot_sequence.power_up_us * cycles_per_us);

    // Each command on its own, with its BUSY wait.
    struct Timed {
        const char* name;
        uint32_t cycles;
        bool ok;
    };
    Timed steps[5] = {{"clock enable", 0, false},
                      {"precharge all", 0, false},
                      {"auto refresh x8", 0, false},
                      {"load mode register", 0, false},
                      {"normal", 0, false}};
    uint32_t t0 = SysTick->VAL;
    steps[0].ok = Sdram::command(SdramCommand::clock_enable);
    steps[0].cycles = val_delta(t0, SysTick->VAL);
    t0 = SysTick->VAL;
    steps[1].ok = Sdram::command(SdramCommand::precharge_all);
    steps[1].cycles = val_delta(t0, SysTick->VAL);
    t0 = SysTick->VAL;
    steps[2].ok = Sdram::command(SdramCommand::auto_refresh, 8);
    steps[2].cycles = val_delta(t0, SysTick->VAL);
    t0 = SysTick->VAL;
    steps[3].ok = Sdram::command(SdramCommand::load_mode, 1, mode_register);
    steps[3].cycles = val_delta(t0, SysTick->VAL);
    t0 = SysTick->VAL;
    steps[4].ok = Sdram::command(SdramCommand::normal);
    steps[4].cycles = val_delta(t0, SysTick->VAL);

    bool all_ok = true;
    for (const Timed& s : steps) {
        all_ok = all_ok && s.ok;
        print(serial, "  ", s.name, ": ", s.cycles, " core cycles", crlf);
    }
    bench.verdict("every command is taken", all_ok);
    print(serial, "  mode register written: ", hex(mode_register),
          " (burst length 1, sequential, CAS 3, single-location writes)", crlf);

    // What the sequence left in the registers.
    const uint32_t cr2 = Sdram::control_word();
    const uint32_t tr2 = Sdram::timing_word();
    print(serial, "  after: SDCR2=", hex(cr2), " SDTR2=", hex(tr2), " SDRTR=",
          Sdram::refresh_rate(), " SDSR=", hex(static_cast<uint32_t>(Sdram::state())), crlf);
    bench.verdict("the geometry is in SDCR2",
                  (cr2 & (FMC_SDCR1_NC | FMC_SDCR1_NR | FMC_SDCR1_MWID | FMC_SDCR1_NB |
                          FMC_SDCR1_CAS)) ==
                      ((0u << FMC_SDCR1_NC_Pos) | (1u << FMC_SDCR1_NR_Pos) |
                       (1u << FMC_SDCR1_MWID_Pos) | FMC_SDCR1_NB | (3u << FMC_SDCR1_CAS_Pos)));
    bench.verdict("and the bank is not write protected", (cr2 & FMC_SDCR1_WP) == 0u);
    // The three fields SDCR2 does not own went to SDCR1.
    const uint32_t cr1 = FmcSdram<1>::control_word();
    print(serial, "  SDCR1 (the shared three): ", hex(cr1), crlf);
    bench.verdict("SDCLK, RBURST and RPIPE are in SDCR1, not in the bank's own register",
                  ((cr1 & FMC_SDCR1_SDCLK) >> FMC_SDCR1_SDCLK_Pos) == 2u &&
                      (cr1 & FMC_SDCR1_RBURST) != 0u &&
                      (cr1 & FMC_SDCR1_RPIPE) == 0u);
    const uint32_t tr1 = FmcSdram<1>::timing_word();
    bench.verdict("TRC and TRP are in SDTR1, whichever bank asked for them",
                  ((tr1 & FMC_SDTR1_TRC) >> FMC_SDTR1_TRC_Pos) == timing.row_cycle - 1u &&
                      ((tr1 & FMC_SDTR1_TRP) >> FMC_SDTR1_TRP_Pos) == timing.row_precharge - 1u);
    bench.verdict("the refresh timer holds what the arithmetic asked for",
                  Sdram::refresh_rate() == refresh_count);

    // And the device is usable straight out of the sequence.
    fill_words(slice_words, 5);
    bench.verdict("the array answers immediately after the sequence",
                  check_words(slice_words, 5).bad == 0u);
}

// ---- d  the address and data lines -----------------------------------------------------

void td_lines() {
    volatile uint16_t* m = halfwords();

    // The data lines: sixteen walking ones at one address, plus both
    // rails. A shorted or stuck line shows here and nowhere cheaper.
    bool data_ok = true;
    for (uint8_t b = 0; b < 16; ++b) {
        const uint16_t v = static_cast<uint16_t>(1u << b);
        m[0] = v;
        if (m[0] != v) {
            data_ok = false;
            print(serial, "  data line ", b, ": wrote ", hex(v), " read ", hex(m[0]), crlf);
        }
    }
    m[0] = 0x0000;
    data_ok = data_ok && m[0] == 0x0000;
    m[0] = 0xFFFF;
    data_ok = data_ok && m[0] == 0xFFFF;
    bench.verdict("sixteen data lines, each on its own", data_ok);

    // The address lines: a unique word at every power of two, then all
    // of them read back. A missing line aliases two addresses onto each
    // other and the second write is what the first one reads.
    volatile uint32_t* w = words();
    constexpr uint8_t top_bit = 22;   // 8 MB is 1 << 23 bytes
    w[0] = 0xA5A5'0000u;
    for (uint8_t b = 2; b <= top_bit; ++b) {
        w[(1u << b) / 4u] = 0xA5A5'0000u | b;
    }
    bool addr_ok = w[0] == 0xA5A5'0000u;
    if (!addr_ok) {
        print(serial, "  offset 0 aliased: ", hex(w[0]), crlf);
    }
    for (uint8_t b = 2; b <= top_bit; ++b) {
        const uint32_t want = 0xA5A5'0000u | b;
        const uint32_t got = w[(1u << b) / 4u];
        if (got != want) {
            addr_ok = false;
            print(serial, "  address line ", b, " (offset ", hex(1u << b), "): got ",
                  hex(got), " want ", hex(want), crlf);
        }
    }
    print(serial, "  bits 2..", top_bit, " of the byte address walked (", sdram_bytes,
          " bytes)", crlf);
    bench.verdict("every address line reaches a different cell", addr_ok);

    // The top of the array answers, and it is not the bottom.
    w[0] = 0x1111'1111u;
    w[sdram_words - 1u] = 0x2222'2222u;
    bench.verdict("the last word of the device is its own cell",
                  w[0] == 0x1111'1111u && w[sdram_words - 1u] == 0x2222'2222u);
}

// ---- e  the whole array, three widths ----------------------------------------------------

void te_march() {
    uint32_t t0 = millis();
    fill_words(sdram_words, 1);
    const uint32_t write_ms = millis() - t0;
    t0 = millis();
    MarchResult r = check_words(sdram_words, 1);
    const uint32_t read_ms = millis() - t0;
    report("word march", r, 4);
    print(serial, "  ", sdram_bytes / 1024u, " KB written in ", write_ms, " ms, read in ",
          read_ms, " ms", crlf);
    bench.verdict("the whole array holds 32-bit writes", r.bad == 0u);

    volatile uint16_t* h = halfwords();
    t0 = millis();
    for (uint32_t i = 0; i < sdram_halfwords; ++i) {
        h[i] = half_of(i, 2);
    }
    const uint32_t hw_write = millis() - t0;
    uint32_t bad = 0;
    uint32_t first = 0;
    t0 = millis();
    for (uint32_t i = 0; i < sdram_halfwords; ++i) {
        if (h[i] != half_of(i, 2)) {
            if (bad == 0u) {
                first = i;
            }
            ++bad;
        }
    }
    const uint32_t hw_read = millis() - t0;
    print(serial, "  halfword march: ", bad, " bad, written in ", hw_write, " ms, read in ",
          hw_read, " ms", crlf);
    if (bad != 0u) {
        print(serial, "  first bad halfword at ", hex(Sdram::base + first * 2u), crlf);
    }
    bench.verdict("the whole array holds 16-bit writes - the device's own width", bad == 0u);

    volatile uint8_t* b = bytes();
    t0 = millis();
    for (uint32_t i = 0; i < sdram_bytes; ++i) {
        b[i] = byte_of(i, 3);
    }
    const uint32_t by_write = millis() - t0;
    bad = 0;
    t0 = millis();
    for (uint32_t i = 0; i < sdram_bytes; ++i) {
        if (b[i] != byte_of(i, 3)) {
            if (bad == 0u) {
                first = i;
            }
            ++bad;
        }
    }
    const uint32_t by_read = millis() - t0;
    print(serial, "  byte march: ", bad, " bad, written in ", by_write, " ms, read in ",
          by_read, " ms", crlf);
    bench.verdict("the whole array holds 8-bit writes - the byte lanes do their work",
                  bad == 0u);

    // The three widths see the same bytes: a word written, read back as
    // two halfwords and four bytes, little-endian throughout.
    volatile uint32_t* w = words();
    w[0] = 0x1234'5678u;
    const bool mixed = h[0] == 0x5678u && h[1] == 0x1234u && b[0] == 0x78u && b[1] == 0x56u &&
                       b[2] == 0x34u && b[3] == 0x12u;
    print(serial, "  0x12345678 as halfwords ", hex(h[0]), " ", hex(h[1]), ", as bytes ",
          hex(b[0]), " ", hex(b[1]), " ", hex(b[2]), " ", hex(b[3]), crlf);
    bench.verdict("one word, three widths, the same bytes", mixed);

    // ES0206 2.3.5: the CPU reading the FMC with interrupts live. The
    // masked read is the workaround; the live one is the measurement.
    fill_words(slice_words, 4);
    const MarchResult live = check_words(slice_words, 4);
    const MarchResult masked = check_words_masked(slice_words, 4);
    print(serial, "  ES0206 2.3.5 over ", slice_words * 4u / 1024u,
          " KB: interrupts live ", live.bad, " bad, masked in 16 KB chunks ", masked.bad,
          " bad", crlf);
    bench.verdict("the errata-compliant read is exact", masked.bad == 0u);
}

// ---- f  checkerboard and moving inversion --------------------------------------------------

void tf_patterns() {
    volatile uint32_t* w = words();

    for (uint8_t phase = 0; phase < 2; ++phase) {
        const uint32_t even = phase == 0 ? 0x5555'5555u : 0xAAAA'AAAAu;
        const uint32_t odd = phase == 0 ? 0xAAAA'AAAAu : 0x5555'5555u;
        for (uint32_t i = 0; i < sdram_words; ++i) {
            w[i] = (i & 1u) ? odd : even;
        }
        uint32_t bad = 0;
        uint32_t first = 0;
        for (uint32_t i = 0; i < sdram_words; ++i) {
            if (w[i] != ((i & 1u) ? odd : even)) {
                if (bad == 0u) {
                    first = i;
                }
                ++bad;
            }
        }
        print(serial, "  checkerboard ", phase == 0 ? "5555/AAAA" : "AAAA/5555", ": ", bad,
              " bad", crlf);
        if (bad != 0u) {
            print(serial, "  first at ", hex(Sdram::base + first * 4u), crlf);
        }
        bench.verdict(phase == 0 ? "checkerboard holds" : "and its inverse holds", bad == 0u);
    }

    // Moving inversion: fill with zeros, walk up inverting each cell and
    // checking it, then walk down doing the same. What it catches that a
    // straight march does not is a cell that a NEIGHBOUR's write
    // disturbs.
    for (uint32_t i = 0; i < sdram_words; ++i) {
        w[i] = 0;
    }
    uint32_t bad_up = 0;
    for (uint32_t i = 0; i < sdram_words; ++i) {
        if (w[i] != 0u) {
            ++bad_up;
        }
        w[i] = 0xFFFF'FFFFu;
    }
    uint32_t bad_down = 0;
    for (uint32_t i = sdram_words; i != 0u; --i) {
        if (w[i - 1u] != 0xFFFF'FFFFu) {
            ++bad_down;
        }
        w[i - 1u] = 0;
    }
    uint32_t bad_final = 0;
    for (uint32_t i = 0; i < sdram_words; ++i) {
        if (w[i] != 0u) {
            ++bad_final;
        }
    }
    print(serial, "  moving inversion: ", bad_up, " up, ", bad_down, " down, ", bad_final,
          " left over", crlf);
    bench.verdict("no cell is disturbed by its neighbour's write",
                  bad_up == 0u && bad_down == 0u && bad_final == 0u);
}

// ---- g  the refresh holds --------------------------------------------------------------

void tg_refresh_holds() {
    const uint32_t count = Sdram::refresh_rate();
    print(serial, "  COUNT=", count, " - a row every ",
          (count + 20u) * 1000u / (sdram_hz / 1'000'000u), " ns, ", rows, " rows in ",
          (count + 20u) * rows / (sdram_hz / 1000u), " ms", crlf);
    fill_words(sdram_words, 7);
    MarchResult r = check_words(sdram_words, 7);
    report("immediately", r, 4);
    bench.verdict("the array is exact as soon as it is written", r.bad == 0u);

    wait_ms(1000);
    r = check_words(sdram_words, 7);
    report("after 1 s", r, 4);
    bench.verdict("and after a second untouched", r.bad == 0u);

    wait_ms(10000);
    r = check_words(sdram_words, 7);
    report("after 11 s", r, 4);
    bench.verdict("and after eleven", r.bad == 0u);
    bench.verdict("with no refresh error raised in all that time", !Sdram::refresh_error());
}

// ---- h  the refresh error flag ------------------------------------------------------------

volatile uint32_t refresh_interrupts = 0;

void th_refresh_error() {
    bench.verdict("no refresh error stands at the working rate", !Sdram::refresh_error());

    // The floor of the field: a refresh request every 41 memory cycles,
    // 456 ns, against a device whose row cycle is six. Whether that is
    // faster than the controller can serve is the SILICON's answer and
    // not a rule, so both probes below print rather than judge.
    refresh_interrupts = 0;
    Sdram::clear_refresh_error();
    Sdram::refresh_interrupt(true);
    Fmc::enable_interrupt();
    bench.verdict("the counter's floor is accepted", Sdram::refresh_rate(41u));

    fill_words(slice_words, 11);
    const MarchResult r = check_words(slice_words, 11);
    const bool flagged_march = Sdram::refresh_error();
    const uint32_t taken_march = refresh_interrupts;
    print(serial, "  COUNT=41 under a ", slice_words * 4u / 1024u, " KB march: RE ",
          flagged_march ? "RAISED" : "clear", ", ", taken_march, " interrupts, ", r.bad,
          " bad words", crlf);
    bench.verdict("the data survives the fastest refresh the field can ask for", r.bad == 0u);

    // The other side of it: in power-down the CONTROLLER has to leave
    // the mode, precharge, refresh and go back for every request
    // (37.7.4), which is the longest a refresh can take here.
    Sdram::clear_refresh_error();
    refresh_interrupts = 0;
    (void)Sdram::power_down();
    wait_ms(100);
    const bool flagged_pd = Sdram::refresh_error();
    const uint32_t taken_pd = refresh_interrupts;
    (void)Sdram::normal();
    print(serial, "  COUNT=41 through 100 ms of power-down: RE ",
          flagged_pd ? "RAISED" : "clear", ", ", taken_pd, " interrupts", crlf);
    const MarchResult after_pd = check_words(slice_words, 11);
    bench.verdict("and it survives that too", after_pd.bad == 0u);

    Fmc::disable_interrupt();
    Sdram::refresh_interrupt(false);
    Sdram::clear_refresh_error();
    bench.verdict("the flag reads clear after CRE, whether it stood or not",
                  !Sdram::refresh_error());
    bench.verdict("the working rate goes back", Sdram::refresh_rate(refresh_count));
    print(serial, "  COUNT back to ", Sdram::refresh_rate(), crlf);
    if (!flagged_march && !flagged_pd) {
        print(serial, "  neither probe overtook the controller: the error flag and its "
                      "vector stay unmeasured on this device",
              crlf);
    }
}

// ---- i  the refresh withheld ----------------------------------------------------------

/// FMC_SDCLK's pad, read as an input: a pad in an alternate function
/// still drives its own input buffer, so this is the WIRE. Two thousand
/// samples a few core cycles apart see both levels of a 90 MHz clock
/// and only one of a stopped one. Bit 0 means a low was seen, bit 1 a
/// high.
using SdClkPad = Pin<'G', 8>;

uint32_t sdclk_levels() {
    uint32_t seen = 0;
    for (uint32_t i = 0; i < 2000u; ++i) {
        seen |= SdClkPad::read() ? 2u : 1u;
    }
    return seen;
}

void ti_decay() {
    // FIRST, WHAT 37.7.5 SAYS AND WHAT THE SILICON DOES. "If the value
    // programmed in the register is 0, no refresh is carried out"; the
    // driver refuses a COUNT below the floor of 41, so the only way to
    // put a smaller one in the register is to store it here, through
    // the resource's own register view, and read it back.
    const uint32_t running = Sdram::refresh_rate();
    Sdram::regs().SDRTR = Sdram::regs().SDRTR & ~FMC_SDRTR_COUNT;
    const uint32_t after_zero = Sdram::refresh_rate();
    Sdram::regs().SDRTR = (Sdram::regs().SDRTR & ~FMC_SDRTR_COUNT) |
                          (10u << FMC_SDRTR_COUNT_Pos);
    const uint32_t after_ten = Sdram::refresh_rate();
    (void)Sdram::refresh_rate(refresh_count);
    print(serial, "  SDRTR COUNT running=", running, ", after storing 0: ", after_zero,
          ", after storing 10: ", after_ten, ", restored: ", Sdram::refresh_rate(), crlf);
    bench.verdict("a COUNT of zero does not reach the register", after_zero != 0u);
    bench.verdict("nor does any value below the floor of 41",
                  after_ten == after_zero && after_zero < 41u);
    bench.verdict("so 37.7.5's zero cannot stop the refresh, and the driver refuses it",
                  !Sdram::refresh_rate(0u) && Sdram::refresh_rate() == refresh_count);

    // WHAT ACTUALLY STOPS THE CLOCK, measured on SDCLK's own pad in
    // three states. The refresh timer is decremented BY the memory
    // clock (37.7.5), so a stopped clock is the only thing that leaves
    // this device unrefreshed - and 37.7.5's "00: SDCLK clock disabled"
    // turns out not to be it.
    const uint32_t lv_running = sdclk_levels();
    (void)Sdram::memory_clock(SdramClock::off);
    const uint32_t lv_field_off = sdclk_levels();
    Fmc::reset();
    const uint32_t lv_in_reset = sdclk_levels();
    const bool up = Sdram::initialize(clock, geometry, timing, boot_sequence);
    print(serial, "  SDCLK's pad, levels seen (1 low, 2 high, 3 both): running ", lv_running,
          ", SDCR1.SDCLK cleared ", lv_field_off, ", the block held in reset ", lv_in_reset,
          crlf);
    bench.verdict("the pad toggles while the controller runs", lv_running == 3u);
    bench.verdict("a reset of the block stops it", lv_in_reset != 3u);
    bench.verdict("and the sequence brings the device back after that reset", up);
    if (lv_field_off == 3u) {
        print(serial, "  SDCR1.SDCLK = 00 did NOT stop the pad: the field does not gate "
                      "the clock on this silicon, and the refresh timer counts on",
              crlf);
    }
    fill_words(slice_words, 19);
    bench.verdict("the array is exact again", check_words(slice_words, 19).bad == 0u);

    // The dark span, with the controller in reset: no clock, no
    // command, no refresh. The way back is the whole sequence, which
    // costs the device's charge nothing.
    print(serial, "  1 MB written, the CONTROLLER held in reset, the array left dark", crlf);
    uint32_t span = 100;
    uint32_t first_bad_span = 0;
    MarchResult worst{};
    for (uint8_t step = 0; step < 11; ++step) {
        fill_words(decay_words, 13);
        Fmc::reset();
        wait_ms(span);
        if (!Sdram::initialize(clock, geometry, timing, boot_sequence)) {
            print(serial, "  ", span, " ms dark: the sequence did not come back", crlf);
            break;
        }
        const MarchResult r = check_words(decay_words, 13);
        print(serial, "  ", span, " ms dark: ", r.bad, " bad words");
        if (r.bad != 0u) {
            print(serial, ", first at ", hex(Sdram::base + r.first_index * 4u), " got ",
                  hex(r.first_got), " want ", hex(r.first_want));
        }
        print(serial, crlf);
        if (r.bad != 0u) {
            first_bad_span = span;
            worst = r;
            break;
        }
        span *= 2u;
    }
    if (first_bad_span != 0u) {
        print(serial, "  the array starts losing bits between ", first_bad_span / 2u,
              " ms and ", first_bad_span, " ms unrefreshed (", worst.bad, " of ",
              decay_words, " words at the upper bound)", crlf);
    } else {
        print(serial, "  1 MB still exact after ", span / 2u,
              " ms unrefreshed - the sweep ends here, not the device's memory", crlf);
    }
    bench.verdict("the memory clock and the refresh rate are both back",
                  Sdram::memory_clock() == geometry.clock &&
                      Sdram::refresh_rate() == refresh_count);
    fill_words(slice_words, 17);
    const MarchResult after = check_words(slice_words, 17);
    bench.verdict("and the array is sound again once it is refreshed", after.bad == 0u);
}

// ---- j  the throughput ladder -------------------------------------------------------------

constexpr uint32_t bench_bytes = 4096;
alignas(4) volatile uint8_t sram_a[bench_bytes];
alignas(4) volatile uint8_t sram_b[bench_bytes];

uint32_t copy_words(volatile uint32_t* dst, const volatile uint32_t* src, uint32_t count) {
    const uint32_t v0 = SysTick->VAL;
    for (uint32_t i = 0; i < count; ++i) {
        dst[i] = src[i];
    }
    return val_delta(v0, SysTick->VAL);
}

uint32_t copy_halfwords(volatile uint16_t* dst, const volatile uint16_t* src, uint32_t count) {
    const uint32_t v0 = SysTick->VAL;
    for (uint32_t i = 0; i < count; ++i) {
        dst[i] = src[i];
    }
    return val_delta(v0, SysTick->VAL);
}

void line(const char* what, uint32_t cycles) {
    const uint32_t per100 = cycles == 0u ? 0u : (bench_bytes * 100u) / cycles;
    print(serial, "  ", what, ": ", cycles, " cycles (", per100, " bytes per 100 cycles, ",
          cycles / cycles_per_us, " us)", crlf);
}

void tj_throughput() {
    for (uint32_t i = 0; i < bench_bytes; ++i) {
        sram_a[i] = static_cast<uint8_t>(i * 7u + 1u);
        sram_b[i] = 0;
    }
    volatile uint32_t* src32 = reinterpret_cast<volatile uint32_t*>(sram_a);
    volatile uint32_t* dst32 = reinterpret_cast<volatile uint32_t*>(sram_b);
    volatile uint32_t* ext = words();
    constexpr uint32_t count32 = bench_bytes / 4u;

    const uint32_t sram_to_sram = copy_words(dst32, src32, count32);
    const uint32_t sram_to_ext = copy_words(ext, src32, count32);
    const uint32_t ext_to_sram = copy_words(dst32, ext, count32);
    const uint32_t ext_to_ext = copy_words(ext + count32, ext, count32);

    line("SRAM  -> SRAM  (words)", sram_to_sram);
    line("SRAM  -> SDRAM (words)", sram_to_ext);
    line("SDRAM -> SRAM  (words)", ext_to_sram);
    line("SDRAM -> SDRAM (words)", ext_to_ext);

    bool same = true;
    for (uint32_t i = 0; i < bench_bytes; ++i) {
        if (sram_b[i] != sram_a[i]) {
            same = false;
        }
    }
    bench.verdict("what came back out of the window is what went in", same);
    bench.verdict("internal SRAM is the fastest of the four",
                  sram_to_sram <= sram_to_ext && sram_to_sram <= ext_to_sram);
    bench.verdict("and a copy inside the SDRAM is the slowest",
                  ext_to_ext >= sram_to_ext && ext_to_ext >= ext_to_sram);

    volatile uint16_t* src16 = reinterpret_cast<volatile uint16_t*>(sram_a);
    volatile uint16_t* ext16 = halfwords();
    const uint32_t half_out = copy_halfwords(ext16, src16, bench_bytes / 2u);
    print(serial, "  the beat: a 16-bit device takes a word in two memory cycles", crlf);
    line("SRAM  -> SDRAM (halfwords)", half_out);
    print(serial, "  word write ", sram_to_ext, " cycles against halfword write ", half_out,
          " cycles for the same ", bench_bytes, " bytes", crlf);
}

// ---- k  a DMA block through the window ----------------------------------------------------

using DmaBlock = Dma<2>;
using DmaWork = DmaStream<2, 0>;

bool wait_complete(uint32_t spins = 4'000'000u) {
    while (!DmaWork::flag(DmaFlag::complete)) {
        if (spins-- == 0u) {
            return false;
        }
    }
    return true;
}

/// One memory-to-memory block on a FRESH controller: the F446's stall
/// rule (docs/stm32f4/dma.md) costs nothing here and keeps the numbers
/// about the memory rather than about the stream's history.
uint32_t timed_dma(volatile void* from, volatile void* to, uint16_t count32) {
    DmaTransfer t{};
    t.peripheral = from;
    t.memory = to;
    t.count = count32;
    t.config.direction = DmaDirection::memory_to_memory;
    t.config.peripheral_increment = true;
    t.config.memory_increment = true;
    t.config.peripheral_width = DmaWidth::word;
    t.config.memory_width = DmaWidth::word;
    t.config.use_fifo = true;
    t.config.fifo_threshold = DmaFifoThreshold::full;
    t.config.memory_burst = DmaBurst::incr4;
    DmaBlock::init();
    if (!DmaWork::prepare(t)) {
        return 0;
    }
    const uint32_t v0 = SysTick->VAL;
    if (!DmaWork::trigger() || !wait_complete()) {
        return 0;
    }
    return val_delta(v0, SysTick->VAL);
}

void tk_dma() {
    constexpr uint16_t count32 = bench_bytes / 4u;
    for (uint32_t i = 0; i < bench_bytes; ++i) {
        sram_a[i] = static_cast<uint8_t>(i * 13u + 5u);
        sram_b[i] = 0;
    }
    volatile uint32_t* ext = words();
    for (uint32_t i = 0; i < count32; ++i) {
        ext[i] = 0;
    }

    const uint32_t out = timed_dma(sram_a, ext, count32);
    bench.verdict("a stream carries a block INTO the window", out != 0u);
    bool ok = true;
    volatile uint8_t* extb = bytes();
    for (uint32_t i = 0; i < bench_bytes; ++i) {
        if (extb[i] != sram_a[i]) {
            ok = false;
        }
    }
    bench.verdict("byte-exact in the device", ok);

    const uint32_t back = timed_dma(ext, sram_b, count32);
    bench.verdict("and a second one carries it back OUT", back != 0u);
    ok = true;
    for (uint32_t i = 0; i < bench_bytes; ++i) {
        if (sram_b[i] != sram_a[i]) {
            ok = false;
        }
    }
    bench.verdict("byte-exact back in internal SRAM", ok);

    line("DMA2 SRAM  -> SDRAM (words, 4-beat bursts)", out);
    line("DMA2 SDRAM -> SRAM  (words, 4-beat bursts)", back);
    const uint32_t cpu = copy_words(ext, reinterpret_cast<volatile uint32_t*>(sram_a), count32);
    line("the CPU's own loop, for comparison", cpu);
    print(serial, "  ES0206 2.3.5 gives the DMA the read the CPU cannot make safely", crlf);
}

// ---- l  self-refresh and power-down -------------------------------------------------------

void tl_low_power() {
    fill_words(slice_words, 21);

    bench.verdict("self-refresh is entered", Sdram::self_refresh());
    const SdramState in_self = Sdram::state();
    print(serial, "  after the command: MODES2=", static_cast<uint32_t>(in_self), crlf);
    bench.verdict("and the status register says so", in_self == SdramState::self_refresh);
    wait_ms(200);
    // 37.7.4: an access alone brings the device out. Read one word, then
    // ask where the bank is.
    const uint32_t peek = words()[0];
    const SdramState after_access = Sdram::state();
    print(serial, "  one read of ", hex(peek), " later: MODES2=",
          static_cast<uint32_t>(after_access), crlf);
    bench.verdict("an access alone leaves self-refresh, with no command",
                  after_access == SdramState::normal);
    bench.verdict("normal() is idempotent from there", Sdram::normal());
    MarchResult r = check_words(slice_words, 21);
    report("after 200 ms of self-refresh", r, 4);
    bench.verdict("the device kept itself alive, byte for byte", r.bad == 0u);

    bench.verdict("power-down is entered", Sdram::power_down());
    const SdramState in_pd = Sdram::state();
    print(serial, "  after the command: MODES2=", static_cast<uint32_t>(in_pd), crlf);
    bench.verdict("and the status register says so", in_pd == SdramState::power_down);
    wait_ms(200);
    bench.verdict("normal() brings it back", Sdram::normal());
    bench.verdict("and the bank is normal again", Sdram::state() == SdramState::normal);
    r = check_words(slice_words, 21);
    report("after 200 ms of power-down", r, 4);
    bench.verdict("the CONTROLLER kept it alive, byte for byte", r.bad == 0u);

    // Write protection, while we are among the things that stop writes.
    Sdram::write_protect(true);
    bench.verdict("the bank can be made read-only", Sdram::write_protect());
    Sdram::write_protect(false);
    bench.verdict("and writable again", !Sdram::write_protect());
    r = check_words(slice_words, 21);
    bench.verdict("with the data untouched throughout", r.bad == 0u);
}

// ---- m  the timing floor -------------------------------------------------------------------

/// Program `t`, then walk the device through every command the seven
/// delays govern before marching a slice: a precharge (TRP), a
/// self-refresh entry (TRAS) and its exit (TXSR), a mode-register load
/// (TMRD), and then the activates, reads and writes of the march itself
/// (TRCD, TRC, TWR). Without those four commands a march says nothing
/// about four of the seven fields. True when the slice comes back exact.
bool march_at(const SdramTiming& t, bool& refused) {
    refused = !Sdram::timing(t);
    if (refused) {
        return false;
    }
    (void)Sdram::command(SdramCommand::precharge_all);
    (void)Sdram::command(SdramCommand::self_refresh);
    (void)Sdram::command(SdramCommand::normal);
    (void)Sdram::command(SdramCommand::load_mode, 1, mode_register);
    fill_words(slice_words, 23);
    return check_words(slice_words, 23).bad == 0u;
}

void sweep(const char* name, uint8_t SdramTiming::*field) {
    SdramTiming t = timing;
    const uint8_t working = timing.*field;
    uint8_t last_good = working;
    uint8_t refused_at = 0;
    uint8_t failed_at = 0;
    for (uint8_t v = static_cast<uint8_t>(working - 1u); v >= 1u; --v) {
        t = timing;
        t.*field = v;
        bool refused = false;
        if (march_at(t, refused)) {
            last_good = v;
        } else if (refused) {
            refused_at = v;
            break;
        } else {
            failed_at = v;
            break;
        }
    }
    print(serial, "  ", name, ": ", working, " cycles in use, exact down to ", last_good);
    if (failed_at != 0u) {
        print(serial, ", the march fails at ", failed_at);
    }
    if (refused_at != 0u) {
        print(serial, ", the driver refuses ", refused_at, " and below");
    }
    print(serial, crlf);
    bool ignored = false;
    (void)march_at(timing, ignored);
}

void tm_timing_floor() {
    print(serial, "  each SDTR field alone, the others as configured: a precharge, a "
                  "self-refresh round trip, a mode-register load and a ",
          slice_words * 4u / 1024u, " KB march at ", sdram_hz / 1'000'000u, " MHz", crlf);
    sweep("TRCD (row to column)", &SdramTiming::row_to_column);
    sweep("TRP  (row precharge)", &SdramTiming::row_precharge);
    sweep("TRC  (row cycle)", &SdramTiming::row_cycle);
    sweep("TRAS (self refresh time)", &SdramTiming::self_refresh);
    sweep("TXSR (exit self refresh)", &SdramTiming::exit_self_refresh);
    sweep("TMRD (load mode to active)", &SdramTiming::load_mode_to_active);
    sweep("TWR  (write recovery)", &SdramTiming::write_recovery);

    bool refused = false;
    bench.verdict("the configured timings are restored and the slice is exact",
                  march_at(timing, refused) && !refused);
    bench.verdict("and SDTR2 holds them again", Sdram::timing_word() != 0u);
    const MarchResult r = check_words(slice_words, 23);
    bench.verdict("the whole slice, one more time", r.bad == 0u);
}

// ---- n  the static controller ----------------------------------------------------------

void tn_static() {
    using B2 = FmcNorPsram<2>;
    print(serial, "  no static memory is wired to this board: registers only, no window",
          crlf);

    constexpr StaticConfig sram_cfg{.memory = StaticMemory::sram,
                                    .width = StaticWidth::bits16,
                                    .extended_mode = true};
    constexpr StaticTiming read_t{.address_setup = 3,
                                  .address_hold = 2,
                                  .data_phase = 6,
                                  .bus_turnaround = 1,
                                  .clock_divide = 4,
                                  .data_latency = 3,
                                  .access = StaticAccess::mode_a};
    constexpr StaticTiming write_t{.address_setup = 1,
                                   .address_hold = 1,
                                   .data_phase = 3,
                                   .bus_turnaround = 1,
                                   .clock_divide = 4,
                                   .data_latency = 3,
                                   .access = StaticAccess::mode_a};

    bench.verdict("a 16-bit SRAM is described", B2::configure(sram_cfg));
    bench.verdict("with read timings", B2::timing(read_t, sram_cfg));
    bench.verdict("and write timings of its own", B2::write_timing(write_t, sram_cfg));
    const uint32_t bcr = B2::control_word();
    const uint32_t btr = B2::timing_word();
    const uint32_t bwtr = B2::write_timing_word();
    print(serial, "  BCR2=", hex(bcr), " BTR2=", hex(btr), " BWTR2=", hex(bwtr), crlf);
    bench.verdict("MTYP says SRAM and MWID says sixteen bits",
                  ((bcr & FMC_BCR1_MTYP) >> FMC_BCR1_MTYP_Pos) == 0u &&
                      ((bcr & FMC_BCR1_MWID) >> FMC_BCR1_MWID_Pos) == 1u);
    bench.verdict("the extended mode is on and the wait signal enabled, active high",
                  (bcr & FMC_BCR1_EXTMOD) != 0u && (bcr & FMC_BCR1_WAITEN) != 0u &&
                      (bcr & FMC_BCR1_WAITPOL) != 0u);
    bench.verdict("configure() leaves the bank DISABLED", (bcr & FMC_BCR1_MBKEN) == 0u);
    bench.verdict("the read timing decodes back",
                  ((btr & FMC_BTR1_ADDSET) >> FMC_BTR1_ADDSET_Pos) == read_t.address_setup &&
                      ((btr & FMC_BTR1_DATAST) >> FMC_BTR1_DATAST_Pos) == read_t.data_phase &&
                      ((btr & FMC_BTR1_CLKDIV) >> FMC_BTR1_CLKDIV_Pos) ==
                          read_t.clock_divide - 1u &&
                      ((btr & FMC_BTR1_DATLAT) >> FMC_BTR1_DATLAT_Pos) ==
                          read_t.data_latency - 2u);
    bench.verdict("and the write timing is its own",
                  ((bwtr & FMC_BWTR1_DATAST) >> FMC_BWTR1_DATAST_Pos) == write_t.data_phase);

    B2::enable();
    bench.verdict("the bank enables", B2::enabled());
    B2::disable();
    bench.verdict("and disables again", !B2::enabled());

    // The two tasks, each on a bank of its own, and the NOR's three
    // shapes. Nothing is accessed; each is released straight away.
    bench.verdict("the SRAM task takes a bank",
                  FmcSram<3>::init(StaticWidth::bits8, read_t));
    FmcSram<3>::release();
    bench.verdict("the SRAM task in the extended mode too",
                  FmcSram<3>::init(StaticWidth::bits32, read_t, write_t));
    FmcSram<3>::release();
    bench.verdict("the NOR task, asynchronous", FmcNor<4>::init(StaticWidth::bits16, read_t));
    bench.verdict("multiplexed", FmcNor<4>::init_multiplexed(StaticWidth::bits16, read_t));
    bench.verdict("and in the synchronous burst",
                  FmcNor<4>::init_burst(StaticWidth::bits16, read_t));
    FmcNor<4>::release();

    // Everything back where reset left it: this suite owns the SDRAM
    // banks and borrows the static ones.
    for (uint8_t b = 0; b < 4; ++b) {
        FmcNorPsram<1>::regs().BTCR[2u * b] = boot.bcr[b];
        FmcNorPsram<1>::regs().BTCR[2u * b + 1u] = boot.btr[b];
        FmcNorPsram<1>::write_regs().BWTR[2u * b] = boot.bwtr[b];
    }
    bench.verdict("the four static banks are back at their reset values",
                  FmcNorPsram<1>::control_word() == boot.bcr[0] &&
                      FmcNorPsram<2>::control_word() == boot.bcr[1] &&
                      FmcNorPsram<3>::control_word() == boot.bcr[2] &&
                      FmcNorPsram<4>::control_word() == boot.bcr[3]);
    bench.verdict("and the SDRAM never noticed", check_words(slice_words, 23).bad == 0u);
}

void banner() {
    print(serial, crlf, "test_stm32f4_fmc - the flexible memory controller", crlf);
    bench.menu();
}

}   // namespace

extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// The controller's one vector. Only the SDRAM's refresh error is armed
/// here; a program with a NAND on the same block would call its body
/// beside this one.
extern "C" void FMC_IRQHandler() {
    if (Sdram::refresh_isr()) {
        refresh_interrupts = refresh_interrupts + 1u;
    }
}

int main() {
    // What the silicon held before a line of this program ran. The gate
    // is read out of the RCC, so reading it does not open it; the
    // controller's own registers need the clock, and what they hold
    // behind it is their reset value.
    boot.gate = brio::Fmc::clock();
    brio::Fmc::clock(true);
    boot.sdcr1 = brio::FmcSdram<1>::control_word();
    boot.sdcr2 = brio::FmcSdram<2>::control_word();
    boot.sdtr1 = brio::FmcSdram<1>::timing_word();
    boot.sdtr2 = brio::FmcSdram<2>::timing_word();
    boot.sdrtr = brio::FmcSdram<1>::regs().SDRTR;
    boot.sdsr = brio::FmcSdram<1>::regs().SDSR;
    boot.bcr[0] = brio::FmcNorPsram<1>::control_word();
    boot.bcr[1] = brio::FmcNorPsram<2>::control_word();
    boot.bcr[2] = brio::FmcNorPsram<3>::control_word();
    boot.bcr[3] = brio::FmcNorPsram<4>::control_word();
    boot.btr[0] = brio::FmcNorPsram<1>::timing_word();
    boot.btr[1] = brio::FmcNorPsram<2>::timing_word();
    boot.btr[2] = brio::FmcNorPsram<3>::timing_word();
    boot.btr[3] = brio::FmcNorPsram<4>::timing_word();
    boot.bwtr[0] = brio::FmcNorPsram<1>::write_timing_word();
    boot.bwtr[1] = brio::FmcNorPsram<2>::write_timing_word();
    boot.bwtr[2] = brio::FmcNorPsram<3>::write_timing_word();
    boot.bwtr[3] = brio::FmcNorPsram<4>::write_timing_word();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

    // The pads first, then the sequence: a command issued into pads that
    // are still GPIO inputs reaches nothing, and a read of a bank that
    // never got its sequence hangs the machine (ES0206 2.3.3).
    claim_pads();
    boot.initialized = Sdram::initialize(clock, geometry, timing, boot_sequence);

    brio::enable_interrupts();

    bench.letter('a', "the block, and what it held at reset", ta_block);
    bench.letter('b', "what the driver refuses", tb_refusals);
    bench.letter('c', "the initialization sequence, command by command", tc_sequence);
    bench.letter('d', "the address and data lines, one bit at a time", td_lines);
    bench.letter('e', "the whole array in three access widths", te_march);
    bench.letter('f', "checkerboard and moving inversion", tf_patterns);
    bench.letter('g', "the array held over eleven seconds", tg_refresh_holds);
    bench.letter('h', "the refresh error flag and its interrupt", th_refresh_error);
    // The decay sweep doubles its dark span up to a hundred seconds and
    // only stops early if the array loses a bit, so it is asked for by
    // name rather than left to lengthen every regression run.
    bench.letter('i', "the refresh withheld: where the array decays", ti_decay, false);
    bench.letter('j', "the throughput ladder against internal SRAM", tj_throughput);
    bench.letter('k', "a DMA memory-to-memory block through the window", tk_dma);
    bench.letter('l', "self-refresh and power-down", tl_low_power);
    bench.letter('m', "the timing floor", tm_timing_floor);
    bench.letter('n', "the NOR/PSRAM controller, registers only", tn_static);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED",
                    " sdram=", boot.initialized ? "up" : "FAILED", brio::crlf);
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
