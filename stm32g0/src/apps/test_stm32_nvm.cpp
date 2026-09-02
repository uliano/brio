// test_stm32_nvm - the STM32G0's FLASH (RM0444 ch. 3) and the block heap
// over it (util/nv_heap.hpp on its THIRD silicon).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the Nucleo's own ST-LINK virtual COM
// port (USART2, PA2/PA3, AF1) and everything measured is inside the chip.
//
// WHERE IT WRITES, AND WHY THAT IS SAFE. ld/stm32g0b1re.ld gives the
// linker BANK 1 only, so the whole image lives in 0x0800_0000..0x0803_FFFF
// and physical bank 2 above it is storage. Every erase and every program
// this suite performs is in bank 2, and stm32g0/nvm_flash.hpp's bounds
// checks refuse anything else - so the running image cannot be damaged
// by a slip in a letter. Two pages at the BOTTOM of the storage bank are
// the raw scratch (the heap places blocks as high as they fit, so it
// reaches those last); the heap owns the rest up to the journal's attic,
// which this suite never touches (test_stm32_journal does).
//
// What is exercised, letter by letter:
//   a  the geometry: the size register against the chapter's tables and
//      against the partition's constants, the bank mapping including
//      nSWAP_BANK, and the linker's own boundary
//   b  the option bytes DECODED and printed, cross-checked against the
//      reset campaign's own registers - and never written
//   c  the lock: LOCK at boot, the keyed unlock, the refusals a locked
//      FLASH_CR earns. NO WRONG KEY IS EVER WRITTEN (3.3.6: that locks
//      the register until the next system reset and hard-faults)
//   d  what an erase and a program cost, the erased pattern, the
//      round trip - and 3.3.8 step 7 against 3.7.4 bit 0 on EOP
//   e  the error vocabulary, provoked on purpose: PROGERR, SIZERR,
//      PGAERR, PGSERR, and this driver's own refusals beside them
//   f  READ-WHILE-WRITE (3.3.9) MEASURED: the turns the CPU completes
//      inside a bank-2 page erase
//   g  fast programming: a whole row under one high-voltage ramp,
//      against 32 ordinary double words
//   h  the FLASH interrupt through the NVIC
//   i  util/nv_heap.hpp on this silicon: mount, alloc/append/seal,
//      find, rewrite, survival across a remount
//   w  the wear this run spent, in page erases, against the declared
//      budget. It PRINTS and then zeroes the counters, so each z run
//      reports its own budget and the letter is re-runnable
//
//   v  verify the survivors (by name only, not in z): the blocks letter
//      i wrote, after whatever happened in between - the letter to run
//      after reflashing another app and coming back
//
// THE ENDURANCE BUDGET. DS13560 table 49 gives this part 10 kcycles
// minimum per page. Letter w prints what a run cost; at the time of
// writing one z is about twenty page erases spread over half a dozen
// pages, so the busiest page is good for hundreds of runs.
//
// The suite is re-runnable in one power-on: nothing here is one-way.
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <optional>
#include <span>

#include "stm32g0/clock.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/nvm_flash.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/nv_heap.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter s lives in
//
// INLINE, and in .noinit, for the two reasons test_stm32_platform gives:
// the section must survive the crt (the linker script marks .noinit
// NOLOAD and startup neither loads nor zeroes it), and gcc gives an
// inline variable with a section attribute a COMDAT group where a plain
// one gets none - the platform's own panic_record_ is a static inline
// member, so this must be inline too or the link fails with a section
// type conflict.
//
// Its magic word is not decoration: RM0444 promises nothing about SRAM
// across a reset, so every read of this object is guarded.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x4E13;
inline constexpr uint16_t token_canary = 0xC3A5;

struct Token {
    uint16_t magic;
    uint16_t canary;
    uint8_t leg;        ///< which misstep the reset we are waiting for staged
    uint16_t pass;      ///< letter s's tally so far
    uint16_t fail;
    uint32_t sr[4];     ///< FLASH_SR right after each misstep
    uint8_t busy[4];    ///< and whether CFGBSY was still standing
    uint32_t ret[4];    ///< what provoke() answered
};
[[gnu::section(".noinit")]] inline Token token;


namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

using Led = Pin<'A', 5>;   // LD4

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The media, with a meter on it
//
// The heap runs over the real MainFlash; this wrapper only counts what
// passes through, because "the wear budget of a run" is a number the
// suite has to be able to PRINT rather than estimate. The static_asserts
// are what keep it honest: every constant is the real medium's, so
// nothing about the heap's geometry is different for being metered.
// ---------------------------------------------------------------------------
struct MeteredFlash {
    MeteredFlash() = delete;

    static constexpr uint32_t erase_size = MainFlash::erase_size;
    static constexpr uint32_t write_cell = MainFlash::write_cell;
    static constexpr uint32_t flash_end = MainFlash::flash_end;
    static constexpr uint8_t zone_count = MainFlash::zone_count;

    static std::array<FlashZone, zone_count> zones() { return MainFlash::zones(); }
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        ++reads;
        MainFlash::read(addr, dst);
    }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        ++programs;
        return MainFlash::program(addr, src);
    }
    static bool erase(uint32_t addr) {
        ++erases;
        return MainFlash::erase(addr);
    }
    static uint32_t build_id() { return MainFlash::build_id(); }

    static void clear() { reads = programs = erases = 0; }

    static inline uint32_t reads = 0;
    static inline uint32_t programs = 0;
    static inline uint32_t erases = 0;
};

static_assert(FlashMedia<MeteredFlash>);
static_assert(MeteredFlash::erase_size == 2048u);
static_assert(MeteredFlash::write_cell == 8u);

using Heap = NvHeap<MeteredFlash, 8, 2>;
Heap heap;

/// The two records letters i and v speak about.
constexpr uint16_t rec_table = 0x4E01;
constexpr uint16_t rec_short = 0x4E02;
constexpr uint32_t table_len = 300;
constexpr uint32_t short_len = 24;

/// Erases this suite performs OUTSIDE the metered media (the raw letters
/// drive stm32g0/flash.hpp directly), so that letter w can add them in.
uint32_t raw_erases = 0;

/// The two pages of the storage bank the raw letters own. They are the
/// BOTTOM of the heap's zone, which a top-down allocator reaches last.
constexpr uint32_t scratch_a =
    MainFlashPartition::address_of(MainFlashPartition::storage_base);
constexpr uint32_t scratch_b = scratch_a + Flash::page_size;

/// Set by the FLASH handler in letter h.
volatile uint16_t flash_irqs = 0;
volatile uint32_t flash_irq_flags = 0;

// ---------------------------------------------------------------------------
// A cycle-resolution stopwatch (the one test_stm32_platform uses)
// ---------------------------------------------------------------------------
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;
uint32_t cycles_to_us(uint32_t c) { return c / cycles_per_us; }

/// Wait for the console to be physically empty. Called before anything
/// that reboots the board: a ring that still holds bytes loses them, and
/// a suite that loses its own last line is unreadable.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz / 500u) {
    }
}

// ---------------------------------------------------------------------------
// Payload helpers
//
// VOLATILE for the reason every other target's storage suite pays: these
// buffers are compared against flash the CPU itself programmed through a
// side channel the optimizer cannot see, and gcc folds such a read-back
// into whatever was last stored.
// ---------------------------------------------------------------------------
volatile uint8_t out_buf[320];
volatile uint8_t in_buf[320];

uint8_t pattern(uint32_t i, uint8_t seed) {
    return static_cast<uint8_t>(seed ^ (i * 7u + 0x5Bu));
}

void fill(uint32_t len, uint8_t seed) {
    for (uint32_t i = 0; i < len; ++i) {
        out_buf[i] = pattern(i, seed);
    }
}

std::span<const uint8_t> out_span(uint32_t len) {
    return std::span<const uint8_t>(const_cast<const uint8_t*>(out_buf), len);
}
std::span<uint8_t> in_span(uint32_t len) {
    return std::span<uint8_t>(const_cast<uint8_t*>(in_buf), len);
}

/// Is the flash at `addr` fully erased over `len` bytes?
bool is_erased(uint32_t addr, uint32_t len) {
    Flash::read(addr, in_span(len));
    for (uint32_t i = 0; i < len; ++i) {
        if (in_buf[i] != 0xFFu) {
            return false;
        }
    }
    return true;
}

/// Erase a raw scratch page, counting it for letter w.
uint32_t raw_erase(uint32_t addr) {
    ++raw_erases;
    return Flash::erase_page(addr);
}

void print_mask(uint32_t m) {
    print(serial, hex(m));
    if (m == 0u) { print(serial, " (clean)"); return; }
    if (m & FlashFlag::refused) print(serial, " REFUSED");
    if (m & FlashFlag::operation_error) print(serial, " OPERR");
    if (m & FlashFlag::program_error) print(serial, " PROGERR");
    if (m & FlashFlag::write_protect_error) print(serial, " WRPERR");
    if (m & FlashFlag::alignment_error) print(serial, " PGAERR");
    if (m & FlashFlag::size_error) print(serial, " SIZERR");
    if (m & FlashFlag::sequence_error) print(serial, " PGSERR");
    if (m & FlashFlag::miss_error) print(serial, " MISSERR");
    if (m & FlashFlag::fast_error) print(serial, " FASTERR");
    if (m & FlashFlag::read_protect_error) print(serial, " RDERR");
    if (m & FlashFlag::option_error) print(serial, " OPTVERR");
}

// =============================================================================
// a - the geometry
// =============================================================================
void ta_geometry() {
    const uint32_t kb = flash_size_kb();
    print(serial, "  size register: ", kb, " KB; banks=", Flash::bank_count(),
          " pages/bank=", Flash::pages_per_bank(), " swapped=",
          Flash::banks_swapped() ? 1u : 0u, crlf);
    print(serial, "  page=", Flash::page_size, " row=", Flash::row_size,
          " cell=", Flash::cell_size, " subpage=", Flash::subpage_size, crlf);
    print(serial, "  linker rom ends at ",
          hex(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_rom_end))),
          "; storage ", hex(scratch_a), " .. ",
          hex(MainFlashPartition::address_of(MainFlashPartition::storage_end)),
          "  heap ", hex(scratch_a), " .. ",
          hex(MainFlashPartition::address_of(MainFlash::flash_end)),
          "  attic ",
          hex(MainFlashPartition::address_of(MainFlashPartition::journal_base)),
          " .. ",
          hex(MainFlashPartition::address_of(MainFlashPartition::storage_end)),
          crlf);
    print(serial, "  build id ", MainFlash::build_id(), crlf);

    // Table 11: 512 KB, two banks of 128 pages of 2 Kbytes.
    bench.verdict("the size register says 512 KB", kb == 512u);
    bench.verdict("and the part is therefore in TWO banks whatever DUAL_BANK "
                  "says (3.3.2: the 512 KB device always is)",
                  Flash::bank_count() == 2u);
    bench.verdict("128 pages per bank, 2 Kbytes each",
                  Flash::pages_per_bank() == 128u &&
                      Flash::page_count() == 256u);

    // The linker's half of the bargain: nothing may be placed in bank 2.
    const uint32_t rom_end =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_rom_end));
    bench.verdict("the linker stops at the bank boundary - no section of "
                  "this image can reach the storage",
                  rom_end == scratch_a);
    bench.verdict("so the partition accepts this silicon and this link",
                  MainFlashPartition::geometry_matches_silicon());

    // The two media are adjacent, non-overlapping, and each anchored to
    // its own flash_end.
    bench.verdict("the heap's zone stops exactly where the attic starts",
                  MainFlash::zones()[0].ceiling == MainFlashPartition::journal_base &&
                      MainFlash::zones()[0].floor == MainFlashPartition::storage_base);
    bench.verdict("the attic is the top two pages of the bank",
                  MainFlashJournalZone::zones()[0].floor ==
                          MainFlashPartition::journal_base &&
                      MainFlashJournalZone::zones()[0].ceiling ==
                          MainFlashPartition::storage_end &&
                      MainFlashPartition::journal_bytes == 2u * Flash::page_size);
    bench.verdict("the heap's map home is below the attic",
                  Heap::map_home + 2u * Heap::erase_size <=
                      MainFlashPartition::journal_base);

    // THE ADDRESSING DECISION, checked rather than trusted: the media
    // speaks OFFSETS, because NvHeap numbers erase units in a uint16_t
    // and this bank's absolute pages start at 65664.
    bench.verdict("an absolute-address media could NOT have worked here: "
                  "0x08040000 / 2048 is past the uint16_t page number",
                  (scratch_a / Flash::page_size) > 0xFFFFu &&
                      (MainFlash::flash_end / MainFlash::erase_size) <= 0xFFFFu);

    // 3.3.2: the erase is bound to the PHYSICAL bank and ignores the
    // swap, so this arithmetic is the one place nSWAP_BANK matters.
    const bool swapped = Flash::banks_swapped();
    bench.verdict("a bank-1 address erases through the bank the swap bit "
                  "says it does",
                  Flash::erase_bank_of(Flash::base) ==
                      (swapped ? FlashBank::bank2 : FlashBank::bank1));
    bench.verdict("and a storage address through the other one",
                  Flash::erase_bank_of(scratch_a) ==
                      (swapped ? FlashBank::bank1 : FlashBank::bank2));
    bench.verdict("this board is NOT swapped (nSWAP_BANK reads 1), so the "
                  "storage really is physical bank 2",
                  !swapped);

    // PNB is the page's index WITHIN its bank, which is what makes the
    // first page of the storage bank page 0 and not page 128.
    bench.verdict("PNB restarts at zero in the second bank",
                  Flash::page_of(scratch_a) == 0u &&
                      Flash::page_of(scratch_b) == 1u &&
                      Flash::page_of(Flash::base + 127u * Flash::page_size) == 127u);
}

// =============================================================================
// b - the option bytes, read and never written
// =============================================================================
void tb_options() {
    const uint32_t optr = FlashOptions::raw();
    print(serial, "  FLASH_OPTR=", hex(optr), " RDP=", hex(FlashOptions::rdp_code()),
          " (level ", static_cast<uint8_t>(FlashOptions::rdp()), ")", crlf);
    print(serial, "  BOR ", FlashOptions::bor_enabled() ? "on" : "off",
          " rise=", FlashOptions::bor_rising_level(),
          " fall=", FlashOptions::bor_falling_level(),
          "; reset on stop/standby/shutdown = ",
          FlashOptions::reset_on_stop() ? 1u : 0u,
          FlashOptions::reset_on_standby() ? 1u : 0u,
          FlashOptions::reset_on_shutdown() ? 1u : 0u, crlf);
    print(serial, "  IWDG_SW=", FlashOptions::iwdg_software() ? 1u : 0u,
          " IWDG_STOP=", FlashOptions::iwdg_runs_in_stop() ? 1u : 0u,
          " IWDG_STDBY=", FlashOptions::iwdg_runs_in_standby() ? 1u : 0u,
          " WWDG_SW=", FlashOptions::wwdg_software() ? 1u : 0u,
          " RAM parity check=", FlashOptions::ram_parity_check() ? 1u : 0u, crlf);
    print(serial, "  nBOOT_SEL=", FlashOptions::boot0_from_option() ? 1u : 0u,
          " nBOOT0=", FlashOptions::nboot0() ? 1u : 0u,
          " nBOOT1=", FlashOptions::nboot1() ? 1u : 0u,
          " NRST_MODE=", FlashOptions::nrst_mode(),
          " IRHEN=", FlashOptions::internal_reset_holder() ? 1u : 0u,
          " DUAL_BANK bit=", FlashOptions::dual_bank_bit() ? 1u : 0u,
          " nSWAP_BANK=", Flash::banks_swapped() ? 0u : 1u, crlf);

    const FlashWrpArea w1a = FlashOptions::wrp(FlashBank::bank1, 0);
    const FlashWrpArea w1b = FlashOptions::wrp(FlashBank::bank1, 1);
    const FlashWrpArea w2a = FlashOptions::wrp(FlashBank::bank2, 0);
    const FlashWrpArea w2b = FlashOptions::wrp(FlashBank::bank2, 1);
    print(serial, "  WRP1A ", w1a.start, "..", w1a.end, " WRP1B ", w1b.start,
          "..", w1b.end, " WRP2A ", w2a.start, "..", w2a.end, " WRP2B ",
          w2b.start, "..", w2b.end, crlf);
    const FlashPcropArea p1a = FlashOptions::pcrop(FlashBank::bank1, 0);
    const FlashPcropArea p2a = FlashOptions::pcrop(FlashBank::bank2, 0);
    print(serial, "  PCROP1A ", p1a.start, "..", p1a.end, " PCROP2A ",
          p2a.start, "..", p2a.end, "  PCROP_RDP=",
          FlashOptions::pcrop_erased_on_rdp_regression() ? 1u : 0u, crlf);
    print(serial, "  FLASH_SECR=", hex(FLASH->SECR), " SEC_SIZE=",
          FlashOptions::securable_pages(FlashBank::bank1), "/",
          FlashOptions::securable_pages(FlashBank::bank2), " BOOT_LOCK=",
          FlashOptions::boot_lock() ? 1u : 0u, " DBG_SWEN=",
          FlashAccel::debug_access() ? 1u : 0u, crlf);

    // A board that can be flashed at all is at RDP level 0.
    bench.verdict("read protection is level 0 - which is the only level a "
                  "board being debugged can be at",
                  FlashOptions::rdp() == FlashRdpLevel::level0 &&
                      FlashOptions::rdp_code() == 0xAAu);
    bench.verdict("the debug port is enabled in FLASH_ACR (3.5.5)",
                  FlashAccel::debug_access());
    bench.verdict("BOOT_LOCK is clear - the state ES0548 2.2.9 warns a "
                  "mismatched option write can leave a board in",
                  !FlashOptions::boot_lock());

    // Nothing is protected, which is what makes every erase in this
    // suite legal (table 14: a protected page is not erased and WRPERR
    // is set instead).
    bench.verdict("no WRP area is defined on either bank",
                  w1a.empty() && w1b.empty() && w2a.empty() && w2b.empty());
    bench.verdict("no PCROP area is defined either",
                  p1a.empty() && p2a.empty());
    bench.verdict("and no securable memory area",
                  FlashOptions::securable_pages(FlashBank::bank1) == 0u &&
                      FlashOptions::securable_pages(FlashBank::bank2) == 0u);

    // THE CROSS-CHECK. The option bytes and the RCC describe the same
    // two watchdogs from opposite ends: WWDG_SW = 0 (a hardware window
    // watchdog) is what makes the RCC hand the WWDG its bus clock at
    // reset, which test_stm32_platform letter a reads from the other
    // side.
    bench.verdict("the WWDG is the SOFTWARE one here, which is why nothing "
                  "was feeding it before this suite started",
                  FlashOptions::wwdg_software());
    bench.verdict("and the IWDG is the software one too (reset.hpp's "
                  "Iwdg::start() is what would arm it)",
                  FlashOptions::iwdg_software());

    // OPTVERR is set at every system reset when the option loader could
    // not trust what it read (3.7.4), so its absence is a statement
    // about this board's option area.
    bench.verdict("the option loader is happy: OPTVERR does not stand",
                  (Flash::status() & FlashFlag::option_error) == 0u);
}

// =============================================================================
// c - the lock
// =============================================================================
void tc_lock() {
    // NO WRONG KEY IS EVER WRITTEN HERE. 3.3.6 makes a wrong sequence a
    // bus error - a HardFault - AND locks FLASH_CR until the next system
    // reset, so the only thing a letter could learn from provoking it is
    // that the board reboots. The claim is stated in flash.hpp and left
    // to the document.
    const bool locked_at_entry = Flash::locked();
    print(serial, "  at entry FLASH_CR is ", locked_at_entry ? "LOCKED" : "open",
          ", OPTLOCK ", Flash::option_locked() ? "set" : "clear", crlf);

    bench.verdict("FLASH_CR is locked when no operation is in flight - every "
                  "verb in nvm_flash.hpp opens it for its own microseconds "
                  "and shuts it again",
                  locked_at_entry);
    bench.verdict("the option bytes are locked on top of that (OPTLOCK), and "
                  "this stratum has no verb that could open them",
                  Flash::option_locked());

    // A locked FLASH_CR refuses, and refuses SOFTLY: nothing is written
    // and the return says so.
    const uint32_t refused_erase = Flash::erase_page(scratch_a);
    const uint32_t refused_prog = Flash::program(scratch_a, out_span(8));
    bench.verdict("an erase through a locked FLASH_CR is refused before the "
                  "silicon is touched", refused_erase == FlashFlag::refused);
    bench.verdict("and so is a program", refused_prog == FlashFlag::refused);

    bench.verdict("KEY1 then KEY2 opens it", Flash::unlock() && !Flash::locked());
    bench.verdict("a second unlock() is a no-op and NOT a second key pair - "
                  "writing a key into an open KEYR is itself the wrong "
                  "sequence", Flash::unlock() && !Flash::locked());
    bench.verdict("lock() closes it again", Flash::lock() && Flash::locked());

    // The driver's own refusals, all before any silicon is touched.
    (void)Flash::unlock();
    bench.verdict("a misaligned page erase is refused",
                  Flash::erase_page(scratch_a + 4u) == FlashFlag::refused);
    bench.verdict("an erase outside the flash is refused",
                  Flash::erase_page(0x2000'0000UL) == FlashFlag::refused);
    bench.verdict("a program at an address that is not a double word is "
                  "refused", Flash::program(scratch_a + 4u, out_span(8)) ==
                                 FlashFlag::refused);
    bench.verdict("a program of a size that is not a whole number of double "
                  "words is refused",
                  Flash::program(scratch_a, out_span(6)) == FlashFlag::refused);
    bench.verdict("a fast row at an address that is not a row is refused",
                  Flash::fast_program_row(scratch_a + 8u,
                                          out_span(Flash::row_size)) ==
                      FlashFlag::refused);
    (void)Flash::lock();

    // And the media's own bounds: the image is not reachable through it.
    bench.verdict("the storage media refuses to erase anything below its "
                  "floor - the running image is out of reach BY BOUNDS, not "
                  "only by convention",
                  !MainFlash::erase(0u) &&
                      !MainFlash::program(0u, out_span(8)));
    bench.verdict("and refuses the attic, which belongs to the journal",
                  !MainFlash::erase(MainFlashPartition::journal_base));
}

// =============================================================================
// d - what it costs, and the EOP dispute
// =============================================================================
void td_cost() {
    (void)Flash::unlock();

    uint32_t t0 = cycles_now();
    const uint32_t err_erase = raw_erase(scratch_a);
    const uint32_t erase_cycles = cycles_now() - t0;

    bench.verdict("a page erase reports no error", err_erase == 0u);
    bench.verdict("and the page reads as erased, every byte",
                  is_erased(scratch_a, 256));
    print(serial, "  page erase ", cycles_to_us(erase_cycles),
          " us (DS13560 table 48: 22.0 typ, 40.0 max ms... in MICROseconds "
          "22000/40000)", crlf);
    bench.verdict("the erase lands inside the datasheet's typ..max band",
                  cycles_to_us(erase_cycles) >= 15'000u &&
                      cycles_to_us(erase_cycles) <= 45'000u);

    // One double word, then eight of them: the per-cell cost and the
    // fixed overhead separate themselves.
    fill(64, 0x11);
    t0 = cycles_now();
    const uint32_t err_one = Flash::program(scratch_a, out_span(8));
    const uint32_t one_cycles = cycles_now() - t0;

    t0 = cycles_now();
    const uint32_t err_eight = Flash::program(scratch_a + 64u, out_span(64));
    const uint32_t eight_cycles = cycles_now() - t0;

    bench.verdict("one double word programs clean", err_one == 0u);
    bench.verdict("and eight in one call too", err_eight == 0u);
    print(serial, "  program: 1 double word ", cycles_to_us(one_cycles),
          " us, 8 of them ", cycles_to_us(eight_cycles), " us (",
          cycles_to_us(eight_cycles) / 8u, " us each; DS13560 tprog 85 typ / "
          "125 max)", crlf);
    bench.verdict("a double word costs what table 48 says (85 us typ, 125 max)",
                  cycles_to_us(one_cycles) >= 50u && cycles_to_us(one_cycles) <= 200u);
    bench.verdict("and eight cost eight times it, so the ramp is per double "
                  "word and not per call",
                  cycles_to_us(eight_cycles) >= 6u * cycles_to_us(one_cycles));

    // The round trip. Reading through Flash::read, i.e. as a load from
    // the mapped address, which is all a read of flash ever is.
    Flash::read(scratch_a + 64u, in_span(64));
    bool exact = true;
    for (uint32_t i = 0; i < 64u; ++i) {
        exact = exact && (in_buf[i] == pattern(i, 0x11));
    }
    bench.verdict("the 64 bytes read back byte for byte", exact);
    bench.verdict("and the cells NOT written are still erased",
                  is_erased(scratch_a + 128u, 64));

    // THE DOCUMENTARY DISPUTE. 3.3.8 step 7 tells the programmer to
    // check EOP after every program; 3.7.4 bit 0 says EOP "is set only
    // if the end of operation interrupts are enabled (EOPIE=1)". They
    // cannot both be right, and the answer decides whether a polled
    // driver may wait on EOP at all. The NVIC line is NOT enabled here,
    // so this measures the FLAG and not the interrupt.
    (void)Flash::interrupts(false, false, false);
    const uint32_t err_no_eopie = Flash::program(scratch_a + 192u, out_span(8));
    const uint32_t sr_no_eopie = Flash::last_status();

    (void)Flash::interrupts(true, false, false);
    const uint32_t err_eopie = Flash::program(scratch_a + 200u, out_span(8));
    const uint32_t sr_eopie = Flash::last_status();
    (void)Flash::interrupts(false, false, false);
    Flash::clear_errors();

    print(serial, "  EOP: with EOPIE clear SR=", hex(sr_no_eopie),
          ", with EOPIE set SR=", hex(sr_eopie), crlf);
    bench.verdict("both programs succeeded whatever EOP did",
                  err_no_eopie == 0u && err_eopie == 0u);
    bench.verdict("3.7.4 wins over 3.3.8 step 7: EOP does NOT rise with EOPIE "
                  "clear, so a polled driver that waits for it waits for "
                  "ever - flash.hpp judges by CFGBSY and the error bits",
                  (sr_no_eopie & FlashFlag::eop) == 0u);
    bench.verdict("and it DOES rise with EOPIE set, with no interrupt enabled "
                  "in the NVIC to take it",
                  (sr_eopie & FlashFlag::eop) != 0u);

    (void)Flash::lock();
}

// =============================================================================
// e - the error vocabulary, provoked on purpose
// =============================================================================
void te_errors() {
    (void)Flash::unlock();
    (void)raw_erase(scratch_b);

    // PROGERR: a cell written once cannot take a second, non-zero value
    // (3.3.8). This is the silicon stating util/nv_heap.hpp's write_cell
    // contract from its own side.
    fill(8, 0x33);
    const uint32_t first = Flash::program(scratch_b, out_span(8));
    fill(8, 0x77);
    const uint32_t second = Flash::program(scratch_b, out_span(8));
    print(serial, "  second write of a written cell -> ");
    print_mask(second);
    print(serial, crlf);
    bench.verdict("the first write of an erased cell is clean", first == 0u);
    bench.verdict("the second raises PROGERR - a cell is programmed ONCE "
                  "between erases, which is exactly the FlashMedia contract",
                  (second & FlashFlag::program_error) != 0u);
    // 3.3.8's "PGSERR is set also if PROGERR ... is set due to a PREVIOUS
    // programming error" is about an error STILL STANDING when the next
    // operation starts - which is why both sequences in the chapter open
    // with "check and clear all error flags". flash.hpp does that as its
    // own first act, so a clean caller sees the real cause ALONE.
    bench.verdict("PROGERR comes ALONE: PGSERR is the punishment for starting "
                  "an operation with an old error still standing, and this "
                  "file clears them first (3.3.8 step 2)",
                  (second & FlashFlag::sequence_error) == 0u);
    bench.verdict("the flash still holds the FIRST value: a refused program "
                  "changes nothing",
                  [] {
                      Flash::read(scratch_b, in_span(8));
                      for (uint32_t i = 0; i < 8u; ++i) {
                          if (in_buf[i] != pattern(i, 0x33)) return false;
                      }
                      return true;
                  }());

    // ERRATUM ES0548 2.2.3 is UNREACHABLE from anything built on this
    // file, and the reason is the verdict above: nothing here ever asks
    // for 3.3.8's all-zeros exception, because nothing programs a cell
    // twice. The erratum is stated in the header and in the doc; it is
    // not staged, because staging it means deliberately writing a cell
    // twice and the answer is already known to be "it fails".

    // THE OTHER THREE ERRORS OF 3.3.8 ARE NOT STAGED HERE, and the
    // reason is a measurement rather than caution: a misstep that sends
    // the flash half a double word leaves CFGBSY standing until the next
    // system reset, so provoking one ENDS the session. They are letter
    // s's, one per boot, with a reset behind each.
    print(serial, "  (SIZERR, PGAERR and PGSERR are letter s's: each costs a "
          "reboot)", crlf);

    // A clean program still works afterwards: the page is usable, the
    // engine is not wedged.
    (void)raw_erase(scratch_b);
    fill(8, 0x5A);
    bench.verdict("after all of that the engine still programs normally",
                  Flash::program(scratch_b, out_span(8)) == 0u);
    (void)Flash::lock();
}

// =============================================================================
// f - read-while-write, measured
// =============================================================================
void tf_rww() {
    (void)Flash::unlock();

    // THE WITNESS IS THE TURN COUNT, not the duration. A stalled CPU
    // takes just as long to see the erase finish as a running one does;
    // what it cannot do is execute anything while it waits. flash.hpp's
    // wait loop lives in bank 1 and counts its own turns, so the number
    // below is the number of times the CPU fetched, decoded and executed
    // that loop while bank 2 was being erased.
    const uint32_t t0 = cycles_now();
    const uint32_t err = raw_erase(scratch_a);
    const uint32_t cycles = cycles_now() - t0;
    const uint32_t turns = Flash::last_wait_turns();

    print(serial, "  bank-2 page erase: ", cycles_to_us(cycles), " us, ",
          turns, " turns of a bank-1 polling loop (",
          turns == 0u ? 0u : cycles / turns, " cycles per turn)", crlf);

    bench.verdict("the erase succeeded", err == 0u);
    bench.verdict("READ-WHILE-WRITE IS REAL (3.3.9): the CPU executed tens of "
                  "thousands of instructions out of bank 1 while bank 2 was "
                  "being erased - a stalled bus would have allowed none",
                  turns > 10'000u);
    bench.verdict("and the turns account for the whole duration at a "
                  "plausible few cycles each, so nothing was lost to a stall "
                  "in the middle",
                  turns != 0u && (cycles / turns) <= 32u);

    // A CONTROL IS NOT POSSIBLE and the letter says so rather than
    // pretending: the only same-bank erase available is an erase of the
    // bank the image runs from, and the honest version of that
    // experiment ends the session. What 3.3.6 promises for that case is
    // recorded in the doc, not measured here.
    print(serial, "  (no control: the only same-bank erase would be an erase "
          "of the running image)", crlf);

    // The same claim on the PROGRAM side, where each double word is a
    // separate high-voltage pulse and the wait is per cell.
    fill(8, 0x2C);
    (void)Flash::program(scratch_a, out_span(8));
    const uint32_t prog_turns = Flash::last_wait_turns();
    print(serial, "  bank-2 double-word program: ", prog_turns,
          " turns for the last cell", crlf);
    bench.verdict("a program in bank 2 leaves bank 1 running too",
                  prog_turns > 100u);

    (void)Flash::lock();
}

// =============================================================================
// g - fast programming
// =============================================================================
void tg_fast() {
    (void)Flash::unlock();
    (void)raw_erase(scratch_b);

    for (uint32_t i = 0; i < Flash::row_size; ++i) {
        out_buf[i] = pattern(i, 0xA1);
    }

    // THE CALLER MASKS. 3.3.8: the 32 double words must arrive at most
    // ~20 us apart or MISSERR stops the row, and the whole row must fit
    // in the silicon's 7 ms time-out or FASTERR does. An interrupt in
    // the middle is exactly what those two flags are for, so the row is
    // written with interrupts off - flash.hpp does not do it for the
    // caller, because a driver that masks behind the application's back
    // decides the system's latency.
    // THE STOPWATCH HAS TO CHANGE FOR THIS ONE MEASUREMENT, and the
    // first version of the letter paid for not noticing: cycles_now()
    // is ticks x period + phase, and inside a critical section SysTick's
    // INTERRUPT is masked, so the tick count freezes while the counter
    // keeps wrapping - the sum then goes BACKWARDS every time the
    // window happens to straddle a millisecond, which is exactly the
    // one-run-in-three failure it produced. SysTick's own VAL, with the
    // single wrap folded in, is valid with interrupts masked and good
    // for one whole tick period; the row takes well under that, and the
    // verdict below refuses to believe a figure that says otherwise.
    const uint32_t reload = SysTick->LOAD;
    uint32_t fast_err = 0;
    uint32_t v0 = 0;
    uint32_t v1 = 0;
    {
        Stm32Platform::CriticalSection cs;
        v0 = SysTick->VAL;
        fast_err = Flash::fast_program_row(scratch_b, out_span(Flash::row_size));
        v1 = SysTick->VAL;
    }
    const uint32_t fast_cycles =
        v0 >= v1 ? v0 - v1 : v0 + reload + 1u - v1;

    bool fast_exact = true;
    Flash::read(scratch_b, in_span(Flash::row_size));
    for (uint32_t i = 0; i < Flash::row_size; ++i) {
        fast_exact = fast_exact && (in_buf[i] == pattern(i, 0xA1));
    }

    // The same 256 bytes the ordinary way, in the next row.
    const uint32_t t1 = cycles_now();
    const uint32_t slow_err =
        Flash::program(scratch_b + Flash::row_size, out_span(Flash::row_size));
    const uint32_t slow_cycles = cycles_now() - t1;

    print(serial, "  row of 32 double words: FSTPG ", cycles_to_us(fast_cycles),
          " us -> ");
    print_mask(fast_err);
    print(serial, ", standard ", cycles_to_us(slow_cycles), " us -> ");
    print_mask(slow_err);
    print(serial, "  (DS13560 tprog_row 1.7 fast / 2.7 normal ms typ)", crlf);

    bench.verdict("a fast row programs clean under a critical section",
                  fast_err == 0u);
    bench.verdict("and every one of its 256 bytes is what was handed in",
                  fast_exact);
    bench.verdict("the ordinary path programs the same row size clean too",
                  slow_err == 0u);
    bench.verdict("FAST IS FASTER, which is the whole reason the mode exists: "
                  "one high-voltage ramp for 32 double words instead of 32",
                  fast_cycles < slow_cycles);
    bench.verdict("the ORDINARY row is what table 48 says a row costs in "
                  "normal programming - 2.7 ms typ, 4.6 max",
                  cycles_to_us(slow_cycles) >= 2'000u &&
                      cycles_to_us(slow_cycles) <= 4'600u);
    // THE FAST ROW BEATS ITS OWN DATASHEET, and by a factor: table 48
    // gives 1.7 ms typ for a fast row and this silicon does it in a
    // third of that. The number that explains it is in 3.3.8's own note
    // - "maximum time between two double words write requests is the
    // time programming (around 20 us)" - and 32 x 20 us is 640. So the
    // datasheet's typical is conservative and the note is exact.
    bench.verdict("and the FAST row comes in at about 20 us per double word, "
                  "which is 3.3.8's own figure and well under table 48's "
                  "1.7 ms typical",
                  cycles_to_us(fast_cycles) >= 400u &&
                      cycles_to_us(fast_cycles) <= 999u);

    // The refusals this mode has of its own: a partial row is not a row.
    bench.verdict("a fast program of anything but a whole row is refused",
                  Flash::fast_program_row(scratch_b, out_span(8)) ==
                      FlashFlag::refused);

    (void)Flash::lock();
}

// =============================================================================
// h - the interrupt
// =============================================================================
void th_interrupt() {
    (void)Flash::unlock();
    flash_irqs = 0;
    flash_irq_flags = 0;

    Nvic::clear_pending(Flash::irq());
    Nvic::enable(Flash::irq());
    (void)Flash::interrupts(true, true, false);

    const uint32_t err = raw_erase(scratch_a);

    // The handler has already run by the time the erase returns: it
    // fires the moment EOP rises, which is before CFGBSY falls back.
    const uint16_t seen = flash_irqs;
    const uint32_t flags = flash_irq_flags;

    (void)Flash::interrupts(false, false, false);
    Nvic::disable(Flash::irq());
    Flash::clear_errors();

    print(serial, "  FLASH_IRQHandler ran ", seen, " time(s), flags=");
    print_mask(flags);
    print(serial, crlf);

    bench.verdict("the erase succeeded", err == 0u);
    bench.verdict("the FLASH line reached the NVIC and the handler ran",
                  seen >= 1u);
    bench.verdict("and what it was given to clear was EOP",
                  (flags & FlashFlag::eop) != 0u);
    bench.verdict("no error flag came with it",
                  (flags & FlashFlag::errors) == 0u);

    // With the enables down again the same operation raises nothing.
    flash_irqs = 0;
    Nvic::enable(Flash::irq());
    (void)raw_erase(scratch_a);
    const uint16_t after = flash_irqs;
    Nvic::disable(Flash::irq());
    bench.verdict("with EOPIE clear the same erase raises no interrupt at "
                  "all - the enable bit gates the FLAG, not just the line",
                  after == 0u);

    (void)Flash::lock();
}

// =============================================================================
// i - util/nv_heap.hpp on this silicon
// =============================================================================
void ti_heap() {
    const uint32_t erases0 = MeteredFlash::erases;
    const uint32_t programs0 = MeteredFlash::programs;

    const Heap::MountReport& r = heap.mount();
    print(serial, "  mount: status=", static_cast<uint8_t>(r.status),
          " survivors=", r.survivors, " lost=", r.lost, " seq=", r.seq,
          " build=", r.build_id, " page=", heap.map_page(),
          " free=", heap.free_pages(0), " pages", crlf);

    bench.verdict("the heap mounts", r.mounted());
    bench.verdict("A MOUNT IS READ-ONLY: booting costs no erase and no "
                  "program at all",
                  MeteredFlash::erases == erases0 &&
                      MeteredFlash::programs == programs0);

    // A block bigger than one page, so the placement really has to
    // reserve two erase units and the payload spans them.
    fill(table_len, 0x6D);
    bool wrote = false;
    uint32_t table_addr = 0;
    if (auto w = heap.alloc(rec_table, table_len)) {
        table_addr = w->address();
        wrote = w->append(out_span(table_len)) && w->seal();
    }
    bench.verdict("a block is allocated, filled and sealed", wrote);

    const std::optional<NvBlock<MeteredFlash>> found = heap.find(rec_table);
    bench.verdict("and find() hands it back with its length",
                  found && found->length == table_len);
    bool exact = false;
    if (found) {
        exact = found->read(0, in_span(table_len));
        for (uint32_t i = 0; i < table_len && exact; ++i) {
            exact = in_buf[i] == pattern(i, 0x6D);
        }
    }
    bench.verdict("every byte of it reads back from the flash", exact);
    print(serial, "  block ", hex(rec_table), " at ",
          hex(MainFlashPartition::address_of(table_addr)), ", ", table_len,
          " bytes in ", (table_len + Heap::erase_size - 1u) / Heap::erase_size,
          " page(s)", crlf);
    bench.verdict("it was placed inside the heap's zone and nowhere near the "
                  "attic or the image",
                  table_addr >= MainFlashPartition::storage_base &&
                      table_addr + table_len <= Heap::map_home);

    // A second, small block: two live records at once.
    fill(short_len, 0x2E);
    bool wrote2 = false;
    if (auto w = heap.alloc(rec_short, short_len)) {
        wrote2 = w->append(out_span(short_len)) && w->seal();
    }
    // AT LEAST two: test_stm32_journal's coexistence letter writes a
    // block of its own into this same heap, which is the partition
    // working rather than interference.
    bench.verdict("a second block lives beside the first",
                  wrote2 && heap.count() >= 2u && heap.find(rec_short));

    // A REMOUNT is the only honest verification: everything above could
    // have been answered out of the RAM index.
    const uint32_t seq_before = heap.sequence();
    const Heap::MountReport& r2 = heap.mount();
    bench.verdict("a fresh mount finds both blocks and loses neither",
                  r2.status == NvHeapStatus::ok && r2.survivors >= 2u &&
                      r2.lost == 0u && heap.find(rec_table) &&
                      heap.find(rec_short));
    bench.verdict("the map's sequence number is the one the last publish left",
                  r2.seq == seq_before);
    bench.verdict("and the map version records the image that wrote it",
                  r2.build_id == MainFlash::build_id());

    bool still = false;
    if (const auto f2 = heap.find(rec_table)) {
        still = f2->read(0, in_span(table_len));
        for (uint32_t i = 0; i < table_len && still; ++i) {
            still = in_buf[i] == pattern(i, 0x6D);
        }
    }
    bench.verdict("the big block's payload survives the remount byte for byte",
                  still);

    // rewrite(): same address, same reservation, new contents.
    fill(short_len, 0x91);
    bool rewrote = false;
    if (auto w = heap.rewrite(rec_short)) {
        rewrote = w->append(out_span(short_len)) && w->seal();
    }
    bool rew_ok = false;
    if (const auto f3 = heap.find(rec_short)) {
        rew_ok = f3->read(0, in_span(short_len));
        for (uint32_t i = 0; i < short_len && rew_ok; ++i) {
            rew_ok = in_buf[i] == pattern(i, 0x91);
        }
    }
    bench.verdict("rewrite() replaces a block in place", rewrote && rew_ok);

    // The map rotation: a mutation lands on the OTHER map page each time,
    // which is what spreads its wear and makes the publish atomic.
    const uint8_t page_a = heap.map_page();
    fill(short_len, 0xC4);
    if (auto w = heap.rewrite(rec_short)) {
        (void)(w->append(out_span(short_len)) && w->seal());
    }
    const uint8_t page_b = heap.map_page();
    print(serial, "  map versions ping-pong: page ", page_a, " -> ", page_b,
          ", seq now ", heap.sequence(), crlf);
    bench.verdict("the map pair really ping-pongs", page_a != page_b);

    // A refusal that costs nothing: no gap in the zone can take it.
    bench.verdict("a block larger than the whole zone is refused, and "
                  "nothing is erased for it",
                  !heap.alloc(0x4E09, 400u * 1024u).has_value());

    print(serial, "  this letter cost ", MeteredFlash::erases - erases0,
          " page erase(s) and ", MeteredFlash::programs - programs0,
          " program(s)", crlf);
}

// =============================================================================
// s - the malformed programs of 3.3.8, one per boot
//
// THIS LETTER REBOOTS THE BOARD, four times, which is why it is not in
// `z`: `z` has to be one console session a tool can judge from a single
// capture. Run it with
//     python3 tools/bench.py run E s --app test_stm32_nvm
//             --expect="pass," --timeout 250
//
// WHY A REBOOT PER MISSTEP, and it is a finding rather than caution.
// 3.3.8 names three ways to get a program wrong and gives each a flag;
// what the chapter does not say is that WHETHER THE ENGINE COMES BACK
// depends on the LENGTH of what was sent and not on the offence. 3.7.4
// makes CFGBSY fall only when a complete double word has gone into the
// flash - so a misstep that sends eight bytes' worth of accesses in the
// wrong shape raises its flag and is over, while one that sends a legal
// first word and no second raises NOTHING and leaves the flash
// interface dead until the next system reset. Legs 1..3 are the first
// kind, leg 4 is the second, and the last boot proves a reset really is
// enough.
// =============================================================================
constexpr uint8_t misstep_count = 4;

void leg_reset(uint8_t next_leg) {
    token.leg = next_leg;
    print(serial, "  resetting...", crlf);
    console_drain();
    Reset::software();
    for (;;) {
    }
}

void stage_misstep(uint8_t leg) {
    static const Flash::Misstep steps[misstep_count] = {
        Flash::Misstep::half_word_store,
        Flash::Misstep::unaligned_double_word,
        Flash::Misstep::store_without_pg,
        Flash::Misstep::half_double_word,
    };
    const uint8_t i = static_cast<uint8_t>(leg - 1u);
    (void)Flash::unlock();
    (void)raw_erase(scratch_b);
    token.ret[i] = Flash::provoke(steps[i], scratch_b + 512u * i);
    token.sr[i] = Flash::last_status();
    token.busy[i] = Flash::config_busy() ? 1u : 0u;
    leg_reset(static_cast<uint8_t>(leg + 1u));
}

const char* misstep_name(uint8_t i) {
    switch (i) {
    case 0: return "eight bytes in four HALF-WORD accesses";
    case 1: return "two words straddling the double-word boundary";
    case 2: return "a whole double word with PG and FSTPG clear";
    default: return "a legal FIRST word and no second";
    }
}

void ts_missteps() {
    token.magic = token_magic;
    token.canary = token_canary;
    token.pass = 0;
    token.fail = 0;
    for (uint8_t i = 0; i < misstep_count; ++i) {
        token.sr[i] = 0;
        token.busy[i] = 0;
        token.ret[i] = 0;
    }
    bench.reset_tally();
    print(serial, "  four malformed programs, one per boot (3.3.8)", crlf);
    stage_misstep(1);
}

void ts_resume() {
    bench.resume_tally(token.pass, token.fail);
    const uint8_t leg = token.leg;

    // Report the misstep the previous boot staged.
    const uint8_t i = static_cast<uint8_t>(leg - 2u);
    print(serial, "  ", misstep_name(i), " -> SR=", hex(token.sr[i]), " ret=",
          hex(token.ret[i]), " CFGBSY-after=", token.busy[i], " (");
    print_mask(token.sr[i] & FlashFlag::errors);
    print(serial, ")", crlf);

    if (leg <= misstep_count) {
        static const uint32_t wanted[3] = {FlashFlag::size_error,
                                           FlashFlag::alignment_error,
                                           FlashFlag::sequence_error};
        static const char* const flags[3] = {"SIZERR", "PGAERR", "PGSERR"};
        bench.verdict(flags[i], " is what the silicon raised, and the engine "
                      "came back: a misstep that sends a WHOLE double word's "
                      "worth of accesses is over when it is reported",
                      (token.sr[i] & wanted[i]) != 0u && token.busy[i] == 0u);
        token.pass = bench.passed();
        token.fail = bench.failed();
        stage_misstep(leg);
        return;
    }

    // The last boot: leg 4's evidence is printed above, and this is
    // where the two halves of the finding are judged together.
    bench.verdict("A LEGAL FIRST WORD WITH NO SECOND RAISES NO FLAG AT ALL - "
                  "3.3.8 has no error for it, because from the engine's side "
                  "nothing has gone wrong yet",
                  (token.sr[3] & FlashFlag::errors) == 0u);
    bench.verdict("and it WEDGES FLASH_SR.CFGBSY: no further flash operation "
                  "is possible and a store into FLASH_CR would be a HardFault "
                  "(3.7.5). The chapter says CFGBSY falls when a complete "
                  "double word is finished; it does not say what happens when "
                  "one never is",
                  token.busy[3] != 0u);
    bench.verdict("the driver saw it and refused rather than writing FLASH_CR",
                  token.ret[3] == FlashFlag::refused);
    bench.verdict("none of the three COMPLETE missteps wedged anything",
                  token.busy[0] == 0u && token.busy[1] == 0u &&
                      token.busy[2] == 0u);

    bench.verdict("and a system reset really does clear it: this boot came up "
                  "with the engine idle",
                  !Flash::config_busy() && !Flash::busy());
    (void)Flash::unlock();
    bench.verdict("FLASH_CR unlocks again", !Flash::locked());
    const uint32_t err = raw_erase(scratch_b);
    fill(8, 0x7E);
    const uint32_t perr = Flash::program(scratch_b, out_span(8));
    (void)Flash::lock();
    bench.verdict("an erase works again after the wedge", err == 0u);
    bench.verdict("and so does a program", perr == 0u);
    bench.verdict("the token crossed all four resets intact",
                  token.magic == token_magic && token.canary == token_canary);

    token.leg = 0;
    token.magic = 0;
    bench.end_letter();
}

// =============================================================================
// w - the wear this run spent
// =============================================================================
void tw_wear() {
    const uint32_t total = MeteredFlash::erases + raw_erases;
    print(serial, "  page erases this run: ", MeteredFlash::erases,
          " through the heap + ", raw_erases, " raw = ", total, crlf);
    print(serial, "  programs: ", MeteredFlash::programs, ", reads: ",
          MeteredFlash::reads, crlf);
    print(serial, "  DS13560 table 49 gives a page 10000 cycles minimum; the "
          "busiest page here takes a handful per run", crlf);

    // The budget is a VERDICT and not a remark: a letter set that starts
    // erasing hundreds of pages per run has changed into something that
    // should not be run in a loop, and the suite should say so before a
    // board pays for it.
    bench.verdict("a z run stays well inside a sane erase budget",
                  total <= 60u);
    bench.verdict("and the heap's own share of it is small - a mutation is "
                  "one map page plus the block's own",
                  MeteredFlash::erases <= 30u);

    MeteredFlash::clear();
    raw_erases = 0;
}

// =============================================================================
// v - the survivors, after whatever happened in between
// =============================================================================
void tv_verify() {
    const Heap::MountReport& r = heap.mount();
    print(serial, "  mount: status=", static_cast<uint8_t>(r.status),
          " survivors=", r.survivors, " lost=", r.lost, " seq=", r.seq,
          " build=", r.build_id, " (this image is ", MainFlash::build_id(),
          ")", crlf);

    bench.verdict("the heap still mounts", r.mounted());
    // AT LEAST two: test_stm32_journal's coexistence letter writes a
    // block of its own into the same heap, which is the partition
    // working rather than interference.
    bench.verdict("letter i's two blocks are still there and nothing is lost",
                  r.survivors >= 2u && r.lost == 0u);

    bool table_ok = false;
    if (const auto f = heap.find(rec_table)) {
        table_ok = f->length == table_len && f->read(0, in_span(table_len));
        for (uint32_t i = 0; i < table_len && table_ok; ++i) {
            table_ok = in_buf[i] == pattern(i, 0x6D);
        }
    }
    bench.verdict("the big block is byte-exact", table_ok);

    bool short_ok = false;
    if (const auto f = heap.find(rec_short)) {
        short_ok = f->length == short_len && f->read(0, in_span(short_len));
        for (uint32_t i = 0; i < short_len && short_ok; ++i) {
            short_ok = in_buf[i] == pattern(i, 0xC4);
        }
    }
    bench.verdict("and the small one holds what letter i last wrote into it",
                  short_ok);

    // THE POINT OF THIS LETTER. tools/bench.py flashes through OpenOCD's
    // `program <elf> verify`, which erases only the sectors the image
    // occupies - and the image occupies bank 1 alone. So a reflash of a
    // DIFFERENT app leaves bank 2 untouched, and a build id older than
    // the running image is the proof that these bytes predate it.
    print(serial, "  (a build id older than this image means the blocks "
          "crossed a reflash)", crlf);
    bench.verdict("the map was written by an image, and validity is judged by "
                  "the CHECKSUM and never by which one",
                  r.build_id != 0u);
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_stm32_nvm - STM32G0B1RE flash (RM0444 ch. 3) + "
          "util/nv_heap.hpp, clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// Letter h binds this; every other letter runs with the line disabled.
extern "C" void FLASH_IRQHandler() {
    const uint32_t flags = brio::Flash::isr();
    flash_irq_flags |= flags;
    flash_irqs = static_cast<uint16_t>(flash_irqs + 1);
}

extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::Stm32Platform>(0x3F);
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::enable_interrupts();

    bench.letter('a', "the geometry: banks, pages and the linker's boundary",
                 ta_geometry);
    bench.letter('b', "the option bytes, decoded and never written", tb_options);
    bench.letter('c', "the lock and the refusals", tc_lock);
    bench.letter('d', "erase and program: what they cost, and EOP", td_cost);
    bench.letter('e', "the error vocabulary, provoked on purpose", te_errors);
    bench.letter('f', "read-while-write, measured", tf_rww);
    bench.letter('g', "fast programming, a whole row at a time", tg_fast);
    bench.letter('h', "the FLASH interrupt through the NVIC", th_interrupt);
    bench.letter('i', "util/nv_heap.hpp on this silicon", ti_heap);
    bench.letter('w', "the wear this run spent", tw_wear);
    bench.letter('s', "FOUR REBOOTS: the malformed programs of 3.3.8",
                 ts_missteps, false);
    bench.letter('v', "the survivors, after a reflash (not in z)", tv_verify,
                 false);

    // A pending token means a leg of letter s is waiting to be judged:
    // resume it instead of printing a banner nobody asked for.
    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        ts_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=",
                    clock_ok ? "PLL64" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " flash=",
                    brio::flash_size_kb(), "KB banks=", brio::Flash::bank_count(),
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
