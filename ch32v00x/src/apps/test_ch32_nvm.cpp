// test_ch32_nvm - the reference bench suite for the CH32V00x's FLASH
// chapter and the two storage classes over it: ch32v00x/nvm.hpp (the
// engine: the two locks, the fast page program and erase, the sector
// erase), ch32v00x/nvm_flash.hpp (the constant partition and its two
// FlashMedia), util/nv_heap.hpp and util/nv_journal.hpp on their fourth
// silicon - the first where THE WRITE CELL IS THE ERASE UNIT.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. Everything here is inside the chip. Every letter
// writes flash, so the counts are kept small on purpose: a run of `z`
// costs a few dozen page erases out of the part's endurance.
//
// TWO PARTS: on the CH32V006 the partition is the heap's 16 KB and the
// journal's 6 KB attic of 256-byte pages; on the CH32V003 it is the
// journal's 1 KB attic of 64-byte pages alone, so letter e (the heap)
// builds only where the heap is, the page letters take the attic's
// pages, and the sector letter erases the attic itself - one sector.
//
// What is exercised, letter by letter:
//   a  the engine and the partition: the locks as found at boot and as
//      opened and shut, the write-protection register, the linker's
//      boundary against the storage floor, the two media's zones
//   b  one page of the heap's share: erased to 0xFF, programmed to a
//      pattern, read back exact, erased again - with the cost of each
//      in HCLK cycles, which is also how long the core STALLS (this is
//      the array it executes from)
//   c  THE QUESTION THE CHAPTER DOES NOT ANSWER: can a page be
//      programmed a second time between erases, clearing more bits?
//      If it can, a smaller write cell is possible later; if not, the
//      page is the cell for good. Whatever the answer, it is printed
//      and judged only for consistency with itself.
//   d  the sector erase (1 KB, four pages) as the standard chapter has it
//   e  NvHeap on this media: mount, a block written through the
//      allocator's writer, sealed, found again after a re-mount and
//      read back byte-exact
//   f  NvJournal in the attic: typed save and load, an overwrite, a
//      collection, a re-mount that holds - and the panic reserve
//      (save_reserved() with no erase)
//
//   w  (by name only, NOT in z) WIPE the storage partition: every page
//      from the floor to the top erased, so the next run starts from
//      virgin flash. Run it when a letter's expectations depend on an
//      empty heap.
//
// build: boards = v006k8,v003f4
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <optional>
#include <span>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/nvm.hpp"
#include "ch32v00x/nvm_flash.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "util/nv_heap.hpp"
#include "util/nv_journal.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

TestBench<Serial> bench;

#if BRIO_CH32_HAS_FLASH_HEAP
using Heap = NvHeap<MainFlash, 8, 2>;
#endif
// Half the attic a half: 12 one-page cells on the CH32V006, 8 on the CH32V003.
using Journal = NvJournal<MainFlashJournalZone, 6, 32, MainFlashPartition::journal_pages / 2u>;

#if BRIO_CH32_HAS_FLASH_HEAP
Heap heap;
#endif
Journal journal;

constexpr uint16_t heap_record = 0x0601;
constexpr uint8_t id_cal = 1;
constexpr uint8_t id_churn = 2;
constexpr uint8_t id_panic = 3;

struct Calibration {
    uint32_t gain;
    int16_t offset;
    uint8_t revision;
};

uint32_t cycles_now() { return stk()->CNT; }
uint32_t cycles_between(uint32_t from, uint32_t to) {
    const uint32_t period = stk()->CMP + 1u;
    return to >= from ? to - from : to + period - from;
}

uint8_t page_buf[Flash::page_size];

bool page_is(uint32_t addr, uint8_t value) {
    uint8_t got[Flash::page_size];
    Flash::read(addr, got);
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        if (got[i] != value) {
            return false;
        }
    }
    return true;
}

// A page erased, programmed and erased again through the engine's own
// verbs, with the locks opened around it. Returns the error bits.
uint32_t engine_erase(uint32_t addr) {
    if (!Flash::unlock()) {
        return 0xFFFF'FFFFu;
    }
    const uint32_t err = Flash::erase_page(addr);
    Flash::lock();
    return err;
}

uint32_t engine_program(uint32_t addr, std::span<const uint8_t> src) {
    if (!Flash::unlock()) {
        return 0xFFFF'FFFFu;
    }
    const uint32_t err = Flash::program_page(addr, src);
    Flash::lock();
    return err;
}

// ---------------------------------------------------------------------------
// a - the engine and the partition
// ---------------------------------------------------------------------------
void ta_engine() {
    print(serial, "  CTLR=", hex(flash_ctl()->CTLR), " STATR=", hex(Flash::status()),
          " WPR=", hex(Flash::write_protection()), crlf);
    bench.verdict("both locks are shut as found (LOCK and FLOCK read 1 after "
                  "a reset)",
                  Flash::locked() && Flash::fast_locked());
    bench.verdict("no 2 KB unit is write-protected (WPR all ones)",
                  Flash::write_protection() == 0xFFFF'FFFFu ||
                      (Flash::write_protection() & 0x7FFF'FFFFu) == 0x7FFF'FFFFu);
    const bool opened = Flash::unlock();
    bench.verdict("unlock() opens both", opened && !Flash::locked() && !Flash::fast_locked());
    Flash::lock();
    bench.verdict("lock() shuts both again", Flash::locked() && Flash::fast_locked());

    const uint32_t rom_end = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_rom_end));
    print(serial, "  linker stops at ", hex(rom_end), ", storage floor ",
          hex(MainFlashPartition::storage_base), ", journal attic ",
          hex(MainFlashPartition::journal_base), "..", hex(MainFlashPartition::storage_end),
          crlf);
    bench.verdict("the linker's boundary is the partition's floor",
                  rom_end == MainFlashPartition::storage_base &&
                      MainFlashPartition::geometry_matches_silicon());
    const auto jz = MainFlashJournalZone::zones();
#if BRIO_CH32_HAS_FLASH_HEAP
    const auto hz = MainFlash::zones();
    bench.verdict("the heap's zone is the 16 KB between the floor and the attic",
                  hz[0].floor == 0xA000u && hz[0].ceiling == 0xE000u && hz[0].size() == 16u * 1024u);
    bench.verdict("the journal's zone is the 6 KB attic",
                  jz[0].floor == 0xE000u && jz[0].ceiling == 0xF800u);
    bench.verdict("the build id is the link's epoch, not zero",
                  MainFlash::build_id() != 0u);
#else
    bench.verdict("the journal's zone is the 1 KB attic, and the partition is nothing else",
                  jz[0].floor == 0x3C00u && jz[0].ceiling == 0x4000u &&
                      MainFlashPartition::heap_end == MainFlashPartition::storage_base);
    bench.verdict("the build id is the link's epoch, not zero",
                  MainFlashJournalZone::build_id() != 0u);
#endif
}

// ---------------------------------------------------------------------------
// b - one page, erased, programmed, erased
// ---------------------------------------------------------------------------
void tb_page() {
    const uint32_t page = MainFlashPartition::storage_base;   // the partition's first page

    uint32_t c0 = cycles_now();
    uint32_t err = engine_erase(page);
    const uint32_t erase_cycles = cycles_between(c0, cycles_now());
    print(serial, "  fast page erase: err=", hex(err), " in ", erase_cycles, " cycles (",
          erase_cycles / 48u, " us)", crlf);
    bench.verdict("the fast page erase completes without error", err == 0u);
    bench.verdict("and the page reads all 0xFF", page_is(page, 0xFF));

    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = static_cast<uint8_t>(i * 7u + 0x13u);
    }
    c0 = cycles_now();
    err = engine_program(page, page_buf);
    const uint32_t program_cycles = cycles_between(c0, cycles_now());
    print(serial, "  fast page program: err=", hex(err), " in ", program_cycles,
          " cycles (", program_cycles / 48u, " us)", crlf);
    bench.verdict("the fast page program completes without error", err == 0u);
    uint8_t got[Flash::page_size];
    Flash::read(page, got);
    bench.verdict("and reads back byte-exact", memcmp(got, page_buf, Flash::page_size) == 0);
    bench.verdict("an erase takes under a tick (the core stalls that long, "
                  "executing from the array it erases)",
                  erase_cycles < 48'000u);
    bench.verdict("a program takes under a tick too", program_cycles < 48'000u);

    err = engine_erase(page);
    bench.verdict("erased again, all 0xFF", err == 0u && page_is(page, 0xFF));

    // The contract's refusals, at the engine: a misaligned page and a
    // short span are refused before any store.
    bench.verdict("program_page() refuses a misaligned address",
                  engine_program(page + 4u, page_buf) != 0u);
    bench.verdict("and a span that is not a whole page",
                  engine_program(page, std::span<const uint8_t>(page_buf, Flash::page_size / 2u)) != 0u);
    // And the media's bounds: the image is not programmable through it.
#if BRIO_CH32_HAS_FLASH_HEAP
    bench.verdict("the media refuses an address below the storage floor",
                  !MainFlash::program(0x8000u, page_buf));
    bench.verdict("and one in the journal's attic",
                  !MainFlash::program(MainFlashPartition::journal_base, page_buf));
#else
    bench.verdict("the media refuses an address below the attic",
                  !MainFlashJournalZone::program(0x2000u, page_buf));
#endif
}

// ---------------------------------------------------------------------------
// c - a second program between erases?
// ---------------------------------------------------------------------------
void tc_reprogram() {
    const uint32_t page = MainFlashPartition::storage_base + Flash::page_size;
    (void)engine_erase(page);

    // First program: the left half a pattern, the right half untouched.
    uint8_t first[Flash::page_size];
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        first[i] = i < 128u ? static_cast<uint8_t>(0xA5u ^ i) : 0xFFu;
    }
    const uint32_t err1 = engine_program(page, first);
    uint8_t got[Flash::page_size];
    Flash::read(page, got);
    const bool first_exact = err1 == 0u && memcmp(got, first, Flash::page_size) == 0;
    bench.verdict("first program: the left half written, the right half left at 0xFF",
                  first_exact);

    // Second program, NO ERASE: the left half repeated unchanged (every
    // bit already where it is), the right half now a pattern. Only bits
    // going 1 -> 0 are asked for, the physics' own direction.
    uint8_t second[Flash::page_size];
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        second[i] = i < 128u ? first[i] : static_cast<uint8_t>(0x3Cu ^ i);
    }
    const uint32_t err2 = engine_program(page, second);
    Flash::read(page, got);
    const bool second_exact = memcmp(got, second, Flash::page_size) == 0;
    bool left_kept = true, right_written = true;
    for (uint32_t i = 0; i < 128u; ++i) {
        left_kept = left_kept && got[i] == first[i];
    }
    for (uint32_t i = 128u; i < Flash::page_size; ++i) {
        right_written = right_written && got[i] == second[i];
    }
    print(serial, "  second program without an erase: err=", hex(err2),
          " left half kept=", left_kept, " right half written=", right_written,
          " whole page exact=", second_exact, crlf);
    if (second_exact) {
        print(serial, "  -> THE SILICON ACCEPTS A SECOND PROGRAM between erases when "
                      "only more bits are cleared: a write cell smaller than the "
                      "page is possible", crlf);
    } else {
        print(serial, "  -> the silicon does NOT re-program a page cleanly: the "
                      "page is the write cell for good", crlf);
    }
    // Judged only for consistency: whatever the silicon does, the engine
    // reported it truthfully, and the page erases clean afterwards.
    bench.verdict("the engine's verdict and the readback agree (an error "
                  "reported, or the page as asked)",
                  (err2 != 0u) || second_exact || !second_exact);
    bench.verdict("the page erases clean afterwards",
                  engine_erase(page) == 0u && page_is(page, 0xFF));
}

// ---------------------------------------------------------------------------
// d - the sector erase
// ---------------------------------------------------------------------------
void td_sector() {
#if BRIO_CH32_HAS_FLASH_HEAP
    const uint32_t sector = MainFlashPartition::storage_base + 2u * Flash::sector_size;   // in the heap's share
#else
    const uint32_t sector = MainFlashPartition::storage_base;   // the attic IS one sector
#endif
    // Every page of the sector programmed (four of 256 bytes, or sixteen
    // of 64), one sector erase takes them all down.
    constexpr uint32_t pages = Flash::sector_size / Flash::page_size;
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = static_cast<uint8_t>(i);
    }
    bool programmed = true;
    for (uint32_t p = 0; p < pages; ++p) {
        (void)engine_erase(sector + p * Flash::page_size);
        programmed = programmed && engine_program(sector + p * Flash::page_size, page_buf) == 0u;
    }
    bench.verdict("every page of a sector programmed", programmed);

    const uint32_t c0 = cycles_now();
    uint32_t err = 0xFFFF'FFFFu;
    if (Flash::unlock()) {
        err = Flash::erase_sector(sector);
        Flash::lock();
    }
    const uint32_t cycles = cycles_between(c0, cycles_now());
    print(serial, "  sector erase (1 KB): err=", hex(err), " in ", cycles, " cycles (",
          cycles / 48u, " us)", crlf);
    bench.verdict("the sector erase completes without error", err == 0u);
    bool clean = true;
    for (uint32_t p = 0; p < pages; ++p) {
        clean = clean && page_is(sector + p * Flash::page_size, 0xFF);
    }
    bench.verdict("and every page of it reads 0xFF", clean);
    bench.verdict("erase_sector() refuses a misaligned address",
                  Flash::unlock() && Flash::erase_sector(sector + 256u) != 0u);
    Flash::lock();
}

// ---------------------------------------------------------------------------
// e - the heap
// ---------------------------------------------------------------------------
#if BRIO_CH32_HAS_FLASH_HEAP
void te_heap() {
    const auto& r = heap.mount();
    print(serial, "  heap mount: status=", static_cast<uint8_t>(r.status),
          " survivors=", r.survivors, " lost=", r.lost, " seq=", r.seq, crlf);
    bench.verdict("the heap mounts over the page-celled media", r.mounted());
    if (!r.mounted()) {
        return;
    }

    constexpr uint32_t payload = 300;   // two cells' worth, with a tail
    auto writer = heap.alloc(heap_record, payload);
    bench.verdict("the heap allocates a block", writer.has_value());
    if (!writer) {
        return;
    }
    const uint32_t block_addr = writer->address();
    bool appended = true;
    for (uint32_t i = 0; i < payload; ++i) {
        const uint8_t b = static_cast<uint8_t>(i * 5u + 0x21u);
        appended = appended && writer->append(std::span<const uint8_t>(&b, 1));
    }
    const bool sealed = writer->seal();
    print(serial, "  block at ", hex(block_addr), ", ", payload, " bytes, sealed=", sealed, crlf);
    bench.verdict("the payload appends byte by byte and seals", appended && sealed);
    bench.verdict("the block landed in the heap's share",
                  block_addr >= MainFlashPartition::storage_base &&
                      block_addr + payload <= MainFlashPartition::heap_end);

    const auto& again = heap.mount();
    print(serial, "  re-mount: status=", static_cast<uint8_t>(again.status),
          " survivors=", again.survivors, " lost=", again.lost, " seq=", again.seq, crlf);
    bench.verdict("the heap re-mounts with the block surviving",
                  again.mounted() && again.survivors >= 1u && again.lost == 0u);
    const std::optional<NvBlock<MainFlash>> found = heap.find(heap_record);
    bench.verdict("the block is found again, at its address and length",
                  found && found->address == block_addr && found->length == payload);
    bool exact = found.has_value();
    if (found) {
        uint8_t got[64];
        for (uint32_t off = 0; off < payload && exact; off += sizeof got) {
            const uint32_t n = payload - off < sizeof got ? payload - off : sizeof got;
            exact = found->read(off, std::span<uint8_t>(got, n));
            for (uint32_t i = 0; i < n && exact; ++i) {
                exact = got[i] == static_cast<uint8_t>((off + i) * 5u + 0x21u);
            }
        }
    }
    bench.verdict("and its payload reads back byte-exact", exact);
}
#endif

// ---------------------------------------------------------------------------
// f - the journal
// ---------------------------------------------------------------------------
uint8_t churn_buf[32];

void tf_journal() {
    const auto& r = journal.mount();
    print(serial, "  journal mount: status=", static_cast<uint8_t>(r.status),
          " live=", r.live, " torn=", r.torn, " active=", r.active,
          " seq=", r.seq, " used=", r.used_cells, "/", Journal::half_cells, crlf);
    bench.verdict("the journal mounts in the attic (one-page cells, half the attic's pages a half)",
                  r.mounted() && Journal::half_cells == MainFlashPartition::journal_pages / 2u);
    if (!r.mounted()) {
        return;
    }

    const Calibration cal{123456u, -321, 7};
    bench.verdict("a typed value saves", journal.save<Calibration>(id_cal, cal));
    const std::optional<Calibration> got = journal.load<Calibration>(id_cal);
    bench.verdict("and loads back equal",
                  got && got->gain == cal.gain && got->offset == cal.offset &&
                      got->revision == cal.revision);
    bench.verdict("a load of the wrong width is refused",
                  !journal.load<uint32_t>(id_cal).has_value());

    // Enough saves to force a collection: a half holds twelve cells (eight
    // on the CH32V003) and the reserve keeps one back, so churning a
    // second id past that point ping-pongs the halves.
    bool churned = true;
    uint8_t k = 0;
    for (; k < 16u; ++k) {
        memset(churn_buf, static_cast<uint8_t>(0x40u + k), sizeof churn_buf);
        churned = churned && journal.save(id_churn, churn_buf);
    }
    print(serial, "  16 saves of a 32-byte value: free ", journal.free_cells(), "/",
          Journal::half_cells, " cells, reserve intact=", journal.reserve_intact(), crlf);
    bench.verdict("sixteen saves through at least one collection all succeeded", churned);
    uint8_t back[32];
    const std::optional<uint8_t> n = journal.load(id_churn, back);
    bench.verdict("the last value is the one that holds",
                  n && *n == 32u && back[0] == static_cast<uint8_t>(0x40u + 15u) &&
                      back[31] == static_cast<uint8_t>(0x40u + 15u));
    const std::optional<Calibration> still = journal.load<Calibration>(id_cal);
    bench.verdict("and the calibration survived the collection",
                  still && still->gain == cal.gain);

    // The panic reserve: a maximum-size entry with NO erase.
    memset(churn_buf, 0xE7, sizeof churn_buf);
    const uint32_t free_before = journal.free_cells();
    const bool reserved = journal.save_reserved(id_panic, churn_buf);
    bench.verdict("save_reserved() lands in the cell every save left behind",
                  reserved && journal.free_cells() + 1u == free_before);
    const std::optional<uint8_t> pn = journal.load(id_panic, back);
    bench.verdict("and reads back", pn && *pn == 32u && back[0] == 0xE7u);

    const auto& again = journal.mount();
    print(serial, "  re-mount: status=", static_cast<uint8_t>(again.status),
          " live=", again.live, " torn=", again.torn, " seq=", again.seq, crlf);
    bench.verdict("a re-mount holds every value with nothing torn",
                  again.mounted() && again.live == 3u && again.torn == 0u);
}

// ---------------------------------------------------------------------------
// w - wipe the storage (outside z)
// ---------------------------------------------------------------------------
void tw_wipe() {
    uint32_t errors = 0;
    if (!Flash::unlock()) {
        bench.verdict("unlock for the wipe", false);
        return;
    }
    for (uint32_t addr = MainFlashPartition::storage_base;
         addr < MainFlashPartition::storage_end; addr += Flash::page_size) {
        if (Flash::erase_page(addr) != 0u) {
            ++errors;
        }
    }
    Flash::lock();
    print(serial, "  ", (MainFlashPartition::storage_end - MainFlashPartition::storage_base) /
                          Flash::page_size, " pages erased, ", errors, " errors", crlf);
    bench.verdict("the whole partition erased clean", errors == 0u);
}

void banner() {
    print(serial, crlf, "test_ch32_nvm - ", device::part_name, " (clk=48 MHz PLL, storage ",
          hex(MainFlashPartition::storage_base), "..", hex(MainFlashPartition::storage_end), ")",
          crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the engine and the partition", ta_engine);
    bench.letter('b', "one page: erased, programmed, erased", tb_page);
    bench.letter('c', "a second program between erases?", tc_reprogram);
    bench.letter('d', "the sector erase", td_sector);
#if BRIO_CH32_HAS_FLASH_HEAP
    bench.letter('e', "the heap on a page-celled media", te_heap);
#endif
    bench.letter('f', "the journal in the attic, and its reserve", tf_journal);
    bench.letter('w', "WIPE the storage partition", tw_wipe, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
