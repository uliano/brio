// The storage backends family smoke TU: stm32g0/nvm_flash.hpp and the two
// target-independent users it exists for (util/nv_heap.hpp and
// util/nv_journal.hpp) against every device header.
//
// The partition's constants are ARITHMETIC, so they hold on every
// variant; what does not hold everywhere is the SILICON, and that is a
// runtime question by construction - the device-select macro is
// STM32G0B1xx for the 128, 256 and 512 Kbyte members alike, so only the
// flash size register knows. geometry_matches_silicon() is that question,
// and a part that answers no gets a zone no user can fit in and two
// mounts that refuse having written nothing.
#include "stm32g0/nvm_flash.hpp"
#include "util/nv_heap.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

// ---- the partition ---------------------------------------------------------
static_assert(MainFlashPartition::bank_bytes == 256u * 1024u);
static_assert(MainFlashPartition::storage_base == 0x40000u);
static_assert(MainFlashPartition::storage_end == 0x80000u);
static_assert(MainFlashPartition::journal_pages == 2);
static_assert(MainFlashPartition::journal_bytes == 4096u);
static_assert(MainFlashPartition::journal_base == 0x7F000u);
static_assert(MainFlashPartition::heap_end == MainFlashPartition::journal_base);
static_assert(MainFlashPartition::address_of(0) == 0x0800'0000UL);
static_assert(MainFlashPartition::address_of(MainFlashPartition::storage_base) ==
              0x0804'0000UL);

// ---- both backends satisfy the contract ------------------------------------
static_assert(FlashMedia<MainFlash>);
static_assert(FlashMedia<MainFlashJournalZone>);
static_assert(MainFlash::erase_size == 2048u && MainFlash::write_cell == 8u);
static_assert(MainFlashJournalZone::erase_size == 2048u &&
              MainFlashJournalZone::write_cell == 8u);
static_assert(MainFlash::flash_end == MainFlashPartition::journal_base);
static_assert(MainFlashJournalZone::flash_end == MainFlashPartition::storage_end);

// THE ADDRESSING DECISION, asserted rather than remembered: NvHeap
// numbers erase units in a uint16_t, so a media whose addresses were
// ABSOLUTE could not describe this bank at all (0x08040000 / 2048 is
// 65664). Offsets from 0x08000000 are what make the page numbers fit -
// the AVR backend's convention, not a new one.
static_assert(0x0804'0000UL / MainFlash::erase_size > 0xFFFFu);
static_assert(MainFlashJournalZone::flash_end / MainFlashJournalZone::erase_size <=
              0xFFFFu);

// ---- the two users, at the geometry the bench runs -------------------------
using BenchHeap = NvHeap<MainFlash, 8, 2>;
using BenchJournal = NvJournal<MainFlashJournalZone, 6, 32, 1>;

// One page per half is 256 write cells; a 32-byte value costs
// ceil((12 + 32) / 8) = 6 of them, so the journal's own invariant
// (max_ids + 2) x max_entry_cells <= half_cells reads (6 + 2) x 6 <= 256.
static_assert(BenchJournal::half_cells == 256u);
static_assert(BenchJournal::max_entry_cells == 6u);
static_assert(BenchJournal::reserve_cells == BenchJournal::max_entry_cells);
static_assert(BenchJournal::journal_home == MainFlashPartition::journal_base);
static_assert(BenchJournal::half_base(1) + BenchJournal::half_bytes ==
              MainFlashPartition::storage_end);
// The heap's map home is two pages below the attic, so the two users
// cannot reach each other.
static_assert(BenchHeap::map_home + 2u * BenchHeap::erase_size <=
              MainFlashPartition::journal_base);
// The same geometry the host suite test/test_nv_journal sweeps as "the
// STM32G0's": 2048-byte erase unit, 8-byte cell.
static_assert(BenchJournal::erase_size == 2048u && BenchJournal::write_cell == 8u);

// A journal with room for far more ids than the bench uses still fits in
// one page per half - which is what the 2048/8 geometry buys over the
// samc's 256/64.
using WideJournal = NvJournal<MainFlashJournalZone, 32, 32, 1>;
static_assert((32u + 2u) * WideJournal::max_entry_cells <=
              WideJournal::half_cells);

BenchHeap heap;
BenchJournal journal;

void media_verbs() {
    static uint8_t buf[16];
    (void)MainFlashPartition::geometry_matches_silicon();
    (void)MainFlash::zones()[0].size();
    (void)MainFlashJournalZone::zones()[0].size();
    MainFlash::read(MainFlashPartition::storage_base,
                    std::span<uint8_t>(buf, 8));
    MainFlashJournalZone::read(MainFlashPartition::journal_base,
                               std::span<uint8_t>(buf, 8));
    (void)MainFlash::build_id();
    (void)MainFlashJournalZone::build_id();
    if (false) {
        (void)MainFlash::erase(MainFlashPartition::storage_base);
        (void)MainFlash::program(MainFlashPartition::storage_base,
                                 std::span<const uint8_t>(buf, 8));
        (void)MainFlashJournalZone::erase(MainFlashPartition::journal_base);
        (void)MainFlashJournalZone::program(MainFlashPartition::journal_base,
                                            std::span<const uint8_t>(buf, 8));
    }
}

void heap_verbs() {
    (void)heap.mount();
    (void)heap.report();
    (void)heap.mounted();
    (void)heap.sequence();
    (void)heap.map_page();
    (void)heap.count();
    (void)heap.free_pages(0);
    (void)heap.find(1);
    if (false) {
        static uint8_t buf[16];
        if (auto w = heap.alloc(1, 16)) {
            (void)w->append(std::span<const uint8_t>(buf, 16));
            (void)w->seal();
        }
        if (auto w = heap.rewrite(1)) {
            (void)w->failed();
        }
    }
}

void journal_verbs() {
    static uint8_t buf[32];
    (void)journal.mount();
    (void)journal.report();
    (void)journal.mounted();
    (void)journal.sequence();
    (void)journal.active_half();
    (void)journal.used_cells();
    (void)journal.free_cells();
    (void)journal.reserve_intact();
    (void)journal.collect_pending();
    (void)journal.count();
    (void)journal.has(0);
    (void)journal.load(0, std::span<uint8_t>(buf, 32));
    (void)journal.load<uint32_t>(0);
    (void)journal.last_error();
    if (false) {
        (void)journal.save(0, std::span<const uint8_t>(buf, 8));
        (void)journal.save<uint32_t>(0, 42u);
        (void)journal.save_reserved(0, std::span<const uint8_t>(buf, 8));
        (void)journal.save_reserved<uint32_t>(0, 42u);
        (void)journal.collect();
    }
}

// The panic reporter over the journal: the breadcrumb that survives a
// POWER LOSS and not only a reset (kernel/panic.hpp's Reporter shape).
using Breadcrumb = JournalPanic<journal, 5>;
void panic_paths() {
    (void)Breadcrumb::peek();
    (void)Breadcrumb::pending();
    if (false) {
        Breadcrumb::report(PanicCode::assert_failed, 7);
        (void)Breadcrumb::save(PanicRecord{panic_magic, 2, 3});
        (void)Breadcrumb::take();
    }
}
