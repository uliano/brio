// Family smoke TU for util/nv_journal.hpp over samc/nvm_flash.hpp: the
// value journal and the RWWEE partition it lives in must COMPILE on the
// E, G and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The partition is a pair of CONSTANTS on every variant - the RWWEE
// array is 8 KB across the whole family and nothing the linker places
// can reach it - so the assertions below are arithmetic and not
// discovery. They are worth making because the two media divide ONE
// array and a change to either bound that did not move the other would
// leave the heap and the journal overlapping in silence.

#include <stdint.h>

#include <span>

#include "kernel/panic.hpp"
#include "samc/nvm.hpp"
#include "samc/nvm_flash.hpp"
#include "util/nv_heap.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

// ---- the partition ----------------------------------------------------------

static_assert(RwweePartition::journal_rows == 4u);
static_assert(RwweePartition::journal_bytes == 1024u);
static_assert(RwweePartition::journal_base == Nvm::rwwee_end - 1024u);
static_assert(RwweePartition::heap_end == RwweePartition::journal_base);

// The two regions meet exactly once and never overlap.
static_assert(RwweeFlash::flash_end == RwweeJournalZone::flash_end -
                                           RwweePartition::journal_bytes);
static_assert(RwweeJournalZone::flash_end == Nvm::rwwee_end);
static_assert(RwweeFlash::erase_size == RwweeJournalZone::erase_size);
static_assert(RwweeFlash::write_cell == RwweeJournalZone::write_cell);

static_assert(FlashMedia<RwweeFlash>);
static_assert(FlashMedia<RwweeJournalZone>);

// ---- the journal this target's attic can hold -------------------------------

using Journal = NvJournal<RwweeJournalZone, 6, 32, 2>;

static_assert(Journal::erase_size == 256u);
static_assert(Journal::write_cell == 64u);
static_assert(Journal::half_bytes == 512u);
static_assert(Journal::half_cells == 8u);
// A 32-byte value plus the 12-byte header rounds up to ONE page here,
// which is what makes six ids plus the reserve fit at all.
static_assert(Journal::max_entry_bytes == 64u);
static_assert(Journal::max_entry_cells == 1u);
static_assert(Journal::reserve_cells == 1u);
static_assert(Journal::journal_home == RwweePartition::journal_base);
static_assert(Journal::half_base(0) == RwweePartition::journal_base);
static_assert(Journal::half_base(1) == RwweePartition::journal_base + 512u);

// The layout constants the on-flash format is described by.
static_assert(Journal::magic == 0x4A4Eu);
static_assert(Journal::header_bytes == 12u);
static_assert(Journal::off_magic == 0u);
static_assert(Journal::off_id == 2u);
static_assert(Journal::off_length == 3u);
static_assert(Journal::off_seq == 4u);
static_assert(Journal::off_crc == 8u);
static_assert(Journal::off_reserved == 10u);
static_assert(Journal::crc_covered_header == 8u);

// The heap still fits its own share, map pair and all.
using Heap = NvHeap<RwweeFlash, 8, 2>;
static_assert(Heap::map_home == RwweePartition::heap_end - 2u * 256u);
static_assert(Heap::map_home > Nvm::rwwee_base);

// ---- every verb instantiates ------------------------------------------------

Journal journal;

struct Calibration {
    int32_t offset;
    uint16_t gain;
};

using Panic = JournalPanic<journal, 5>;

void use() {
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
    (void)journal.last_error();
    (void)journal.has(0);

    uint8_t buf[32]{};
    (void)journal.save(0, std::span<const uint8_t>(buf, sizeof buf));
    (void)journal.save_reserved(1, std::span<const uint8_t>(buf, 4));
    (void)journal.load(0, std::span<uint8_t>(buf, sizeof buf));
    (void)journal.template save<Calibration>(2, Calibration{-1, 2});
    (void)journal.template save_reserved<Calibration>(3, Calibration{-1, 2});
    (void)journal.template load<Calibration>(2);
    (void)journal.collect();
    if (journal.count() != 0) {
        (void)journal.entry(0);
    }

    Panic::report(PanicCode::assert_failed, 1);
    (void)Panic::peek();
    (void)Panic::pending();
    (void)Panic::take();
    (void)Panic::save(PanicRecord{panic_magic, 0, 0});
}
