// Flash family smoke TU: the engine's grains and keys, the partition's
// arithmetic, and the two media over it satisfying FlashMedia - with
// the heap and the journal instantiated on them at their real geometry.
#include "ch32v00x/nvm.hpp"
#include "ch32v00x/nvm_flash.hpp"
#include "util/nv_heap.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

static_assert(Flash::page_size == 256 && Flash::sector_size == 1024);
static_assert(Flash::alias_of(0xA000) == 0x0800A000);
static_assert(MainFlashPartition::storage_base == 0xA000);
static_assert(MainFlashPartition::journal_base == 0xE000);
static_assert(MainFlashPartition::storage_end == 0xF800);
static_assert(MainFlash::write_cell == MainFlash::erase_size, "the page is the cell here");
static_assert(FlashMedia<MainFlash> && FlashMedia<MainFlashJournalZone>);

using Heap = NvHeap<MainFlash, 8, 2>;
using Journal = NvJournal<MainFlashJournalZone, 8, 32, 12>;

Heap heap;
Journal journal;

void flash_verbs() {
    (void)Flash::unlock();
    (void)Flash::locked();
    (void)Flash::fast_locked();
    (void)Flash::busy();
    uint8_t page[Flash::page_size] = {};
    (void)Flash::program_page(0xA000, page);
    (void)Flash::erase_page(0xA000);
    (void)Flash::erase_sector(0xA000);
    (void)Flash::write_protection();
    (void)Flash::status();
    Flash::lock();
    (void)MainFlashPartition::geometry_matches_silicon();
    (void)MainFlash::zones();
    (void)MainFlashJournalZone::build_id();
    (void)heap.mount();
    (void)journal.mount();
}
