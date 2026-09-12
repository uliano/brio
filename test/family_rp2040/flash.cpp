// Flash family smoke TU: the engine over the bootrom, the cache, the
// partition and its two FlashMedia with the heap and the journal on
// them.
#include "rp2040/flash.hpp"
#include "rp2040/nvm_flash.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

static_assert(Flash::size_bytes == 2u * 1024u * 1024u);
static_assert(Flash::sector_size == 4096u && Flash::page_size == 256u);
static_assert(FlashJedecId{.manufacturer = 0xEF, .type = 0x40, .capacity = 0x15}.bytes() == Flash::size_bytes);
static_assert(QspiFlashPartition::storage_base == 0x1F0000u);
static_assert(QspiFlashPartition::journal_base == 0x1FC000u);
static_assert(QspiFlash::flash_end == QspiFlashPartition::journal_base);
static_assert(QspiFlashJournalZone::flash_end == Flash::size_bytes);
static_assert(FlashMedia<QspiFlash> && FlashMedia<QspiFlashJournalZone>);

NvHeap<QspiFlash, 8, 2> heap;
NvJournal<QspiFlashJournalZone, 6, 32, 2> journal;
static_assert(decltype(journal)::half_cells == 32u);

uint8_t page[Flash::page_size];

void flash_verbs() {
    (void)Flash::init();
    (void)Flash::ready();
    (void)Flash::rom_version();
    (void)Flash::stage_crc();
    (void)Flash::stage_crc_stored();
    (void)Flash::in_window(page);
    (void)Flash::address(0);
    (void)Flash::uncached_address(0);
    Flash::read(QspiFlashPartition::storage_base, page);
    (void)Flash::erase(QspiFlashPartition::storage_base, Flash::sector_size);
    (void)Flash::erase_sector(QspiFlashPartition::storage_base);
    (void)Flash::program(QspiFlashPartition::storage_base, page);
    const uint8_t tx[2] = {0x05, 0};
    uint8_t rx[2];
    (void)Flash::command(tx, rx);
    (void)Flash::jedec_id();
    (void)Flash::unique_id();
    (void)Flash::status_register();

    (void)Xip::enabled();
    Xip::enable(true);
    Xip::power_down(false);
    (void)Xip::powered_down();
    Xip::fault_on_bad_write(false);
    (void)Xip::faults_on_bad_write();
    Xip::flush();
    (void)Xip::flush_ready();
    (void)Xip::hits();
    (void)Xip::accesses();
    Xip::reset_counters();

    (void)QspiFlashPartition::geometry_matches_silicon();
    (void)QspiFlash::zones();
    (void)QspiFlash::program(QspiFlashPartition::storage_base, page);
    (void)QspiFlash::erase(QspiFlashPartition::storage_base);
    (void)QspiFlash::build_id();
    (void)QspiFlashJournalZone::zones();
    (void)heap.mount();
    (void)journal.mount();
}
