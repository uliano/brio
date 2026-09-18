// Flash storage family smoke TU: the partition's geometry, the
// FlashMedia contract satisfied, and every verb of the medium
// instantiated once. Nothing stands ON the contract here - neither the
// block heap nor the value journal is instantiated on this family, by
// the decision the header states.
#include "rp2350/nvm_flash.hpp"

using namespace brio;

// The contract itself, and the two arithmetic facts util/nv_heap.hpp
// depends on.
static_assert(FlashMedia<QspiFlash>);
static_assert(QspiFlash::flash_end % QspiFlash::erase_size == 0u);
static_assert(QspiFlash::flash_end / QspiFlash::erase_size <= 0xFFFFu);

// The partition: the top 64 kB of a 16 MB chip, sixteen sectors of
// 4096, the cell a 256-byte page.
static_assert(QspiFlashPartition::storage_end == Flash::size_bytes);
static_assert(QspiFlashPartition::storage_end - QspiFlashPartition::storage_base == 64u * 1024u);
static_assert(QspiFlashPartition::storage_sectors * QspiFlashPartition::sector == 64u * 1024u);
static_assert(QspiFlash::erase_size == Flash::sector_size);
static_assert(QspiFlash::write_cell == Flash::page_size);
static_assert(QspiFlash::zone_count == 1u);

// The compile-time half of the bounds check: inside, at the edges, and
// past them.
static_assert(QspiFlashPartition::contains(QspiFlashPartition::storage_base, 256u));
static_assert(QspiFlashPartition::contains(QspiFlashPartition::storage_end - 256u, 256u));
static_assert(!QspiFlashPartition::contains(QspiFlashPartition::storage_base - 256u, 256u));
static_assert(!QspiFlashPartition::contains(QspiFlashPartition::storage_end - 256u, 512u));
static_assert(!QspiFlashPartition::contains(0u, 256u));

static_assert(QspiFlashPartition::cell_aligned(QspiFlashPartition::storage_base, 256u));
static_assert(!QspiFlashPartition::cell_aligned(QspiFlashPartition::storage_base + 1u, 256u));
static_assert(!QspiFlashPartition::cell_aligned(QspiFlashPartition::storage_base, 255u));

uint8_t cell[QspiFlash::write_cell];

void nvm_flash_verbs() {
    (void)QspiFlashPartition::geometry_matches_silicon();

    const auto zones = QspiFlash::zones();
    (void)zones[0].floor;
    (void)zones[0].ceiling;

    QspiFlash::read(QspiFlashPartition::storage_base, cell);
    (void)QspiFlash::program(QspiFlashPartition::storage_base, cell);
    (void)QspiFlash::erase(QspiFlashPartition::storage_base);
    (void)QspiFlash::build_id();
}
