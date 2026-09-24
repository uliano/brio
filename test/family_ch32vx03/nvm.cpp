// Flash family smoke TU: the program and erase engine, the option
// bytes, the electronic signature and the FlashMedia over the top of
// the array - every verb, on every part.
//
// Nothing here is per instance: there is one flash controller and one
// signature block on every part of the series. What IS per part is the
// ARRAY (32, 64 or 128 KB) and therefore where the medium's zone
// begins, so this file asks device:: for the size and checks the
// partition's arithmetic against it rather than naming an address.
#include <stddef.h>

#include "ch32vx03/nvm.hpp"
#include "ch32vx03/nvm_flash.hpp"

using namespace brio;

// ---- the register map (RM 32.4's table 32-2) --------------------------------
static_assert(offsetof(FlashRegs, KEYR) == 0x04);
static_assert(offsetof(FlashRegs, OBKEYR) == 0x08);
static_assert(offsetof(FlashRegs, STATR) == 0x0C);
static_assert(offsetof(FlashRegs, CTLR) == 0x10);
static_assert(offsetof(FlashRegs, ADDR) == 0x14);
static_assert(offsetof(FlashRegs, OBR) == 0x1C);
static_assert(offsetof(FlashRegs, WPR) == 0x20);
static_assert(offsetof(FlashRegs, MODEKEYR) == 0x24);

// ---- the geometry (32.1, 32.2.1, the part table) ----------------------------
static_assert(Flash::page_size == 256u);
static_assert(Flash::sector_size == 4096u);
static_assert(Flash::block_size == 32u * 1024u);
static_assert(Flash::half_word_size == 2u);
static_assert(Flash::array_bytes == device::flash_bytes);
static_assert(Flash::array_base == 0x08000000u);
static_assert(Flash::alias_base == 0x00000000u);
static_assert(Flash::alias_of(0x1234u) == 0x08001234u);
static_assert(Flash::sector_size % Flash::page_size == 0u);

// THE ERASED PATTERN IS NOT ALL ONES (32.5.4, 32.5.7), which is the one
// fact about this array a reader has to hold.
static_assert(Flash::erased_word == 0xE339E339u);
static_assert(Flash::erased_word != 0xFFFFFFFFu);

// The refusals are bits no STATR flag uses, so a refusal is never
// mistaken for a silicon error.
static_assert((Flash::refused & flash_statr_w1c) == 0u);
static_assert((Flash::refused_rate & flash_statr_w1c) == 0u);
static_assert(Flash::refused != Flash::refused_rate);

// ---- the rate contract (32.1, datasheet table 4-17) -------------------------
static_assert(flash_access_max_hz == 60'000'000u);
static_assert(flash_safe_sysclk_hz == 2u * flash_access_max_hz);

using Boot = Clock<ClockSource::internal, 8'000'000>;
using Safe = Clock<ClockSource::pll, 96'000'000>;
using Full = Clock<ClockSource::pll, 144'000'000>;
using Paced = DynamicClock<Rates<Safe, Full>>;

static_assert(!Boot::flash_needs_halving);
static_assert(!Safe::flash_needs_halving);
static_assert(Full::flash_needs_halving);

// ---- the option bytes (32.4.6, 32.6) ----------------------------------------
static_assert(flash_rdpr_unprotected == 0xA5u);
static_assert(FlashOptionArea::base == 0x1FFFF800u);

// The two reset-related bits are stated the way a reader thinks about
// them, which is the INVERSE of the register's own sense (32.4.6: 0 =
// "system will be reset when entering Stop mode").
constexpr FlashOptions all_clear{false, false, false, false, false, 0xFFFFFFFFu};
static_assert(!all_clear.sector_protected(0));
static_assert(!all_clear.address_protected(0x1000u));
constexpr FlashOptions first_locked{false, false, false, false, false, 0xFFFFFFFEu};
static_assert(first_locked.sector_protected(0));
static_assert(!first_locked.sector_protected(1));
static_assert(first_locked.address_protected(0u));
static_assert(first_locked.address_protected(0x0FFFu));
static_assert(!first_locked.address_protected(0x1000u));
// The last bit of WPR covers sectors 31 to 127 (32.6's WRPR3 note), so
// every index above 30 asks the same bit.
constexpr FlashOptions top_locked{false, false, false, false, false, 0x7FFFFFFFu};
static_assert(top_locked.sector_protected(31));
static_assert(top_locked.sector_protected(127));
static_assert(!top_locked.sector_protected(30));

// ---- the medium (util/nv_heap.hpp's contract) -------------------------------
using Store = MainFlash<Safe>;
static_assert(FlashMedia<Store>);

static_assert(MainFlashPartition::zone_bytes == 4096u);
static_assert(MainFlashPartition::zone_pages == 16u);
static_assert(MainFlashPartition::storage_end == device::flash_bytes);
static_assert(MainFlashPartition::storage_base == device::flash_bytes - 4096u);
static_assert(Store::erase_size == 256u && Store::write_cell == 256u);
static_assert(Store::flash_end == MainFlashPartition::storage_end);
static_assert(Store::zone_count == 1);
static_assert(Store::erased_word == Flash::erased_word);

// The two compile-time predicates a program pins a constant address
// with, and what each of them refuses.
static_assert(MainFlashPartition::contains(MainFlashPartition::storage_base, 4096u));
static_assert(!MainFlashPartition::contains(MainFlashPartition::storage_base - 1u, 1u));
static_assert(!MainFlashPartition::contains(MainFlashPartition::storage_base, 4097u));
static_assert(MainFlashPartition::cell_aligned(MainFlashPartition::storage_base, 512u));
static_assert(!MainFlashPartition::cell_aligned(MainFlashPartition::storage_base + 1u, 256u));
static_assert(!MainFlashPartition::cell_aligned(MainFlashPartition::storage_base, 64u));

// The medium is ONE WRITE-PROTECTION UNIT, which is what makes it
// lockable without touching the image's own sectors (32.1's note 1).
static_assert(MainFlashPartition::zone_bytes == device::flash_protect_bytes);
static_assert(MainFlashPartition::storage_base % device::flash_protect_bytes == 0u);

// ---- every verb ------------------------------------------------------------
static uint8_t page_buffer[Flash::page_size];
static uint8_t read_buffer[64];

void flash_verbs() {
    // the locks
    (void)Flash::unlock();
    (void)Flash::unlock_standard();
    Flash::lock();
    (void)Flash::locked();
    (void)Flash::fast_locked();
    (void)Flash::options_locked();

    // the status and the interrupt
    (void)Flash::status();
    (void)Flash::busy();
    (void)Flash::write_busy();
    (void)Flash::operation_ended();
    (void)Flash::errors();
    Flash::clear_flags();
    Flash::interrupts(true, true);
    (void)Flash::end_interrupt();
    (void)Flash::error_interrupt();
    (void)Flash::isr();

    // the access clock
    (void)Flash::access_clock_whole();
    (void)Flash::access_clock_whole(Boot{}, true);
    (void)Flash::access_clock_whole(Safe{}, false);
    (void)Flash::access_hz(Safe{});
    (void)Flash::hclk_of(Safe{});

    // the enhanced read mode
    (void)Flash::enhanced_read();
    (void)Flash::enhanced_read(false);

    // reading
    Flash::read(0u, std::span<uint8_t>(read_buffer, sizeof read_buffer));
    (void)Flash::erased(MainFlashPartition::storage_base, Flash::page_size);

    // erasing and programming, at a rate that needs no halving
    (void)Flash::erase_page(Safe{}, MainFlashPartition::storage_base);
    (void)Flash::erase_sector(Safe{}, MainFlashPartition::storage_base);
    (void)Flash::erase_block32(Safe{}, 0u);
    (void)Flash::erase_chip(Safe{});
    (void)Flash::erase_chip(Safe{}, ChipErasePolicy::refuse);
    (void)Flash::program_half_word(Safe{}, MainFlashPartition::storage_base, 0x1234u);
    (void)Flash::program_page(Safe{}, MainFlashPartition::storage_base,
                              std::span<const uint8_t>(page_buffer, Flash::page_size));

    // and under a clock whose rate is a fact of the moment: the same
    // verbs, refused with a CODE rather than at compile time
    (void)Flash::erase_page(Paced{}, MainFlashPartition::storage_base);
    (void)Flash::program_page(Paced{}, MainFlashPartition::storage_base,
                              std::span<const uint8_t>(page_buffer, Flash::page_size));
    (void)Flash::access_clock_whole(Paced{}, true);

    // the option bytes, read-only
    (void)Flash::options();
    (void)Flash::write_protection();
    (void)Flash::read_protected();
    (void)FlashOptions::read();
    (void)FlashOptionArea::read();
    (void)FlashOptionArea::consistent();
    (void)FlashOptionArea::raw()[0];

    // the signature
    (void)Flash::uid();
    (void)Flash::size_kbytes();
    (void)DeviceUid::read();
    (void)flash_size_kbytes();
    (void)esig_flash_kbytes();
    (void)esig_uid_word(0);
}

void media_verbs() {
    (void)MainFlashPartition::geometry_matches_silicon();
    (void)Store::zones();
    Store::read(MainFlashPartition::storage_base, std::span<uint8_t>(read_buffer, sizeof read_buffer));
    (void)Store::program(MainFlashPartition::storage_base,
                         std::span<const uint8_t>(page_buffer, Flash::page_size));
    (void)Store::erase(MainFlashPartition::storage_base);
    (void)Store::build_id();
}
