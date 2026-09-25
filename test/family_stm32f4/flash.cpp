// The embedded flash memory interface: family smoke TU (RM0090 ch. 3,
// RM0390 ch. 3, RM0383 ch. 3).
//
// This chapter is the stratum's clearest case of THE DEVICE HEADER NOT
// BEING THE AUTHORITY, in both directions at once, and the assertions
// below are mostly about that: the pack declares a second bank's bits on
// a part that has one bank (the F446's FLASH_CR_MER2, FLASH_OPTCR_DB1M,
// FLASH_OPTCR_BFB2) and fails to declare a PCROP the manual describes in
// full (the F411's FLASH_OPTCR_SPRMOD). So what a part's flash interface
// IS comes from `flash_facts()`, keyed on the part class, and a fixture
// is the one place that may ask both sources and state where they part.
//
// The geometry is stated as a RULE and not a table - four sectors of 16
// Kbytes, one of 64, then 128 Kbytes to the end of the bank - so the
// checks on it are checks of the rule's arithmetic, the size register
// being a run-time read this TU cannot make.
#include "stm32f4/flash.hpp"

using namespace brio;

// ---- the reserve's own answers ----------------------------------------------

// The sector shape, and the numbering that skips four codes at the bank
// boundary (RM0090 3.9.8: 12..15 are "not allowed" and bank 2 starts at
// 0b10000).
static_assert(flash_small_sectors == 4);
static_assert(flash_small_sector_bytes == 16u * 1024u);
static_assert(flash_middle_sector_bytes == 64u * 1024u);
static_assert(flash_large_sector_bytes == 128u * 1024u);
static_assert(flash_bank2_first_sector == 12);
static_assert(flash_sector_number_code(0) == 0);
static_assert(flash_sector_number_code(11) == 11);
static_assert(flash_sector_number_code(12) == 16, "bank 2's first sector is SNB 0b10000");
static_assert(flash_sector_number_code(23) == 27);

// A class whose chapter was read says everything about itself; one that
// was not says nothing at all, and stm32f4/flash.hpp then has no sector
// map (neg/flash_bank2_erase_on_a_single_bank.cpp is the compile-time
// half of the same rule).
static_assert(flash_facts().known == Flash::geometry_known);
static_assert(!flash_facts().known || flash_facts().max_sectors_per_bank >= 8);
static_assert(!flash_facts().known || flash_facts().wrp_bits >= 8);
static_assert(flash_facts().snb_bits == 4 || flash_facts().snb_bits == 5);
static_assert(flash_facts().snb_bits == 5 ? flash_facts().dual_bank_capable : true,
              "five bits of SNB exist only to carry a bank");
static_assert(flash_facts().has_option_register1 == flash_facts().dual_bank_capable,
              "OPTCR1 is bank 2's write protection and exists with bank 2");
static_assert(!flash_facts().has_dual_bank_option || flash_facts().dual_bank_capable);
static_assert(!flash_facts().has_dual_bank_boot || flash_facts().dual_bank_capable);

// ASKED TWICE: RDERR is the one bit of this chapter the pack gets right
// on every part, so where the manual was read the two sources must agree.
static_assert(!flash_facts().known ||
                  (flash_facts().has_read_error == (flash_read_error_flag() != 0u)),
              "FLASH_SR.RDERR: the header and the manual agree wherever the manual was read");
static_assert(FlashFlag::read_protect_error == flash_read_error_flag());

// WHERE THEY DISAGREE, named. Two headers of the pack are wrong about
// this chapter, and both are bench parts, so both are asserted here
// rather than described in a comment.
#if defined(STM32F446xx)
static_assert(flash_bank2_mass_erase_mask() != 0u,
              "the F446's header declares FLASH_CR_MER2 ...");
static_assert(!flash_facts().dual_bank_capable,
              "... and RM0390 3.3 gives the part 512 Kbytes in ONE bank");
static_assert(flash_facts().max_sectors_per_bank == 8 && flash_facts().wrp_bits == 8);
#endif
#if defined(STM32F411xE)
static_assert(flash_pcrop_mode_mask() != 0u,
              "RM0383 3.6.5 gives the F411 PCROP, whatever SPRMOD the header omits");
static_assert(flash_facts().has_pcrop_mode && flash_facts().wrp_bits == 8);
static_assert(!flash_facts().dual_bank_capable && flash_bank2_mass_erase_mask() == 0u);
#endif
#if defined(STM32F429xx) || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
static_assert(flash_facts().dual_bank_capable && flash_facts().snb_bits == 5);
static_assert(flash_bank2_mass_erase_mask() == FLASH_CR_MER2, "RM0090 calls the bit MER1");
static_assert(flash_facts().has_option_register1 && flash_facts().max_sectors_per_bank == 12);
#endif
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
static_assert(flash_facts().known && !flash_facts().dual_bank_capable);
static_assert(!flash_facts().has_pcrop_mode && !flash_facts().has_read_error,
              "RM0090 3.6.6 gives RDERR to the F42x/F43x class alone");
static_assert(flash_facts().max_sectors_per_bank == 12 && flash_facts().wrp_bits == 12);
#endif

// The system memory, the OTP and the two option-byte blocks.
static_assert(flash_system_memory_base == 0x1FFF0000UL);
static_assert(flash_otp_base == 0x1FFF7800UL && flash_otp_bytes == 512);
static_assert(flash_otp_lock_base == flash_otp_base + flash_otp_bytes);
static_assert(flash_otp_blocks == 16);
static_assert(flash_option_bytes_base == 0x1FFFC000UL);
static_assert(flash_option_bytes_base_bank2 == 0x1FFEC000UL);
static_assert(flash_irq() == FLASH_IRQn);

// ---- the parallelism ----------------------------------------------------------

static_assert(flash_parallelism_bytes(FlashParallelism::x8) == 1);
static_assert(flash_parallelism_bytes(FlashParallelism::x16) == 2);
static_assert(flash_parallelism_bytes(FlashParallelism::x32) == 4);
static_assert(flash_parallelism_bytes(FlashParallelism::x64) == 8);
static_assert(flash_parallelism_min_mv(FlashParallelism::x32) == 2700, "table 6");
static_assert(flash_parallelism_min_mv(FlashParallelism::x64) == 0,
              "x64's condition is VPP, not a VDD - and it is refused at compile time "
              "(neg/flash_parallelism_needs_vpp.cpp)");

// ---- the flag vocabulary ------------------------------------------------------

// The two names that are the manual's and not the header's, pinned to
// their bits so a rename in the pack cannot pass unnoticed.
static_assert(FlashFlag::operation_error == (1UL << 1), "OPERR, which the header spells SOP");
static_assert(FlashFlag::busy == (1UL << 16));
static_assert(FlashFlag::eop == (1UL << 0));
static_assert(FlashFlag::write_protect_error == (1UL << 4));
static_assert(FlashFlag::alignment_error == (1UL << 5));
static_assert(FlashFlag::parallelism_error == (1UL << 6));
static_assert(FlashFlag::sequence_error == (1UL << 7));
static_assert((FlashFlag::errors & FlashFlag::busy) == 0u, "BSY is not an error");
static_assert((FlashFlag::clearable & FlashFlag::busy) == 0u, "and it is read-only");
static_assert((FlashFlag::refused & FlashFlag::clearable) == 0u,
              "the driver's own bit is one no store can reach");
static_assert(FlashFlag::refused == 0x80000000UL);

// ---- the wait states and the accelerator --------------------------------------

static_assert(FlashWaitStates::max_latency == 7 || FlashWaitStates::max_latency == 15,
              "three bits of LATENCY, or four where the ladder reaches 180 MHz");
static_assert(FlashWaitStates::needed_for(16'000'000u) == (sysclk_ladder().known ? 0u : 0xFFu));

// ---- the sector map's arithmetic ----------------------------------------------

static_assert(Flash::sector_size_in_bank(0) == 16u * 1024u);
static_assert(Flash::sector_size_in_bank(3) == 16u * 1024u);
static_assert(Flash::sector_size_in_bank(4) == 64u * 1024u);
static_assert(Flash::sector_size_in_bank(5) == 128u * 1024u);
static_assert(Flash::sector_size_in_bank(11) == 128u * 1024u);
static_assert(Flash::base == 0x08000000UL);
static_assert(Flash::key1 == 0x45670123UL && Flash::key2 == 0xCDEF89ABUL);
static_assert(Flash::option_key1 == 0x08192A3BUL && Flash::option_key2 == 0x4C5D6E7FUL);
static_assert(Flash::dual_bank_capable == flash_facts().dual_bank_capable);

// A sector descriptor is arithmetic and can be checked without silicon.
constexpr FlashSector s7{7, 1, 0x08060000UL, 128u * 1024u};
static_assert(s7.contains(0x08060000UL) && s7.contains(0x0807FFFFUL));
static_assert(!s7.contains(0x0805FFFFUL) && !s7.contains(0x08080000UL));

// ---- every verb, once ---------------------------------------------------------

void flash_read_side_verbs() {
    (void)FlashWaitStates::get();
    (void)FlashWaitStates::set(0);
    (void)FlashWaitStates::needed_for(100'000'000u);

    FlashAccel::prefetch(true);
    (void)FlashAccel::prefetch();
    FlashAccel::icache(false);
    (void)FlashAccel::icache();
    FlashAccel::dcache(false);
    (void)FlashAccel::dcache();
    (void)FlashAccel::icache_reset();
    (void)FlashAccel::dcache_reset();
    FlashAccel::flush_caches();
    FlashAccel::enable_all();

    (void)DeviceUid::read();
    (void)flash_size_kbytes();
    (void)DeviceIdcode::read();
}

void flash_geometry_verbs() {
    (void)Flash::size_bytes();
    (void)Flash::in_main_flash(0x08000000UL);
    (void)Flash::bank_count();
    (void)Flash::bank_bytes();
    (void)Flash::sectors_per_bank();
    (void)Flash::sector_count();
    (void)Flash::sector_at(0);
    (void)Flash::sector(7);
    (void)Flash::sector_of(0x08010000UL);
}

void flash_engine_verbs() {
    uint8_t buffer[8] = {};
    Flash::read(0x08000000UL, buffer);
    (void)Flash::blank(0x08000000UL, sizeof(buffer));
    (void)Flash::read_otp(0, buffer);
    (void)Flash::otp_block_locked(0);
    (void)Flash::otp_block_locked(16);

    (void)Flash::status();
    (void)Flash::busy();
    (void)Flash::errors();
    Flash::clear(FlashFlag::eop);
    Flash::clear_errors();
    (void)Flash::last_status();
    (void)Flash::last_wait_turns();
    (void)Flash::wait_ready();

    (void)Flash::locked();
    (void)Flash::unlock();
    (void)Flash::lock();

    // The erase engine at all three legal parallelisms, and the program
    // engine at each of its store widths.
    (void)Flash::erase_sector(7);
    (void)Flash::erase_sector<FlashParallelism::x8>(7);
    (void)Flash::erase_sector<FlashParallelism::x16>(7);
    (void)Flash::erase_sector<FlashParallelism::x32>(7);
    (void)Flash::mass_erase<FlashEraseAll::erase_every_user_sector>();
    (void)Flash::mass_erase<FlashEraseAll::erase_every_user_sector, FlashParallelism::x8>();
    (void)Flash::bank_erase<FlashBank::bank1>(FlashEraseAll::no);
    if constexpr (Flash::dual_bank_capable) {
        // Bank 2 exists only on the F42x/F43x class, and naming it
        // elsewhere is a compile error, not a run-time false.
        (void)Flash::bank_erase<FlashBank::bank2>(FlashEraseAll::erase_every_user_sector);
    }

    const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    (void)Flash::program(0x08060000UL, data);
    (void)Flash::program<FlashParallelism::x8>(0x08060000UL, data);
    (void)Flash::program<FlashParallelism::x16>(0x08060000UL, data);
    (void)Flash::program<FlashParallelism::x32>(0x08060000UL, data);
    (void)Flash::program_word(0x08060000UL, 0xDEADBEEFUL);

    (void)Flash::provoke(Flash::Misstep::store_without_pg, 0x08060000UL);
    (void)Flash::provoke(Flash::Misstep::wrong_access_width, 0x08060000UL);
    (void)Flash::provoke(Flash::Misstep::unaligned_word, 0x08060000UL);
    (void)Flash::provoke(Flash::Misstep::write_protected_area, 0x08060000UL);

    // The lockout is a verb of its own because it ends the engine's life
    // until the next reset; the fixture only has to prove it exists.
    Flash::provoke_wrong_key();

    (void)Flash::irq();
    (void)Flash::interrupts(true, true);
    (void)Flash::end_of_operation_interrupt();
    (void)Flash::error_interrupt();
    (void)Flash::isr();
}

void flash_option_verbs() {
    (void)FlashOptions::raw();
    (void)FlashOptions::raw_bank2();
    (void)FlashOptions::rdp_code();
    (void)FlashOptions::rdp();
    (void)FlashOptions::bor_level();
    (void)FlashOptions::bor_off();
    (void)FlashOptions::iwdg_software();
    (void)FlashOptions::reset_on_stop();
    (void)FlashOptions::reset_on_standby();
    (void)FlashOptions::pcrop_mode();
    (void)FlashOptions::dual_bank_boot();
    (void)FlashOptions::dual_bank_1m();
    (void)FlashOptions::nwrp(0);
    (void)FlashOptions::write_protected(0);
    (void)FlashOptions::pcrop_protected(0);
    (void)FlashOptions::locked();
    (void)FlashOptions::unlock();
    (void)FlashOptions::lock();
    (void)FlashOptions::protect(7, true);
    (void)FlashOptions::protect(7, false);
    // A sector no part of the family has: every verb answers, none of
    // them names a register.
    (void)FlashOptions::nwrp(31);
    (void)FlashOptions::protect(31, true);
}
