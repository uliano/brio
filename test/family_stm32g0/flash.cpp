// FLASH family smoke TU: the whole of stm32g0/flash.hpp (RM0444 ch. 3)
// against every device header the pack ships for this stratum.
//
// WHAT THIS TU IS FOR ON THIS PARTICULAR CHAPTER: the second bank is the
// family's real split. The G0B1 declares FLASH_CR.BKER/MER2, FLASH_SR.BSY2,
// the two bank-2 option bits and the bank-2 ECC and protection
// registers; the G071 and G031 declare NONE of them, and ECC2R is not
// even a member of their FLASH_TypeDef. So a driver that names
// FLASH->ECC2R does not compile on two thirds of the family, and this TU
// is what says so before a bench does. Everything the preprocessor has
// to ask lives in stm32g0/device_tables.hpp; nothing below is guarded.
#include "stm32g0/device_tables.hpp"
#include "stm32g0/flash.hpp"

using namespace brio;

// ---- the wait states, table 13's Range 1 and Range 2 columns ---------------
static_assert(FlashWaitStates::for_hz(16'000'000UL) == 0);
static_assert(FlashWaitStates::for_hz(24'000'000UL) == 0);
static_assert(FlashWaitStates::for_hz(24'000'001UL) == 1);
static_assert(FlashWaitStates::for_hz(48'000'000UL) == 1);
static_assert(FlashWaitStates::for_hz(64'000'000UL) == 2);
static_assert(FlashWaitStates::for_hz_range2(8'000'000UL) == 0);
static_assert(FlashWaitStates::for_hz_range2(16'000'000UL) == 1);
static_assert(FlashWaitStates::max_latency == 2);

// ---- the geometry, 3.2 and 3.3.1 ------------------------------------------
static_assert(Flash::base == 0x0800'0000UL);
static_assert(Flash::page_size == 2048);
static_assert(Flash::subpage_size == 512);
static_assert(Flash::row_size == 256);
static_assert(Flash::cell_size == 8);
static_assert(Flash::cells_per_row == 32);
static_assert(Flash::max_pages_per_bank == 128);
static_assert(Flash::otp_base == 0x1FFF'7000UL && Flash::otp_size == 1024);
static_assert(Flash::option_bytes_base == 0x1FFF'7800UL);
// 3.3.6's two keys, spelled out so a typo is a compile failure and not a
// board that needs a power cycle.
static_assert(Flash::key1 == 0x4567'0123UL);
static_assert(Flash::key2 == 0xCDEF'89ABUL);

// ---- the status vocabulary -------------------------------------------------
// Every error bit is in `errors`, and the driver's own `refused` is NOT a
// silicon bit: bits 31:19 of FLASH_SR are Reserved.
static_assert((FlashFlag::errors & FlashFlag::program_error) != 0);
static_assert((FlashFlag::errors & FlashFlag::size_error) != 0);
static_assert((FlashFlag::errors & FlashFlag::alignment_error) != 0);
static_assert((FlashFlag::errors & FlashFlag::sequence_error) != 0);
static_assert((FlashFlag::errors & FlashFlag::write_protect_error) != 0);
static_assert((FlashFlag::errors & FlashFlag::miss_error) != 0);
static_assert((FlashFlag::errors & FlashFlag::fast_error) != 0);
static_assert((FlashFlag::errors & FlashFlag::option_error) != 0);
static_assert((FlashFlag::errors & FlashFlag::eop) == 0);
static_assert((FlashFlag::clearable & FlashFlag::eop) != 0);
static_assert((FlashFlag::errors & FlashFlag::refused) == 0);
static_assert(FlashFlag::refused == 0x8000'0000UL);
static_assert((FlashFlag::busy & FlashFlag::bank1_busy) != 0);
// BSY2 exists exactly where the second bank does.
static_assert((FlashFlag::bank2_busy != 0u) == flash_dual_bank_capable);

// ---- the reserve's own claims ---------------------------------------------
static_assert((flash_cr_bank_select != 0u) == flash_dual_bank_capable);
static_assert((flash_cr_mass_erase2 != 0u) == flash_dual_bank_capable);
static_assert((flash_optr_dual_bank != 0u) == flash_dual_bank_capable);
static_assert((flash_optr_swap_bank != 0u) == flash_dual_bank_capable);

void flash_geometry() {
    (void)flash_size_kb();
    (void)Flash::size_bytes();
    (void)Flash::page_count();
    (void)Flash::bank_count();
    (void)Flash::bank_size();
    (void)Flash::pages_per_bank();
    (void)Flash::banks_swapped();
    (void)Flash::erase_bank_of(Flash::base);
    (void)Flash::page_of(Flash::base);
    (void)Flash::in_main_flash(Flash::base);
}

void flash_accelerators() {
    (void)FlashWaitStates::get();
    (void)FlashWaitStates::set(2);
    (void)FlashAccel::prefetch();
    FlashAccel::prefetch(false);
    (void)FlashAccel::instruction_cache();
    FlashAccel::instruction_cache(true);
    FlashAccel::flush_instruction_cache();
    (void)FlashAccel::user_flash_empty();
    (void)FlashAccel::debug_access();
}

void flash_status() {
    (void)Flash::status();
    (void)Flash::busy();
    (void)Flash::config_busy();
    (void)Flash::errors();
    Flash::clear(FlashFlag::clearable);
    Flash::clear_errors();
    (void)Flash::last_status();
    (void)Flash::last_wait_turns();
    (void)Flash::wait_ready();
}

void flash_lock() {
    (void)Flash::locked();
    (void)Flash::option_locked();
    (void)Flash::unlock();
    (void)Flash::lock();
}

void flash_operations() {
    static uint8_t buf[Flash::row_size];
    // Named, not run: every one of these costs an erase cycle, and
    // mass_erase(bank1) would end the session.
    if (false) {
        (void)Flash::erase_page(Flash::base);
        (void)Flash::mass_erase(FlashBank::bank1);
        (void)Flash::mass_erase(FlashBank::bank2);
        (void)Flash::program(Flash::base, std::span<const uint8_t>(buf, 8));
        (void)Flash::fast_program_row(
            Flash::base, std::span<const uint8_t>(buf, Flash::row_size));
        (void)Flash::provoke(Flash::Misstep::half_word_store, Flash::base);
        (void)Flash::provoke(Flash::Misstep::unaligned_double_word, Flash::base);
        (void)Flash::provoke(Flash::Misstep::store_without_pg, Flash::base);
    }
    Flash::read(Flash::base, std::span<uint8_t>(buf, 8));
    (void)Flash::read_otp(0, std::span<uint8_t>(buf, 8));
}

void flash_interrupt() {
    static_assert(Flash::irq() == FLASH_IRQn);
    (void)Flash::interrupts(true, true, true);
    (void)Flash::isr();
    (void)Flash::last_isr_ecc();
}

void flash_ecc() {
    // Bank 2's ECC register is a struct member the single-bank headers do
    // not declare, which is why it is reached through the reserve's
    // pointer and not by name.
    const FlashEccStatus one = Flash::ecc(FlashBank::bank1);
    const FlashEccStatus two = Flash::ecc(FlashBank::bank2);
    (void)one.corrected;
    (void)one.detected;
    (void)one.system_flash;
    (void)one.double_word_offset;
    (void)two.present;
    Flash::clear_ecc(FlashBank::bank1);
    Flash::clear_ecc(FlashBank::bank2);
    Flash::ecc_correction_interrupt(FlashBank::bank1, true);
    Flash::ecc_correction_interrupt(FlashBank::bank2, false);
}

void flash_options() {
    (void)FlashOptions::raw();
    (void)FlashOptions::rdp_code();
    (void)(FlashOptions::rdp() == FlashRdpLevel::level0);
    (void)FlashOptions::bor_enabled();
    (void)FlashOptions::bor_rising_level();
    (void)FlashOptions::bor_falling_level();
    (void)FlashOptions::reset_on_stop();
    (void)FlashOptions::reset_on_standby();
    (void)FlashOptions::reset_on_shutdown();
    (void)FlashOptions::iwdg_software();
    (void)FlashOptions::iwdg_runs_in_stop();
    (void)FlashOptions::iwdg_runs_in_standby();
    (void)FlashOptions::wwdg_software();
    (void)FlashOptions::ram_parity_check();
    (void)FlashOptions::dual_bank_bit();
    (void)FlashOptions::banks_swapped();
    (void)FlashOptions::boot0_from_option();
    (void)FlashOptions::nboot0();
    (void)FlashOptions::nboot1();
    (void)FlashOptions::nrst_mode();
    (void)FlashOptions::internal_reset_holder();
    (void)FlashOptions::pcrop_erased_on_rdp_regression();
    (void)FlashOptions::boot_lock();
    for (uint8_t area = 0; area < 2u; ++area) {
        const FlashWrpArea w1 = FlashOptions::wrp(FlashBank::bank1, area);
        const FlashWrpArea w2 = FlashOptions::wrp(FlashBank::bank2, area);
        (void)w1.empty();
        (void)w2.empty();
        const FlashPcropArea p1 = FlashOptions::pcrop(FlashBank::bank1, area);
        const FlashPcropArea p2 = FlashOptions::pcrop(FlashBank::bank2, area);
        (void)p1.empty();
        (void)p2.empty();
    }
    (void)FlashOptions::securable_pages(FlashBank::bank1);
    (void)FlashOptions::securable_pages(FlashBank::bank2);
}
