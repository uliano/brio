// Family smoke TU for samc/nvm.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three).
//
// NVMCTRL is one instance on every member of the family and its register
// layout does not vary; what DOES vary between the 15/16/17/18 sizes is
// the geometry, and the whole point of the constants below is that they
// come from the device header rather than from a constant this driver
// invents. The three variants swept here are all 18A, so the numbers are
// the same on all three - the assertions are still worth making, because
// they are what would catch a future 15/16/17 header being wired up
// wrongly.

#include <stdint.h>

#include <span>

#include "samc/clock.hpp"
#include "samc/nvm.hpp"

using namespace brio;

// ---- geometry, as chapter 27 fixes it ---------------------------------------

static_assert(Nvm::page_size == 64u);
static_assert(Nvm::row_size == 256u);
static_assert(Nvm::pages_per_row == 4u);
static_assert(Nvm::row_size % Nvm::page_size == 0u);
static_assert(Nvm::main_size == 262144u, "the 18A parts carry 256 KB");
static_assert(Nvm::rwwee_size == 8192u, "and 8 KB of RWWEE (table 9-3)");
static_assert(Nvm::main_pages * Nvm::page_size == Nvm::main_size);
static_assert(Nvm::rwwee_pages * Nvm::page_size == Nvm::rwwee_size);
static_assert(Nvm::rwwee_base == 0x00400000UL);
static_assert(Nvm::region_count == 16u);
static_assert(Nvm::region_size == Nvm::main_size / 16u);

// The user row is 0x00804000 and NOT the header's NVMCTRL_USER_PAGE_OFFSET,
// which names the base of the whole auxiliary space: the two differ by
// 0x4000, and mixing them up reads calibration data as fuses.
static_assert(Nvm::user_row == 0x00804000UL);
static_assert(Nvm::aux_base == 0x00800000UL);
static_assert(Nvm::user_row - Nvm::aux_base == 0x4000UL);

// ---- ADDR's encoding, as measured on silicon --------------------------------
//
// Section-relative half-words (27.8.8), confirmed at the bench by reading
// ADDR back after a page-buffer load on each array.
static_assert(Nvm::addr_field(NvmArray::rwwee, 0x00400100UL) == 0x80u);
static_assert(Nvm::addr_field(NvmArray::main, 0x00020100UL) == 0x10080u);
static_assert(Nvm::addr_field(NvmArray::main, 0u) == 0u);
static_assert(Nvm::addr_field(NvmArray::rwwee, Nvm::rwwee_base) == 0u);

static_assert(Nvm::in_array(NvmArray::main, Nvm::main_size - 1u));
static_assert(!Nvm::in_array(NvmArray::main, Nvm::main_size));
static_assert(!Nvm::in_array(NvmArray::main, Nvm::rwwee_base));
static_assert(Nvm::in_array(NvmArray::rwwee, Nvm::rwwee_end - 1u));
static_assert(!Nvm::in_array(NvmArray::rwwee, Nvm::rwwee_end));

static_assert(Nvm::region_of(0u) == 0u);
static_assert(Nvm::region_of(Nvm::region_size) == 1u);
static_assert(Nvm::region_of(Nvm::main_size - 1u) == 15u);

// ---- the wait-state verb, now owned by this header --------------------------

static_assert(FlashWaitStates::for_hz(19'000'000) == 0);
static_assert(FlashWaitStates::for_hz(24'000'000) == 1);
static_assert(FlashWaitStates::for_hz(48'000'000) == 2);

// ---- user row decoding, against table 27-2 / 27-3 ---------------------------
//
// The production setting is 0x7 for both fields = nothing reserved. The
// tables are geometric from there, which is what these pin.
constexpr NvmUserRow production{0xB15088FFUL, 0xFFFFF8BBUL};
static_assert(production.bootprot_field() == 0x7u);
static_assert(production.bootprot_rows() == 0u);
static_assert(production.eeprom_field() == 0x7u);
static_assert(production.eeprom_rows() == 0u);
static_assert(production.bodvdd_level() == 0x8u, "the production BODVDD level");
static_assert(!production.wdt_enabled());
static_assert(!production.wdt_always_on());

constexpr NvmUserRow protected_row{0xB1508800UL | 0x1u | (0x2u << 4), 0xFFFFF8BBUL};
static_assert(protected_row.bootprot_field() == 0x1u);
static_assert(protected_row.bootprot_rows() == 64u, "table 27-2: 0x1 -> 64 rows");
static_assert(protected_row.bootprot_bytes() == 16384u);
static_assert(protected_row.eeprom_field() == 0x2u);
static_assert(protected_row.eeprom_rows() == 16u, "table 27-3: 0x2 -> 16 rows");
static_assert(protected_row.eeprom_bytes() == 4096u);

// ---- every verb instantiates ------------------------------------------------

void block_verbs() {
    constexpr NvmConfig cfg{
        .wait_states = 2,
        .cache = true,
        .read_mode = NvmReadMode::deterministic,
        .sleep_power = NvmSleepPower::disabled,
        .manual_write = true,
    };
    Nvm::init<cfg>();          // compile-time: an impossible field is a build error
    (void)Nvm::init(cfg);      // run-time: an impossible field is a false return
    (void)Nvm::config_valid(cfg);
    (void)Nvm::ctrlb();
    (void)Nvm::cache_enabled();
    Nvm::cache(false);
    (void)Nvm::read_mode();
    (void)Nvm::manual_write();

    (void)Nvm::param();
    (void)Nvm::param_main_pages();
    (void)Nvm::param_rwwee_pages();
    (void)Nvm::param_page_size();
    (void)Nvm::geometry_matches();
}

void status_verbs() {
    (void)Nvm::ready();
    (void)Nvm::flags();
    (void)Nvm::armed();
    Nvm::clear_flags(NvmFlag::ready | NvmFlag::error);
    (void)Nvm::pending();
    Nvm::arm(NvmFlag::ready);
    Nvm::disarm(NvmFlag::error);
    (void)Nvm::status_bits();
    (void)Nvm::status();
    Nvm::clear_status();
    (void)Nvm::take_status();
    (void)Nvm::security_bit();
    (void)Nvm::isr();
}

void command_verbs() {
    (void)Nvm::command(NVMCTRL_CTRLA_CMD_PBC_Val);
    (void)Nvm::outcome();
    Nvm::address(NvmArray::rwwee, Nvm::rwwee_base);
    (void)Nvm::address();
    (void)Nvm::clear_page_buffer();
    (void)Nvm::invalidate_cache();
    (void)Nvm::enter_power_reduction();
    (void)Nvm::exit_power_reduction();
    (void)Nvm::power_reduced();
}

void memory_verbs() {
    static uint8_t page[Nvm::page_size];

    (void)Nvm::erase_row(NvmArray::main, 0x00010000UL);
    (void)Nvm::erase_row(NvmArray::rwwee, Nvm::rwwee_base);
    (void)Nvm::program_page(NvmArray::main, 0x00010000UL,
                            std::span<const uint8_t>(page));
    (void)Nvm::program_page(NvmArray::rwwee, Nvm::rwwee_base,
                            std::span<const uint8_t>(page));
    Nvm::read(Nvm::rwwee_base, std::span<uint8_t>(page));
    (void)Nvm::read_word(Nvm::user_row);

    (void)Nvm::locks();
    (void)Nvm::region_locked(3);
    (void)Nvm::lock_region(0x00010000UL);
    (void)Nvm::unlock_region(0x00010000UL);
}

void factory_views() {
    (void)NvmUserRow::read().bootprot_bytes();
    (void)NvmCalibration::read().osc32k_calib();
    (void)NvmCalibration::read().cal48m_5v();
    (void)NvmCalibration::read().cal48m_3v3();
    (void)NvmTemperatureCalibration::read().tsens_gain();
    (void)NvmTemperatureCalibration::read().tsens_offset();
    (void)DeviceSerial::read().word[3];
}
