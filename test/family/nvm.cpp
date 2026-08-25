// NVMCTRL family smoke TU: every package must compile this
// (instantiation only - nothing here is ever executed).
//
// NVMCTRL is the same block on all eight AVR128DA/DB packages: the
// register set, every command code, every error code and the whole
// geometry (128 KB Flash in 512-byte pages, 512 bytes of EEPROM, a
// 32-byte User Row) are identical, so there are no package tiers to
// gate. What this TU proves is that the surface instantiates
// everywhere, that the enums really carry the device header's own
// codes, and that the compile-time verbs accept what the declared Flash
// layout allows.
#include "avrdx/nvm.hpp"

using namespace brio;

// The geometry is the device header's, not a copy of it.
static_assert(flash_size == PROGMEM_SIZE);
static_assert(flash_page_size == PROGMEM_PAGE_SIZE);
static_assert(eeprom_size == EEPROM_SIZE);
static_assert(eeprom_base == EEPROM_START);
static_assert(userrow_size == USER_SIGNATURES_SIZE);
static_assert(userrow_base == USER_SIGNATURES_START);

// The error codes ARE the register's (11.5.3). The device headers list
// two codes where the data sheet lists one collision code; both are
// named here so neither document can drift unnoticed.
static_assert(static_cast<uint8_t>(NvmError::none) ==
              (NVMCTRL_ERROR_NOERROR_gc >> NVMCTRL_ERROR_gp));
static_assert(static_cast<uint8_t>(NvmError::invalid_command) ==
              (NVMCTRL_ERROR_ILLEGALCMD_gc >> NVMCTRL_ERROR_gp));
static_assert(static_cast<uint8_t>(NvmError::write_protect) ==
              (NVMCTRL_ERROR_ILLEGALSADDR_gc >> NVMCTRL_ERROR_gp));
static_assert(static_cast<uint8_t>(NvmError::command_collision) ==
              (NVMCTRL_ERROR_DOUBLESELECT_gc >> NVMCTRL_ERROR_gp));
static_assert(static_cast<uint8_t>(NvmError::ongoing_program) ==
              (NVMCTRL_ERROR_ONGOINGPROG_gc >> NVMCTRL_ERROR_gp));

// The erase spans are page/byte COUNTS, and each maps to its own
// command (tables 11-4 and 11-5).
static_assert(erase_pages(FlashErase::page) == 1);
static_assert(erase_pages(FlashErase::pages32) == 32);
static_assert(erase_bytes(EepromErase::byte) == 1);
static_assert(erase_bytes(EepromErase::bytes32) == 32);

// The DB errata's "EEWP" bit does not exist on this family: CTRLB is
// FLMAPLOCK, FLMAP, APPDATAWP, BOOTRP, APPCODEWP and nothing else.
// Guarded so that a future device header that DOES grow the bit makes
// this TU fail rather than leaving the doc's claim silently wrong.
#if defined(NVMCTRL_EEWP_bm)
#error "this family has grown an EEPROM write-protect bit: nvm.md is wrong"
#endif

// The sections a fuse geometry produces, at compile time (table 11-2).
using WholeBoot = FlashLayout<0>;                 // shipping default
using BootOnly = FlashLayout<128, 0>;             // BOOT 0..64K, APPCODE above
using WithData = FlashLayout<128, 192>;           // ... and APPDATA above 96K
using NoAppcode = FlashLayout<128, 64>;           // CODESIZE <= BOOTSIZE

static_assert(WholeBoot::boot_end == flash_size);
static_assert(!WholeBoot::writable(0, flash_size));
static_assert(BootOnly::boot_end == 0x10000u);
static_assert(BootOnly::appcode_end == flash_size);
static_assert(BootOnly::section_of(0) == FlashSection::boot);
static_assert(BootOnly::section_of(0x10000u) == FlashSection::appcode);
static_assert(!BootOnly::writable(0, 0x200u));
static_assert(BootOnly::writable(0x10000u, 0x10200u));
static_assert(WithData::appcode_end == 0x18000u);
static_assert(WithData::section_of(0x18000u) == FlashSection::appdata);
static_assert(NoAppcode::appcode_end == NoAppcode::boot_end);
static_assert(NoAppcode::section_of(0x10000u) == FlashSection::appdata);

volatile uint8_t nvm_sink;

void nvm_state() {
    nvm_sink = static_cast<uint8_t>(Nvm::flash_busy());
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_busy());
    nvm_sink = static_cast<uint8_t>(Nvm::busy());
    nvm_sink = static_cast<uint8_t>(Nvm::error());
    Nvm::clear_error();
    nvm_sink = static_cast<uint8_t>(Nvm::wait_idle());
    nvm_sink = static_cast<uint8_t>(Nvm::wait_flash());
    nvm_sink = static_cast<uint8_t>(Nvm::wait_eeprom());
    Nvm::clear_command();
    Nvm::select(NvmCommand::eeprom_erase_write);
    nvm_sink = static_cast<uint8_t>(Nvm::selected());
    Nvm::select(NvmCommand::none);
}

// Every command code IS the register's (11.5.1), and the two that erase
// a whole memory are deliberately absent from the enum.
static_assert(static_cast<uint8_t>(NvmCommand::none) == NVMCTRL_CMD_NONE_gc);
static_assert(static_cast<uint8_t>(NvmCommand::noop) == NVMCTRL_CMD_NOOP_gc);
static_assert(static_cast<uint8_t>(NvmCommand::flash_write) == NVMCTRL_CMD_FLWR_gc);
static_assert(static_cast<uint8_t>(NvmCommand::flash_page_erase) ==
              NVMCTRL_CMD_FLPER_gc);
static_assert(static_cast<uint8_t>(NvmCommand::flash_erase32) ==
              NVMCTRL_CMD_FLMPER32_gc);
static_assert(static_cast<uint8_t>(NvmCommand::eeprom_write) == NVMCTRL_CMD_EEWR_gc);
static_assert(static_cast<uint8_t>(NvmCommand::eeprom_erase_write) ==
              NVMCTRL_CMD_EEERWR_gc);
static_assert(static_cast<uint8_t>(NvmCommand::eeprom_byte_erase) ==
              NVMCTRL_CMD_EEBER_gc);
static_assert(static_cast<uint8_t>(NvmCommand::eeprom_erase32) ==
              NVMCTRL_CMD_EEMBER32_gc);
static_assert(static_cast<uint8_t>(NvmCommand::reserved) != NVMCTRL_CMD_CHER_gc &&
              static_cast<uint8_t>(NvmCommand::reserved) != NVMCTRL_CMD_EECHER_gc);

void nvm_ctrlb() {
    nvm_sink = Nvm::flmap();
    nvm_sink = static_cast<uint8_t>(Nvm::flmap_locked());
    nvm_sink = static_cast<uint8_t>(Nvm::set_flmap(2));
    Nvm::lock_flmap();
    Nvm::protect_appcode();
    Nvm::protect_appdata();
    Nvm::protect_boot_read();
    nvm_sink = static_cast<uint8_t>(Nvm::appcode_protected());
    nvm_sink = static_cast<uint8_t>(Nvm::appdata_protected());
    nvm_sink = static_cast<uint8_t>(Nvm::boot_read_protected());
    Nvm::vectors_in_boot();
    nvm_sink = static_cast<uint8_t>(Nvm::vectors_in_boot_armed());
}

void nvm_geometry() {
    nvm_sink = static_cast<uint8_t>(Nvm::boot_end() >> 16);
    nvm_sink = static_cast<uint8_t>(Nvm::appcode_end() >> 16);
    nvm_sink = static_cast<uint8_t>(Nvm::section_of(0x10000u));
    nvm_sink = static_cast<uint8_t>(Nvm::writable(0x10000u, 0x14000u));
    nvm_sink = static_cast<uint8_t>(BootOnly::matches_fuses());
    const FlashRange r = Nvm::scratch_region();
    nvm_sink = static_cast<uint8_t>(r.empty());
    nvm_sink = static_cast<uint8_t>(r.contains(r.begin, r.end));
    nvm_sink = static_cast<uint8_t>(r.size() >> 8);
    nvm_sink = static_cast<uint8_t>(Nvm::image_low_end() >> 8);
    nvm_sink = static_cast<uint8_t>(Nvm::rodata_load_start() >> 8);
    nvm_sink = static_cast<uint8_t>(Nvm::rodata_load_end() >> 8);
}

void nvm_flash() {
    uint8_t buf[8]{};
    nvm_sink = Nvm::flash_read(0x18000u);
    nvm_sink = static_cast<uint8_t>(Nvm::flash_read_word(0x18000u));
    Nvm::flash_read(0x18000u, buf, sizeof buf);
    nvm_sink = static_cast<uint8_t>(Nvm::flash_blank(0x18000u, 0x18008u));
    nvm_sink = static_cast<uint8_t>(Nvm::erase(0x10000u));
    nvm_sink = static_cast<uint8_t>(Nvm::erase(0x10000u, FlashErase::pages2));
    nvm_sink = static_cast<uint8_t>(Nvm::erase(0x10000u, FlashErase::pages4));
    nvm_sink = static_cast<uint8_t>(Nvm::erase(0x10000u, FlashErase::pages8));
    nvm_sink = static_cast<uint8_t>(Nvm::erase(0x10000u, FlashErase::pages16));
    nvm_sink = static_cast<uint8_t>(Nvm::erase(0x10000u, FlashErase::pages32));
    nvm_sink = static_cast<uint8_t>(Nvm::write_word(0x10000u, 0x1234u));
    nvm_sink = static_cast<uint8_t>(Nvm::write_block(0x10000u, buf, sizeof buf));
    nvm_sink = static_cast<uint8_t>(
        Nvm::erase_ignoring_protection(0x10000u, FlashErase::pages2));
    nvm_sink = static_cast<uint8_t>(
        Nvm::erase_at<BootOnly, 0x10000u, FlashErase::pages32>());
    nvm_sink = static_cast<uint8_t>(Nvm::write_word_at<BootOnly, 0x10000u>(1));
}

void nvm_eeprom() {
    uint8_t buf[4]{};
    nvm_sink = Nvm::eeprom_read(0);
    Nvm::eeprom_read(0, buf, sizeof buf);
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_write(0, 0x5Au));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_erase_write(1, 0xA5u));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_erase(0));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_erase(0, EepromErase::bytes2));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_erase(0, EepromErase::bytes4));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_erase(0, EepromErase::bytes8));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_erase(0, EepromErase::bytes16));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_erase(0, EepromErase::bytes32));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_write_block(0, buf, sizeof buf));
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_poke(0, 0x5Au));
    nvm_sink = static_cast<uint8_t>(
        Nvm::eeprom_erase_at<0, EepromErase::bytes32>());
    nvm_sink = static_cast<uint8_t>(Nvm::eeprom_ready_flag());
    Nvm::clear_eeprom_ready_flag();
    Nvm::enable_eeprom_ready_interrupt(true);
    Nvm::eeready();
}

void nvm_userrow() {
    uint8_t row[userrow_size]{};
    nvm_sink = Nvm::userrow_read(0);
    Nvm::userrow_read(0, row, userrow_size);
    nvm_sink = static_cast<uint8_t>(Nvm::userrow_write(31, 0x5Au));
    nvm_sink = static_cast<uint8_t>(Nvm::userrow_write_at<31>(0x5Au));
    nvm_sink = static_cast<uint8_t>(Nvm::userrow_write_block(0, row, 4));
    nvm_sink = static_cast<uint8_t>(Nvm::userrow_erase());
}

void nvm_sigrow() {
    nvm_sink = Sigrow::device_id(0);
    nvm_sink = static_cast<uint8_t>(Sigrow::device_id() >> 16);
    for (uint8_t i = 0; i < Sigrow::serial_bytes; ++i) {
        nvm_sink = Sigrow::serial(i);
    }
    nvm_sink = static_cast<uint8_t>(Sigrow::tempsense0());
    nvm_sink = static_cast<uint8_t>(Sigrow::tempsense1());
}

// The EEPROM as the util-stratum store, and the three things built on
// it: the record layout, the interrupt-paced writer AO and the
// persistent panic record. They are target-independent, but their only
// real backend is here, so this TU is where they get compiled for every
// package.
#include "avrdx/platform_avr.hpp"
#include "util/nv_writer.hpp"
#include "util/persistent_panic.hpp"

struct Settings {
    uint16_t baud_div;
    uint8_t flags;
};

using Store = EepromStore;
using SettingsRecord = NvRecord<Settings, Store, 16>;
using Panic = PersistentPanic<Store, 0>;
using Writer = NvWriter<Store, AvrPlatform>;

static_assert(nv_record_size<Settings> == 4 + sizeof(Settings));
static_assert(SettingsRecord::end <= eeprom_size);
static_assert(Panic::end <= SettingsRecord::begin,
              "the panic slot and the settings record must not overlap");

void nvm_util() {
    const std::optional<Settings> s = SettingsRecord::load();
    nvm_sink = static_cast<uint8_t>(s.has_value());
    nvm_sink = static_cast<uint8_t>(SettingsRecord::store(Settings{12, 3}));
    nvm_sink = static_cast<uint8_t>(SettingsRecord::valid());
    nvm_sink = static_cast<uint8_t>(SettingsRecord::clear());

    Panic::report(PanicCode::assert_failed, 7);
    nvm_sink = static_cast<uint8_t>(Panic::pending());
    const std::optional<PanicRecord> r = Panic::take();
    nvm_sink = r ? r->code : 0;
    nvm_sink = static_cast<uint8_t>(Panic::save(PanicRecord{panic_magic, 1, 2}));

    static const uint8_t payload[3] = {1, 2, 3};
    Writer::init();
    post<Writer>(NvWrite{32, payload, sizeof payload, ReplyTo<NvDone>{}});
    nvm_sink = Writer::rejected_count();
    while (const std::optional<Writer::Event> e = Writer::queue.pop()) {
        Writer::dispatch(*e);
    }
}
