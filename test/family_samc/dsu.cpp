// Family smoke TU for samc/dsu.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The DSU is one instance with one register map on every member of the
// family. What varies is the DEVICE the DID reports, which is the point
// of the block - so nothing here asserts a DID value; what is pinned is
// the decode, the length arithmetic and the identifier the reserve
// probes out of the header.

#include <stdint.h>

#include "samc/dsu.hpp"

using namespace brio;

static_assert(Dsu::pac_id == 33);

// The revision letter is what the errata matrix is indexed by, and it is
// the one derived value this header computes.
static_assert(DsuDeviceId{0, 0, 0, 0, 0, 0, 0}.revision_letter() == 'A');
static_assert(DsuDeviceId{0, 0, 0, 0, 0, 5, 0}.revision_letter() == 'F');

// AMOD is one two-bit field with two meanings; both namings are the
// chapter's (tables 13-3 and 13-5), and zero is what a CPU-side caller
// wants either way.
static_assert(static_cast<uint8_t>(DsuAccessMode::array_or_exit_on_error) == 0);
static_assert(static_cast<uint8_t>(DsuAccessMode::eeprom_or_pause_on_error) == 1);

static_assert(DsuStatus::done == 0x01);
static_assert(DsuStatus::crstext == 0x02);
static_assert(DsuStatus::bus_error == 0x04);
static_assert(DsuStatus::fail == 0x08);
static_assert(DsuStatus::protection_error == 0x10);

void verbs() {
    Dsu::bus_clock(true);
    (void)Dsu::init();
    Dsu::release();

    (void)Dsu::did_raw();
    const DsuDeviceId id = Dsu::device_id();
    (void)id.processor;
    (void)id.family;
    (void)id.series;
    (void)id.die;
    (void)id.revision;
    (void)id.devsel;
    (void)id.revision_letter();

    (void)Dsu::status();
    Dsu::clear_status();
    (void)Dsu::done();
    (void)Dsu::bus_error();
    (void)Dsu::failed();
    (void)Dsu::protection_error();
    (void)Dsu::cpu_reset_extended();
    Dsu::release_cpu_reset();
    (void)Dsu::status_b();
    (void)Dsu::device_protected();
    (void)Dsu::debugger_present();
    (void)Dsu::hot_plugging_enabled();
    (void)Dsu::dcc_dirty(0);
    (void)Dsu::status_c_raw();
    (void)Dsu::dcfg_raw(0);

    (void)Dsu::dcc(0);
    Dsu::dcc(1, 0x12345678UL);

    Dsu::set_address(0x20001000UL, DsuAccessMode::array_or_exit_on_error);
    Dsu::set_length_words(64);
    (void)Dsu::address_raw();
    (void)Dsu::length_raw();
    (void)Dsu::data();
    Dsu::data(0xFFFFFFFFUL);
    Dsu::reset_module();
    (void)Dsu::wait_done(1000);

    (void)Dsu::crc32(0, 64);
    (void)Dsu::crc32_raw(0, 64, 0xFFFFFFFFUL);

    const DsuMbistResult m = Dsu::mbist(0x20001000UL, 16, false, 1000);
    (void)m.done;
    (void)m.failed;
    (void)m.phase;
    (void)m.bit_index;

    (void)Dsu::rom_entry(0);
    (void)Dsu::rom_entry_present(1);
    (void)Dsu::rom_entry_offset(0);
    (void)Dsu::rom_end();
    (void)Dsu::rom_memtype();
    (void)Dsu::rom_pid(4);
    (void)Dsu::rom_cid(0);
    (void)Dsu::rom_partnum();
    (void)Dsu::regs().DSU_DID;
}
