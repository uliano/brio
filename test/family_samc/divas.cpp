// Family smoke TU for samc/divas.hpp: every verb must COMPILE on the E,
// G and J 18A headers (tools/check_samc.sh sweeps all three).
//
// DIVAS is one instance at one address on every member of the family and
// has no package variation at all. What this fixture pins is the pair of
// ADDRESSES - one from the device header, one spelled from the data
// sheet's memory map because no header in this pack defines it - and
// that both bus paths instantiate.

#include <stdint.h>

#include "samc/divas.hpp"

using namespace brio;

// The high-speed bus address is the header's.
static_assert(Divas::ahb_base == 0x48000000UL);

// The IOBUS alias is the DATA SHEET's: table 9-1 puts the IOBUS region
// at 0x60000000 and figure 8-3 places PORT at its base with DIVAS at
// +0x200. No device header in this pack defines it.
static_assert(Divas::iobus_base == 0x60000200UL);
static_assert(Divas::iobus_base != Divas::ahb_base);

void verbs() {
    Divas::bus_clock(true);
    (void)Divas::bus_clock();

    Divas::configure(false);
    Divas::configure(true, true);
    (void)Divas::ctrla();
    (void)Divas::signed_division();
    (void)Divas::leading_zero_disabled();

    (void)Divas::status();
    (void)Divas::busy();
    (void)Divas::divide_by_zero();
    Divas::clear_divide_by_zero();
    (void)Divas::wait_idle(100);

    // Both bus paths, both operations, both signednesses.
    const DivasResult a = Divas::divide_unsigned(1000, 7);
    const DivasResult b = Divas::divide_unsigned<DivasBus::iobus>(1000, 7);
    const DivasSignedResult c = Divas::divide_signed(-1000, 7);
    const DivasSignedResult d = Divas::divide_signed<DivasBus::iobus>(-1000, 7);
    const DivasResult e = Divas::square_root(1'000'000UL);
    const DivasResult f = Divas::square_root<DivasBus::iobus>(1'000'000UL);
    (void)a.result;
    (void)a.remainder;
    (void)b.result;
    (void)c.result;
    (void)d.remainder;
    (void)e.result;
    (void)f.remainder;

    (void)Divas::regs().DIVAS_STATUS;
    (void)Divas::io_regs().DIVAS_STATUS;
    (void)Divas::at<DivasBus::ahb>().DIVAS_STATUS;
}
