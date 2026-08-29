// Family smoke TU for samc/pac.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The PAC is one instance with one register map on every member of the
// family, so what this fixture pins is the IDENTIFIER ARITHMETIC of
// 11.7.1 - the one decision this driver makes rather than reads from a
// register - and the bridge count the register map really has.

#include <stdint.h>

#include "samc/pac.hpp"

using namespace brio;

// PERID = 32 x bridge + index, and the bit within STATUSn/INTFLAGn is
// the index. Checked in both directions.
static_assert(Pac::bridge_of(0) == 0);
static_assert(Pac::bridge_of(31) == 0);
static_assert(Pac::bridge_of(32) == 1);
static_assert(Pac::bridge_of(64) == 2);
static_assert(Pac::bit_of(0) == 1UL);
static_assert(Pac::bit_of(33) == (1UL << 1));
static_assert(Pac::bit_of(87) == (1UL << 23), "CCL is bridge C bit 23");
static_assert(Pac::perid_of(2, 23) == 87);
static_assert(Pac::perid_of(1, 4) == 36, "the MTB, whose id table 12-3 omits");

// The three bridges this register map has. A fourth exists on the C21N
// and neither its header nor a board is here, so an identifier that
// would land on it must be refused rather than aimed at bridge C.
static_assert(Pac::bridge_count == 3);
static_assert(Pac::id_valid(0));
static_assert(Pac::id_valid(87));
static_assert(!Pac::id_valid(96), "bridge D is the C21N's alone");
static_assert(!Pac::id_valid(0xFFFF));

// The PAC's own identifier, from the device header's ID_ table.
static_assert(Pac::pac_id == 0);

// The keys are the register's, not an invention of this header.
static_assert(static_cast<uint8_t>(PacKey::off) == 0);
static_assert(static_cast<uint8_t>(PacKey::clear) == 1);
static_assert(static_cast<uint8_t>(PacKey::set) == 2);
static_assert(static_cast<uint8_t>(PacKey::lock) == 3);

// The event generator this peripheral publishes into evsys.hpp.
static_assert(Pac::ev_gen_error == 86);

void verbs() {
    Pac::bus_clock(true);

    (void)Pac::write_control(Pac::pac_id, PacKey::off);
    (void)Pac::protect(87);
    (void)Pac::unprotect(87);
    (void)Pac::lock(87);
    (void)Pac::is_protected(87);

    (void)Pac::status(0);
    (void)Pac::flags(1);
    Pac::clear_flags(2, 0xFFFFFFFFUL);
    (void)Pac::flagged(33);
    Pac::clear_flag(33);
    (void)Pac::ahb_flags();
    Pac::clear_ahb_flags();
    Pac::clear_ahb_flags(PacAhbFlag::divas);
    Pac::clear_all_flags();
    (void)Pac::any_error();

    (void)Pac::armed();
    Pac::arm(true);
    Pac::event_output(true);
    (void)Pac::event_output();
    (void)Pac::irq();

    const Pac::Report r = Pac::isr();
    (void)r.any();
}
