// Family smoke TU for samc/eic.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The EIC block itself is identical across this family - sixteen lines,
// two CONFIG registers, one vector. What DOES vary is which PADS are
// bonded to it, and that is exactly what this fixture pins: the pad-to-
// line map is read out of the device header, it is irregular, and the
// pads that only the bigger packages bond are asserted absent on the
// smaller ones.

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/eic.hpp"
#include "samc/pin.hpp"

using namespace brio;

static_assert(Eic::line_count == 16);
static_assert(Eic::config_regs == 2);
static_assert(Eic::gclk_id == EIC_GCLK_ID);
static_assert(Eic::valid_line(15) && !Eic::valid_line(16));

// The EVSYS generator codes are published HERE (evsys.hpp owns the
// fabric, not the vocabulary) and are contiguous from EXTINT0 = 0x0E.
static_assert(Eic::event_generator(0) == 0x0E);
static_assert(Eic::event_generator(15) == 0x1D);

// ---- the clock question, which is the chapter's real subtlety --------------
//
// 26.5.3 / 26.6.3: level detection without a filter, and asynchronous
// edge detection, need no clock at all; filtering and synchronous edge
// detection do.
static_assert(!eic_needs_clock(EicLineConfig{.sense = EicSense::high}));
static_assert(!eic_needs_clock(EicLineConfig{.sense = EicSense::low}));
static_assert(eic_needs_clock(EicLineConfig{.sense = EicSense::high, .filter = true}));
static_assert(eic_needs_clock(EicLineConfig{.sense = EicSense::rising}));
static_assert(!eic_needs_clock(
    EicLineConfig{.sense = EicSense::rising, .asynchronous = true}));
static_assert(eic_needs_clock(EicLineConfig{.sense = EicSense::both, .filter = true}));

// The one thing 26.8.10 forbids outright: the filter with asynchronous
// detection.
static_assert(eic_line_config_valid(EicLineConfig{.sense = EicSense::rising}));
static_assert(!eic_line_config_valid(
    EicLineConfig{.sense = EicSense::rising, .filter = true, .asynchronous = true}));
static_assert(!eic_nmi_config_valid(
    EicNmiConfig{.sense = EicSense::rising, .filter = true, .asynchronous = true}));

// ---- the pad table: irregular, and the header is the authority -------------
//
// Any "pin number modulo 16" rule dies on these four lines. They are the
// reason this file has a generated table and not a formula.
static_assert(eic_extint_line('A', 16) == 0);
static_assert(eic_extint_line('A', 24) == 12);
static_assert(eic_extint_line('A', 27) == 15);
static_assert(eic_extint_line('A', 0) == 0);

// PA08 is the NMI pad, so it has no EXTINT line at all; PA26 and PA29 do
// not exist on any variant.
static_assert(eic_extint_line('A', 8) < 0);
static_assert(eic_extint_line('A', 26) < 0);
static_assert(eic_nmi_pad('A', 8));
static_assert(!eic_nmi_pad('A', 9));

// Bonded on every variant of the family.
static_assert(extint_exists<'A', 0>);
static_assert(extint_exists<'A', 16>);
static_assert(ExtInt<Pin<'A', 16>>::line == 0);
static_assert(ExtInt<Pin<'A', 16>>::event_generator == 0x0E);

// PB22/PB23 (the bench board's button and LED) exist on G and J but not
// on E; PB16/PB17 are J only. The assertions are therefore written per
// variant, which is the point of sweeping three headers.
#if defined(__SAMC21E18A__)
static_assert(!extint_exists<'B', 22>);
static_assert(!extint_exists<'B', 16>);
#elif defined(__SAMC21G18A__)
static_assert(extint_exists<'B', 22> && ExtInt<Pin<'B', 22>>::line == 6);
static_assert(!extint_exists<'B', 16>);
#else
static_assert(extint_exists<'B', 22> && ExtInt<Pin<'B', 22>>::line == 6);
static_assert(extint_exists<'B', 16> && ExtInt<Pin<'B', 16>>::line == 0);
static_assert(ExtInt<Pin<'B', 30>>::line == 14);
#endif

void verbs() {
    constexpr EicLineConfig edge{.sense = EicSense::rising, .event_out = true};
    constexpr EicLineConfig level{.sense = EicSense::low};

    (void)Eic::init();
    Eic::bus_clock(true);
    (void)Eic::clock(0);
    (void)Eic::clock_select(EicClock::ulp32k);
    (void)Eic::clock_select();
    (void)Eic::reset();
    (void)Eic::configure_line(0, edge);
    (void)Eic::configure_line(15, level);
    (void)Eic::line_config(0);
    (void)Eic::release_line(0);
    (void)Eic::enable(true);
    (void)Eic::enabled();
    (void)Eic::flags();
    Eic::clear_flags(Eic::line_mask(3));
    Eic::arm(Eic::line_mask(3));
    Eic::disarm(Eic::line_mask(3));
    (void)Eic::armed();
    (void)Eic::flag(3);
    Eic::clear_flag(3);
    (void)Eic::isr();
    (void)Eic::irq();

    (void)Eic::nmi_configure(EicNmiConfig{.sense = EicSense::falling,
                                          .asynchronous = true});
    (void)Eic::nmi_config();
    (void)Eic::nmi_flag();
    Eic::clear_nmi_flag();
    (void)Eic::take_nmi();

    using Line0 = ExtInt<Pin<'A', 16>>;
    Line0::claim(PinPull::up);
    (void)Line0::configure(edge);
    (void)Line0::flag();
    Line0::clear_flag();
    Line0::arm(true);
    (void)Line0::armed();
    Line0::release();

    using Nmi = ExtNmi<Pin<'A', 8>>;
    Nmi::claim(PinPull::up);
    (void)Nmi::configure(EicNmiConfig{.sense = EicSense::rising,
                                      .asynchronous = true});
    (void)Nmi::flag();
    Nmi::clear_flag();
    Nmi::release();

    Eic::release();
}
