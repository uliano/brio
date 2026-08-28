// Family smoke TU for samc/osc32kctrl.hpp: every verb must COMPILE on
// the E, G and J 18A headers (tools/check_samc.sh sweeps all three).
//
// OSC32KCTRL is one instance on every member of the family and nothing
// in it varies by package. What this fixture pins is the LEGALITY the
// driver adds on top of the registers - field widths, and the rule that
// an oscillator with both outputs disabled is a clock nobody can reach.

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/osc32kctrl.hpp"

using namespace brio;

// Field widths, from the register drawings of 21.8.9 and 21.8.10.
static_assert(Osc32k::calib_max == 0x7F, "OSC32K.CALIB is seven bits");
static_assert(Osculp32k::calib_max == 0x1F, "OSCULP32K.CALIB is five bits");
static_assert(Osc32k::startup_max == 0x7);
static_assert(Xosc32k::startup_max == 0x7);

// The RTCSEL codes are the header's, and all six are legal (21.6.7).
static_assert(static_cast<uint8_t>(RtcClock::ulp_1k) == 0);
static_assert(static_cast<uint8_t>(RtcClock::osc_32k) == 3);
static_assert(static_cast<uint8_t>(RtcClock::xosc_32k) == 5);

// AN OSCILLATOR WITH NO OUTPUT IS A CLOCK NOBODY CAN REACH. 21.6.4 asks
// for EN32K or EN1K to be enabled before a consumer is pointed at it;
// enabling neither is a configuration that would run and do nothing.
static_assert(!Osc32k::config_valid(
                  Osc32kConfig{.enable_32k = false, .enable_1k = false}));
static_assert(!Xosc32k::config_valid(
                  Xosc32kConfig{.enable_32k = false, .enable_1k = false}));
static_assert(Osc32k::config_valid(Osc32kConfig{.enable_32k = false, .enable_1k = true}));
static_assert(Osc32k::config_valid(Osc32kConfig{.calib = 0x7F}));
static_assert(!Osc32k::config_valid(Osc32kConfig{.calib = 0x80}), "past the field");
static_assert(!Osc32k::config_valid(Osc32kConfig{.startup = 8}), "past the field");
static_assert(!Xosc32k::config_valid(Xosc32kConfig{.startup = 8}));

void block_verbs() {
    (void)Osc32kctrl::irq();
    (void)Osc32kctrl::status();
    (void)Osc32kctrl::osc32k_ready();
    (void)Osc32kctrl::xosc32k_ready();
    (void)Osc32kctrl::clock_failing();
    (void)Osc32kctrl::clock_switched();
    (void)Osc32kctrl::flags();
    (void)Osc32kctrl::armed();
    Osc32kctrl::clear_flags();
    Osc32kctrl::arm(Osc32kFlag::osc32k_ready);
    Osc32kctrl::disarm(Osc32kFlag::all);
    (void)Osc32kctrl::isr();
    Osc32kctrl::rtc_clock(RtcClock::ulp_32k);
    (void)Osc32kctrl::rtc_clock();
}

void osc32k_verbs() {
    (void)Osc32k::factory_calib();
    (void)Osc32k::init(Osc32kConfig{.calib = 0x40, .enable_32k = true});
    (void)Osc32k::reg();
    (void)Osc32k::enabled();
    (void)Osc32k::locked();
    (void)Osc32k::ready();
    (void)Osc32k::calib();
    (void)Osc32k::retrim(0x20);
    Osc32k::stop();
    Osc32k::lock();
}

void osculp32k_verbs() {
    (void)Osculp32k::calib();
    (void)Osculp32k::calib(0x10);
    (void)Osculp32k::locked();
    Osculp32k::lock();
}

void xosc32k_verbs() {
    (void)Xosc32k::init(Xosc32kConfig{.crystal = true});
    (void)Xosc32k::reg();
    (void)Xosc32k::enabled();
    (void)Xosc32k::locked();
    (void)Xosc32k::ready();
    Xosc32k::failure_detector(true, true);
    (void)Xosc32k::failure_detector();
    Xosc32k::switch_back();
    Xosc32k::stop();
    Xosc32k::lock();
}
