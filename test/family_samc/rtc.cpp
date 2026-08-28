// Family smoke TU for samc/rtc.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The RTC is ONE instance on every member of this family and its
// registers do not vary by package, so what this fixture pins is what
// the driver DECIDES rather than reads: the prescaler arithmetic, the
// 1 Hz reachability rule that chapter 24 never states, the frequency
// correction's ppb scale, the calendar's packing and its leap-year
// rule, and every configuration refusal.

#include <stdint.h>

#include "samc/rtc.hpp"

using namespace brio;

// ---- the prescaler ----------------------------------------------------------

static_assert(rtc_prescaler_divisor(RtcPrescaler::off) == 1,
              "OFF divides by one - what it stops is the periodic events");
static_assert(rtc_prescaler_divisor(RtcPrescaler::div1) == 1);
static_assert(rtc_prescaler_divisor(RtcPrescaler::div2) == 2);
static_assert(rtc_prescaler_divisor(RtcPrescaler::div1024) == 1024);

// THE RULE MODE 2 DEPENDS ON AND CHAPTER 24 NEVER WRITES DOWN: the
// clock/calendar needs 1 Hz, the prescaler stops at /1024, so a
// 32.768 kHz source cannot reach it and a 1.024 kHz one can.
static_assert(rtc_prescaler_for_hz(1024, 1) == RtcPrescaler::div1024);
static_assert(!rtc_prescaler_for_hz(32768, 1).has_value(),
              "32768/1024 is 32 Hz - the calendar needs a 1 kHz source");
static_assert(rtc_prescaler_for_hz(32768, 32) == RtcPrescaler::div1024);
static_assert(rtc_prescaler_for_hz(32768, 16384) == RtcPrescaler::div2);
// A divide-by-one answers DIV1 and never OFF: they divide the same and
// only one of them keeps the periodic events alive.
static_assert(rtc_prescaler_for_hz(1024, 1024) == RtcPrescaler::div1);
static_assert(!rtc_prescaler_for_hz(1000, 3).has_value(), "not a whole divisor");
static_assert(!rtc_prescaler_for_hz(0, 1).has_value());
static_assert(!rtc_prescaler_for_hz(1024, 0).has_value());

// PERn taps prescaler bit n+2, so it runs at f/2^(n+3) (24.6.8.1).
static_assert(rtc_periodic_mhz(32768, 0) == 4096u * 1000u);
static_assert(rtc_periodic_mhz(32768, 7) == 32u * 1000u);
static_assert(rtc_periodic_mhz(1024, 7) == 1000u, "1 Hz, expressed in millihertz");
static_assert(rtc_periodic_mhz(32768, 8) == 0u, "there are eight of them");

// ---- the frequency correction ----------------------------------------------

static_assert(rtc_correction_ppb(0) == 0);
// One step is 1e9/983040 ppb - the chapter's "1.017 ppm resolution".
static_assert(rtc_correction_ppb(1) == 1017u);
static_assert(rtc_correction_ppb(127) == 129'191u, "about 129 ppm at the end");
static_assert(rtc_correction_ppb(127) > rtc_correction_ppb(64));
// The 64-bit intermediate is what keeps this exact: 127 x 1e9 is thirty
// times past a 32-bit product.
static_assert(127ULL * 1'000'000'000ULL > 0xFFFFFFFFULL);

// ---- the calendar -----------------------------------------------------------

// 24.12.9's own rule, which is NOT the Gregorian one: YEAR[1:0] == 0.
static_assert(rtc_is_leap(0) && rtc_is_leap(4) && rtc_is_leap(60));
static_assert(!rtc_is_leap(1) && !rtc_is_leap(2) && !rtc_is_leap(3));
static_assert(rtc_days_in_month(2, 0) == 29, "year offset 0 is a leap year");
static_assert(rtc_days_in_month(2, 1) == 28);
static_assert(rtc_days_in_month(1, 0) == 31 && rtc_days_in_month(4, 0) == 30);
static_assert(rtc_days_in_month(12, 3) == 31);
static_assert(rtc_days_in_month(0, 0) == 0 && rtc_days_in_month(13, 0) == 0);

// The packing round-trips, and the field positions are the header's.
constexpr RtcClockValue sample{.second = 58, .minute = 59, .hour = 23,
                               .day = 31, .month = 12, .year = 63};
static_assert(RtcClockValue::from_register(sample.to_register()) == sample);
static_assert((sample.to_register() & RTC_MODE2_CLOCK_SECOND_Msk) >>
                  RTC_MODE2_CLOCK_SECOND_Pos == 58u);
static_assert((sample.to_register() & RTC_MODE2_CLOCK_YEAR_Msk) >>
                  RTC_MODE2_CLOCK_YEAR_Pos == 63u);
// The whole word, field by field: 23:59:58 on 31 December of year 63 -
// one second short of the top value 24.6.2.5 says wraps to zero.
static_assert(sample.to_register() == 0xFF3F'7EFAUL);

// 12-hour representation puts AM/PM in HOUR[4], which is bit 16.
constexpr RtcClockValue pm_sample{.hour = 11, .day = 1, .month = 1, .pm = true};
static_assert((pm_sample.to_register() & (1UL << 16)) != 0u);
static_assert(RtcClockValue::from_register(pm_sample.to_register(), true).pm);
static_assert(RtcClockValue::from_register(pm_sample.to_register(), true).hour ==
              11u);
// AND THE SAME BITS READ AS 24-HOUR ARE HOUR 27, WHICH IS THE POINT:
// only CTRLA.CLKREP says which reading is meant, so from_register() is
// told and Rtc::clock_value() asks the register.
static_assert(RtcClockValue::from_register(pm_sample.to_register()).hour == 27u);
static_assert(!RtcClockValue::from_register(pm_sample.to_register()).pm);

// Validity, and it depends on which representation is in force.
static_assert(sample.valid());
static_assert(!RtcClockValue{.second = 60, .day = 1, .month = 1}.valid());
static_assert(!RtcClockValue{.hour = 24, .day = 1, .month = 1}.valid());
static_assert(!RtcClockValue{.day = 30, .month = 2, .year = 0}.valid(),
              "February has 29 days even in a leap year");
static_assert(RtcClockValue{.day = 29, .month = 2, .year = 0}.valid());
static_assert(!RtcClockValue{.day = 29, .month = 2, .year = 1}.valid());
static_assert(!RtcClockValue{.day = 0, .month = 1}.valid(), "days start at 1");
static_assert(!RtcClockValue{.day = 1, .month = 13}.valid());
static_assert(!RtcClockValue{.day = 1, .month = 1, .year = 64}.valid());
// A 24-hour value carrying PM is nonsense; hour 0 is nonsense in 12-hour.
static_assert(!RtcClockValue{.day = 1, .month = 1, .pm = true}.valid(false));
static_assert(!RtcClockValue{.hour = 0, .day = 1, .month = 1}.valid(true));
static_assert(RtcClockValue{.hour = 12, .day = 1, .month = 1}.valid(true));

// ---- configuration legality -------------------------------------------------

static_assert(rtc_compare_count(RtcMode::count32) == 1);
static_assert(rtc_compare_count(RtcMode::count16) == 2);
static_assert(rtc_compare_count(RtcMode::clock) == 1);

static_assert(rtc_config_valid(RtcConfig{}));
static_assert(rtc_config_valid(RtcConfig{.mode = RtcMode::clock,
                                         .prescaler = RtcPrescaler::div1024,
                                         .match_clear = true,
                                         .twelve_hour = true}));
static_assert(!rtc_config_valid(RtcConfig{.mode = RtcMode::count16,
                                          .match_clear = true}),
              "MATCHCLR is valid only in modes 0 and 2 (24.12.1)");
static_assert(!rtc_config_valid(RtcConfig{.twelve_hour = true}),
              "CLKREP is mode 2's alone");
static_assert(!rtc_config_valid(
                  RtcConfig{.prescaler = static_cast<RtcPrescaler>(0xC)}),
              "0xC..0xF are Reserved");

// EVCTRL's two rules, both of which live between the structs.
static_assert(rtc_event_config_valid(RtcConfig{},
                                     RtcEventConfig{.compare_out = 0x1}));
static_assert(!rtc_event_config_valid(RtcConfig{},
                                      RtcEventConfig{.compare_out = 0x2}),
              "there is no CMPEO1 outside mode 1");
static_assert(rtc_event_config_valid(RtcConfig{.mode = RtcMode::count16},
                                     RtcEventConfig{.compare_out = 0x3}));
static_assert(!rtc_event_config_valid(RtcConfig{.mode = RtcMode::clock},
                                      RtcEventConfig{.compare_out = 0x2}),
              "mode 2 has one alarm");
static_assert(!rtc_event_config_valid(
                  RtcConfig{.prescaler = RtcPrescaler::off},
                  RtcEventConfig{.periodic_out = 0x01}),
              "the prescaler OFF silently kills every periodic event");
static_assert(rtc_event_config_valid(RtcConfig{.prescaler = RtcPrescaler::off},
                                     RtcEventConfig{.compare_out = 0x1}),
              "a compare event does not come from the prescaler");

// ---- the EVSYS codes this peripheral publishes ------------------------------

static_assert(Rtc::compare_generator(0) == EVENT_ID_GEN_RTC_CMP_0);
static_assert(Rtc::compare_generator(1) == EVENT_ID_GEN_RTC_CMP_1);
static_assert(Rtc::alarm_generator == Rtc::compare_generator(0),
              "the alarm IS compare 0 under another name");
static_assert(Rtc::overflow_generator == EVENT_ID_GEN_RTC_OVF);
static_assert(Rtc::periodic_generator(0) == EVENT_ID_GEN_RTC_PER_0);
static_assert(Rtc::periodic_generator(7) == EVENT_ID_GEN_RTC_PER_7);

// ---- the EVCTRL field widths, which are NOT the same in all three modes -----
//
// The trap the driver has to dodge: the device header's group mask for
// the compare event outputs is ONE bit in the mode 0 view and TWO in the
// mode 1 view, at the same position. A driver writing the common control
// surface through the mode 0 macro would silently drop CMPEO1.
static_assert(RTC_MODE0_EVCTRL_CMPEO_Msk == (0x1u << 8));
static_assert(RTC_MODE1_EVCTRL_CMPEO_Msk == (0x3u << 8));
static_assert(RTC_MODE0_EVCTRL_PEREO_Msk == 0xFFu,
              "all eight periodic outputs, in every mode");
static_assert(RTC_MODE0_EVCTRL_OVFEO_Msk == RTC_MODE1_EVCTRL_OVFEO_Msk);
static_assert(RTC_MODE0_EVCTRL_OVFEO_Msk == RTC_MODE2_EVCTRL_OVFEO_Msk);

// ---- the flag bits ----------------------------------------------------------

static_assert(RtcFlag::compare(0) == RtcFlag::compare0);
static_assert(RtcFlag::compare(1) == RtcFlag::compare1);
static_assert(RtcFlag::periodic(0) == 0x0001u);
static_assert(RtcFlag::periodic(7) == 0x0080u);
static_assert(RtcFlag::periodic_all == 0x00FFu);
static_assert((RtcFlag::all & RtcFlag::overflow) != 0u);

void verbs() {
    constexpr RtcConfig cfg0{.mode = RtcMode::count32,
                             .prescaler = RtcPrescaler::div1024,
                             .match_clear = true};
    constexpr RtcConfig cfg1{.mode = RtcMode::count16,
                             .prescaler = RtcPrescaler::div2};
    constexpr RtcConfig cfg2{.mode = RtcMode::clock,
                             .prescaler = RtcPrescaler::div1024,
                             .twelve_hour = true};

    (void)Rtc::irq();
    Rtc::bus_clock(true);
    (void)Rtc::init();
    (void)Rtc::reset();
    (void)Rtc::sync_flags();
    (void)Rtc::sync_wait(0u);
    (void)Rtc::enabled();
    (void)Rtc::enable(true);

    (void)Rtc::config_valid(cfg0);
    (void)Rtc::configure(cfg0);
    (void)Rtc::configure<cfg1>();
    (void)Rtc::configure<cfg2>();
    (void)Rtc::ctrla();
    (void)Rtc::mode();
    (void)Rtc::prescaler();
    (void)Rtc::match_clear();
    (void)Rtc::twelve_hour();
    (void)Rtc::read_sync(true);
    (void)Rtc::read_sync();
    (void)Rtc::event_config(cfg0, RtcEventConfig{.compare_out = 0x1,
                                                 .overflow_out = true});
    (void)Rtc::evctrl();

    (void)Rtc::count32_raw();
    (void)Rtc::count32();
    (void)Rtc::set_count32(0);
    (void)Rtc::comp32();
    (void)Rtc::set_comp32(0x1000);

    (void)Rtc::count16_raw();
    (void)Rtc::count16();
    (void)Rtc::set_count16(0);
    (void)Rtc::period16();
    (void)Rtc::set_period16(1000);
    (void)Rtc::comp16(0);
    (void)Rtc::set_comp16(1, 500);

    (void)Rtc::clock_register_raw();
    (void)Rtc::clock_register();
    (void)Rtc::clock_value();
    (void)Rtc::set_clock(sample);
    (void)Rtc::alarm_register();
    (void)Rtc::alarm();
    (void)Rtc::set_alarm(sample);
    (void)Rtc::alarm_mask();
    (void)Rtc::set_alarm_mask(RtcAlarmMask::hour_minute_second);

    (void)Rtc::set_frequency_correction(true, 127);
    (void)Rtc::correction_value();
    (void)Rtc::correction_negative();

    (void)Rtc::flags();
    Rtc::clear_flags();
    (void)Rtc::armed();
    Rtc::arm(RtcFlag::overflow);
    Rtc::disarm();
    (void)Rtc::isr();
    Rtc::debug_run(true);
    (void)Rtc::debug_run();

    (void)Rtc::regs0().RTC_CTRLA;
    (void)Rtc::regs1().RTC_PER;
    (void)Rtc::regs2().RTC_MASK;
    Rtc::release();
}
