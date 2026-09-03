// RTC family smoke TU: the RTC domain, the calendar, both alarms, the
// wake-up timer, the smooth calibrator and the backup registers
// (stm32g0/rtc.hpp). Every part of the family carries ONE RTC, ONE TAMP
// with five backup registers and one shared vector, so what this TU
// proves is that the file compiles against each device header, that the
// constexpr arithmetic agrees with the chapter's own tables, and that
// the predicates refuse what the silicon refuses.
#include "stm32g0/rtc.hpp"

using namespace brio;

// ---- the prescalers (30.3.4) ------------------------------------------------

// The chapter's own example: 32.768 kHz -> 127 / 255 -> 1 Hz.
static_assert(rtc_prescalers_for(32768).async == 127);
static_assert(rtc_prescalers_for(32768).sync == 255);
static_assert(rtc_ck_spre_hz(rtc_prescalers_for(32768), 32768) == 1);
static_assert(rtc_ck_apre_hz(rtc_prescalers_for(32768), 32768) == 256);
// The asynchronous factor is taken as HIGH as the chapter recommends.
static_assert(rtc_prescalers_for(32000).async == 127);
static_assert(rtc_prescalers_for(32000).sync == 249);
static_assert(rtc_ck_spre_hz(rtc_prescalers_for(32000), 32000) == 1);
// A rate past the chapter's ~4 MHz ceiling has no exact pair, and says
// so with a value the 7-bit field cannot hold (the hsidiv_for habit).
static_assert(rtc_prescalers_for(5'000'000).async == 0xFF);
static_assert(rtc_prescalers_valid(RtcPrescalers{}));
static_assert(!rtc_prescalers_valid(RtcPrescalers{.async = 128}));
static_assert(!rtc_prescalers_valid(RtcPrescalers{.sync = 0x8000}));

// 30.6.3's own formula, at both ends of the counter.
static_assert(rtc_subsecond_ms(255, 255) == 0);
static_assert(rtc_subsecond_ms(0, 255) == 996);
static_assert(rtc_subsecond_ms(128, 255) == 496);

// ---- the calendar (30.6.1, 30.6.2) -----------------------------------------

static_assert(rtc_to_bcd(59) == 0x59);
static_assert(rtc_from_bcd(0x59) == 59);
static_assert(rtc_from_bcd(rtc_to_bcd(99)) == 99);

// The leap rule of a one-century calendar: divisible by four, no
// exception to apply (the field carries no century).
static_assert(rtc_days_in_month(2, 24) == 29);
static_assert(rtc_days_in_month(2, 25) == 28);
static_assert(rtc_days_in_month(4, 24) == 30);
static_assert(rtc_days_in_month(12, 24) == 31);
static_assert(rtc_days_in_month(13, 24) == 0);

static_assert(rtc_datetime_valid(RtcDateTime{}));
static_assert(rtc_datetime_valid({.day = 29, .month = 2, .year = 24}));
static_assert(!rtc_datetime_valid({.day = 29, .month = 2, .year = 25}));
static_assert(!rtc_datetime_valid({.hour = 24}));
static_assert(!rtc_datetime_valid({.weekday = 0}));   // 30.6.2: forbidden

// The registers round-trip.
constexpr RtcDateTime sample{.hour = 23, .minute = 59, .second = 58,
                             .day = 29, .month = 2, .year = 24, .weekday = 4};
static_assert(rtc_decode(rtc_time_register(sample),
                         rtc_date_register(sample)).hour == 23);
static_assert(rtc_decode(rtc_time_register(sample),
                         rtc_date_register(sample)).day == 29);
static_assert(rtc_decode(rtc_time_register(sample),
                         rtc_date_register(sample)).weekday == 4);
static_assert(rtc_decode(rtc_time_register(sample),
                         rtc_date_register(sample)).year == 24);

// ---- the alarms (30.6.14) ---------------------------------------------------

static_assert(rtc_alarm_valid(RtcAlarm{}));
static_assert(!rtc_alarm_valid(RtcAlarm{.second = 60, .mask_seconds = false}));
static_assert(!rtc_alarm_valid(RtcAlarm{.subsecond_mask = 16}));
static_assert(!rtc_alarm_valid(RtcAlarm{.day = 8, .weekday_select = true,
                                        .mask_date = false}));
// Every field masked is the "every second" alarm, and its register is
// the four mask bits and nothing else.
static_assert((rtc_alarm_register(RtcAlarm{}) &
               (RTC_ALRMAR_MSK1 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK3 |
                RTC_ALRMAR_MSK4)) ==
              (RTC_ALRMAR_MSK1 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK3 |
               RTC_ALRMAR_MSK4));

// ---- the wake-up timer (30.6.6) ---------------------------------------------

static_assert(rtc_wakeup_divider(RtcWakeupClock::div16) == 16);
static_assert(rtc_wakeup_divider(RtcWakeupClock::ck_spre) == 0);
static_assert(rtc_wakeup_valid(RtcWakeupClock::div16, 0xFFFF));
static_assert(!rtc_wakeup_valid(RtcWakeupClock::div16, 0x10000));
// 30.6.6 calls RTCCLK/2 with a reload of zero forbidden.
static_assert(!rtc_wakeup_valid(RtcWakeupClock::div2, 0));
static_assert(rtc_wakeup_valid(RtcWakeupClock::div4, 0));
static_assert(rtc_wakeup_clock_hz(RtcWakeupClock::div16, 32768,
                                  rtc_prescalers_for(32768)) == 2048);
static_assert(rtc_wakeup_clock_hz(RtcWakeupClock::ck_spre, 32768,
                                  rtc_prescalers_for(32768)) == 1);

// ---- smooth calibration (30.6.9) --------------------------------------------

static_assert(rtc_calibration_valid(RtcCalibration{}));
static_assert(!rtc_calibration_valid({.minus = 0x200}));
// The two stuck-bit notes as refusals.
static_assert(!rtc_calibration_valid({.minus = 3,
                                      .window = RtcCalibrationWindow::seconds8}));
static_assert(rtc_calibration_valid({.minus = 4,
                                     .window = RtcCalibrationWindow::seconds8}));
static_assert(!rtc_calibration_valid({.minus = 1,
                                      .window = RtcCalibrationWindow::seconds16}));
// The chapter's own range, in the units this file reports.
static_assert(rtc_calibration_ppb({.plus = true}) > 488'000);
static_assert(rtc_calibration_ppb({.minus = 511}) < -487'000);
static_assert(rtc_calibration_ppb({}) == 0);

// ---- the geometry the reserve answers for -----------------------------------

static_assert(Tamp::backup_count == 5);
static_assert(Rtc::exti_line == 19);
static_assert(Tamp::exti_line == 21);
static_assert(rtc_irq() == RTC_TAMP_IRQn);

// ---- the verbs --------------------------------------------------------------

void domain_verbs() {
    RtcDomain::pwr_bus_clock(true);
    (void)RtcDomain::pwr_bus_clock();
    RtcDomain::apb_clock(true);
    (void)RtcDomain::apb_clock();
    RtcDomain::unlock(true);
    (void)RtcDomain::unlocked();
    (void)RtcDomain::bdcr();
    RtcDomain::lse_enable(true);
    (void)RtcDomain::lse_enabled();
    (void)RtcDomain::lse_ready();
    (void)RtcDomain::lse_wait_ready(10);
    (void)RtcDomain::lse_drive(LseDrive::medium_low);
    (void)RtcDomain::lse_drive();
    (void)RtcDomain::lse_bypass(false);
    (void)RtcDomain::lse_bypass();
    (void)RtcDomain::lse_css(true);
    (void)RtcDomain::lse_css();
    (void)RtcDomain::lse_css_failed();
    (void)RtcDomain::select(RtcClockSource::lsi);
    (void)RtcDomain::selected();
    RtcDomain::enable(true);
    (void)RtcDomain::enabled();
    RtcDomain::lsco(false);
    (void)RtcDomain::lsco();
    (void)RtcDomain::open(RtcClockSource::lsi, false);
    static_assert(RtcDomain::css_exti_line == 31);
    static_assert(RtcDomain::lse_tim16_ti1_code == 2);
    if (false) {
        RtcDomain::reset();   // wipes the domain: named, not called
    }
}

void rtc_verbs() {
    Rtc::unlock();
    Rtc::lock();
    (void)Rtc::cr();
    (void)Rtc::icsr();
    (void)Rtc::prer();
    (void)Rtc::wutr();
    (void)Rtc::calr();
    (void)Rtc::status();
    (void)Rtc::masked_status();
    (void)Rtc::calendar_set();
    (void)Rtc::in_init();
    (void)Rtc::synchronized();
    (void)Rtc::shift_pending();
    (void)Rtc::recalibration_pending();
    (void)Rtc::bypass_shadow();
    Rtc::bypass_shadow(true);
    (void)Rtc::enter_init();
    (void)Rtc::exit_init();
    Rtc::exit_init_raw();
    (void)Rtc::wait_sync();
    (void)Rtc::set_prescalers(rtc_prescalers_for(32768));
    (void)Rtc::prescalers();
    (void)Rtc::set_calendar(sample);
    (void)Rtc::init(rtc_prescalers_for(32768), sample);

    RtcReading r{};
    (void)Rtc::read(r);
    (void)Rtc::time_of_hour_ms();
    static_assert(Rtc::elapsed_ms(3'599'000, 1'000) == 2000);
    static_assert(Rtc::elapsed_ms(1000, 2000) == 1000);

    Rtc::shift_hour(true);
    Rtc::daylight_flag(true);
    (void)Rtc::daylight_flag();

    (void)Rtc::set_alarm(RtcAlarmId::a, RtcAlarm{});
    (void)Rtc::set_alarm(RtcAlarmId::b, RtcAlarm{}, false);
    (void)Rtc::alarm_enabled(RtcAlarmId::a);
    Rtc::clear_alarm(RtcAlarmId::b);

    (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 3);
    (void)Rtc::wakeup_enabled();
    (void)Rtc::wakeup_write_allowed();
    Rtc::clear_wakeup();

    (void)Rtc::calibrate(RtcCalibration{});
    (void)Rtc::calibration();
    Rtc::calr_unprotected(0);

    Rtc::timestamp_enable(true);
    (void)Rtc::timestamp_enabled();
    (void)Rtc::timestamp();

    (void)Rtc::flag(RtcFlag::alarm_a);
    Rtc::clear_flags(RtcFlag::all);
    (void)Rtc::wake_line_open();
    (void)Rtc::isr();
    (void)Rtc::debug_freeze();
    Rtc::debug_freeze(false);
    static_assert(Rtc::wakeup_tim16_ti1_code == 3);
}

void tamp_verbs() {
    (void)Tamp::backup(0);
    (void)Tamp::backup(0, 0x1234u);
    (void)Tamp::config1();
    (void)Tamp::config2();
    (void)Tamp::filter();
    (void)Tamp::interrupts();
    (void)Tamp::status();
    (void)Tamp::masked_status();
    (void)Tamp::any_armed();
    (void)Tamp::any_internal_armed();
    (void)Tamp::erase_source_armed();
    Tamp::clear_flags(0);
    static_assert(Tamp::irq() == RTC_TAMP_IRQn);

    // The detection half, whole.
    (void)Tamp::filter_config({.filter = TamperFilter::samples8,
                               .sampling = TamperSampling::div256,
                               .precharge = TamperPrecharge::cycles8,
                               .pullup = false});
    (void)Tamp::filter_mode();
    (void)Tamp::armed(1);
    (void)Tamp::arm({.index = 1,
                     .trigger = TamperTrigger::high_level_or_falling_edge,
                     .erase_backups = false,
                     .masked = false,
                     .interrupt = true});
    (void)Tamp::disarm(1);
    (void)Tamp::internal_tamper(brio::tamp_internal_first, false);
    (void)Tamp::flag(TampFlag::external);
    (void)Tamp::wake_line_open();
    (void)Tamp::isr();
    static_assert(Tamp::exti_line == 21);
    static_assert(Tamp::input_count >= 2 && Tamp::input_count <= 3);

    // The two internal-index edges, from the manual's own numbering.
    static_assert(internal_tamper_flag(2) == 0u);
    static_assert(internal_tamper_flag(7) == 0u);
    static_assert(tamper_flag(0) == 0u);
    static_assert(tamper_flag(1) == TampFlag::tamper1);

    // The sampling arithmetic, on a stated rate the way every other
    // ratio in this file takes one.
    static_assert(tamper_sampling_hz(TamperSampling::div32768, 32768) == 1);
    static_assert(tamper_sampling_hz(TamperSampling::div256, 32768) == 128);
    static_assert(tamper_sampling_divider(TamperSampling::div2048) == 2048);

    // The new Rtc verbs of the same chapter pair.
    (void)Rtc::subsecond();
    (void)Rtc::reference_clock();
    (void)Rtc::reference_clock(false);
    (void)Rtc::shift(true, 128);
    Rtc::timestamp_on_tamper(false);
    (void)Rtc::timestamp_on_tamper();
    Rtc::timestamp_internal(false);
    (void)Rtc::timestamp_internal();
}

// ---- the other prescaler split (the timebase's, not the calendar's) --------

// rtc_prescalers_for() maximizes the ASYNCHRONOUS factor (30.3.4's
// low-power advice); rtc_prescalers_for_resolution() minimizes it, which
// is what a sub-second stopwatch wants.
static_assert(rtc_prescalers_for_resolution(32768).async == 0);
static_assert(rtc_prescalers_for_resolution(32768).sync == 32767);
static_assert(rtc_ck_spre_hz(rtc_prescalers_for_resolution(32768), 32768) == 1);
static_assert(rtc_prescalers_for_resolution(33000).sync + 1 >= 1000);
static_assert(rtc_prescalers_for_resolution(5'000'000).async == 0xFF);
