// RTC family smoke TU: stm32f4/rtc.hpp - the backup domain (RtcDomain:
// PWR_CR.DBP, the whole of RCC_BDCR, RCC_CSR's LSI, RCC_CFGR's RTCPRE)
// and the peripheral (Rtc: the calendar, both alarms, the wake-up timer,
// both calibrators, the shift, the timestamp, the tamper inputs, the
// RTC_OUT pad, the backup registers and three ISR bodies).
//
// What this TU proves on every header of the pack: that the whole verb
// set compiles, that the reserve's two RTC facts (the backup-register
// count derived from RTC_TypeDef, the pad facts keyed on the part class)
// carry through every expression built on them, and that the constexpr
// arithmetic - BCD both ways, the prescaler searches, the wake-up and
// calibration prices - folds to the numbers the chapter states.
#include <stdint.h>

#include "stm32f4/rtc.hpp"

using namespace brio;

// ---- the reserve's own facts --------------------------------------------------

static_assert(rtc_backup_registers() == 20);   // 80 bytes on every header of the pack
static_assert(Rtc::backup_count == rtc_backup_registers());
static_assert(Rtc::tamper_inputs >= 1 && Rtc::tamper_inputs <= 2);
static_assert(Rtc::has_second_pad == rtc_pad_facts().has_af2);
static_assert(!rtc_pad_facts().known || rtc_pad_facts().tamper_inputs >= 1);
static_assert(rtc_af1_port == 'C' && rtc_af1_pin == 13);
static_assert(Rtc::alarm_exti_line == 17 && Rtc::tamper_stamp_exti_line == 21 &&
              Rtc::wakeup_exti_line == 22);

// The second tamper's flag is present exactly where the second input is.
static_assert((RtcFlag::tamper2 != 0u) == (rtc_pad_facts().tamper_inputs >= 2u));
static_assert((RtcFlag::all & RtcFlag::tamper1) != 0u);

// ---- BCD, both ways -----------------------------------------------------------

static_assert(rtc_to_bcd(0) == 0x00 && rtc_to_bcd(9) == 0x09);
static_assert(rtc_to_bcd(10) == 0x10 && rtc_to_bcd(59) == 0x59 && rtc_to_bcd(99) == 0x99);
static_assert(rtc_from_bcd(0x00) == 0 && rtc_from_bcd(0x23) == 23 && rtc_from_bcd(0x99) == 99);
static_assert(rtc_from_bcd(rtc_to_bcd(37)) == 37);

static_assert(rtc_days_in_month(1, 24) == 31 && rtc_days_in_month(4, 24) == 30);
static_assert(rtc_days_in_month(2, 24) == 29 && rtc_days_in_month(2, 25) == 28);
static_assert(rtc_days_in_month(0, 24) == 0 && rtc_days_in_month(13, 24) == 0);

static_assert(rtc_datetime_valid(RtcDateTime{23, 59, 59, 29, 2, 24, 4}));
static_assert(!rtc_datetime_valid(RtcDateTime{24, 0, 0, 1, 1, 0, 1}));    // hour
static_assert(!rtc_datetime_valid(RtcDateTime{0, 0, 0, 29, 2, 25, 1}));   // no 29 February
static_assert(!rtc_datetime_valid(RtcDateTime{0, 0, 0, 1, 1, 0, 0}));     // weekday 0 forbidden

// The registers a setting makes, and the decode that undoes it.
constexpr RtcDateTime sample{13, 45, 7, 29, 2, 24, 4};
static_assert(rtc_time_register(sample) == 0x00134507u);
static_assert(rtc_date_register(sample) == 0x00248229u);
static_assert(rtc_decode(rtc_time_register(sample), rtc_date_register(sample)).hour == 13);
static_assert(rtc_decode(rtc_time_register(sample), rtc_date_register(sample)).second == 7);
static_assert(rtc_decode(rtc_time_register(sample), rtc_date_register(sample)).day == 29);
static_assert(rtc_decode(rtc_time_register(sample), rtc_date_register(sample)).weekday == 4);
static_assert(rtc_decode(rtc_time_register(sample), rtc_date_register(sample)).year == 24);

// ---- prescalers ---------------------------------------------------------------

static_assert(rtc_prescalers_valid(RtcPrescalers{127, 255}));
static_assert(!rtc_prescalers_valid(RtcPrescalers{128, 255}));
static_assert(rtc_prescalers_for(32768).async == 127 && rtc_prescalers_for(32768).sync == 255);
static_assert(rtc_prescalers_for_resolution(32768).async == 0 &&
              rtc_prescalers_for_resolution(32768).sync == 32767);
static_assert(rtc_ck_spre_hz(RtcPrescalers{127, 255}, 32768) == 1);
static_assert(rtc_ck_apre_hz(RtcPrescalers{127, 255}, 32768) == 256);
static_assert(rtc_cycles_per_second(RtcPrescalers{127, 255}) == 32768);
static_assert(rtc_prescalers_for(1'000'000).async == 124 &&
              rtc_prescalers_for(1'000'000).sync == 7999);
static_assert(rtc_subsecond_ms(255, 255) == 0 && rtc_subsecond_ms(0, 255) == 996);
static_assert(rtc_subsecond_ms(300, 255) == 0);   // past PREDIV_S: a shift's overhang

static_assert(rtc_shadow_read_allowed(100'000'000, 32768));
static_assert(!rtc_shadow_read_allowed(100'000, 32768));

// ---- the HSE branch -----------------------------------------------------------

static_assert(rtc_hse_divider_for(25'000'000) == 25);   // the 25 MHz crystal reaches 1 MHz
static_assert(rtc_hse_divider_for(8'000'000) == 8);
static_assert(rtc_hse_divider_for(1'000'000) == 0);     // /1 is "no clock"
static_assert(rtc_hse_divider_for(32'000'000) == 0);    // past the 5-bit field
static_assert(rtc_hse_divider_for(12'288'000) == 0);    // not a whole megahertz

// ---- alarms -------------------------------------------------------------------

static_assert(rtc_alarm_valid(RtcAlarm{}));
static_assert(!rtc_alarm_valid(RtcAlarm{.subsecond_mask = 16}));
static_assert(!rtc_alarm_valid(RtcAlarm{.second = 60, .mask_seconds = false}));
static_assert(!rtc_alarm_valid(RtcAlarm{.day = 8, .weekday_select = true, .mask_date = false}));
static_assert(rtc_alarm_valid(RtcAlarm{.day = 7, .weekday_select = true, .mask_date = false}));

// The default alarm is "every second": all four masks set, no sub-second
// comparison.
static_assert(rtc_alarm_register(RtcAlarm{}) ==
              (RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1 |
               (1u << RTC_ALRMAR_DU_Pos)));
static_assert(rtc_alarm_subsecond_register(RtcAlarm{.subsecond_mask = 15, .subsecond = 128}) ==
              ((15u << RTC_ALRMASSR_MASKSS_Pos) | 128u));

// ---- the wake-up timer --------------------------------------------------------

static_assert(rtc_wakeup_divider(RtcWakeupClock::div16) == 16);
static_assert(rtc_wakeup_divider(RtcWakeupClock::ck_spre) == 0);
static_assert(rtc_wakeup_valid(RtcWakeupClock::div16, 0));
static_assert(!rtc_wakeup_valid(RtcWakeupClock::div2, 0));        // 17.6.6's forbidden pair
static_assert(!rtc_wakeup_valid(RtcWakeupClock::div16, 0x10000)); // past the field
static_assert(rtc_wakeup_clock_hz(RtcWakeupClock::div2, 32768, RtcPrescalers{}) == 16384);
static_assert(rtc_wakeup_clock_hz(RtcWakeupClock::ck_spre, 32768, RtcPrescalers{}) == 1);

// ---- calibration --------------------------------------------------------------

static_assert(rtc_calibration_valid(RtcCalibration{}));
static_assert(!rtc_calibration_valid(RtcCalibration{.minus = 512}));
static_assert(!rtc_calibration_valid(RtcCalibration{
    .minus = 1, .window = RtcCalibrationWindow::seconds16}));
static_assert(!rtc_calibration_valid(RtcCalibration{
    .minus = 2, .window = RtcCalibrationWindow::seconds8}));
static_assert(rtc_calibration_valid(RtcCalibration{
    .minus = 4, .window = RtcCalibrationWindow::seconds8}));
// CALP alone is +488.5 ppm, CALM at its top is -487.1 ppm (17.3.11).
static_assert(rtc_calibration_ppb(RtcCalibration{.plus = true}) == 488'281);
static_assert(rtc_calibration_ppb(RtcCalibration{.minus = 511}) == -487'327);
static_assert(rtc_calibration_ppb(RtcCalibration{}) == 0);

static_assert(rtc_coarse_calibration_valid(RtcCoarseCalibration{false, 31}));
static_assert(!rtc_coarse_calibration_valid(RtcCoarseCalibration{false, 32}));
static_assert(rtc_coarse_calibration_ppb(RtcCoarseCalibration{false, 31}) == 126'139);
static_assert(rtc_coarse_calibration_ppb(RtcCoarseCalibration{true, 31}) == -63'085);

// ---- tamper -------------------------------------------------------------------

static_assert(tamper_sampling_divider(TamperSampling::div32768) == 32768);
static_assert(tamper_sampling_divider(TamperSampling::div256) == 256);
static_assert(tamper_sampling_hz(TamperSampling::div256, 32768) == 128);
static_assert(tamper_input_valid(TamperInput{1}));
static_assert(!tamper_input_valid(TamperInput{0}));
static_assert(tamper_input_valid(TamperInput{2}) == (rtc_pad_facts().tamper_inputs >= 2u));
static_assert(Rtc::tamper_flag(1) == RtcFlag::tamper1);
static_assert(Rtc::tamper_flag(3) == 0u);

// ---- the verbs, named once each -----------------------------------------------

void domain_verbs() {
    RtcDomain::pwr_bus_clock(true);
    (void)RtcDomain::pwr_bus_clock();
    (void)RtcDomain::unlock(true);
    (void)RtcDomain::unlocked();
    (void)RtcDomain::bdcr();
    RtcDomain::reset();
    RtcDomain::lse_enable(true);
    (void)RtcDomain::lse_enabled();
    (void)RtcDomain::lse_ready();
    (void)RtcDomain::lse_wait_ready();
    (void)RtcDomain::lse_wait_ready(1000);
    (void)RtcDomain::lse_bypass(true);
    (void)RtcDomain::lse_bypass();
    (void)RtcDomain::has_lse_mode();
    (void)RtcDomain::lse_mode(LseMode::high_drive);
    (void)RtcDomain::lse_mode();
    Rcc::lsi_enable(true);
    (void)Rcc::lsi_enabled();
    (void)Rcc::lsi_ready();
    (void)Rcc::lsi_wait_ready();
    (void)RtcDomain::hse_divider(25);
    (void)RtcDomain::hse_divider();
    (void)RtcDomain::selected();
    (void)RtcDomain::select(RtcClockSource::lse);
    RtcDomain::enable(true);
    (void)RtcDomain::enabled();
    (void)RtcDomain::open(RtcClockSource::lsi);
    (void)RtcDomain::open(RtcClockSource::hse_divided, true);
}

void calendar_verbs() {
    Rtc::unlock();
    Rtc::lock();
    (void)Rtc::cr();
    (void)Rtc::isr();
    (void)Rtc::prer();
    (void)Rtc::wutr();
    (void)Rtc::calr();
    (void)Rtc::calibr();
    (void)Rtc::tafcr();
    (void)Rtc::status();
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
    (void)Rtc::set_prescalers(RtcPrescalers{});
    (void)Rtc::set_prescalers<RtcPrescalers{0, 32767}>();
    (void)Rtc::prescalers();
    (void)Rtc::set_calendar(sample);
    (void)Rtc::init(RtcPrescalers{}, sample);
    RtcReading r{};
    (void)Rtc::read(r);
    (void)Rtc::subsecond();
    Rtc::shift_hour(true);
    Rtc::daylight_flag(true);
    (void)Rtc::daylight_flag();
    (void)Rtc::shift(true, 128);
    (void)Rtc::reference_clock();
    (void)Rtc::reference_clock(true);
}

void alarm_verbs() {
    (void)Rtc::alarm_enable_bit(RtcAlarmId::b);
    (void)Rtc::alarm_interrupt_bit(RtcAlarmId::b);
    (void)Rtc::alarm_write_flag(RtcAlarmId::b);
    (void)Rtc::alarm_flag(RtcAlarmId::b);
    (void)Rtc::alarm_enabled(RtcAlarmId::a);
    (void)Rtc::set_alarm(RtcAlarmId::a, RtcAlarm{.second = 30, .mask_seconds = false});
    (void)Rtc::set_alarm<RtcAlarm{.subsecond_mask = 15, .subsecond = 64}>(RtcAlarmId::b, false);
    Rtc::clear_alarm(RtcAlarmId::a);
}

void wakeup_verbs() {
    (void)Rtc::wakeup_enabled();
    (void)Rtc::wakeup_write_allowed();
    (void)Rtc::set_wakeup(RtcWakeupClock::div16, 2047);
    (void)Rtc::set_wakeup<RtcWakeupClock::ck_spre_high, 0>(false);
    Rtc::clear_wakeup();
}

void calibration_verbs() {
    (void)Rtc::calibrate(RtcCalibration{.plus = true});
    (void)Rtc::calibrate<RtcCalibration{.minus = 256}>();
    (void)Rtc::calibration();
    Rtc::calr_unprotected(0);
    (void)Rtc::coarse_calibrate(true, RtcCoarseCalibration{true, 7});
    (void)Rtc::coarse_calibrate(false);
    (void)Rtc::coarse_calibration_enabled();
    (void)Rtc::coarse_calibration();
}

void timestamp_and_output_verbs() {
    Rtc::timestamp_enable(true, true);
    (void)Rtc::timestamp_enabled();
    Rtc::timestamp_interrupt(true);
    (void)Rtc::timestamp();
    Rtc::alarm_output(RtcOutput::wakeup, true, true);
    (void)Rtc::alarm_output();
    (void)Rtc::calibration_output(true, RtcCalibOutput::hz1);
    (void)Rtc::calibration_output();
    (void)Rtc::timestamp_pad(true);
    (void)Rtc::tamper_pad(false);
}

void tamper_verbs() {
    (void)Rtc::tamper_armed(1);
    (void)Rtc::any_tamper_armed();
    (void)Rtc::tamper_config(TamperConfig{TamperFilter::samples4, TamperSampling::div1024,
                                          TamperPrecharge::cycles8, false});
    (void)Rtc::tamper_filter();
    (void)Rtc::tamper_arm(TamperInput{1, TamperTrigger::high_level_or_falling_edge});
    (void)Rtc::tamper_arm(TamperInput{2});
    (void)Rtc::tamper_disarm(1);
    Rtc::tamper_interrupt(true);
    (void)Rtc::tamper_interrupt();
    Rtc::timestamp_on_tamper(true);
    (void)Rtc::timestamp_on_tamper();
}

void backup_and_interrupt_verbs() {
    (void)Rtc::backup(0);
    (void)Rtc::backup(0, 0x5A5Au);
    (void)Rtc::backup<Rtc::backup_count - 1u>();
    Rtc::backup<0>(0x1234u);
    (void)Rtc::flag(RtcFlag::wakeup);
    Rtc::clear_flags(RtcFlag::all);
    (void)Rtc::masked_status();
    (void)Rtc::enabled_mask();
    Rtc::wake_line_open(Rtc::alarm_exti_line);
    (void)Rtc::wake_line_is_open(Rtc::wakeup_exti_line);
    (void)Exti::pending(Rtc::tamper_stamp_exti_line);
    (void)Exti::clear(Rtc::tamper_stamp_exti_line);
    (void)Rtc::debug_freeze();
    Rtc::debug_freeze(true);
    (void)Rtc::alarm_irq();
    (void)Rtc::wakeup_irq();
    (void)Rtc::tamper_stamp_irq();
}

// The three ISR bodies, as an app binds them.
extern "C" void RTC_Alarm_IRQHandler() { (void)brio::Rtc::alarm_isr(); }
extern "C" void RTC_WKUP_IRQHandler() { (void)brio::Rtc::wakeup_isr(); }
extern "C" void TAMP_STAMP_IRQHandler() { (void)brio::Rtc::tamper_stamp_isr(); }
