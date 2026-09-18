// RTC family smoke TU: the backup domain (PWR's write enable and the
// whole of RCC_BDCTLR) and the counter (the prescaler arithmetic, the
// configuration window, the two reads, the events and the alarm's
// second path through EXTI line 17) - every verb, on every part.
//
// Two things here ARE per part and are asked of the part table rather
// than named: whether the package brings out the 32 kHz oscillator
// pads, and what RTCSEL's third choice divides the HSE by - which on
// this device class is one of two numbers keyed on the lot.
#include <stddef.h>

#include "ch32v203/rtc.hpp"

using namespace brio;

// ---- the register maps ------------------------------------------------------
static_assert(offsetof(RtcRegs, CTLRH) == 0x00);
static_assert(offsetof(RtcRegs, CTLRL) == 0x04);
static_assert(offsetof(RtcRegs, PSCRH) == 0x08);
static_assert(offsetof(RtcRegs, PSCRL) == 0x0C);
static_assert(offsetof(RtcRegs, DIVH) == 0x10);
static_assert(offsetof(RtcRegs, DIVL) == 0x14);
static_assert(offsetof(RtcRegs, CNTH) == 0x18);
static_assert(offsetof(RtcRegs, CNTL) == 0x1C);
static_assert(offsetof(RtcRegs, ALRMH) == 0x20);
static_assert(offsetof(RtcRegs, ALRML) == 0x24);
static_assert(offsetof(PwrRegs, CTLR) == 0x00);
static_assert(offsetof(PwrRegs, CSR) == 0x04);

// ---- the bits ---------------------------------------------------------------
static_assert(pwr_dbp == (1u << 8));
static_assert(rcc_lseon == 1u && rcc_lserdy == 2u && rcc_lsebyp == 4u);
static_assert(rcc_rtcsel_mask == (0x3u << 8) && rcc_rtcen == (1u << 15));
static_assert(rcc_bdrst == (1u << 16));
static_assert(rtc_ctlrl_flags == (rtc_secf | rtc_alrf | rtc_owf | rtc_rsf));
static_assert((rtc_ctlrl_flags & rtc_rtoff) == 0u);   // RTOFF is read-only
static_assert((rtc_ctlrl_flags & rtc_cnf) == 0u);
static_assert(rtc_ctlrh_enables == (rtc_secie | rtc_alrie | rtc_owie));

// The alarm's second path is EXTI line 17, which is what makes it a
// wake source: exti.hpp names the line, this file only uses it.
static_assert(Rtc::wake_line == Exti::line_rtc_alarm);
static_assert(Exti::implemented(Rtc::wake_line));

// ---- the prescaler's arithmetic (6.3.4) -------------------------------------
static_assert(rtc_prescaler_max == 0x000FFFFFu);
static_assert(rtc_prescaler_valid(rtc_prescaler_max));
static_assert(!rtc_prescaler_valid(rtc_prescaler_max + 1u));

// 6.3.4's own example: 0x7FFF out of 32768 Hz is a one-second tick.
static_assert(rtc_prescaler_for(32768u) == 0x7FFFu);
static_assert(rtc_tick_hz(32768u, 0x7FFFu) == 1u);
static_assert(rtc_prescaler_for(32768u, 2u) == 16383u);
static_assert(rtc_prescaler_for(32768u, 32768u) == 0u);

// EXACT OR NOTHING: a ratio that is not whole, and one the twenty-bit
// field cannot hold, both answer the sentinel.
static_assert(rtc_prescaler_for(32768u, 3u) == rtc_prescaler_none);
static_assert(rtc_prescaler_for(0u) == rtc_prescaler_none);
static_assert(rtc_prescaler_for(32768u, 0u) == rtc_prescaler_none);
static_assert(!rtc_prescaler_valid(rtc_prescaler_none));

// An RTCCLK of 8 MHz - the crystal UNDIVIDED, which RTCSEL cannot
// actually select - is past the field: 2^20 is the longest division
// this prescaler makes.
static_assert(rtc_prescaler_for(8'000'000u) == rtc_prescaler_none);
static_assert(rtc_prescaler_for(1'048'576u) == rtc_prescaler_max);
static_assert(rtc_prescaler_for(1'048'577u) == rtc_prescaler_none);

// The crystal over the smaller of this class's two HSE divisions is
// still a whole number of hertz, so a one-second tick off it is exact.
static_assert(rtc_prescaler_for(8'000'000u / 128u) == 62499u);
static_assert(rtc_tick_hz(8'000'000u / 128u, 62499u) == 1u);

// The LSI's nominal rate makes a whole second at its typical corner,
// which is what a program with no crystal uses - and the RC's spread is
// what makes that a NOMINAL second.
static_assert(rtc_prescaler_for(device::lsi_typ_hz) == device::lsi_typ_hz - 1u);
static_assert(device::lsi_min_hz < device::lsi_max_hz);

// ---- the HSE's division, which the LOT decides on this class ----------------
static_assert(device::rtc_hse_div[0] == 512u || device::rtc_hse_div[0] == 128u);
static_assert(rtc_hse_divider_known == (device::rtc_hse_div[0] == device::rtc_hse_div[1]));
static_assert(rtc_hse_clock_hz(8'000'000u, 0) == 8'000'000u / device::rtc_hse_div[0]);
static_assert(rtc_hse_clock_hz(8'000'000u, 1) == 8'000'000u / device::rtc_hse_div[1]);
// An index past the pair asks the second entry rather than reading off
// the end.
static_assert(rtc_hse_clock_hz(8'000'000u, 7) == rtc_hse_clock_hz(8'000'000u, 1));

// ---- the configuration ------------------------------------------------------
static_assert(rtc_config_valid(RtcConfig{}));
static_assert(rtc_config_valid(RtcConfig{.prescaler = rtc_prescaler_max}));
static_assert(!rtc_config_valid(RtcConfig{.prescaler = rtc_prescaler_max + 1u}));
static_assert(!rtc_config_valid(RtcConfig{.prescaler = rtc_prescaler_none}));

// The 32 kHz pads are PC14/PC15, which two thirds of this series do not
// bond: the domain states it and a board file reads it.
static_assert(RtcDomain::has_lse_pins == device::has_port('C'));

// ---- every verb -------------------------------------------------------------
void domain_verbs() {
    RtcDomain::pwr_bus_clock(true);
    (void)RtcDomain::pwr_bus_clock();
    (void)RtcDomain::unlock(true);
    (void)RtcDomain::unlocked();
    (void)RtcDomain::bdctlr();
    RtcDomain::reset();

    RtcDomain::lse_enable(true);
    (void)RtcDomain::lse_enabled();
    (void)RtcDomain::lse_ready();
    (void)RtcDomain::lse_wait_ready();
    (void)RtcDomain::lse_wait_ready(1000u);
    (void)RtcDomain::lse_bypass(true);
    (void)RtcDomain::lse_bypass();

    (void)RtcDomain::select(RtcClockSource::lse);
    (void)RtcDomain::select(RtcClockSource::lsi);
    (void)RtcDomain::select(RtcClockSource::hse_divided);
    (void)RtcDomain::select(RtcClockSource::none);
    (void)RtcDomain::selected();
    RtcDomain::enable(true);
    (void)RtcDomain::enabled();
    (void)RtcDomain::open(RtcClockSource::lse);
    (void)RtcDomain::open(RtcClockSource::lsi, true);
}

void counter_verbs() {
    (void)Rtc::write_finished();
    (void)Rtc::wait_write_finished();
    (void)Rtc::wait_write_finished(1000u);
    (void)Rtc::synchronized();
    (void)Rtc::synchronize();
    (void)Rtc::synchronize(1000u);
    (void)Rtc::begin_config();
    (void)Rtc::in_config();
    (void)Rtc::end_config();

    (void)Rtc::configure<RtcConfig{.prescaler = 0x7FFFu}>();
    (void)Rtc::configure({.prescaler = rtc_prescaler_for(32768u),
                          .count = 1000u,
                          .alarm = 1010u,
                          .set_count = true,
                          .set_alarm = true});
    (void)Rtc::prescaler(0x7FFFu);
    (void)Rtc::count(0u);
    (void)Rtc::alarm(0xFFFFFFFFu);
    (void)Rtc::count();
    (void)Rtc::divider();

    (void)Rtc::interrupts(rtc_secie | rtc_alrie | rtc_owie);
    (void)Rtc::interrupts();
    (void)Rtc::flags();
    (void)Rtc::second();
    (void)Rtc::alarmed();
    (void)Rtc::overflowed();
    Rtc::clear(rtc_secf);
    Rtc::clear(rtc_alrf | rtc_owf);
    (void)Rtc::isr();

    (void)Rtc::arm_wake(true);
    (void)Rtc::arm_wake_event(true);
    (void)Rtc::alarm_isr();
    (void)Rtc::regs().CTLRL;
}
