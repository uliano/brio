// Family smoke TU for samc/reset.hpp: every verb must COMPILE on the E,
// G and J 18A headers (tools/check_samc.sh sweeps all three).
//
// RSTC and the WDT are one instance each on every member of the family
// and neither register layout varies, so there is no package gating to
// check here - what this fixture pins instead is the ARITHMETIC and the
// legality rule, both of which are decisions this driver makes rather
// than reads from the header.

#include <stdint.h>

#include "samc/platform_sam.hpp"
#include "samc/reset.hpp"

using namespace brio;

// ---- the period encoding, against the chapter's own endpoints ---------------
//
// 23.6.2.4 promises "12 possible WDT time-out periods, selectable from
// 8ms to 16s"; the field is a cycle count of 8 << n at a nominal
// 1.024 kHz. These are the two ends and one middle.
static_assert(wdt_nominal_ms(WdtCycles::cyc8) == 8u);
static_assert(wdt_nominal_ms(WdtCycles::cyc1024) == 1000u);
static_assert(wdt_nominal_ms(WdtCycles::cyc16384) == 16000u);
static_assert(wdt_nominal_ms(WdtCycles::cyc16) == 2u * wdt_nominal_ms(WdtCycles::cyc8));

// The three fields share one encoding - that is why there is one enum.
static_assert(static_cast<uint8_t>(WdtCycles::cyc8) == WDT_CONFIG_PER_CYC8_Val);
static_assert(static_cast<uint8_t>(WdtCycles::cyc8) == WDT_CONFIG_WINDOW_CYC8_Val);
static_assert(static_cast<uint8_t>(WdtCycles::cyc8) == WDT_EWCTRL_EWOFFSET_CYC8_Val);

// ---- the one legality rule the driver enforces ------------------------------
//
// In NORMAL mode an early-warning offset at or past the period means the
// reset arrives first and the interrupt never does (23.6.8.2). In WINDOW
// mode the offset is not used at all, so the same numbers are fine.
static_assert(Watchdog::config_valid(WdtConfig{
    .period = WdtCycles::cyc1024, .early_warning = true,
    .ew_offset = WdtCycles::cyc512}));
static_assert(!Watchdog::config_valid(WdtConfig{
    .period = WdtCycles::cyc512, .early_warning = true,
    .ew_offset = WdtCycles::cyc512}));
static_assert(!Watchdog::config_valid(WdtConfig{
    .period = WdtCycles::cyc256, .early_warning = true,
    .ew_offset = WdtCycles::cyc1024}));
static_assert(Watchdog::config_valid(WdtConfig{
    .period = WdtCycles::cyc256, .window_mode = true, .early_warning = true,
    .ew_offset = WdtCycles::cyc1024}), "window mode ignores the offset");
static_assert(Watchdog::config_valid(WdtConfig{
    .period = WdtCycles::cyc8, .early_warning = false,
    .ew_offset = WdtCycles::cyc16384}), "no warning asked for, no rule to break");

// ---- every verb instantiates ------------------------------------------------

void reset_verbs() {
    (void)Reset::cause_bits();
    (void)Reset::cause();
    (void)Reset::power_supply_reset();
    (void)Reset::user_reset();
    (void)Reset::warm();
    // Reset::software() is [[noreturn]] and is exercised by the bench
    // suite, not here: a family TU that called it would still compile,
    // but naming it in a function with other statements would make them
    // unreachable.
}

void watchdog_verbs() {
    constexpr WdtConfig cfg{
        .period = WdtCycles::cyc2048,
        .window_mode = false,
        .window = WdtCycles::cyc8,
        .early_warning = true,
        .ew_offset = WdtCycles::cyc1024,
        .always_on = false,
    };
    (void)Watchdog::arm<cfg>();
    (void)Watchdog::arm(cfg);

    Watchdog::bus_clock(true);
    (void)Watchdog::busy();
    (void)Watchdog::sync();

    (void)Watchdog::ctrla();
    (void)Watchdog::enabled();
    (void)Watchdog::window_mode();
    (void)Watchdog::always_on();
    (void)Watchdog::config();
    (void)Watchdog::period();
    (void)Watchdog::window();
    (void)Watchdog::ew_offset();

    Watchdog::clear();
    (void)Watchdog::disable();
    Watchdog::force_reset();

    (void)Watchdog::flags();
    (void)Watchdog::armed();
    Watchdog::clear_flags();
    (void)Watchdog::early_warning_flag();
    Watchdog::arm_interrupt(true);
    (void)Watchdog::isr();
}

// The panic reporter and the fault body are template/plain functions the
// app binds; naming them here is what checks they instantiate against a
// real Platform.
void panic_glue() {
    ResetReporter::report(PanicCode::assert_failed, 0);
}

[[noreturn]] void fault_body() { hard_fault_reset<SamPlatform>(0x42); }
