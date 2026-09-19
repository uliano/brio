// POWMAN family smoke TU: the always-on block whole - the power states
// and the legality table as constants, the state machine's flags, the
// four GPIO power-up sources, the debugger's standing request, the
// sequencer, the regulator and the brown-out detector decoded read-only,
// the scratch words, the interrupts - and the always-on timer beside
// them: the counter, the alarm in both duties, the three tick sources
// and the divisor a measurement writes.
#include "rp2350/clock.hpp"
#include "rp2350/powman.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;

// ---- the state codes ARE the datasheet's four-bit field ------------------
static_assert(state_code(PowerState::p0_0) == 0x0u);
static_assert(state_code(PowerState::p1_0) == 0x8u);
static_assert(state_code(PowerState::p1_4) == 0xCu);
static_assert(state_code(PowerState::p1_7) == 0xFu);
static_assert(!is_low_power_state(PowerState::p0_3));
static_assert(is_low_power_state(PowerState::p1_3));

// ---- 6.2.3's table, every row the register description states ------------
static_assert(power_transition_legal(PowerState::p0_0, PowerState::p1_0));
static_assert(power_transition_legal(PowerState::p0_0, PowerState::p1_7));
static_assert(power_transition_legal(PowerState::p0_0, PowerState::p0_3));
static_assert(power_transition_legal(PowerState::p0_3, PowerState::p0_0));
static_assert(power_transition_legal(PowerState::p1_0, PowerState::p0_0));
static_assert(power_transition_legal(PowerState::p1_1, PowerState::p0_1));
static_assert(power_transition_legal(PowerState::p1_7, PowerState::p0_0));
// A P1.m state may only be left for a P0.m one.
static_assert(!power_transition_legal(PowerState::p1_0, PowerState::p1_1));
static_assert(!power_transition_legal(PowerState::p1_7, PowerState::p1_0));
// No request may power one domain up and another down at once: P0.1
// (SRAM1 off) to P1.2 (SRAM0 off) would do both.
static_assert(!power_transition_legal(PowerState::p0_1, PowerState::p1_2));
static_assert(!power_transition_legal(PowerState::p0_2, PowerState::p0_1));

// ---- the two voltage tables ---------------------------------------------
static_assert(vreg_vsel_mv(0x0Bu) == 1100u);   // the reset code
static_assert(vreg_vsel_mv(0u) == 550u);
static_assert(vreg_vsel_mv(31u) == 3300u);
static_assert(bod_vsel_mv(0x09u) == 860u);     // the reset code
static_assert(bod_vsel_mv(0u) == 473u);
static_assert(bod_vsel_mv(17u) == 1204u);
static_assert(bod_vsel_mv(18u) == 0u);
static_assert(Powman::high_temp_threshold_c(5u) == 125u);

// ---- what a power-up source may be --------------------------------------
static_assert(pwrup_source_valid(0u));
static_assert(pwrup_source_valid(PwrupQspiSource::ss));
static_assert(pwrup_source_valid(PwrupQspiSource::sclk));
static_assert(!pwrup_source_valid(54u));
static_assert(!pwrup_source_valid(63u));
static_assert(pwrup_bit(PowerUpSource::alarm) == (1UL << 6));
static_assert(pwrup_bit(PowerUpSource::coresight) == (1UL << 5));

void powman_state() {
    (void)Powman::current_state();
    (void)Powman::requested_state();
    (void)Powman::changing();
    (void)Powman::waiting();
    (void)Powman::bad_software_request();
    (void)Powman::bad_hardware_request();
    (void)Powman::powerup_while_waiting();
    (void)Powman::request_ignored();
    Powman::clear_request_flags();
    (void)Powman::wait_settled();
    (void)Powman::powerup_armed();
    (void)Powman::request_state(PowerState::p0_1);
    (void)Powman::request_state(PowerState::p0_0);
    (void)Powman::power_down(PowerState::p1_0);
    (void)Powman::power_down(PowerState::p1_7);
}

void powman_password() {
    (void)Powman::bad_password();
    Powman::clear_bad_password();
}

void powman_pwrups() {
    (void)Powman::pwrup(0, PwrupConfig{});
    (void)Powman::pwrup(1, PwrupConfig{.source = 19,
                                       .mode = PwrupMode::edge,
                                       .direction = PwrupDirection::high_rising});
    (void)Powman::pwrup(2, PwrupConfig{.source = PwrupQspiSource::sd0,
                                       .mode = PwrupMode::level,
                                       .direction = PwrupDirection::low_falling});
    (void)Powman::pwrup(3, PwrupConfig{.source = 0, .mode = PwrupMode::level,
                                       .direction = PwrupDirection::high_rising});
    for (uint8_t s = 0; s < 4u; ++s) {
        (void)Powman::pwrup_enabled(s);
        (void)Powman::pwrup_source(s);
        (void)Powman::pwrup_raw(s);
        (void)Powman::pwrup_status(s);
        Powman::pwrup_clear(s);
        Powman::pwrup_disable(s);
    }
    Powman::pwrup_disable_all();
    (void)Powman::current_powerup_requests();
    (void)Powman::last_powerup_requests();
    (void)Powman::last_powerup_source();
}

void powman_debug() {
    (void)Powman::debug_powerup_pending();
    Powman::ignore_debug_powerup(true);
    Powman::ignore_debug_powerup(false);
    (void)Powman::ignoring_debug_powerup();
}

void powman_sequencer() {
    const SequencerConfig c = Powman::sequencer();
    (void)c.using_fast_powck;
    (void)c.using_bod_lp;
    (void)c.using_vreg_lp;
    (void)c.use_fast_powck;
    (void)c.run_lposc_in_lp;
    (void)c.use_bod_hp;
    (void)c.use_bod_lp;
    (void)c.use_vreg_hp;
    (void)c.use_vreg_lp;
    (void)c.hw_pwrup_sram0;
    (void)c.hw_pwrup_sram1;
    (void)Powman::sram_stays_down_on_powerup(false, false);
    (void)Powman::run_lposc_in_low_power(true);
}

void powman_supplies() {
    const VregStatus v = Powman::vreg();
    (void)v.vsel;
    (void)v.mv;
    (void)v.high_impedance;
    (void)v.updating;
    (void)v.vout_ok;
    (void)v.starting_up;
    (void)v.unlocked;
    (void)v.voltage_limit_off;
    (void)v.isolated;
    (void)v.high_temp_code;
    (void)v.high_temp_c;
    const VregLowPower e = Powman::vreg_low_power_entry();
    const VregLowPower x = Powman::vreg_low_power_exit();
    (void)e.vsel;
    (void)e.mv;
    (void)e.linear;
    (void)e.high_impedance;
    (void)x.linear;
    const BodStatus b = Powman::bod();
    (void)b.enabled;
    (void)b.vsel;
    (void)b.mv;
    (void)b.isolated;
    (void)b.lp_entry_enabled;
    (void)b.lp_entry_mv;
    (void)b.lp_exit_enabled;
    (void)b.lp_exit_mv;
}

void powman_scratch() {
    for (uint8_t n = 0; n < 8u; ++n) {
        Powman::scratch(n, 0xA5A5'0000u | n);
        (void)Powman::scratch(n);
    }
    for (uint8_t n = 0; n < 4u; ++n) {
        (void)Powman::boot_word(n);
    }
}

void powman_interrupts() {
    (void)Powman::raw_interrupts();
    Powman::clear_raw(Powman::Interrupt::vreg_output_low);
    Powman::interrupt(Powman::Interrupt::timer, true);
    Powman::interrupt(Powman::Interrupt::all, false);
    (void)Powman::interrupts();
    Powman::force(Powman::Interrupt::state_req_ignored, true);
    Powman::force(Powman::Interrupt::pwrup_while_waiting, false);
    (void)Powman::forced();
    (void)Powman::pending();
    (void)Powman::irq_pow();
    (void)Powman::irq_timer();
}

// ---- the always-on timer -------------------------------------------------

void aon_counter() {
    (void)AonTimer::running();
    AonTimer::run(false);
    (void)AonTimer::set(1'700'000'000'000ULL);
    AonTimer::run(true);
    AonTimer::clear();
    (void)AonTimer::now();
    (void)AonTimer::now_low();
}

void aon_alarm() {
    AonTimer::alarm_at(1234ULL);
    (void)AonTimer::alarm_in(500u);
    (void)AonTimer::alarm_time();
    AonTimer::alarm_enable(false);
    (void)AonTimer::alarm_enabled();
    (void)AonTimer::alarm_fired();
    AonTimer::clear_alarm();
    AonTimer::powerup_on_alarm(true);
    (void)AonTimer::powerup_on_alarm();
    AonTimer::interrupt(true);
    (void)AonTimer::interrupt_enabled();
    (void)AonTimer::pending();
    (void)AonTimer::isr();
    (void)AonTimer::irq();
}

void aon_source() {
    (void)AonTimer::source();
    (void)AonTimer::synchronised_to_gpio_1hz();
    (void)AonTimer::use_xosc();
    (void)AonTimer::use_lposc();
    (void)AonTimer::time_reference(AonTimeRefPin::gpio12, false);
    (void)AonTimer::time_reference(AonTimeRefPin::gpio20, true);
    (void)AonTimer::time_reference(AonTimeRefPin::gpio14, false);
    (void)AonTimer::time_reference(AonTimeRefPin::gpio22, false);
}

void aon_divisor() {
    (void)AonTimer::lposc_khz_int();
    (void)AonTimer::lposc_khz_frac();
    (void)AonTimer::xosc_khz_int();
    (void)AonTimer::xosc_khz_frac();
    (void)AonTimer::declared_hz();
    (void)AonTimer::lposc_khz(32u, 0xC49Cu);
    (void)AonTimer::xosc_khz(12000u, 0u);
    constexpr SysClock c;
    (void)AonTimer::calibrate(c);
    (void)AonTimer::calibrate(c, 10u);
    (void)AonTimer::init(c);
}
