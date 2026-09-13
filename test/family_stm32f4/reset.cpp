// Reset family smoke TU: the reset flags, both watchdogs, the four
// fault vectors and the panic bodies over them (stm32f4/reset.hpp) -
// every verb named once so that every header of the pack compiles them,
// plus the constants the chapter's tables can be checked against at
// compile time.
//
// WHAT THIS TU PROVES ACROSS THE FAMILY: that RCC_CSR's seven flags,
// IWDG's four registers, WWDG's three and the DBGMCU freeze bits are
// spelled the same on all twenty-three headers (they are), that no
// header declares an IWDG window register, and that the WWDG's
// prescaler field is two bits wide everywhere.
#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/reset.hpp"

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the reserve's facts ------------------------------------------------

// No window on this family's IWDG: the driver has no window verb, and
// this is what makes that a checked claim.
static_assert(!iwdg_has_window());
static_assert(wwdg_prescaler_codes() == 4);
static_assert(wwdg_present());

// ---- the flags are seven distinct bits, and PINRSTF is one of them ------

static_assert((ResetFlag::all & ResetFlag::pin) != 0u);
static_assert(ResetFlag::watchdog ==
              (ResetFlag::window_watchdog | ResetFlag::independent_watchdog));
static_assert(ResetFlag::supply == (ResetFlag::power_on | ResetFlag::brown_out));
static_assert(Reset::pin_only(ResetFlag::pin));
static_assert(!Reset::pin_only(ResetFlag::pin | ResetFlag::software));
static_assert(!Reset::pin_only(0u));

// ---- IWDG: the dividers and RM0090 table 107's max column ---------------

static_assert(iwdg_divider(IwdgPrescaler::div4) == 4);
static_assert(iwdg_divider(IwdgPrescaler::div256) == 256);
static_assert(iwdg_nominal_ms(IwdgPrescaler::div4, 0x0FFF) == 512);
static_assert(iwdg_nominal_ms(IwdgPrescaler::div8, 0x0FFF) == 1024);
static_assert(iwdg_nominal_ms(IwdgPrescaler::div64, 0x0FFF) == 8192);
static_assert(iwdg_nominal_ms(IwdgPrescaler::div256, 0x0FFF) == 32768);
// The rate is the caller's: the same setting at a slower LSI is longer.
static_assert(iwdg_nominal_ms(IwdgPrescaler::div8, 0x0EEE, 30'000) >
              iwdg_nominal_ms(IwdgPrescaler::div8, 0x0EEE, 34'000));
static_assert(iwdg_config_valid(IwdgConfig{}));
static_assert(iwdg_config_valid(IwdgConfig{IwdgPrescaler::div256, 0}));
static_assert(!iwdg_config_valid(IwdgConfig{IwdgPrescaler::div4, 0x1000}));

// ---- WWDG: the steps and RM0090 table 109 (30 MHz PCLK1) ----------------

static_assert(wwdg_step_cycles(WwdgPrescaler::div1) == 4096);
static_assert(wwdg_step_cycles(WwdgPrescaler::div8) == 32768);
static_assert(wwdg_timeout_us(30'000'000, WwdgPrescaler::div1, 0x40) == 136);
static_assert(wwdg_timeout_us(30'000'000, WwdgPrescaler::div1, 0x7F) == 8738);
static_assert(wwdg_timeout_us(30'000'000, WwdgPrescaler::div2, 0x7F) == 17476);
static_assert(wwdg_timeout_us(30'000'000, WwdgPrescaler::div4, 0x7F) == 34952);
static_assert(wwdg_timeout_us(30'000'000, WwdgPrescaler::div8, 0x7F) == 69905);
// A counter at or below 0x3F is past the reset, and a rate below a
// megahertz is refused rather than answered wrongly.
static_assert(wwdg_timeout_us(30'000'000, WwdgPrescaler::div1, 0x3F) == 0);
static_assert(wwdg_timeout_us(500'000, WwdgPrescaler::div1, 0x7F) == 0);
static_assert(wwdg_config_valid(WwdgConfig{}));
static_assert(!wwdg_config_valid(WwdgConfig{WwdgPrescaler::div1, 0x3F}));
static_assert(!wwdg_config_valid(WwdgConfig{WwdgPrescaler::div1, 0x80}));

// ---- FaultRecord: the predicates over the core's status words -----------

static_assert(FaultRecord{}.empty());
static_assert(FaultRecord{SCB_CFSR_DIVBYZERO_Msk, 0, 0}.usage_fault());
static_assert(!FaultRecord{SCB_CFSR_DIVBYZERO_Msk, 0, 0}.bus_fault());
static_assert(FaultRecord{0, SCB_HFSR_FORCED_Msk, 0}.escalated());
static_assert(!FaultRecord{SCB_CFSR_DIVBYZERO_Msk, 0, 0}.address_valid());

// ---- every verb, named once ---------------------------------------------

void reset_verbs() {
    (void)Reset::flags();
    Reset::clear_flags();
    (void)Reset::take_flags();
    (void)Reset::pin_only(Reset::flags());
    // Reset::software() is [[noreturn]]: naming it here would end the
    // function. Its address is taken instead, which compiles the body.
    (void)static_cast<void (*)()>(&Reset::software);
}

void iwdg_verbs() {
    (void)Rcc::lsi_enabled();
    Rcc::lsi_enable(true);
    (void)Rcc::lsi_ready();
    (void)Rcc::lsi_wait_ready();

    Iwdg::refresh();
    Iwdg::unlock();
    Iwdg::start();
    (void)Iwdg::running();
    (void)Iwdg::status();
    (void)Iwdg::busy();
    (void)Iwdg::busy(IWDG_SR_RVU_Msk);
    (void)Iwdg::sync();
    (void)Iwdg::sync(IWDG_SR_PVU_Msk);
    (void)Iwdg::prescaler();
    (void)Iwdg::prescaler_bits();
    (void)Iwdg::reload();
    (void)Iwdg::configure(IwdgConfig{IwdgPrescaler::div8, 0x0EEE});
    (void)Iwdg::configure<IwdgConfig{IwdgPrescaler::div256, 0x0FFF}>();
    (void)Iwdg::arm(IwdgConfig{IwdgPrescaler::div32, 0x0100});
    (void)Iwdg::force_reset();
    (void)Iwdg::debug_frozen();
}

void wwdg_verbs() {
    Wwdg::bus_clock(true);
    (void)Wwdg::bus_clock();
    (void)Wwdg::irq();
    (void)Wwdg::cr();
    (void)Wwdg::cfr();
    (void)Wwdg::enabled();
    (void)Wwdg::counter();
    (void)Wwdg::prescaler();
    (void)Wwdg::window();
    (void)Wwdg::early_wakeup_enabled();
    (void)Wwdg::in_window();
    (void)Wwdg::configure(WwdgConfig{WwdgPrescaler::div8, 0x5A, true});
    (void)Wwdg::configure<WwdgConfig{WwdgPrescaler::div4, 0x60, false}>();
    Wwdg::refresh();
    Wwdg::refresh(0x60);
    Wwdg::start();
    Wwdg::start(0x70);
    Wwdg::force_reset();
    (void)Wwdg::flag();
    Wwdg::clear_flag();
    (void)Wwdg::isr();
    (void)Wwdg::debug_frozen();
}

void fault_verbs() {
    Faults::enable(true, true, true);
    (void)Faults::mem_enabled();
    (void)Faults::bus_enabled();
    (void)Faults::usage_enabled();
    Faults::divide_by_zero_trap(true);
    (void)Faults::divide_by_zero_trap();
    Faults::unaligned_trap(false);
    (void)Faults::unaligned_trap();
    const FaultRecord r = Faults::read();
    (void)r.empty();
    (void)r.mem_fault();
    (void)r.bus_fault();
    (void)r.usage_fault();
    (void)r.escalated();
    (void)r.address_valid();
    Faults::clear();
    (void)Faults::take();
}

// The two panic entry points: the Reporter panic() hands over to, and
// the body an app binds to a fault vector.
void panic_bodies() {
    (void)static_cast<void (*)(PanicCode, uint8_t)>(&ResetReporter::report);
    (void)static_cast<void (*)(uint8_t)>(&hard_fault_reset<P>);
}
