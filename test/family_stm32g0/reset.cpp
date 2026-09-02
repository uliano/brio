// Reset family smoke TU: the reset flags, both watchdogs and the panic
// paths over them (stm32g0/reset.hpp). Everything here is the same on
// every G0 variant - the RCC_CSR flag set, the IWDG's four registers,
// the WWDG's three and its unshared vector at position 0 - so what this
// TU proves is that the file compiles against each device header, that
// the constexpr arithmetic agrees with the two documents' own tables,
// and that the compile-time configure<> twins accept what they must.
#include "kernel/panic.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/reset.hpp"

using namespace brio;

// DS13560 table 73, the /4 and /256 rows at the nominal 32 kHz LSI.
static_assert(iwdg_divider(IwdgPrescaler::div4) == 4);
static_assert(iwdg_divider(IwdgPrescaler::div256) == 256);
static_assert(iwdg_nominal_ms(IwdgPrescaler::div4, 0x0FFF) == 512);
static_assert(iwdg_nominal_ms(IwdgPrescaler::div256, 0x0FFF) == 32768);
static_assert(iwdg_nominal_ms(IwdgPrescaler::div32, 0x0FFF) == 4096);
// The reload counts from RL down THROUGH zero, hence RL + 1 steps: the
// /4 minimum of table 73 is 0.125 ms, which is one step at 32 kHz.
static_assert(iwdg_nominal_ms(IwdgPrescaler::div4, 0, 32'000) == 0);

// The two refusals, as predicates.
static_assert(iwdg_config_valid(IwdgConfig{}));
static_assert(!iwdg_config_valid(IwdgConfig{.window = 0}));
static_assert(!iwdg_config_valid(IwdgConfig{.reload = 0x1000}));
static_assert(!iwdg_config_valid(IwdgConfig{.window = 0x1000}));

// RM0444 29.3.4's own worked example, translated to this family's
// ceiling: 4096 x 2^3 x (63 + 1) PCLK cycles is 43.69 ms at 48 MHz and
// 32.768 ms at 64.
static_assert(wwdg_step_cycles(WwdgPrescaler::div8) == 32768);
static_assert(wwdg_timeout_us(48'000'000UL, WwdgPrescaler::div8, 0x7F) == 43'690);
static_assert(wwdg_timeout_us(64'000'000UL, WwdgPrescaler::div8, 0x7F) == 32'768);
static_assert(wwdg_timeout_us(64'000'000UL, WwdgPrescaler::div1, 0x7F) == 4'096);
// One step from the warning to the reset, at the family's ceiling.
static_assert(wwdg_timeout_us(64'000'000UL, WwdgPrescaler::div128, 0x40) == 8'192);

static_assert(wwdg_config_valid(WwdgConfig{}));
static_assert(!wwdg_config_valid(WwdgConfig{.window = 0x3F}));
static_assert(!wwdg_config_valid(WwdgConfig{.window = 0x80}));

// The flags are seven distinct bits and PINRSTF is one of them.
static_assert((ResetFlag::all & ResetFlag::pin) == ResetFlag::pin);
static_assert(ResetFlag::watchdog ==
              (ResetFlag::window_watchdog | ResetFlag::independent_watchdog));
static_assert(Reset::pin_only(ResetFlag::pin));
static_assert(!Reset::pin_only(ResetFlag::pin | ResetFlag::software));

void reset_verbs() {
    (void)Reset::flags();
    (void)Reset::take_flags();
    Reset::clear_flags();
}

void iwdg_verbs() {
    (void)Rcc::lsi_enable(true);
    (void)Rcc::lsi_enabled();
    (void)Rcc::lsi_wait_ready();

    constexpr IwdgConfig cfg{
        .prescaler = IwdgPrescaler::div256,
        .reload = 0x0FFF,
        .window = 0x0FFF,
    };
    (void)Iwdg::configure<cfg>();
    (void)Iwdg::configure(cfg);
    (void)Iwdg::busy();
    (void)Iwdg::sync();
    (void)Iwdg::status();
    (void)Iwdg::prescaler();
    (void)Iwdg::prescaler_bits();
    (void)Iwdg::reload();
    (void)Iwdg::window();
    Iwdg::unlock();
    Iwdg::refresh();
    (void)Iwdg::debug_freeze();
    Iwdg::debug_freeze(true);
    // start()/arm()/force_reset() are one-way on real silicon; naming
    // them here is what proves they compile.
    if (false) {
        Iwdg::start();
        (void)Iwdg::arm(cfg);
        Iwdg::force_reset();
    }
}

void wwdg_verbs() {
    Wwdg::bus_clock(true);
    (void)Wwdg::bus_clock();
    static_assert(Wwdg::irq() == WWDG_IRQn);

    constexpr WwdgConfig cfg{
        .prescaler = WwdgPrescaler::div128,
        .window = 0x7F,
        .early_wakeup = true,
    };
    (void)Wwdg::configure<cfg>();
    (void)Wwdg::configure(cfg);
    (void)Wwdg::cr();
    (void)Wwdg::cfr();
    (void)Wwdg::enabled();
    (void)Wwdg::counter();
    (void)Wwdg::prescaler();
    (void)Wwdg::window();
    (void)Wwdg::early_wakeup_enabled();
    Wwdg::refresh();
    Wwdg::refresh(0x60);
    (void)Wwdg::flag();
    Wwdg::clear_flag();
    (void)Wwdg::isr();
    (void)Wwdg::debug_freeze();
    Wwdg::debug_freeze(false);
    if (false) {
        Wwdg::start();
        Wwdg::force_reset();
    }
}

// The two panic paths: a Reporter the kernel may be given, and the
// HardFault body an app binds to the vector.
extern "C" void HardFault_Handler();
extern "C" void HardFault_Handler() { hard_fault_reset<Stm32Platform>(0x11); }

void panic_paths() { panic<Stm32Platform, ResetReporter>(PanicCode::assert_failed, 7); }
