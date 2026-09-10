// Reset family smoke TU: the flags as RCC_RSTSCKR carries them, the
// read-and-clear verb, the core's reset request, the reporter that
// resets and the fault body that records.
#include "ch32v00x/platform.hpp"
#include "ch32v00x/reset.hpp"
#include "kernel/panic.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;

static_assert(ResetFlag::software == (1UL << 28));
static_assert(ResetFlag::power == (1UL << 27));
static_assert(ResetFlag::pin == (1UL << 26));
static_assert((ResetFlag::all & rstsckr_rmvf) == 0u, "RMVF is not a flag");
static_assert((ResetFlag::all & rstsckr_lsion) == 0u, "the LSI bits are the clock tree's");
static_assert(ResetFlag::watchdog == (ResetFlag::window_watchdog | ResetFlag::independent_watchdog));

void reset_verbs() {
    (void)Reset::flags();
    Reset::clear_flags();
    (void)Reset::take_flags();
}

[[noreturn]] void reset_now() { Reset::software(); }

[[noreturn]] void die() { panic<P, ResetReporter>(PanicCode::queue_overflow, 1); }

[[noreturn]] void fault_body() { fault_reset<P>(0x51); }

// ---- the two watchdogs (RM ch. 4 and 5) --------------------------------------
static_assert(iwdg_divider(IwdgPrescaler::div256) == 256u);
static_assert(iwdg_timeout_ms(IwdgPrescaler::div32, 999, 124'000UL) == 258u);
static_assert(wwdg_timeout_us(48'000'000UL, WwdgPrescaler::div8, 0x7F) == 43690u);

void watchdog_verbs() {
    (void)Iwdg::arm({.prescaler = IwdgPrescaler::div64, .reload = 0x0FFF});
    (void)Iwdg::configure({.prescaler = IwdgPrescaler::div4, .reload = 100});
    Iwdg::refresh();
    (void)Iwdg::busy();
    (void)Iwdg::sync();
    (void)Iwdg::prescaler();
    (void)Iwdg::reload();
    (void)Iwdg::status();
    (void)Iwdg::debug_freeze();
    Iwdg::debug_freeze(true);
    Iwdg::force_reset();

    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div2, .window = 0x60, .early_wakeup = true});
    Wwdg::start(0x7F);
    Wwdg::refresh(0x70);
    (void)Wwdg::enabled();
    (void)Wwdg::counter();
    (void)Wwdg::prescaler();
    (void)Wwdg::window();
    (void)Wwdg::early_wakeup_enabled();
    (void)Wwdg::flag();
    Wwdg::clear_flag();
    (void)Wwdg::isr();
    (void)Wwdg::bus_clock();
    Wwdg::bus_clock(false);
    (void)Wwdg::debug_freeze();
    Wwdg::debug_freeze(false);
    Wwdg::force_reset();
}
