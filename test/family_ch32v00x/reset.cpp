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
