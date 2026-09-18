// Reset family smoke TU: the causes word over POWMAN's always-on
// latches and the watchdog's REASON, the two reboots and the reporter,
// the fault body both crts can enter - every verb instantiated once, and
// the one verb an architecture has not got called only where it exists.
#include "rp2350/platform.hpp"
#include "rp2350/reset.hpp"

using namespace brio;

using P = Rp2350Platform<>;

// The word's two halves do not overlap, and every bit of `all` is one of
// them: the chip-level latches a system reset leaves standing, and the
// watchdog's pair that a processor's own reset clears.
static_assert((ResetCause::chip_level & ResetCause::watchdog) == 0u);
static_assert((ResetCause::chip_level | ResetCause::watchdog) == ResetCause::all);
static_assert((ResetCause::watchdog_psm & ResetCause::chip_level) != 0u);
static_assert(ResetCause::watchdog == (ResetCause::watchdog_timer | ResetCause::watchdog_force));

void reset_reads() {
    (void)Reset::causes();
    (void)(Reset::causes() & ResetCause::power_on);
    (void)(Reset::causes() & ResetCause::rescue);
    (void)(Reset::causes() & ResetCause::watchdog_psm);
    (void)Reset::rescue_flag();
    (void)Reset::double_tap();
}

/// The reboots, in a function nothing calls: each of them ends the
/// program, and this TU is a compile and not a run.
[[noreturn]] void reset_reboots() {
    ResetReporter::report(PanicCode::assert_failed, 0);
    Reset::software();
}

[[noreturn]] void reset_fault() { fault_reset<P>(0x11); }

/// Reset::core() exists on the Cortex-M33 half alone (the hart's reset
/// controls are the Debug Module's on the other one), so the call sits
/// in a TEMPLATE: the discarded branch of an `if constexpr` inside one
/// is never instantiated, which is exactly the compile-time refusal the
/// verb carries.
template <CoreKind k>
void reset_core_where_it_exists() {
    if constexpr (k == CoreKind::cortex_m33) {
        Reset::core();
    }
}

void reset_core_verb() { reset_core_where_it_exists<core_kind>(); }
