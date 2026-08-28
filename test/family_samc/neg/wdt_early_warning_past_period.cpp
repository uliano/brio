// mcu: samc21j18a
// In NORMAL mode an early-warning offset at or past the period means the
// time-out reset arrives before the interrupt, so the warning the caller
// asked for would never be generated (23.6.8.2). Must not compile.

#include "samc/reset.hpp"

using namespace brio;

void bad() {
    constexpr WdtConfig cfg{
        .period = WdtCycles::cyc256,
        .early_warning = true,
        .ew_offset = WdtCycles::cyc1024,
    };
    (void)Watchdog::arm<cfg>();
}
