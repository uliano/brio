// The reference the DPLL loop sees must be inside 32 kHz .. 2 MHz
// (table 45-52); a 4 MHz generator is refused at compile time.
// mcu: samc21e18a samc21g18a samc21j18a
#include "samc/clock.hpp"

using namespace brio;

void bad() {
    (void)Fdpll::init<FdpllConfig{.reference = DpllReference::gclk,
                                  .reference_hz = 4'000'000,
                                  .ldr = 11}>();
}
