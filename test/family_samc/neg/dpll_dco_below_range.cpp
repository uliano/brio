// The DPLL's DCO must run at 48..96 MHz (table 45-52). The output
// prescaler divides what the DCO made and is not a way below the floor,
// so a ratio producing 24 MHz is refused at compile time.
// mcu: samc21e18a samc21g18a samc21j18a
#include "samc/clock.hpp"

using namespace brio;

void bad() {
    (void)Fdpll::init<FdpllConfig{.reference = DpllReference::gclk,
                                  .reference_hz = 2'000'000,
                                  .ldr = 11}>();
}
