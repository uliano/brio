// DPLLCTRLB.DIV divides the XOSC reference and nothing else (20.8.14).
// Asking for it on the GCLK reference would silently do nothing, so it
// is refused at compile time.
// mcu: samc21e18a samc21g18a samc21j18a
#include "samc/clock.hpp"

using namespace brio;

void bad() {
    (void)Fdpll::init<FdpllConfig{.reference = DpllReference::gclk,
                                  .reference_hz = 2'000'000,
                                  .xosc_div = 1,
                                  .ldr = 23}>();
}
