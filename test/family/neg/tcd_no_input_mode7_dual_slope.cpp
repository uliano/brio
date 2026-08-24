// mcu: avr128db48 avr128da48
// Errata 2.14.3 (DB, every revision incl. B0) / 2.13.3 (DA, every
// revision): halting and waiting for a software restart (INPUTMODE 7)
// does not work in Dual Slope mode. No silicon escapes it, so it is
// refused outright.
#include "avrdx/tcd.hpp"
using namespace brio;
void f() {
    (void)Tcd<0>::init<TcdConfig{.waveform = TcdWaveform::dual_slope,
                                 .compare_a_set = 100,
                                 .compare_b_clear = 999,
                                 .input_a = {.enable = true,
                                             .mode = TcdInputMode::wait_sw}}>();
}
