// mcu: avr128db48 avr128da48
// The other half of errata 2.14.3 / 2.13.3: INPUTMODE 7 does not work
// with CMPASET = 0 either, on every revision of both families.
#include "avrdx/tcd.hpp"
using namespace brio;
void f() {
    (void)Tcd<0>::init<TcdConfig{.waveform = TcdWaveform::one_ramp,
                                 .compare_a_set = 0,
                                 .compare_b_clear = 999,
                                 .input_a = {.enable = true,
                                             .mode = TcdInputMode::wait_sw}}>();
}
