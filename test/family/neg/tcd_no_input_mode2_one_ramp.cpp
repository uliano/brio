// mcu: avr128db48 avr128da48
// Table 25-5: input mode 2 ("execute the opposite compare cycle") is
// "Do not use" in One Ramp mode - there is no opposite ramp to execute.
#include "avrdx/tcd.hpp"
using namespace brio;
void f() {
    (void)Tcd<0>::init<TcdConfig{.waveform = TcdWaveform::one_ramp,
                                 .compare_b_clear = 999,
                                 .input_a = {.enable = true,
                                             .mode = TcdInputMode::exec_wait}}>();
}
