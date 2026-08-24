// mcu: avr128db48 avr128da48
// 25.3.3.5, table 25-7: dithering is "Not supported" in Dual Slope mode
// for every DITHERSEL.
#include "avrdx/tcd.hpp"
using namespace brio;
void f() {
    (void)Tcd<0>::init<TcdConfig{.waveform = TcdWaveform::dual_slope,
                                 .compare_b_clear = 999,
                                 .dither = 8}>();
}
