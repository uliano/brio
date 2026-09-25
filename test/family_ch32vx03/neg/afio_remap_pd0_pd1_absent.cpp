// mcu: ch32v203f8 ch32v203g8 ch32v203rb ch32v303vc
// The oscillator's pads as GPIO: two packages of this series bring out no
// pin of port D at all, the CH32V203RB's OSC_IN/OSC_OUT are function
// pins with no GPIO behind them (RM 10.2.11.2, datasheet note 4), and on
// the LQFP100 CH32V303VC PD0 and PD1 are pins of their own that need no
// remap (its datasheet's note 4). The part table says which
// (device::osc_pads_as_pd0_pd1), so the constant face refuses it.
#include "ch32vx03/afio.hpp"

void f() { brio::Afio::remap<brio::Remap::pd0_pd1, 1>(); }
