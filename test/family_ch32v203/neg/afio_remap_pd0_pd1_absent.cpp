// mcu: ch32v203f8 ch32v203g8 ch32v203rb
// The oscillator's pads as GPIO: two packages of this series bring out no
// pin of port D at all, and the CH32V203RB's OSC_IN/OSC_OUT are function
// pins with no GPIO behind them (RM 10.2.11.2, datasheet note 4). The
// column has no pad on these parts, so the constant face refuses it.
#include "ch32v203/afio.hpp"

void f() { brio::Afio::remap<brio::Remap::pd0_pd1, 1>(); }
