// The full-duplex extension block is not on every part: the F446 pairs
// two whole instances instead, and its header declares no I2S2ext_BASE.
// mcu: stm32f446xx
#include "stm32f4/spi.hpp"
void f() { brio::I2sExt<2>::enable(); }
