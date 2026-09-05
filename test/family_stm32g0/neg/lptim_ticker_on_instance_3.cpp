// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// No part of the pack has a third LPTIM; the ticker refuses the config.
#include "stm32g0/lptim_ticker.hpp"
using Tb = brio::LptimTicker<brio::LptimTickerConfig{.instance = 3}>;
void f() { (void)Tb::ticks(); }
