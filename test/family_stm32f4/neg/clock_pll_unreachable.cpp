// No exact M/N/P produces this rate from HSI within the PLL's windows.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/clock.hpp"
using C = brio::Clock<brio::ClockSource::pll_hsi, 100'000'001>;
void f() { (void)C::init(); }
