// The PLL triple is checked against RM0386 18.12.5's and DS11189 table
// 47's ranges at compile time by init<cfg, clkin>(): a rate no exact
// triple reaches (500 Mbit/s off 8 MHz with the PFD input held at
// 8 MHz) leaves ndiv at 0, and that is refused.
// mcu: stm32f469xx stm32f479xx
#include "stm32f4/dsi.hpp"
constexpr brio::DsiConfig cfg = brio::dsi_config_for(8'000'000u, 500'000'000u);
void f() { (void)brio::Dsi::init<cfg, 8'000'000u>(); }
