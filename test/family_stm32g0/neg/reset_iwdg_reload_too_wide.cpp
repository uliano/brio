// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// IWDG_RLR.RL is twelve bits (RM0444 28.4.3): a reload above 0x0FFF
// would be silently truncated by the register, which is a different
// time-out from the one the caller asked for.
#include "stm32g0/reset.hpp"
using namespace brio;
void f() { (void)Iwdg::configure<IwdgConfig{.reload = 0x1000}>(); }
