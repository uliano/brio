// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// WWDG_CFR.W is seven bits (RM0444 29.5.2): a wider value would be
// truncated into a different window than the one asked for.
#include "stm32g0/reset.hpp"
using namespace brio;
void f() { (void)Wwdg::configure<WwdgConfig{.window = 0x80}>(); }
