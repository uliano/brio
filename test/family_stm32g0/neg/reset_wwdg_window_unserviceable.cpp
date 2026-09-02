// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A WWDG refresh is legal only while the counter is at or below W AND
// above 0x3F (RM0444 29.3.3), so any window below 0x40 leaves no legal
// instant at all - refused, as the IWDG's zero window is.
#include "stm32g0/reset.hpp"
using namespace brio;
void f() { (void)Wwdg::configure<WwdgConfig{.window = 0x3F}>(); }
