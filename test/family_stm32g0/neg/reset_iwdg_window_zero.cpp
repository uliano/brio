// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// An IWDG window of zero leaves no counter value below it, so no refresh
// is ever legal and the reset is certain (RM0444 28.3.2). A watchdog
// that cannot be served must not be armed by accident - Iwdg::
// force_reset() is the deliberate spelling.
#include "stm32g0/reset.hpp"
using namespace brio;
void f() { (void)Iwdg::configure<IwdgConfig{.window = 0}>(); }
