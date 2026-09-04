// mcu: stm32g030xx stm32g050xx stm32g070xx stm32g0b0xx
// The value line's PWR has no CR2 - no programmable voltage detector and
// no supply monitor - and the reserve says so at compile time: Pwr::has_pvd
// is false there, and a program that static_asserts on it is refused. (The
// verbs themselves stay callable and answer false at run time; this is
// the compile-time half of the same fact.)
#include "stm32g0/pwr.hpp"
static_assert(brio::Pwr::has_pvd, "this program needs a PVD");
void f() { (void)brio::Pwr::pvd_enable(true); }
