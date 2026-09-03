// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 26.4.12: with COUNTMODE = 1 "the internal clock provided to the LPTIM
// must not be prescaled (PRESC[2:0] = 000)".
#include "stm32g0/lptim.hpp"
using L = brio::Lptim<1>;
constexpr brio::LptimConfig cfg{.count_external = true,
                                .prescaler = brio::LptimPrescaler::div8};
void f() { (void)L::configure<cfg>(); }
