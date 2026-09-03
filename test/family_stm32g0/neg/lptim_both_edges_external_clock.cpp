// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 26.4.12: clocked FROM Input1, "the LPTIM counter can be updated either
// on rising edges or falling edges of the input1 clock signal but not on
// both rising and falling edges".
#include "stm32g0/lptim.hpp"
using L = brio::Lptim<1>;
constexpr brio::LptimConfig cfg{.clock = brio::LptimClockSource::external_input1,
                                .clock_polarity = brio::LptimClockPolarity::both};
void f() { (void)L::configure<cfg>(); }
