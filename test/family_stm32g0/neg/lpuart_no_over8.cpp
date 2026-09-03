// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The LPUART's baud generator is 256 x fck / LPUARTDIV and nothing else
// (RM0444 34.4.7): there is no OVER8 to ask for.
#include "stm32g0/clock.hpp"
#include "stm32g0/lpuart.hpp"
constexpr brio::UartPins p{.tx = {'C', 1, brio::PinFunction::af1},
                           .rx = {'C', 0, brio::PinFunction::af1}};
constexpr brio::UartOptions o{.over8 = true};
using Bad = brio::LpUart<1, p, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, o>;
void f() { (void)Bad::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 9600); }
