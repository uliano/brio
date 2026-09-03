// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 33.8.14 implements twelve codes; the rest are Reserved, and the
// silicon turns one into 1011 (divide by 256) rather than ignoring it.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins p{.tx = {'A', 9, brio::PinFunction::af1},
                           .rx = {'A', 10, brio::PinFunction::af1}};
constexpr brio::UartOptions o{.prescaler = static_cast<brio::UsartPrescaler>(12)};
using Bad = brio::Uart<1, p, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, o>;
void f() { (void)Bad::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 9600); }
