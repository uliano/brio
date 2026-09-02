// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// TX and RX cannot be the same pad.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
using namespace brio;
constexpr UartPins p{.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 2, PinFunction::af1}};
void f() { (void)Uart<2, p>::init(Clock<ClockSource::pll, 64'000'000>{}, 115200); }
