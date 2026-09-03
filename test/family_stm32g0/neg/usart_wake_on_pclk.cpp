// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A wake from Stop needs a kernel clock that survives Stop - HSI16 or
// LSE (RM0444 33.5.21). PCLK is stopped, so the receiver would never see
// the start bit.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins p{.tx = {'A', 9, brio::PinFunction::af1},
                           .rx = {'A', 10, brio::PinFunction::af1}};
constexpr brio::UartOptions o{.wake_from_stop = brio::UsartWakeSource::start_bit};
using Bad = brio::Uart<1, p, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, o>;
void f() { (void)Bad::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 9600); }
