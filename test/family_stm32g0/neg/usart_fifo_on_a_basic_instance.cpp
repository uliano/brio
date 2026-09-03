// mcu: stm32g0b1xx stm32g071xx
// The FIFOs are a FULL instance's (RM0444 table 184): USART4 is BASIC on
// every part that has it, and FIFOEN would not even stick there.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins p{.tx = {'A', 0, brio::PinFunction::af4},
                           .rx = {'A', 1, brio::PinFunction::af4}};
constexpr brio::UartOptions o{.fifo = true};
using Bad = brio::Uart<4, p, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, o>;
void f() { (void)Bad::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 9600); }
