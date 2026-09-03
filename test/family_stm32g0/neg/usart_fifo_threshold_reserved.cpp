// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// RXFTCFG/TXFTCFG are 000..101 (33.8.4); 110 and 111 are Reserved.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins p{.tx = {'A', 9, brio::PinFunction::af1},
                           .rx = {'A', 10, brio::PinFunction::af1}};
constexpr brio::UartOptions o{
    .fifo = true, .rx_threshold = static_cast<brio::UartFifoThreshold>(6)};
using Bad = brio::Uart<1, p, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, o>;
void f() { (void)Bad::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 9600); }
