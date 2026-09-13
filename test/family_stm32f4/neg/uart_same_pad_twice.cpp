// TX and RX cannot be the same pad.
// mcu: stm32f429xx
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr UartPins pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 9, PinFunction::af7}};
using U = Uart<1, pins>;
void f() { (void)U::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600); }
