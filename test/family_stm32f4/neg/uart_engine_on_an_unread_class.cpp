// No device header of this pack carries a request mapping, so the
// reserve keys the serial slice on the part class - and a class whose
// reference manual was not read has no table. An engine there is
// REFUSED, never run on the neighbouring class's channel: this is the
// same cell that is correct on the F429, F446, F411 and F469.
// mcu: stm32f412zx stm32f401xe
#include "stm32f4/dma.hpp"
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr UartPins pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
using U = Uart<1, pins, 64, 256, DmaTxEngine<2, 7, 4>>;
void f() { (void)U::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600); }
