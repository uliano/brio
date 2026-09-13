// The DMA engine slots take NoDmaEngine alone until stm32f4/dma.hpp
// exists: a slot that names a present engine is refused.
// mcu: stm32f429xx
#include "stm32f4/usart.hpp"
using namespace brio;
struct FakeEngine { static constexpr bool present = true; };
constexpr UartPins pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
using U = Uart<1, pins, 64, 256, FakeEngine>;
void f() { (void)U::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600); }
