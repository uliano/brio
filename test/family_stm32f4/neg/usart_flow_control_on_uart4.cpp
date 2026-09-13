// UART4 has no CTS/RTS (RM0090 table 148): a Uart on it with flow
// control is refused where the part has the instance at all.
// mcu: stm32f429xx stm32f446xx
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr UartPins pins{.tx = {'A', 0, PinFunction::af8}, .rx = {'A', 1, PinFunction::af8}};
constexpr UartOptions flow{.rts = true, .rts_pin = {'A', 15, PinFunction::af8}};
using U = Uart<4, pins, 64, 256, NoDmaEngine, NoDmaEngine, flow>;
void f() { (void)U::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600); }
