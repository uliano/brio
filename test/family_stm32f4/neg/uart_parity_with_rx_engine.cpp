// PARITY IS THE INTERRUPT RECEIVER'S. UartOptions::parity makes the vector
// judge each received word's parity as well as SR.PE (which a write of DR
// can clear while the port transmits); under a receive engine the stream
// moves DR a byte at a time and an eight-bit frame's parity bit dies in the
// transfer, so the option on such a port must be REFUSED rather than
// carried as a promise nothing keeps.
// mcu: stm32f429xx stm32f446xx stm32f411xe stm32f469xx
#include "stm32f4/dma.hpp"
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr UartPins pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr UartOptions parity_port{.parity = true};
using U = Uart<1, pins, 64, 256, NoDmaEngine, DmaRxEngine<2, 2, 4>, parity_port>;
void f() { (void)U::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600); }
