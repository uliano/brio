// With a receive engine the ring is the circular stream's whole storage,
// and SxNDTR counts 65535 items at most: a power-of-two ring of 65536
// bytes cannot be one lap, so the Uart refuses it before the board is
// powered rather than binding a stream to a count it cannot hold.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr UartPins pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
using U = Uart<1, pins, 65536, 256, NoDmaEngine, DmaRxEngine<2, 2, 4>>;
void f() { (void)U::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600); }
