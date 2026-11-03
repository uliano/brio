// Must FAIL: a copy owns its channel through the copy engine's tag, so a copy on
// the channel UART1's transmit owns does not compile.
#include "rp2350/dma.hpp"
#include "rp2350/uart.hpp"
constexpr brio::UartPins u1{.tx = {4, brio::PinFunction::uart}, .rx = {5, brio::PinFunction::uart}};
using Serial = brio::Uart<1, u1, 64, 64, brio::DmaTxEngine<2>, brio::DmaRxEngine<3>>;
using Copier = brio::DmaCopyEngine<2>;
void f() { (void)sizeof(Serial); (void)sizeof(Copier); }
