// Must FAIL: whoever programs a channel owns it. A program's tag for a channel it
// programs by hand, on the channel UART1's receive owns, does not compile.
#include "rp2350/dma.hpp"
#include "rp2350/uart.hpp"
constexpr brio::UartPins u1{.tx = {4, brio::PinFunction::uart}, .rx = {5, brio::PinFunction::uart}};
using Serial = brio::Uart<1, u1, 64, 64, brio::DmaTxEngine<2>, brio::DmaRxEngine<3>>;
struct ByHand {};
static_assert(brio::dma_claim<brio::DmaSlotId<0, 3>, ByHand>);
void f() { (void)sizeof(Serial); }
