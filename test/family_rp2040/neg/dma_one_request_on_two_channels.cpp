// Must FAIL: one request, one channel (the DREQ rule: "multiple channels should
// not be connected to the same DREQ", RP2040 2.5.3.2): UART1's transmit on channel 2,
// then on channel 6. The error names the owner and both slots.
#include "rp2040/dma.hpp"
#include "rp2040/uart.hpp"
constexpr brio::UartPins u1{.tx = {4, brio::PinFunction::uart}, .rx = {5, brio::PinFunction::uart}};
using Both = brio::Uart<1, u1, 64, 64, brio::DmaTxEngine<2>, brio::DmaRxEngine<3>>;
using TxElsewhere = brio::Uart<1, u1, 64, 64, brio::DmaTxEngine<6>, brio::NoDmaEngine>;
void f() { (void)sizeof(Both); (void)sizeof(TxElsewhere); }
