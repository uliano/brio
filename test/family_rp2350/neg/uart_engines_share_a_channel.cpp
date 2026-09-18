// Two engines of one transport must name DIFFERENT DMA channels: a
// transmit run and a receive run on one channel would each abort the
// other. On this chip a request is a FIELD any channel takes, so the
// channel is the identity and the pair below is the same channel twice.
#include "rp2350/dma.hpp"
#include "rp2350/uart.hpp"
constexpr brio::UartPins pins{.tx = {4, brio::PinFunction::uart},
                              .rx = {5, brio::PinFunction::uart}};
using Bad = brio::Uart<1, pins, 64, 64, brio::DmaTxEngine<3>, brio::DmaRxEngine<3>>;
void f() { (void)sizeof(Bad); }
