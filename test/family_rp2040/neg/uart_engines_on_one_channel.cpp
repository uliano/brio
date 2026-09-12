// Must FAIL: a transport's two engines cannot share one DMA channel.
#include "rp2040/dma.hpp"
#include "rp2040/uart.hpp"

constexpr brio::UartPins u1{.tx = {4, brio::PinFunction::uart}, .rx = {5, brio::PinFunction::uart}};
using Both = brio::Uart<1, u1, 64, 64, brio::DmaTxEngine<2>, brio::DmaRxEngine<2>>;
void up() { (void)Both::tx_idle(); }
