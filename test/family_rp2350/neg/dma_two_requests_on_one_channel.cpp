// Must FAIL: one DMA channel has one owner for the whole image
// (docs/design/dma.md, "The ownership rule"): channel 2 is UART1's transmit, so
// SPI0's cannot have it too. The error names the slot and both owners.
#include "rp2350/dma.hpp"
#include "rp2350/spi.hpp"
#include "rp2350/uart.hpp"
constexpr brio::UartPins u1{.tx = {4, brio::PinFunction::uart}, .rx = {5, brio::PinFunction::uart}};
constexpr brio::SpiPins s0{.sck = 18, .tx = 19, .rx = 16};
using Serial = brio::Uart<1, u1, 64, 64, brio::DmaTxEngine<2>, brio::DmaRxEngine<3>>;
using Bus = brio::SpiHost<0, s0, brio::DmaTxEngine<2, uint16_t>, brio::DmaRxEngine<5, uint16_t>>;
void f() { (void)sizeof(Serial); (void)sizeof(Bus); }
