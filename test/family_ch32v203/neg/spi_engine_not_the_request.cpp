// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE CHANNEL IS THE REQUEST (RM 11.2.3, table 11-5): SPI1 transmits on
// DMA channel 3 and on no other, so a transmit engine sitting on channel
// 4 - which is SPI2's receive, or USART1's transmit - would be armed for
// a request that never comes.
#include "ch32v203/dma.hpp"
#include "ch32v203/spi.hpp"

using Wrong = brio::SpiHost<1, brio::spi_default_pins<1>, brio::DmaTxEngine<4>,
                            brio::DmaRxEngine<2>>;
void f() { (void)Wrong::status(); }
