// mcu: ch32v303rc ch32v303vc
// SPI3'S ENGINES ON DMA1. Table 11-3 wires SPI/I2S3's two requests to
// DMA2's channels 1 and 2, and DMA1's first two channels are ADC1's and
// SPI1's receiver's - an engine there would move nothing for SPI3. The
// host compares the whole slot, controller and channel.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/spi.hpp"

using Wrong = brio::SpiHost<3, brio::spi3_default_pins, brio::DmaTxEngine<1, 2>,
                            brio::DmaRxEngine<1, 1>>;
void f() { (void)Wrong::status(); }
