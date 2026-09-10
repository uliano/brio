// mcu: ch32v006k8
// On this family the DMA channel IS the request (RM table 8-2): SPI1
// transmits on channel 3 and receives on channel 2. A host whose
// engines name any other pair must be REFUSED - it would move nothing.
#include "ch32v00x/dma.hpp"
#include "ch32v00x/spi.hpp"

using Bad = brio::SpiHost<1, brio::spi1_default_pins, brio::DmaTxEngine<4>, brio::DmaRxEngine<5>>;
void f() { Bad::release(); }
