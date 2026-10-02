// mcu: ch32v006k8 ch32v003f4
// SPI1's DATAR is sixteen bits: engines whose widest beat is a word
// would store into the reserved half above it, and must be REFUSED.
#include "ch32v00x/dma.hpp"
#include "ch32v00x/spi.hpp"

using Bad = brio::SpiHost<1, brio::spi1_default_pins, brio::DmaTxEngine<3, uint32_t>,
                          brio::DmaRxEngine<2, uint32_t>>;
void f() { Bad::release(); }
