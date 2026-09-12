// mcu: ch32v006k8 ch32v003f4
// On this family the DMA channel IS the request (RM table 8-2): I2C1
// transmits on channel 6 and receives on channel 7. Any other pair must
// be REFUSED.
#include "ch32v00x/dma.hpp"
#include "ch32v00x/i2c.hpp"

using Bad = brio::I2cHost<1, brio::i2c1_default_pins, brio::DmaTxEngine<3>, brio::DmaRxEngine<2>>;
void f() { Bad::release(); }
