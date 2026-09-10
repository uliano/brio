// mcu: ch32v006k8
// The data phase is full duplex and its completion is the RECEIVE
// block's: a host naming one engine and not the other must be REFUSED.
#include "ch32v00x/dma.hpp"
#include "ch32v00x/spi.hpp"

using Bad = brio::SpiHost<1, brio::spi1_default_pins, brio::DmaTxEngine<3>>;
void f() { Bad::release(); }
