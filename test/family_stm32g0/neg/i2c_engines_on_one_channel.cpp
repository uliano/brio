// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Two engines on one DMA channel is one channel armed twice: the second
// arm overwrites the first's descriptor and the direction that lost is
// silently never served.
#include "stm32g0/dma.hpp"
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 9, PinFunction::af6},
};
using Host = I2cHost<1, pins, DmaTxEngine<1, 3>, DmaRxEngine<1, 3>>;
void use() { Host::release(); }
