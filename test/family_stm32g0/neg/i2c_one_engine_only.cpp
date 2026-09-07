// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A write-then-read tenure moves bytes in BOTH directions inside one
// bus tenure, so a host with only one engine would have to hand over to
// the byte pump mid-tenure. Name both or neither.
#include "stm32g0/dma.hpp"
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 9, PinFunction::af6},
};
using Host = I2cHost<1, pins, DmaTxEngine<1, 1>>;
void use() { Host::release(); }
