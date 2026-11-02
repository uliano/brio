// mcu: stm32g0b1xx
// The host never chooses the engine: a Request names `interrupt` (the
// default) or `dma` (util/i2c_bus.hpp), and there is no third value that
// would leave the choice to the host.
#include "stm32g0/dma.hpp"
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 9, PinFunction::af6},
};
using Host = I2cHost<1, pins, DmaTxEngine<1, 1>, DmaRxEngine<1, 2>>;
bool use() {
    Host::Request r{};
    r.addr = 0x42;
    r.engine = I2cEngine::automatic;
    return Host::start(r);
}
