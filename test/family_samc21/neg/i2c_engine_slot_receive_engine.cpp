// mcu: samc21e18a samc21g18a samc21j18a
// The I2C host's one engine slot takes a transmit engine: a receive engine
// named there fails to compile (samc21/i2c.hpp's static_assert), rather than
// failing deep inside the write's block start.
#include "samc21/dmac.hpp"
#include "samc21/i2c.hpp"
using namespace brio;

constexpr I2cPads pads{
    .sda_pin = {'A', 8, PinFunction::c},
    .scl_pin = {'A', 9, PinFunction::c},
};

void f() { (void)I2cHost<0, pads, 0, DmaRxEngine<0>>::status(); }
