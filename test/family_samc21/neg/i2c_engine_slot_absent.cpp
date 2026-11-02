// mcu: samc21e18a samc21g18a samc21j18a
// The I2C host has one engine slot, the transmit engine's: a read runs on
// the interrupt, an I2C receive phase being no shape the image's two-channel
// budget was measured in (samc21/i2c.hpp, docs/samc21/dmac.md). A receive
// engine slot is absent, and naming one fails to compile.
#include "samc21/dmac.hpp"
#include "samc21/i2c.hpp"
using namespace brio;

constexpr I2cPads pads{
    .sda_pin = {'A', 8, PinFunction::c},
    .scl_pin = {'A', 9, PinFunction::c},
};

void f() { (void)I2cHost<0, pads, 0, DmaTxEngine<0>, DmaRxEngine<1>>::status(); }
