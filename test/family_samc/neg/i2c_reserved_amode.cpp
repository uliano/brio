// mcu: samc21j18a
// CTRLB.AMODE 0x3 is Reserved (33.8.2): a cast can put it in the enum,
// the configuration refuses it.
#include "samc/i2c.hpp"
using namespace brio;
constexpr I2cPads pads{.sda_pin = {'A', 8, PinFunction::c},
                       .scl_pin = {'A', 9, PinFunction::c}};
constexpr I2csConfig cfg = [] {
    I2csConfig c{};
    c.pads = pads;
    c.address_mode = static_cast<I2cAddressMode>(0x3);
    c.address = 0x2C;
    return c;
}();
bool f() { return I2cs<0>::configure<cfg>(); }
