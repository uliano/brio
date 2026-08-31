// mcu: samc21j18a
// An I2C address is seven bits; bit 7 set is not an address.
#include "samc/i2c.hpp"
using namespace brio;
constexpr I2cPads pads{.sda_pin = {'A', 8, PinFunction::c},
                       .scl_pin = {'A', 9, PinFunction::c}};
constexpr I2csConfig cfg = [] {
    I2csConfig c{};
    c.pads = pads;
    c.address = 0x93;
    return c;
}();
bool f() { return I2cs<0>::configure<cfg>(); }
