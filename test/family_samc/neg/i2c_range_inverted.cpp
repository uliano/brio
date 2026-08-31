// mcu: samc21j18a
// AMODE range: ADDR is the UPPER limit (33.8.2); inverted bounds match
// nothing the chapter describes.
#include "samc/i2c.hpp"
using namespace brio;
constexpr I2cPads pads{.sda_pin = {'A', 8, PinFunction::c},
                       .scl_pin = {'A', 9, PinFunction::c}};
constexpr I2csConfig cfg = [] {
    I2csConfig c{};
    c.pads = pads;
    c.address_mode = I2cAddressMode::range;
    c.address = 0x10;
    c.second = 0x20;
    return c;
}();
bool f() { return I2cs<0>::configure<cfg>(); }
