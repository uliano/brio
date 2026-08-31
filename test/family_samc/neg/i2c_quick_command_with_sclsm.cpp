// mcu: samc21j18a
// Erratum 1.17.13: quick command with SCLSM = 1 raises a bus error on
// every repeated start - the pair is refused at compile time.
#include "samc/i2c.hpp"
using namespace brio;
constexpr I2cPads pads{.sda_pin = {'A', 8, PinFunction::c},
                       .scl_pin = {'A', 9, PinFunction::c}};
constexpr I2cmConfig cfg = [] {
    I2cmConfig c{};
    c.pads = pads;
    c.baud = I2cBaud{100, 0};
    c.quick_command = true;
    c.scl_stretch_after_ack = true;
    return c;
}();
bool f() { return I2cm<0>::configure<cfg>(); }
