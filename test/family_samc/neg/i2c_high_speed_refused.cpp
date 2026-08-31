// mcu: samc21j18a
// High-speed mode is a deliberate non-feature: errata 1.17.7 and
// 1.17.9 break its repeated starts both ways with no workaround. A
// cast cannot smuggle the code past the refusal.
#include "samc/i2c.hpp"
using namespace brio;
constexpr I2cPads pads{.sda_pin = {'A', 8, PinFunction::c},
                       .scl_pin = {'A', 9, PinFunction::c}};
constexpr I2cmConfig cfg = [] {
    I2cmConfig c{};
    c.pads = pads;
    c.baud = I2cBaud{100, 0};
    c.speed = static_cast<I2cSpeed>(3);
    return c;
}();
bool f() { return I2cm<0>::configure<cfg>(); }
