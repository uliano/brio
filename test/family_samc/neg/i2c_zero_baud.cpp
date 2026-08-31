// mcu: samc21j18a
// The chapter's own note: BAUD and/or BAUDLOW must be nonzero. The
// reset value is not a rate.
#include "samc/i2c.hpp"
using namespace brio;
constexpr I2cPads pads{.sda_pin = {'A', 8, PinFunction::c},
                       .scl_pin = {'A', 9, PinFunction::c}};
constexpr I2cmConfig cfg = [] {
    I2cmConfig c{};
    c.pads = pads;
    return c;
}();
bool f() { return I2cm<0>::configure<cfg>(); }
