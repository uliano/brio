// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// SCL and SDA on the same pad is a wiring mistake no datasheet table can
// catch, and the one thing about a pad list this header CAN check
// besides the pads' existence.
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr I2cPins bad{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 8, PinFunction::af6},
};
using Host = I2cHost<1, bad>;
void use() { Host::release(); }
