// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 32.4.8: "OA2 is always a 7-bit address" - there is no OA2MODE and no
// ten-bit second address on this peripheral, so a value above 0x7F is a
// caller's mistake and not a mode.
#include "stm32g0/i2c.hpp"
using namespace brio;
static_assert(i2c_address_config_valid(
    I2cAddressConfig{.own = 0x10, .second = 0x100, .second_enable = true}));
