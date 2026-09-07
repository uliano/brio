// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A 10-bit address is ten bits: 0x400 is not one, and OAR1's OA1[9:0]
// would silently keep the low ten of whatever it was given.
#include "stm32g0/i2c.hpp"
using namespace brio;
static_assert(i2c_address_config_valid(
    I2cAddressConfig{.own = 0x400, .mode = I2cAddressMode::ten_bit}));
