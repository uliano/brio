// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 32.4.8's own caution: "The target byte control mode is not compatible
// with NOSTRETCH mode. Setting SBC when NOSTRETCH = 1 is not allowed."
// Byte control works by STRETCHING between the eighth and the ninth SCL
// pulse, which is precisely what NOSTRETCH forbids.
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr brio::I2cTiming t{1, 39, 15, 7, 13};
static_assert(i2c_config_valid(
    I2cConfig{.timing = t, .no_stretch = true, .byte_control = true}));
