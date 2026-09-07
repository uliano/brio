// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 32.9.1's own note on CR1.WUPEN: "WUPEN can be set only when
// DNF[3:0] = 0000" - a hardware interlock, not a caution (table 168 puts
// it the other way round: "wake-up from Stop mode on address match not
// supported when the digital filter is enabled"). Refused at the
// configuration so a caller learns it at the verb and not from a bit
// that would not stick.
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr brio::I2cTiming t{1, 39, 15, 7, 13};
static_assert(i2c_config_valid(
    I2cConfig{.timing = t, .filters = {true, 1}, .wake_from_stop = true}));
