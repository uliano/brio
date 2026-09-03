// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 14.3.3: "even polynomials are not supported" - the constant term must
// be there, and no register refuses one without it.
#include "stm32g0/crc.hpp"
constexpr brio::CrcConfig cfg{.polynomial = 0x1020u, .size = brio::CrcWidth::w16};
static_assert(brio::crc_config_valid(cfg), "must be refused");
