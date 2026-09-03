// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 31.6.2, per bit and in those words: "The tamper x interrupt must not
// be enabled when TAMPxMSK is set." A masked flag is cleared by
// hardware, so an interrupt over it would be a vector with nothing to
// read.
#include "stm32g0/rtc.hpp"
static_assert(brio::tamper_input_valid(
    {.index = 1, .masked = true, .interrupt = true},
    brio::TamperFilter::samples2));
