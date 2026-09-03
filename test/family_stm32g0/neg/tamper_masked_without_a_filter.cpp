// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 31.3.4: the masked configuration "is available only when the tamper
// is configured in the Level detection with filtering ... mode". The
// edge detector has no sample train to mask.
#include "stm32g0/rtc.hpp"
static_assert(brio::tamper_input_valid({.index = 1, .masked = true},
                                       brio::TamperFilter::edge));
