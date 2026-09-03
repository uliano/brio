// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The calendar spans one century, so its leap rule is "divisible by
// four" and nothing else - 29 February 2025 is not a date.
#include "stm32g0/rtc.hpp"
static_assert(brio::rtc_datetime_valid({.day = 29, .month = 2, .year = 25}));
