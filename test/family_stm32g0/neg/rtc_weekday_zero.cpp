// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// RTC_DR's WDU field spells 001..111 for Monday..Sunday and calls 000
// "forbidden" (30.6.2) - so a default-constructed weekday is a bug the
// predicate has to catch.
#include "stm32g0/rtc.hpp"
static_assert(brio::rtc_datetime_valid({.weekday = 0}));
