// Must FAIL: the year field is twelve bits (table 551), 4096 is no date.
#include "rp2040/rtc.hpp"

static_assert(brio::rtc_datetime_valid(brio::RtcDateTime{.year = 4096}));
