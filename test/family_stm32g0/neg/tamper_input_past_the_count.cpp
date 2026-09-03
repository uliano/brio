// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The external inputs are 1-based and there are three on the G0B1 and
// TWO on the G071 and the G031 (their headers declare TAMP1E and
// TAMP2E and stop), so a fourth is refused everywhere and a third is
// refused where the part has not got one.
#include "stm32g0/rtc.hpp"
static_assert(brio::tamper_input_valid({.index = 4},
                                       brio::TamperFilter::edge));
