// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// OVSS reaches an 8-bit shift and no further (15.8's table 79), so a
// ninth is not a slower answer, it is a different register.
#include "stm32g0/adc.hpp"
static_assert(brio::adc_config_valid({.oversampling = true, .oversampling_shift = 9}));
