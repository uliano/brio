// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Nineteen multiplexed channels (15.3.8), counted off the header's own
// CHSELx bits: a pad claiming channel 19 is claiming nothing.
#include "stm32g0/adc.hpp"
brio::AnalogIn<brio::Pin<'A', 4>, 19> in;
