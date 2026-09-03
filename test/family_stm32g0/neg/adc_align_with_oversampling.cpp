// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 15.8.1: "the alignment mode is not available when working with
// oversampled data. The ALIGN bit is ignored". A driver that let a
// caller ask for it would be promising something the silicon drops.
#include "stm32g0/adc.hpp"
static_assert(brio::adc_config_valid({.left_aligned = true, .oversampling = true}));
