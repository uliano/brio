// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// RES is two bits with four implemented codes (15.4.2); anything else
// has no full scale and no tSAR, so a config carrying one must be
// refused.
#include "stm32g0/adc.hpp"
static_assert(brio::adc_config_valid({.resolution = static_cast<brio::AdcRes>(5)}));
