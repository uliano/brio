// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 4.2.2: "VPVDFx should always be set to a lower voltage level than
// VPVDRx". A falling threshold above the rising one is a detector that
// chatters, and it is refused rather than programmed.
#include "stm32g0/pwr.hpp"
static_assert(brio::pvd_config_valid({.rising = brio::PvdRising::v2_1,
                                      .falling = brio::PvdFalling::v2_9}));
