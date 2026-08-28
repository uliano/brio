// mcu: samc21j18a
// 38.6.3.2: "Rail-to-rail operation requires a sampling period of four
// cycles. This is achieved by enabling offset compensation
// (SAMPCTRL.OFFCOMP = 1). Rail-to-rail operation should not be used when
// offset compensation is disabled."

#include "samc/adc.hpp"

using namespace brio;

constexpr AdcConfig bad_cfg{
    .rail_to_rail = true,
    .offset_compensation = false,
};

void use() { (void)Adc<0>::init<bad_cfg>(0); }
