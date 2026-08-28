// mcu: samc21e18a samc21g18a samc21j18a
// EVCTRL.STARTINV without EVCTRL.STARTEI inverts an input the converter
// is not listening to - the same refusal samc/adc.hpp, samc/dac.hpp and
// samc/ac.hpp all make.

#include "samc/sdadc.hpp"

using namespace brio;

constexpr SdadcConfig bad_cfg{
    .events = {.invert_start = true},
};

void use() { (void)Sdadc::init<bad_cfg>(0); }
