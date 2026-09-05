// mcu: samc21e18a samc21g18a samc21j18a
// EVCTRL.STARTINV without EVCTRL.STARTEI inverts an input the converter
// is not listening to - the same refusal samc21/adc.hpp, samc21/dac.hpp and
// samc21/ac.hpp all make.

#include "samc21/sdadc.hpp"

using namespace brio;

constexpr SdadcConfig bad_cfg{
    .events = {.invert_start = true},
};

void use() { (void)Sdadc::init<bad_cfg>(0); }
