// mcu: samc21e18a samc21g18a samc21j18a
// EVCTRL.INVEI chooses the edge of the START event input; inverting an
// input whose enable is clear asks the silicon for nothing.

#include "samc/dac.hpp"

using namespace brio;

constexpr DacConfig bad_cfg{
    .events = {.invert_start = true},
};

void use() { (void)Dac::init<bad_cfg>(0); }
