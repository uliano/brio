// mcu: samc21e18a samc21g18a samc21j18a
// CTRLB.REFSEL (41.8.2) implements three codes; 0x3 is Reserved.

#include "samc/dac.hpp"

using namespace brio;

constexpr DacConfig bad_cfg{
    .reference = static_cast<DacRef>(3),
};

void use() { (void)Dac::init<bad_cfg>(0); }
