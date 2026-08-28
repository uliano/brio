// mcu: samc21e18a samc21g18a samc21j18a
// CTRLB.OSR (39.8.3) implements five ratios; 0x5..0x7 are Reserved (the
// register description prints the range as "0x4 - 0xF", which is both a
// typo and impossible in a three-bit field).

#include "samc/sdadc.hpp"

using namespace brio;

constexpr SdadcConfig bad_cfg{
    .osr = static_cast<SdadcOsr>(5),
};

void use() { (void)Sdadc::init<bad_cfg>(0); }
