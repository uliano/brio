// mcu: samc21e18a samc21g18a samc21j18a
// 39.6.2.3: "The first valid sample starts from the third sample onward."
// CTRLB.SKPCNT's reset value is 2 for that reason, and erratum 1.18.3's
// workaround spells it out - "Write CTRLB.SKPCNT to 2 before running
// single conversions". A single conversion that skips fewer than two
// decimation windows returns the filter still filling.

#include "samc/sdadc.hpp"

using namespace brio;

constexpr SdadcConfig bad_cfg{
    .skip_count = 0,
};

void use() { (void)Sdadc::init<bad_cfg>(0); }
