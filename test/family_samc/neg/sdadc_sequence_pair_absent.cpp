// mcu: samc21e18a
// SEQCTRL.SEQEN has one bit per differential pair (39.8.20); a bit for a
// pair this package does not bond asks the sequencer to convert pads
// that are not there.

#include "samc/sdadc.hpp"

using namespace brio;

constexpr SdadcConfig bad_cfg{
    .sequence = 0x3,
};

void use() { (void)Sdadc::init<bad_cfg>(0); }
