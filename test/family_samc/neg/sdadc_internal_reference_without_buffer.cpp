// mcu: samc21e18a samc21g18a samc21j18a
// 39.8.2's Note: "The reference buffer should be enabled (ONREFBUF=1)
// when using the internal INTREF or DAC output as reference" - and for
// the DAC selection that bit IS erratum 1.8.10's whole workaround, live
// on every silicon revision.

#include "samc/sdadc.hpp"

using namespace brio;

constexpr SdadcConfig bad_cfg{
    .reference = SdadcRef::intref,
    .reference_buffer = false,
};

void use() { (void)Sdadc::init<bad_cfg>(0); }
