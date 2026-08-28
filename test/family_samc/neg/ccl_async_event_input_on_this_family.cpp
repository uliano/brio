// mcu: samc21e18a samc21g18a samc21j18a
// INSEL 0xB (ASYNCEVENT) - the event input with the CCL's own edge
// detector switched off, so a LEVEL can be combined with another source
// - is the second N-variant-only code of 37.8.3.

#include "samc/ccl.hpp"

using namespace brio;

constexpr LutConfig bad_cfg{
    .in2 = LutInput::async_event,
    .truth = lut_truth_pass(2),
    .event_in = true,
};

void use() { (void)Lut<3>::configure<bad_cfg>(); }
