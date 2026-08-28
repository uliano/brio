// mcu: samc21e18a samc21g18a samc21j18a
// LUTCTRLn.INVEI inverts the INCOMING event, so it means nothing with
// LUTEI clear - the same refusal ac.hpp's ac_event_control_valid()
// makes for the comparators' own COMPEI/INVEI pair.

#include "samc/ccl.hpp"

using namespace brio;

constexpr LutConfig bad_cfg{
    .truth = lut_truth_pass(0),
    .event_in = false,
    .invert_event_in = true,
};

void use() { (void)Lut<2>::configure<bad_cfg>(); }
