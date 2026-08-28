// mcu: samc21e18a samc21g18a samc21j18a
// INSELy = EVENT points the input multiplexer at this LUT's event line,
// but LUTCTRLn.LUTEI is what lets an incoming event reach it (37.6.3).
// Selecting the source without enabling the line wires nothing.

#include "samc/ccl.hpp"

using namespace brio;

constexpr LutConfig bad_cfg{
    .in0 = LutInput::event,
    .truth = lut_truth_pass(0),
    .event_in = false,
};

void use() { (void)Lut<1>::configure<bad_cfg>(); }
