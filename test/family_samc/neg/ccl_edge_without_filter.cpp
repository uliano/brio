// mcu: samc21e18a samc21g18a samc21j18a
// 37.6.2.6: "In order to avoid unpredictable behavior, either the filter
// or synchronizer must be enabled" with the edge detector. A LUT asking
// for EDGESEL with FILTSEL disabled is asking for that behavior.

#include "samc/ccl.hpp"

using namespace brio;

constexpr LutConfig bad_cfg{
    .truth = lut_truth_pass(0),
    .filter = LutFilter::none,
    .edge_detect = true,
};

void use() { (void)Lut<0>::configure<bad_cfg>(); }
