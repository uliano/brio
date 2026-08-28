// mcu: samc21e18a samc21g18a samc21j18a
// INSEL 0xA (ALT2TC) is "only available on SAM C20/C21 N variants"
// (37.8.3), and no header in this pack declares the enumerator. The
// vocabulary carries the code so the option space is the chapter's
// whole one; the device decides whether it may be asked for.

#include "samc/ccl.hpp"

using namespace brio;

constexpr LutConfig bad_cfg{
    .in0 = LutInput::alt2_tc,
    .truth = lut_truth_pass(0),
};

void use() { (void)Lut<0>::configure<bad_cfg>(); }
