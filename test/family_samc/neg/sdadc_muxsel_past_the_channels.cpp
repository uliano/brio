// mcu: samc21e18a samc21g18a samc21j18a
// INPUTCTRL.MUXSEL (39.8.9) names three differential pairs; 0x3..0xF are
// Reserved on every package.

#include "samc/sdadc.hpp"

using namespace brio;

void use() { (void)Sdadc::select<3>(); }
