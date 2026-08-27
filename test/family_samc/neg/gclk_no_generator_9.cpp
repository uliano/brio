// mcu: samc21e18a samc21g18a samc21j18a
// GCLK_GEN_NUM is 9: generators 0..8 exist, a ninth does not.
#include "samc/clock.hpp"
using namespace brio;
void f() { (void)Gclk<9>::enabled(); }
