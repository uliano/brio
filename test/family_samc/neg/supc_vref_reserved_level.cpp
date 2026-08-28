// VREF.SEL implements three codes out of sixteen (22.8.7); a Reserved
// one must be refused at compile time when the configuration is known
// then.
// mcu: samc21e18a samc21g18a samc21j18a
#include "samc/supc.hpp"

using namespace brio;

void bad() {
    (void)Vref::configure<VrefConfig{.level = static_cast<VrefLevel>(5)}>();
}
