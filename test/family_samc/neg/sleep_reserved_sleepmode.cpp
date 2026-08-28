// mcu: samc21j18a
// SLEEPCFG.SLEEPMODE has THREE implemented codes (19.8.1): IDLE0 0x0,
// IDLE2 0x2 and STANDBY 0x4. There is no IDLE1, and 0x1, 0x3 and
// 0x5..0x7 are Reserved. A caller that names one of them through the
// compile-time twin must not compile - the run-time twin's `false` is
// for a value that is not a constant.

#include "samc/sleep.hpp"

using namespace brio;

bool arm_a_reserved_mode() {
    return Pm::set_sleep_mode<static_cast<SleepMode>(1)>();
}
