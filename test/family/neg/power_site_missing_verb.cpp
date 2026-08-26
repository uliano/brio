// mcu: avr128db28 avr128db48 avr128da28 avr128da48
// A sleep site without armed(): the SleepSite concept is what a target
// must satisfy to carry a power manager, and a site that cannot say what
// is armed cannot support the first-event-after-wake contract. The
// manager must refuse it at the template boundary, not fail somewhere
// inside its body.
#include "avrdx/platform_avr.hpp"
#include "util/power.hpp"

using namespace brio;

struct HalfSite {
    static bool arm(SleepDepth) { return true; }
    static void disarm() {}
    // no armed()
};

using Broken = PowerManager<AvrPlatform, HalfSite>;

void f() { Broken::init(); }
