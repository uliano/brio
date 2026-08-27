// mcu: samc21e18a samc21g18a samc21j18a
// Only ClockSource::internal is implemented; the declared-but-unbuilt
// sources must be a compile error, never a silently wrong clock.
#include "samc/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::crystal, 24'000'000>::init(); }
