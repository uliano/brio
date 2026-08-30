// mcu: samc21e18a samc21g18a samc21j18a
// The timed sleep site's RTC rate must be at least 1024 Hz: slower, and
// one RTC count is coarser than the kernel tick itself, so the resync
// would quantize away the very spans it exists to restore. A 1 Hz
// calendar-style clock is a legal RTC arrangement and an illegal
// timebase witness - refused where the configuration is named.
#include "samc/platform_sam.hpp"
#include "samc/sleep.hpp"
using namespace brio;

constexpr TimedSleepConfig slow{.rtc_hz = 1};

void f() { (void)SamTimedSleepSite<SamPlatform, slow>::init(); }
