// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.MATCHCLR is "valid only in Mode 0 (COUNT32) and Mode 2 (CLOCK)"
// (24.12.1): mode 1's counter wraps at PER and has nothing to clear on
// a match. Here the refusal is asked of the COMPILE-TIME configure<cfg>
// twin, which is what makes an impossible configuration a build error
// instead of a false return nobody reads.

#include "samc/rtc.hpp"

using namespace brio;

constexpr RtcConfig bad{.mode = RtcMode::count16, .match_clear = true};

void go() { (void)Rtc::configure<bad>(); }
