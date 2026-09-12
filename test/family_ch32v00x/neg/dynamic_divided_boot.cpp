// mcu: ch32v006k8 ch32v003f4
// A DynamicClock's Boot must name the root's undivided rate (HPRE 1):
// a boot at 8 MHz (the reset divider) is refused.
#include "ch32v00x/clock.hpp"

struct Follower { static void rebase(uint32_t) {} };
using Dyn = brio::DynamicClock<brio::Clock<brio::ClockSource::internal, 8'000'000>, Follower>;
void f() { (void)Dyn::init(); }
