// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A dynamic clock runs at the rates its pack names and at no others: a
// set<hz>() for a rate that is not in the pack is a compile error, not a
// run-time false, because the rate is written in the source.
#include "ch32vx03/clock.hpp"

using Top = brio::Clock<brio::ClockSource::pll, 96'000'000>;
using Boot = brio::Clock<brio::ClockSource::internal, 8'000'000>;

struct User {
    static void rebase(uint32_t) {}
};

using SysClock = brio::DynamicClock<brio::Rates<Top, Boot>, User>;

void f() { (void)SysClock::set<48'000'000>(); }
