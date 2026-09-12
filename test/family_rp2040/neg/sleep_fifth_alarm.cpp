// Must FAIL: the system timer has four alarms, 0..3.
#include "rp2040/sleep.hpp"

using P = brio::Rp2040Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
using Site = brio::Rp2040TimedSleepSite<P, SysClock, brio::DormantSource::xosc, 4>;
bool f() { return Site::init(); }
