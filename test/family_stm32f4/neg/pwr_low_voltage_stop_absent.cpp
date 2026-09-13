// The F405/F407/F415/F417 class has no MRLVDS/LPLVDS pair: a Stop asked
// for in the low-voltage mode there is refused, not silently plain.
// mcu: stm32f405xx stm32f407xx stm32f415xx stm32f417xx
#include "stm32f4/sleep.hpp"
constexpr brio::SleepSiteConfig cfg{
    .standby = brio::StopConfig{},
    .deep = brio::StopConfig{brio::StopRegulator::low_power, true, true, false},
};
using Boot = brio::Clock<brio::ClockSource::hsi, 16'000'000>;
using Site = brio::Stm32f4SleepSite<Boot, brio::Ticker, cfg>;
void f() { (void)Site::arm(brio::SleepDepth::deep); }
