// RM0390 table 17: the under-drive rows all carry the low-voltage bit -
// under-drive MODIFIES that mode and is not a mode of its own.
// mcu: stm32f429xx stm32f446xx
#include "stm32f4/sleep.hpp"
constexpr brio::SleepSiteConfig cfg{
    .standby = brio::StopConfig{},
    .deep = brio::StopConfig{brio::StopRegulator::low_power, true, false, true},
};
using Boot = brio::Clock<brio::ClockSource::hsi, 16'000'000>;
using Site = brio::Stm32f4SleepSite<Boot, brio::Ticker, cfg>;
void f() { (void)Site::arm(brio::SleepDepth::deep); }
