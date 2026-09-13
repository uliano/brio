// ... and a position past the pack's end is not a rate either.
// mcu: stm32f411xe stm32f429xx stm32f446xx
#include "stm32f4/clock.hpp"
struct User {
    static void rebase(uint32_t) {}
};
using Boot = brio::Clock<brio::ClockSource::hsi, 16'000'000>;
using Sys = brio::DynamicClock<brio::Rates<Boot>, User>;
void f() { (void)Sys::set_index<3>(); }
