// Two signals of one bus cannot share a pad: the second claim would take
// the pad's alternate function away from the first, silently.
// mcu: stm32f446xx stm32f413xx
#include "stm32f4/fmpi2c.hpp"
using namespace brio;
constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4},
                          .sda = {'C', 7, PinFunction::af4},
                          .smba = {'C', 7, PinFunction::af4}};
using Peer = FmpI2cClient<1, pins>;
void f() { (void)Peer::init(Clock<ClockSource::hsi, 16'000'000>{}, FmpI2cAddressConfig{}); }
