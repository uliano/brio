// A pad of a port this part does not bond is not a pad: the F410 has
// ports A, B, C and H and nothing else, so the datasheet's PF14/PF15 pair
// for this signal does not exist there.
// mcu: stm32f410rx stm32f410cx
#include "stm32f4/fmpi2c.hpp"
using namespace brio;
constexpr FmpI2cPins pins{.scl = {'F', 14, PinFunction::af4},
                          .sda = {'F', 15, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
