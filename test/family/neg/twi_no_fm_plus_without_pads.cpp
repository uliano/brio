// mcu: avr128db48 avr128da48
// Fast-mode Plus is a PAD setting as much as a divider one (29.3.3.1):
// a 1 MHz bus with FMPEN off would run on slew-limited drivers.
#include "avrdx/clock.hpp"
#include "avrdx/twi.hpp"
using namespace brio;
void f() {
    (void)Twi<0>::init<TwiConfig{.speed = TwiSpeed::fast_plus_1m}>(24'000'000u);
}
