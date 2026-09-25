// mcu: ch32v303cb ch32v303rb
// A TRIGGER FROM A TIMER THE PART HAS NOT GOT. Table 17-1's code 001 is
// TIM8's TRGO, and the 128 KB CH32V303 has one advanced timer - TIM1 -
// and no TIM5..TIM10 (datasheet table 2-1-1), so the code selects a
// signal nothing drives and the constant configuration is refused.
#include "ch32vx03/dac.hpp"

void f() {
    brio::Dac::configure<1, brio::DacChannelConfig{.triggered = true,
                                                   .trigger = brio::DacTrigger::tim8_trgo}>();
}
