// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f7 ch32x033f8
// On every package but the QFN20 CH32X035F8, PC16 shares its pin with PC11
// and PC17 with PC10, and the datasheet forbids such a pad as an output
// (table 2-1, notes 4 to 7): I2C1_RM 010 is refused there.
#include "ch32x035/clock.hpp"
#include "ch32x035/i2c.hpp"

using Client = brio::I2cClient<1, brio::i2c1_pins_for(2)>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Client::init(clock, {.own = 0x2C});
}
