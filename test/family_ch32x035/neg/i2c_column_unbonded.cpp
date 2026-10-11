// mcu: ch32x035f8 ch32x035f7 ch32x035g8u ch32x035g8r
// I2C1_RM 000 puts SCL on PA10 and SDA on PA11, which these packages do
// not bring out: the column is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/i2c.hpp"

using Host = brio::I2cHost<1, brio::i2c1_pins_for(0)>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
