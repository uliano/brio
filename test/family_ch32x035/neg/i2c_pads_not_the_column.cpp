// mcu: ch32x035f8
// A column is the code's pads in the code's order: SCL PC16 and SDA PC17
// named with code 4 - whose SCL is PC17 - are not a bus.
#include "ch32x035/clock.hpp"
#include "ch32x035/i2c.hpp"

using Host = brio::I2cHost<1, brio::I2cPins{.scl = {'C', 16}, .sda = {'C', 17}, .remap = 4}>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
