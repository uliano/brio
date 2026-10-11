// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// A write-then-read needs both directions inside one tenure: an I2C host
// with a transmit engine and no receive engine is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/i2c.hpp"

using Host = brio::I2cHost<1, brio::i2c1_default_pins, brio::DmaTxEngine<6>>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
