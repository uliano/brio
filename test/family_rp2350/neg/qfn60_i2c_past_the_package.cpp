// GP44/GP45 are I2C0's pads on the die and are bonded on the QFN-80
// alone: in a build that states the QFN-60, an I2C on them must not
// compile. The refusal is the pin chapter's own - the I2C table is the
// DIE's, and what the smaller package has not got is the bond wire.
#include "rp2350/clock.hpp"
#include "rp2350/i2c.hpp"

constexpr brio::I2cPins high_pins{.scl = 45, .sda = 44};
using Bad = brio::I2cHost<0, high_pins>;
using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
void f() { (void)Bad::init(SysClock{}); }
