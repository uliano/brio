// mcu: ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A 10-BIT ADDRESS HAS NO SHAPE IN THE HOST'S REQUEST: the tenure
// util/i2c_bus.hpp describes carries a 7-bit address on every stratum
// (docs/design/i2c-bus.md), so the field is a uint8_t and a 10-bit
// address cannot be spelled into it. The client half matches such an
// address (OADDR1.ADDMODE); the host's header sequence has no user.
#include "ch32v203/i2c.hpp"

using Host = brio::I2cHost<1>;
brio::I2cHost<1>::Request ten_bit{.addr = 0x123};
void f() { (void)Host::start(ten_bit); }
