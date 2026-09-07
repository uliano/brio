// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// ES0548 2.10.1 has no register workaround: with a transmitter honouring
// the standard's own 50 ns of tSU;DAT, Fast-mode Plus needs a kernel
// clock of at least 20 MHz or the device samples the PREVIOUS SDA value.
// So a program that asks for a compile-time Fm+ timing at HSI16's
// 16 MHz gets nothing to unwrap - which is the refusal, and it beats
// DS13560 table 74's own 18 MHz floor.
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr auto t = *i2c_timing_for(16'000'000UL, I2cSpeed::fast_plus_1m);
static_assert(t.presc <= 15);
