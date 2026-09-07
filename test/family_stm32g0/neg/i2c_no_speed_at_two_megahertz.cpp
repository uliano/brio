// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The same erratum's Standard-mode floor is 4 MHz, so a core scaled to
// 2 MHz cannot run even a 100 kHz bus ON PCLK. This is the refusal that
// makes the INDEPENDENT CLOCK worth having: the same program with the
// instance's kernel on HSI16 compiles and runs.
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr auto t = *i2c_timing_for(2'000'000UL, I2cSpeed::standard_100k);
static_assert(t.presc <= 15);
