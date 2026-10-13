// mcu: stm32g0b1xx stm32g031xx
// The I2C host's event pump takes no placement on this family: as an
// InRam instance behind a vector in the flash it was measured no shorter
// (stm32g0/i2c.hpp, docs/stm32g0/i2c.md), so there is nothing to place,
// and a program written for a family whose host does take one is refused
// here rather than handed a tag that would do nothing. The program's way
// is its vector in `.fast_text`.
#include "stm32g0/i2c.hpp"
#include "util/placement.hpp"
constexpr brio::I2cPins pins{
    .scl = {'B', 8, brio::PinFunction::af6},
    .sda = {'B', 9, brio::PinFunction::af6},
};
using H = brio::I2cHost<1, pins, brio::NoDmaEngine, brio::NoDmaEngine, brio::InRam>;
void f();
void f() { (void)H::isr(); }
