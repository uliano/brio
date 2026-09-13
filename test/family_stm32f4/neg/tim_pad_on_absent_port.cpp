// A timer pad names a port letter, and a port this device does not bond
// is a claim that must not compile.
// mcu: stm32f411xe stm32f429xx stm32f446xx stm32f410cx
#include "stm32f4/tim.hpp"
constexpr brio::PinSel nowhere{'Z', 3, brio::PinFunction::af2};
void f() { brio::TimPad<nowhere>::claim(); }
