// mcu: stm32g0b1xx stm32g071xx stm32g031xx stm32g030xx
// A pack needs at least its boot rate.
#include "stm32g0/clock.hpp"
using namespace brio;
using Dyn = DynamicClock<Rates<>>;
bool f() { return Dyn::init(); }
