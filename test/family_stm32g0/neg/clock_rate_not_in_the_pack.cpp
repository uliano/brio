// mcu: stm32g0b1xx stm32g071xx stm32g031xx stm32g030xx
// A DynamicClock runs only at the rates its pack names: set<hz>() with
// a rate outside the pack is a compile error, not a false.
#include "stm32g0/clock.hpp"
using namespace brio;
using Dyn = DynamicClock<Rates<Clock<ClockSource::pll, 64'000'000>,
                               Clock<ClockSource::internal, 16'000'000>>>;
bool f() { return Dyn::set<8'000'000>(); }
