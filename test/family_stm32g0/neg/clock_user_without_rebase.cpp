// mcu: stm32g0b1xx stm32g071xx stm32g031xx stm32g030xx
// The users of a DynamicClock are ClockUsers (util/clock.hpp): a type
// with no static rebase(uint32_t) does not compile where the list is
// written.
#include "stm32g0/clock.hpp"
using namespace brio;
struct Mute {};
using Dyn = DynamicClock<Rates<Clock<ClockSource::pll, 64'000'000>>, Mute>;
bool f() { return Dyn::init(); }
