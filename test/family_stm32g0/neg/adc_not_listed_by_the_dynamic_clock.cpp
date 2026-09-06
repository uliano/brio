// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The ADC derives its clock division from the clock it is given and
// follows a change through rebase() - but only when the dynamic clock
// LISTS it among the users it rebases: one that does not is refused at
// compile time rather than read once and left stale (util/clock.hpp's
// clock_follows, docs/design/clock.md).
#include "stm32g0/adc.hpp"
#include "stm32g0/clock.hpp"
using namespace brio;
struct Other { static void rebase(uint32_t) {} };
using Dyn = DynamicClock<Rates<Clock<ClockSource::pll, 64'000'000>>, Other>;
bool f() { return Adc::init(Dyn{}, AdcConfig{}); }
