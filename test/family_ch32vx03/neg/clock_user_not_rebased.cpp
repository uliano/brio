// mcu: ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A driver whose divisor comes from the clock must be among the users
// that clock rebases, or it would keep the old rate in silence. The
// port below is left out of the pack's user list on purpose.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using P = brio::Ch32vx03Platform<>;
using Serial = brio::Uart<1, P>;

using Top = brio::Clock<brio::ClockSource::pll, 96'000'000>;
using Boot = brio::Clock<brio::ClockSource::internal, 8'000'000>;

struct Other {
    static void rebase(uint32_t) {}
};

using SysClock = brio::DynamicClock<brio::Rates<Top, Boot>, Other>;
constexpr SysClock clock;

void f() { (void)Serial::init(clock, 115200); }
