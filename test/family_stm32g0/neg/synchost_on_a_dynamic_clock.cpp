// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The three USART personalities without a rebase() - SyncHost, IrdaLink,
// Smartcard - take a static clock only; listing one as a user is
// impossible (no rebase, the ClockUser concept) and initializing it
// with a dynamic clock is refused by clock_follows.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
using namespace brio;
constexpr UartPins u1{.tx = {'A', 9, PinFunction::af1}, .rx = {'A', 10, PinFunction::af1}};
constexpr PinRef ck_pad{'A', 8, PinFunction::af1};
using Host = SyncHost<1, u1, ck_pad>;
using Dyn = DynamicClock<Rates<Clock<ClockSource::pll, 64'000'000>>>;
bool f() { return Host::init(Dyn{}, 115200, SyncConfig{}); }
