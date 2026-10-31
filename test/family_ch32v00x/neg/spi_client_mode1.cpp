// mcu: ch32v006k8
// A client that ANSWERS needs CPOL = 1 (RM 16.2.3): in modes 0 and 1 a
// window whose first answer bit is 1 can slip one bit at both ends
// (measured). An answering client in mode 1 configured at compile time
// must be REFUSED.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/spi.hpp"

using Client = brio::SpiClient<1>;
constexpr brio::Clock<brio::ClockSource::pll, 48'000'000> clock;
bool f() { return Client::init<Client::Config{.mode = brio::SpiMode::mode1}>(clock); }
