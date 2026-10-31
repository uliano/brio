// mcu: ch32v006k8
// A client that ANSWERS needs CPOL = 1 (RM 16.2.3): mode 0 is refused at
// compile time like mode 1 - its answers slip one bit, and so does what
// it hears, whenever the first answer bit is 1 and the host's MOSI stands
// low at the select's fall (measured). A listener in mode 0 is served.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/spi.hpp"

using Client = brio::SpiClient<1>;
constexpr brio::Clock<brio::ClockSource::pll, 48'000'000> clock;
bool f() { return Client::init<Client::Config{.mode = brio::SpiMode::mode0, .drive_output = true}>(clock); }
