// mcu: ch32v006k8
// The CH32V006's client is served in modes 0 and 3 alone: mode 2 is
// refused at compile time like mode 1, a listener too (it hears every
// frame one bit early).
#include "ch32v00x/clock.hpp"
#include "ch32v00x/spi.hpp"

using Client = brio::SpiClient<1>;
constexpr brio::Clock<brio::ClockSource::pll, 48'000'000> clock;
bool f() { return Client::init<Client::Config{.mode = brio::SpiMode::mode2, .drive_output = false}>(clock); }
