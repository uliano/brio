// mcu: ch32v006k8
// The CH32V006's client is served in modes 0 and 3 alone: in modes 1 and
// 2 it hears every frame one bit early and sends a fixed pattern
// (measured). A client configured in one of them at compile time must be
// REFUSED.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/spi.hpp"

using Client = brio::SpiClient<1>;
constexpr brio::Clock<brio::ClockSource::pll, 48'000'000> clock;
bool f() { return Client::init<Client::Config{.mode = brio::SpiMode::mode1}>(clock); }
