// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A SpiHost initialized with a DynamicClock that does not list it among
// its Users would keep an SCK ceiling - and a cs_setup timing - resolved
// against a rate that is gone.
#include "stm32g0/clock.hpp"
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {'B', 5, brio::PinFunction::af0},
                             .nss = {}};
using Bus = brio::SpiHost<1, pins>;
using Rates = brio::Rates<brio::Clock<brio::ClockSource::pll, 64'000'000>,
                          brio::Clock<brio::ClockSource::internal, 16'000'000>>;
using Dyn = brio::DynamicClock<Rates>;
void use() {
    constexpr Dyn clock;
    (void)Bus::init(clock);
}
