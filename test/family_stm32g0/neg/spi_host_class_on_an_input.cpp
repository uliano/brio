// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A host drives SCK and MOSI; MISO is its INPUT, and OSPEEDR is an
// output's slew class (RM0444 7.5.3) - an input has none. So the host's
// Config names sck_speed and mosi_speed, and a class for the answer line
// is the CLIENT's field, not this one's.
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {'B', 5, brio::PinFunction::af0},
                             .nss = {'A', 15, brio::PinFunction::af0}};
using Host = brio::SpiHost<1, pins>;
constexpr brio::Clock<brio::ClockSource::pll, 64'000'000> clk{};
void use() { (void)Host::init(clk, Host::Config{.miso_speed = brio::PinSpeed::medium}); }
