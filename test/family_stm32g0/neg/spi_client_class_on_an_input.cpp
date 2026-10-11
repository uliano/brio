// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A client drives ONE pad, MISO: SCK, MOSI and NSS are its inputs, and
// OSPEEDR is an OUTPUT's slew class (RM0444 7.5.3) - an input has none. So
// the client's Config names miso_speed alone, and a class for its clock
// pad is not a field there.
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {'B', 5, brio::PinFunction::af0},
                             .nss = {'A', 15, brio::PinFunction::af0}};
using Peer = brio::SpiClient<1, pins>;
constexpr brio::Clock<brio::ClockSource::pll, 64'000'000> clk{};
void use() { (void)Peer::init(clk, Peer::Config{.sck_speed = brio::PinSpeed::medium}); }
