// A client drives ONE pad, MISO: SCK, MOSI and NSS are its inputs, and
// OSPEEDR is an OUTPUT's slew class (RM0390 7.4.3) - an input has none. So
// the client's Config names miso_speed alone, and a class for its clock
// pad is not a field there.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'B', 13, PinFunction::af5}, .miso = {'B', 14, PinFunction::af5},
                       .mosi = {'B', 15, PinFunction::af5}, .nss = {'B', 12, PinFunction::af5}};
using Peer = SpiClient<2, pins>;
constexpr Clock<ClockSource::hsi, 16'000'000> clk{};
void f() { (void)Peer::init(clk, Peer::Config{.sck_speed = PinSpeed::medium}); }
