// A host drives SCK and MOSI; MISO is its INPUT, and OSPEEDR is an
// output's slew class (RM0390 7.4.3) - an input has none. So the host's
// Config names sck_speed and mosi_speed, and a class for the answer line
// is the CLIENT's field, not this one's.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'B', 13, PinFunction::af5}, .miso = {'B', 14, PinFunction::af5},
                       .mosi = {'B', 15, PinFunction::af5}, .nss = {'B', 12, PinFunction::af5}};
using Host = SpiHost<2, pins>;
constexpr Clock<ClockSource::hsi, 16'000'000> clk{};
void f() { (void)Host::init(clk, Host::Config{.miso_speed = PinSpeed::medium}); }
