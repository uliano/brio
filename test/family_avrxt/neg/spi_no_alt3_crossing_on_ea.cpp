// mcu: avr64ea48 avr64ea32
// SPI0 ALT3 (PA0, PA1, PC0, PC1) spans two ports: refused on the AVR EA
// until the pin hand-over drives a route per signal.
#include "avrxt/clock.hpp"
#include "avrxt/spi.hpp"
using namespace brio;
void f() { (void)SpiHost<0, SpiRoute::alt3>::init(Clock<ClockSource::internal, 20'000'000>{}); }
