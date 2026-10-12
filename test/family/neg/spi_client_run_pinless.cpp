// mcu: avr128db48 avr128da32
// A pinless client has nothing to be clocked by: its init() answers
// false, and a run on it - which reads the select pad - is refused.
#include "avrdx/spi.hpp"
using namespace brio;
uint8_t rx[4];
void f() { (void)SpiClient<0, SpiRoute::none>::receive(std::span<uint8_t>(rx), 100u); }
