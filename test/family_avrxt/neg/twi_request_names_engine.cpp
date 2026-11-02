// mcu: avr128db48 avr128da48 avr64ea48
// The AVR's I2C Request carries no `engine` field: no DMA on this family,
// and the field would make every Request a byte longer (avrxt/twi.hpp,
// has_dma). A Request that names it fails to compile; a portable program
// asks I2cHost::has_dma first.
#include "avrxt/twi.hpp"
using namespace brio;
I2cHost<0>::Request r{.addr = 0x50, .engine = I2cEngine::dma};
