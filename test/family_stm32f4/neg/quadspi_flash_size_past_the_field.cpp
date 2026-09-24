// FSIZE is five bits: a flash of more than 32 address bits, a FIFO
// threshold past the 32-byte FIFO or a chip-select high time past eight
// cycles is refused at compile time by init<cfg>().
// mcu: stm32f469xx stm32f446xx
#include "stm32f4/quadspi.hpp"
constexpr brio::QspiConfig cfg{.address_bits = 33};
void f() { (void)brio::Quadspi::init<cfg>(); }
