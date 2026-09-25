// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb
// I2S ON A PART WITHOUT IT. Chapter 20 describes the audio face for four
// families at once, and the register answers on every SPI2; what decides
// is the datasheet - two I2S on the CH32V303RC and VC (table 2-1-1), none
// on the 128 KB CH32V303 and no I2S row at all in the CH32V203's table
// 2-1. Refused where the part has none.
#include "ch32vx03/spi.hpp"

using Audio = brio::I2s<2>;
void f() { Audio::enable(); }
