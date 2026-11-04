// test_rp2040_i2c_ram - test_rp2040_i2c.cpp with the engined host and the two vectors it
// binds in SRAM: the same letters over `InRam` (util/placement.hpp), so that a vector's way
// in does not go through the flash a load of the DMA is reading. Letter u's question on this
// chip is whether a channel can then be caught behind the STOP; every other letter reads as
// the flash image's.
//
// build: boards = weact2040
// build: monitor_speed = 115200

#define I2C_SUITE_IN_RAM 1
#include "test_rp2040_i2c.cpp"
