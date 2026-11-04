// test_rp2350_i2c_ram - test_rp2350_i2c.cpp with the engined host and the two vectors it
// binds in SRAM: the same letters over `InRam` (util/placement.hpp), which proves the STOP
// entry's cold path from a host placed there, its call out to the flash included.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#define I2C_SUITE_IN_RAM 1
#include "test_rp2350_i2c.cpp"
