// test_stm32f4_rubato_fmpi2c - the bench suite of the STM32F4 stratum's
// second I2C design under Rubato: util/i2c_bus.hpp's arbiter over
// stm32f4/fmpi2c.hpp's FmpI2cHost on FMPI2C1 (PC6/PC7) and its DMA engines,
// against the desk's Nucleo-G0B1RE running its unchanged twi_peer, at 100,
// 400 and 1000 kHz. The same source as test_stm32f4_rubato_i2c, included
// whole: the macro below selects the host, its pads, its engines and its
// four vectors, and that file's header comment is the account of the host,
// the pressure, the peer, the wires and the letters.
//
// build: boards = f446re
// build: monitor_speed = 115200

#define BRIO_SUITE_FMPI2C 1
#include "test_stm32f4_rubato_i2c.cpp"
