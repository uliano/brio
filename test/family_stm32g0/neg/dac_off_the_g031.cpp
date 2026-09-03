// mcu: stm32g031xx
// 16.3's own table: there is no DAC on the STM32G031xx/G041xx, and the
// header declares no DAC1_BASE - so the whole driver is a refusal there
// rather than a block writing into a hole in the address map.
#include "stm32g0/dac.hpp"
void f() { (void)brio::Dac::write(0, 0); }
