// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// AN ERASE AT A RATE THAT WANTS HCLK HALVED. RM 32.1 says the flash
// access clock may not exceed 60 MHz and SCKMOD's reset value already
// halves the system clock, so above 120 MHz an erase or a program asks
// for the tree to be divided around it - which this driver will not do
// behind its caller's back. Under a STATIC clock the refusal is a
// compile error; under a DynamicClock it is a code.
#include "ch32vx03/nvm.hpp"

using Full = brio::Clock<brio::ClockSource::pll, 144'000'000>;

void f() { (void)brio::Flash::erase_page(Full{}, 0u); }
