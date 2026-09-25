// mcu: ch32v303cb ch32v303rb
// A TAIL ADDRESS ON A PART THAT HAS NO TAIL. The CH32V303's datasheet
// gives the non-zero-wait area to the 256K FLASH + 64K SRAM products
// alone (its note 1 to table 2-1-1), so on the 128 KB parts the window
// is the whole of what a program may write, and the first byte past it
// is past the array - refused at compile time where the address is a
// constant, with `refused` where it is not.
#include "ch32vx03/nvm.hpp"

using Safe = brio::Clock<brio::ClockSource::pll, 96'000'000>;

void f() { (void)brio::Flash::erase_sector<brio::Flash::window_bytes>(Safe{}); }
