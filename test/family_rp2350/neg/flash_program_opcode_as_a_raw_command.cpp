// The raw command verb carries READS only: a page program opcode (02h)
// must not reach the chip through it.
#include "rp2350/flash.hpp"
uint8_t rx[4];
void f() { (void)brio::Flash::command<0x02>({}, rx); }
