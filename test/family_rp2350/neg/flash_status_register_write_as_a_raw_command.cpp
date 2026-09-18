// The raw command verb carries READS only: a write of status register 1
// (01h) is how a chip loses its quad-enable bit for good.
#include "rp2350/flash.hpp"
uint8_t rx[4];
void f() { (void)brio::Flash::command<0x01>({}, rx); }
