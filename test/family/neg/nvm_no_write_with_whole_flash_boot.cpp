// mcu: avr128db48 avr128da28
// BOOTSIZE = 0 (the shipping fuse default) makes the ENTIRE Flash one
// BOOT section (table 11-2), and then no address at all can be written
// by the software running in it. An image that declares that geometry
// and still writes Flash is a contradiction the compiler can catch.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::write_word_at<FlashLayout<0>, 0x10000>(0x1234); }
