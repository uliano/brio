// mcu: avr128db48 avr128da28
// 128 KB is 0x00000..0x1FFFF: an address past the end of the Flash is
// not a runtime failure, it is a typo.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::write_word_at<FlashLayout<128, 0>, 0x20000>(0x1234); }
