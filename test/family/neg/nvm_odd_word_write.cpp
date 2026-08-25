// mcu: avr128db48 avr128da28
// 11.3.2: the Flash is word-written and bit 0 of the address pointer is
// IGNORED. An odd address is therefore not half a word, it is a write
// to the even address below - refuse it instead of surprising the
// caller.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::write_word_at<FlashLayout<128, 0>, 0x10001>(0x1234); }
