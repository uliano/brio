// mcu: avr128db48 avr128da28
// 11.3.2: a page erase ignores the low bits of the address, so an
// address inside a page erases the whole page - and an address that is
// not the page's own start hides which page that is.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::erase_at<FlashLayout<128, 0>, 0x10100>(); }
