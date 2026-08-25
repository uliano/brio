// mcu: avr128db48 avr128da28
// Table 11-1: code can never write the section it executes from. Under
// the declared layout (BOOT = the first 64 KB) an address inside BOOT
// must be refused at compile time, not discovered as a WRITEPROTECT
// error on the bench.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::write_word_at<FlashLayout<128, 0>, 0x0800>(0x1234); }
