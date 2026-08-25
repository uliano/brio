// mcu: avr128db48 avr128da28
// The User Row is 32 bytes (11.3.1.4).
#include "avrdx/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::userrow_write_at<32>(0x5A); }
