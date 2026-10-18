// mcu: avr64ea48 avr64ea32
// Flash self-programming is declined on the AVR EA: no page erase from
// the CPU either.
#include "avrxt/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::erase(0x8000u); }
