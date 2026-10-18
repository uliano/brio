// mcu: avr64ea48 avr64ea28
// Flash self-programming is declined on the AVR EA: the AVR DA/DB's
// write verb exists as a name there only to refuse, with the reason.
#include "avrxt/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::write_word(0x8000u, 0x1234u); }
