// mcu: avr64ea48 avr64ea28
// The AVR EA's User Row is written over UPDI, never by the CPU: its
// write path is the declined flash one.
#include "avrxt/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::userrow_write(0, 0x5Au); }
