// mcu: avr64ea48 avr64ea32
// No flash self-programming on the AVR EA: NvmFlash (and a flash heap
// over it) is refused by name.
#include "avrxt/nvm_flash.hpp"
using namespace brio;
void f() {}
