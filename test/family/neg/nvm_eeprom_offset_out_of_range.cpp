// mcu: avr128db48 avr128da28
// The EEPROM is 512 bytes (11.3.1.2): offset 512 is outside it.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::eeprom_erase_at<512>(); }
