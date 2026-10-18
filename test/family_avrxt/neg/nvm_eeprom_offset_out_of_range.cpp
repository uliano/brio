// mcu: avr128db48 avr128da28 avr64ea48
// The EEPROM is 512 bytes (DS40002247B 11.3.1.2, DS40002443A table
// 8-3): offset 512 is outside it.
#include "avrxt/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::eeprom_erase_at<512>(); }
