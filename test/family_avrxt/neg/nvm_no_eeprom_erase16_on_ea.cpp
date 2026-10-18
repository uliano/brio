// mcu: avr64ea48 avr64ea32
// One EEPROM operation reaches one 8-byte page on the AVR EA
// (DS40002443A table 8-3): the AVR DA/DB's 16- and 32-byte erase spans
// have no counterpart.
#include "avrxt/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::eeprom_erase(0, EepromErase::bytes16); }
