// mcu: avr128db48 avr128da28
// Table 11-5: an 8-byte erase matches ADDR[N:3], so offset 4 would
// erase 0..7 and not 4..11.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() { (void)Nvm::eeprom_erase_at<4, EepromErase::bytes8>(); }
