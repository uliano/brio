// mcu: avr128db48 avr128da28
// The same rule for an erase - and this is the one the multi-page erase
// erratum makes sharp: a 32-page erase whose FIRST page is legal would
// take the rest down whatever the hardware checks (DS80000915F 2.7.1,
// DS80000882C 2.7.2). The refusal is about the WHOLE range.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() {
    (void)Nvm::erase_at<FlashLayout<128, 0>, 0x0C000, FlashErase::pages32>();
}
