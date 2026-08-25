// mcu: avr128db48 avr128da28
// Table 11-4: FLMPER32 erases the pages matching FPAGE[N:5], so the
// address must be aligned to the whole 16 KB block it takes down.
#include "avrdx/nvm.hpp"
using namespace brio;
void f() {
    (void)Nvm::erase_at<FlashLayout<128, 0>, 0x10200, FlashErase::pages32>();
}
