// Only the F42x/F43x class has a second bank (RM0090 3.4). The device
// header is no guide here - it declares FLASH_CR_MER2 on the F446 too,
// which RM0390 3.3 gives 512 Kbytes in ONE bank - so a bank-2 erase
// named on a single-bank part must fail to compile rather than store a
// one into a reserved bit and start an erase nobody can name.
// mcu: stm32f446xx stm32f411xe stm32f405xx stm32f407xx stm32f401xe
#include "stm32f4/flash.hpp"
using namespace brio;
void f() {
    (void)Flash::bank_erase<FlashBank::bank2>(FlashEraseAll::erase_every_user_sector);
}
