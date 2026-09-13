// x64 programming needs 8..9 V on the VPP pad (table 6 / table 13), a
// supply no board of this bench has and one the datasheet forbids
// leaving applied for more than an hour: the width is refused where it
// is written, not attempted and blamed on the silicon afterwards.
// mcu: stm32f429xx stm32f446xx stm32f411xe stm32f401xc stm32f405xx
#include "stm32f4/flash.hpp"
using namespace brio;
void f() {
    const uint8_t data[8] = {};
    (void)Flash::program<FlashParallelism::x64>(0x08060000UL, data);
}
