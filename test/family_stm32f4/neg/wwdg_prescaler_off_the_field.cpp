// WWDG_CFR.WDGTB is TWO bits on this family (four codes, /1 to /8 -
// the reserve reads the width off the header's own mask), so a fifth
// code is refused rather than written into the window's field.
// mcu: stm32f429xx stm32f411xe
#include "stm32f4/reset.hpp"
void f() {
    (void)brio::Wwdg::configure<
        brio::WwdgConfig{static_cast<brio::WwdgPrescaler>(4), 0x7F}>();
}
