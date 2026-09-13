// ITR1_RMP is TIM2's option register field; TIM5's is TI4_RMP, so the
// typed verb must refuse the other instance rather than write the wrong
// two bits.
// mcu: stm32f411xe stm32f429xx stm32f446xx
#include "stm32f4/tim.hpp"
void f() { (void)brio::Tim<5>::trigger1_source(brio::Tim2Trigger1::otg_fs_sof); }
