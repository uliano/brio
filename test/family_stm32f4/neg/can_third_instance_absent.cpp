// CAN3 is the F413/F423's alone: on the parts with the CAN1/CAN2 pair
// there is no third block to name.
// mcu: stm32f429xx stm32f446xx stm32f405xx
#include "stm32f4/can.hpp"
void f() { (void)brio::Can<3>::in_init(); }
