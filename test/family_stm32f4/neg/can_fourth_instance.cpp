// Three instances is this family's maximum and only the F413/F423 reach
// it; no part has a fourth.
// mcu: stm32f429xx stm32f446xx stm32f413xx
#include "stm32f4/can.hpp"
void f() { (void)brio::Can<4>::in_init(); }
