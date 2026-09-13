// A mass erase takes down every user sector, the running image with it.
// The confirmation is a TEMPLATE argument so that a suite cannot reach
// the verb through a run-time expression that happens to be wrong: the
// call does not compile unless the source says it in as many letters.
// mcu: stm32f429xx stm32f446xx stm32f411xe stm32f401xc stm32f412zx
#include "stm32f4/flash.hpp"
using namespace brio;
void f() { (void)Flash::mass_erase<FlashEraseAll::no>(); }
