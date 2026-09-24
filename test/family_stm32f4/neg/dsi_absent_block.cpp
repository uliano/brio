// The DSI Host is on the F469/F479 class alone: the F42x/F43x with their
// display interface on pads, the F446, the F411 and the F405 class have
// no block and their device header declares no DSI_BASE - so the
// resource does not exist there rather than answering false.
// mcu: stm32f429xx stm32f446xx stm32f411xe stm32f407xx
#include "stm32f4/dsi.hpp"
void f() { brio::Dsi::clock(true); }
