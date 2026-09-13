// The F401, F411 and F446 have no random number generator at all, and
// their device header declares no RNG_BASE - so the resource does not
// exist there rather than answering false.
// mcu: stm32f411xe stm32f401xe stm32f446xx
#include "stm32f4/rng.hpp"
void f() { (void)brio::Rng::ready(); }
