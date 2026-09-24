// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc
// TIM1's complete mapping is port E's (table 10-15), and no part of this
// family but the LQFP100 bonds a pin of port E: elsewhere the column
// exists in the register and nowhere on the silicon.
#include "ch32v203/afio.hpp"

void f() { brio::Afio::remap<brio::Remap::tim1, 3>(); }
