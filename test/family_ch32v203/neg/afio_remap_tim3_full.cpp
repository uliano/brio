// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb
// TIM3's complete mapping is PC6..PC9 (table 10-17), which only the
// 64-pin and 100-pin parts of this family bond - the manual's note says
// "not supported in packages below 64 pins" and the bonding table
// agrees.
#include "ch32v203/afio.hpp"

void f() { brio::Afio::remap<brio::Remap::tim3, 3>(); }
