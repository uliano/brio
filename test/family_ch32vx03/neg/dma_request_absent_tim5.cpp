// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb ch32v303rb
// Table 11-6's six TIM5 rows are the CH32V20x_D8's: the 32-bit timer
// exists on the 128 KB CH32V203 alone, so its update request - the only
// thing channel 8 answers besides UART4's receiver - is refused on every
// CH32V203 below it, and on the 128 KB CH32V303, which has no TIM5 at
// all.
#include "ch32vx03/dma.hpp"

void f() { (void)brio::DmaRequestOf<brio::DmaRequest::tim5_up>::channel; }
