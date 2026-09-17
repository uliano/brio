// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8
// Table 11-6's six TIM5 rows are the OTHER DEVICE CLASS's: the 32-bit
// timer exists on the 128 KB part alone, so its update request - the
// only thing channel 8 answers besides UART4's receiver - is refused on
// every part below it.
#include "ch32v203/dma.hpp"

void f() { (void)brio::DmaRequestOf<brio::DmaRequest::tim5_up>::channel; }
