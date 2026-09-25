// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb
// A DMA2 ROW ON A PART THAT CANNOT RAISE IT. TIM8's update is table
// 11-3's, a request of the second controller - which no CH32V203 has -
// and of the second advanced timer, which the 128 KB CH32V303 has not
// got either (datasheet table 2-1-1). Refused on the line that asked.
#include "ch32vx03/dma.hpp"

void f() { (void)brio::DmaRequestOf<brio::DmaRequest::tim8_up>::channel; }
