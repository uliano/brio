// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6
// The request table is read through the PART: these seven offer two
// usarts or one, so USART3 raises nothing on them and the channel table
// 11-5 gives it (2 for the transmitter) belongs to silicon this die has
// not got. The two parts with four usarts compile the same line.
#include "ch32v203/dma.hpp"

void f() { (void)brio::DmaRequestOf<brio::DmaRequest::usart3_tx>::channel; }
