// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb
// The CH32V203 has USART1..3 and UART4 and no more, and the 128 KB
// CH32V303 has USART1..3: a fifth instance is refused wherever the part
// has none (the CH32V303RC and VC carry UART5..8).
#include "ch32v203/usart.hpp"

using Fifth = brio::Usart<5>;
void f() { Fifth::bus_clock(true); }
