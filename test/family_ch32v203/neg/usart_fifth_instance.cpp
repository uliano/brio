// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// The family has USART1..3 and UART4 and no more: a fifth instance has
// no base address, whatever the part.
#include "ch32v203/usart.hpp"

using Fifth = brio::Usart<5>;
void f() { Fifth::bus_clock(true); }
