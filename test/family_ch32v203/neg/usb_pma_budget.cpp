// mcu: ch32v203f6 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// The packet memory this block shares with the CAN controller is 512
// bytes: a budget above it is refused.
#include "ch32v203/usb.hpp"

using Greedy = brio::Usbd<1024>;
void f() { Greedy::connect(true); }
