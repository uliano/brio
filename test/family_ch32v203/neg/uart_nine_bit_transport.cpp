// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// The transport's rings carry BYTES: a nine-bit word is the resource's
// read_word()/write_word(), not the Uart task's.
#include "ch32v203/platform.hpp"
#include "ch32v203/usart.hpp"

using Wide = brio::Uart<2, brio::Ch32v203Platform<>, 64, 64,
                        brio::UartFormat{brio::UartBits::nine, brio::UartParity::none}>;
void f() { (void)Wide::baud(); }
