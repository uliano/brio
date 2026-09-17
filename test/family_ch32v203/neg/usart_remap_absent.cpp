// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8
// A COLUMN OF ANOTHER DEVICE CLASS: USART2's remap (table 10-24, the
// PD5/PD6 column) belongs to the CH32V20x_D8, 10.3.2.2's note excludes
// it here, and the field reads back zero on this silicon. A transport
// naming code 1 on a part of this class would believe its pads had
// moved while they had not.
#include "ch32v203/platform.hpp"
#include "ch32v203/usart.hpp"

using Moved = brio::Uart<2, brio::Ch32v203Platform<>, 64, 64, brio::UartFormat{},
                         brio::NoDmaEngine, brio::NoDmaEngine, 1>;
void f() { (void)Moved::baud(); }
