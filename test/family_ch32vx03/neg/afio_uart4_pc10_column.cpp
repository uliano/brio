// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb ch32v303rb
// UART4 ON PC10, ASKED FOR BY ITS PAD: code 0 of table 10-26, which is
// every class's but the CH32V20x_D6's - the CH32V203C8 reads table 10-27,
// whose code 0 is PB0, so no code of that part puts UART4's TX on PC10
// and the constant face refuses the 0xFF afio_usart_code_for() answers.
// The parts with no fourth serial port refuse it too.
#include "ch32vx03/afio.hpp"

void f() {
    brio::Afio::remap<brio::Remap::uart4, brio::afio_usart_code_for(4, brio::Pad{'C', 10})>();
}
