// mcu: ch32v303rc ch32v303vc
// THE SMARTCARD ON A UART. UART6 is TX and RX alone (table 10-29): no CK
// to clock a card, and no SCEN worth writing - chapter 18's opening counts
// three USARTs and five UARTs, and the full modes are the USARTs'. The
// verb is refused at compile time on an instance that is not full.
#include "ch32vx03/usart.hpp"

using Sixth = brio::Usart<6>;
void f() { (void)Sixth::smartcard({.nack = true, .guard_time = 16, .clock_prescaler = 6}); }
