// mcu: ch32v203rb
// THE FLOW-CONTROL PAIR IS A FULL USART'S. On this device class the
// fourth serial port is a UART4 - TX and RX alone, table 10-26 - so RTS
// and CTS are not its to enable, whatever CTLR3's bits read.
#include "ch32v203/platform.hpp"
#include "ch32v203/usart.hpp"

inline constexpr brio::UartOptions pair{.rts = true, .cts = true};
using Flow = brio::Uart<4, brio::Ch32v203Platform<>, 64, 64, brio::UartFormat{},
                        brio::NoDmaEngine, brio::NoDmaEngine, 0, pair>;
void f() { (void)Flow::baud(); }
