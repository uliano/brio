// mcu: samc21e18a samc21g18a samc21j18a
// Hardware handshaking is available asynchronous on the internal clock
// alone (31.6.3.2): a synchronous configuration with the RTS/CTS pinout is
// refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .baud = 11,
    .synchronous = true,
    .pinout = UartPinout::flow_control,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
