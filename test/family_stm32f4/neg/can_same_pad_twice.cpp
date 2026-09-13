// A node's receiver and its transmitter on one pad is not a node: the
// second claim would take the pad from the first, silently.
// mcu: stm32f429xx stm32f446xx
#include "stm32f4/can.hpp"
using namespace brio;
constexpr CanPins pins{.rx = {'A', 11, PinFunction::af9}, .tx = {'A', 11, PinFunction::af9}};
void f() { Can<1>::claim_pads<pins>(); }
