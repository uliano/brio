// A pad on a port this package does not bond is refused where the claim
// is written: the F446 stops at port H.
// mcu: stm32f446xx stm32f429xx
#include "stm32f4/can.hpp"
using namespace brio;
constexpr CanPins pins{.rx = {'Z', 9, PinFunction::af9}, .tx = {'A', 12, PinFunction::af9}};
void f() { Can<1>::claim_pads<pins>(); }
