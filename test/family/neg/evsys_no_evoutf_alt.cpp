// mcu: avr128db48 avr128db28 avr64ea48 avr64ea28
// PF7 as EVOUTF's ALT1: no such position on the AVR DA/DB (17.5.1); the
// UPDI pin on the AVR EA (refused there by name: a GPIO only with
// UPDIPINCFG = 0, which costs the UPDI link without a high-voltage
// programmer).
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EvOut<Pin<'F', 7>>::listen(EventChannel<0>{}); }
