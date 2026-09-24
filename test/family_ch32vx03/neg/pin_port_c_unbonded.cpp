// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8
// PC13 is the tamper and RTC-output pad, and the six smallest packages
// bond no pin of port C at all - VBAT is not a pin on them either.
#include "ch32vx03/pin.hpp"

using Tamper = brio::Pin<'C', 13>;
void f() { Tamper::output(); }
