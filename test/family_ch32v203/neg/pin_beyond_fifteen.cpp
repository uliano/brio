// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A port of this family has sixteen pins, 0..15.
#include "ch32v203/pin.hpp"

using Sixteenth = brio::Pin<'A', 16>;
void f() { Sixteenth::output(); }
