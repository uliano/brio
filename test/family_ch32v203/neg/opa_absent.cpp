// mcu: ch32v203f6
// THE AMPLIFIER THE SMALLEST PACKAGE HAS NOT. Datasheet table 2-1 gives
// the twenty-pin CH32V203F6 one operational amplifier, and the pin
// table says which: that package bonds neither PB15 nor PB0, which are
// OPA1's two positive inputs, so the one it has is OPA2.
#include "ch32v203/opa.hpp"

using First = brio::Opa<1>;
void f() { First::enable(true); }
