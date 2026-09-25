// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// THE DAC ON A PART THAT HAS NONE. RM ch. 17 is the whole family's manual
// and its converter is every CH32V303's (datasheet table 2-1-1); the
// CH32V203's table 2-1 has no DAC row at all, so the converter is refused
// where it is used - the header itself still compiles, for a program that
// serves both series.
#include "ch32vx03/dac.hpp"

void f() { brio::Dac::init(); }
