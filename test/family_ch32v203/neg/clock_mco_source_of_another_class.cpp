// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// The clock output's multiplexer has eight codes in the manual and four
// of them belong to other device classes - PLL2, PLL3 and the Ethernet
// oscillator are the D8C families'. McoSource names only what this
// class has, so asking for one of the others is not a run-time refusal
// but a name that does not exist.
#include "ch32v203/clock.hpp"

void f() { brio::Mco::init(brio::McoSource::pll2); }
