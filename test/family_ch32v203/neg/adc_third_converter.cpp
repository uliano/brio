// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// THERE IS NO THIRD CONVERTER on any part of this family: RM tables
// 12-5 and 12-6 are ADC1's and ADC2's registers and there is no 12-7,
// and the datasheet's table 2-1 counts one or two units per part.
#include "ch32v203/adc.hpp"

using Third = brio::Adc<3>;
void f() { Third::start(); }
