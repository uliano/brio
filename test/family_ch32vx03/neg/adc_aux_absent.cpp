// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A SHORT SAMPLING TIME ON A PART WITHOUT ADCx_AUX. The register that
// switches a channel's upper four codes to 2.5..5.5 cycles names the
// CH32F20x_D8, the D8C, the CH32V30x_D8, the D8C and the CH32V31x_D8C in
// its note (RM 12.3.15) - no CH32V20x - so on every CH32V203 the code
// would mean 41.5 cycles or more and the short time is refused.
#include "ch32vx03/adc.hpp"

void f() { (void)brio::Adc<1>::sample_time(0, brio::AdcShortSampleTime::cycles2_5); }
