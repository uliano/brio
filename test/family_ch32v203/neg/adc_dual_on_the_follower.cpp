// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8
// A DUAL MODE ON THE FOLLOWER. CTLR1's DUALMOD is the master's field
// and 12.3.2 says so in as many words - "these bits in ADC2 are
// reserved" - so a dual mode is ADC1's verb and not a spelling the two
// instances share. The other two ADC1-only bits (the DMA request and
// TSVREFE) answer FALSE on the second converter instead, because a
// program may reach them through a configuration it did not write;
// nobody arrives at a dual mode by accident.
#include "ch32v203/adc.hpp"

void f() { (void)brio::Adc<2>::dual(brio::AdcDualMode::regular_simultaneous); }
