// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE CONVERTER OUT OF SPECIFICATION. ADCCLK is PCLK2 divided by 2, 4,
// 6 or 8 (RM 3.4.2's ADCPRE) and this family's PCLK2 is HCLK undivided,
// so at the 144 MHz the part is rated for the slowest divider still
// gives 18 MHz against a converter rated at 14. The tree is legal and
// the clock task programs it; it is the ADC that refuses, on the line
// that handed it the clock.
#include "ch32v203/adc.hpp"

using TooFast = brio::Clock<brio::ClockSource::pll, 144'000'000>;
void f() { (void)brio::Adc<1>::init(TooFast{}); }
