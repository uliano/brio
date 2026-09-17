// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A RESCALING CLOCK UNDER THE CONVERTER. Every sampling time this
// driver programs and every conversion it reports is in ADCCLK cycles,
// and ADCCLK follows PCLK2: a DynamicClock would move them all at once
// and leave a program holding numbers for a rate it no longer runs at.
// The converter takes a static clock, as the timers do.
#include "ch32v203/adc.hpp"

using Slow = brio::Clock<brio::ClockSource::pll, 48'000'000>;
using Fast = brio::Clock<brio::ClockSource::pll, 96'000'000>;
using Rescaling = brio::DynamicClock<brio::Rates<Slow, Fast>>;

void f() { (void)brio::Adc<1>::init(Rescaling{}); }
