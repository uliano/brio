// mcu: avr64ea48 avr64ea32
// AnalogSampler's Sampled.value is 16 bits: a converter configured for an
// unscaled sum of 32 samples (17 bits) is refused there.
#include "avrxt/adc.hpp"
#include "avrxt/platform.hpp"
#include "util/analog_sampler.hpp"
using namespace brio;
using Conv = AdcConfigured<0, AdcConfig{.accumulate = 32}>;
using S = AnalogSampler<Conv, AvrPlatform, Subscribers<>, AdcInput::temp>;
void f() { S::init(); }
