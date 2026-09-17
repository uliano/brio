// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A PAD THAT IS NOT AN ANALOG INPUT. Sixteen pads carry a channel on
// this family - PA0..PA7, PB0, PB1, PC0..PC5 - and PA13 is not one of
// them on any package (it is the debug port's, and the datasheet's pin
// tables give it no ADC_INx function). AnalogIn says so on the line
// that asked rather than converting whatever channel 0 happens to be.
#include "ch32v203/adc.hpp"

using NotAnInput = brio::AnalogIn<brio::Pin<'A', 13>>;
void f() { NotAnInput::claim(); }
