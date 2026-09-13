// The channel fields of this converter are five bits and the chapter
// documents 0..18; anything above is Reserved and no pad claims it.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/adc.hpp"
void f() { (void)brio::AnalogIn<brio::Pin<'A', 4>, 19>::channel; }
