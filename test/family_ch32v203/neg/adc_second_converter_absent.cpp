// mcu: ch32v203rb
// THE 128 KB PART HAS ONE CONVERTER. Datasheet table 2-1 counts the
// channels beside the units - "10@2" on the parts up to the CH32V203C8
// and "16@1" here - so this part trades ADC2 for six more channels on
// the one it keeps, and Adc<2> is a peripheral that is not there.
#include "ch32v203/adc.hpp"

using Second = brio::Adc<2>;
void f() { Second::start(); }
