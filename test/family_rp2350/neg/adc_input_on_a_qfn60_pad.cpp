// REFUSED IN THE QFN-80: GP26 as an ADC input. The pad map moves with
// the package on this chip - GPIO26..GPIO29 are inputs 0..3 in the
// QFN-60, GPIO40..GPIO47 are inputs 0..7 here (datasheet 12.4.2.1) - so
// GP26 is an ordinary GPIO in this package and the converter cannot
// reach it.
#include "rp2350/adc.hpp"

using namespace brio;

using Wrong = AnalogIn<Pin<26>>;

void claim() { (void)Wrong::claim(); }
