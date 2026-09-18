// This chip has three PIO blocks, PIO0, PIO1 and PIO2 - the RP2040 had
// two, and a fourth exists on neither.
#include "rp2350/pio.hpp"
using Bad = brio::Pio<3>;
void f() { (void)Bad::version(); }
