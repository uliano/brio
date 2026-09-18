// A PIO block has four state machines, 0..3, on both of these chips.
#include "rp2350/pio.hpp"
using Bad = brio::PioSm<2, 4>;
void f() { (void)Bad::address(); }
