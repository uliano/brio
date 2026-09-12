// Must FAIL: a PIO block has four state machines.
#include "rp2040/pio.hpp"

void up() { (void)brio::PioSm<0, 4>::address(); }
