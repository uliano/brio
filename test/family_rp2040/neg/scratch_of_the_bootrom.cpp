// SCRATCH4..7 carry the bootrom's boot magic: not brio's to hand out.
#include "rp2040/watchdog.hpp"
void f() { brio::Scratch<4>::write(0); }
