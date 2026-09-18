// SCRATCH4..7 carry the bootrom's boot redirection, not the program's.
#include "rp2350/watchdog.hpp"
void f() { brio::Scratch<4>::write(0xB007C0D3u); }
