// The DMA has four pacing timers (12.6.8.1: TIMER0..TIMER3, one TREQ
// code each at 0x3b..0x3e), so a fifth has neither a register nor a
// request number and must not compile.
#include "rp2350/dma.hpp"
using Bad = brio::DmaTimer<4>;
void f() { Bad::stop(); }
