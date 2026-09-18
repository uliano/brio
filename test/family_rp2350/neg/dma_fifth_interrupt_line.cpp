// The DMA has FOUR system interrupt lines here (12.6.5: INTE0..INTE3,
// against the RP2040's two), so DMA_IRQ_4 does not exist and a line
// numbered 4 must not compile - it would read and write past the last
// enable bank.
#include "rp2350/dma.hpp"
using Bad = brio::DmaLine<4>;
void f() { Bad::enable(); }
