// An engine reports its completion on one of the four system interrupt
// lines, and a fifth line is refused where the engine names it too - not
// only in DmaLine, because an engine that could be built on a line the
// chip has not got would arm a channel whose interrupt nobody serves.
#include "rp2350/dma.hpp"
using Bad = brio::DmaRxEngine<5, uint8_t, 4>;
void f() { Bad::stop(); }
