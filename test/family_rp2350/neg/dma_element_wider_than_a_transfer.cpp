// A DMA element is ONE bus access wide - 1, 2 or 4 bytes, the whole of
// CTRL.DATA_SIZE (12.6.2.3). An engine over a wider type would move each
// element in pieces the controller never promised to keep together, so
// the element type itself is refused.
#include "rp2350/dma.hpp"
using Bad = brio::DmaTxEngine<0, uint64_t>;
void f() { (void)Bad::size; }
