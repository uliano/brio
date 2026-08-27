// mcu: samc21e18a
// The E package bonds SERCOM0..3 and no more, so SERCOM5 has no DMA
// trigger code there. Asking for one must FAIL, not quietly answer
// `dma_trigger_none` - a channel configured with TRIGSRC 0x00 does not
// report an error, it simply never fires, which is the worst possible
// way for a missing instance to show up. The G and J packages have the
// instance and compile the same line (test/family_samc/dmac.cpp).
#include "samc/dmac.hpp"
using namespace brio;

void f() { (void)dma_trigger_sercom_tx<5>(); }
