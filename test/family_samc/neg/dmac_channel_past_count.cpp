// mcu: samc21e18a samc21g18a samc21j18a
// The DMAC has DMAC_CH_NUM channels (twelve on every variant of this
// family), numbered 0..11. Channel 12 is not a slow channel or a
// reserved one: its descriptor slot would sit one entry PAST the tables
// this driver registers with BASEADDR/WRBADDR, so the controller would
// fetch a descriptor out of whatever follows them in .bss. Refused here
// rather than in RAM.
#include "samc/dmac.hpp"
using namespace brio;

void f() { (void)DmaChannel<DMAC_CH_NUM>::status(); }
