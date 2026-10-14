// Must FAIL: a placement is InFlash or InRam (util/placement.hpp's
// HotPlacement), and a third tag is refused where the type is written.
#include "rp2350/spi.hpp"

using namespace brio;
struct InRom {};
void up() { (void)SpiHost<0, {.sck = 18, .tx = 19, .rx = 16}, NoDmaEngine, NoDmaEngine, InRom>::status(); }
