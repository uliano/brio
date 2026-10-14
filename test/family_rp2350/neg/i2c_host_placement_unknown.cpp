// Must FAIL: a placement is InFlash or InRam (util/placement.hpp's
// HotPlacement), and a third tag is refused where the type is written.
#include "rp2350/i2c.hpp"

using namespace brio;
struct InRom {};
void up() { (void)I2cHost<0, {.scl = 13, .sda = 12}, NoDmaEngine, NoDmaEngine, InRom>::status(); }
