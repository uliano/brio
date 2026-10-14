// Must FAIL: a placement is InFlash or InRam (util/placement.hpp's
// HotPlacement), and a third tag is refused where the type is written.
#include "rp2040/i2c.hpp"

using namespace brio;
struct InRom {};
void up() { (void)I2cHost<1, {.scl = 3, .sda = 2}, NoDmaEngine, NoDmaEngine, InRom>::status(); }
