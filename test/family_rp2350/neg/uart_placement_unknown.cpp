// Must FAIL: a placement is InFlash or InRam (util/placement.hpp's
// HotPlacement), and a third tag is refused where the type is written.
#include "rp2350/uart.hpp"

using namespace brio;
struct InRom {};
void up() { (void)Uart<0, {.tx = {0, PinFunction::uart}, .rx = {1, PinFunction::uart}}, 64, 256, NoDmaEngine, NoDmaEngine, InRom>::dma_faults(); }
