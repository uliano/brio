// mcu: samc21e18a samc21g18a samc21j18a
// A DMA beat is one, two or four bytes (25.10.1 BEATSIZE: the fourth
// code is Reserved). An engine's element type therefore has exactly
// three legal widths, and a three-byte struct is not one of them: taken
// silently it would move three bytes per beat somewhere, or - worse -
// pick a width the end-address arithmetic does not share, and the block
// would run over the wrong memory at full speed. Refused where the type
// is named.
#include "samc/dmac.hpp"
using namespace brio;

struct Three {
    uint8_t a, b, c;
};

void f() { (void)DmaPingPongEngine<0, Three>::beat; }
