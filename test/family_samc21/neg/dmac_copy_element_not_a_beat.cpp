// mcu: samc21e18a samc21g18a samc21j18a
// A copy's element type IS its beat, and 25.10.1 implements three: a
// byte, a halfword, a word. An eight-byte element has no beat, so the
// copy is refused where it is spelled rather than moved as something
// else.
#include <stdint.h>

#include "samc21/dmac.hpp"
using namespace brio;

void f() {
    static uint64_t a[4];
    static uint64_t b[4];
    (void)DmaCopyEngine<0>::copy(a, b, 4u);
}
