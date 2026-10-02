// mcu: samc21e18a samc21g18a samc21j18a
// An engine's element type is the WIDEST beat its binding allows: a run
// of narrower elements moves with their own beat, but a run of WIDER
// ones would need a beat the binding never allowed - a halfword into a
// byte register writes a byte the peripheral never asked for. The
// overload does not exist, so the call does not compile.
#include <span>

#include "samc21/dmac.hpp"
using namespace brio;

void f() {
    static const uint16_t halves[4] = {};
    (void)DmaTxEngine<0>::start(std::span<const uint16_t>(halves));
}
