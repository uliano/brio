// mcu: samc21e18a samc21g18a samc21j18a
// An image holds two DMA channels at most (docs/samc21/dmac.md, "The rule":
// three active channels faulted in every shape measured, erratum 1.10.4), so
// an engine names channel 0 or 1 and a third number does not compile.
#include "samc21/dmac.hpp"
using namespace brio;

void f() { (void)DmaTxEngine<2>::busy(); }
