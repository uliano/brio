// mcu: samc21e18a samc21g18a samc21j18a
// The DMAC has ONE channel in this stratum: DmaTxEngine is channel 0 and
// nothing else, so no image can make two channels concurrent (erratum
// 1.10.4, samc21/dmac.hpp). A second channel cannot even be spelled:
// the engine is not a template over a channel number.
#include "samc21/dmac.hpp"
using namespace brio;

void f() { (void)DmaTxEngine<1>::busy(); }
