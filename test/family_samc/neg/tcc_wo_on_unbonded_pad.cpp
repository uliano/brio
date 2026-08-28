// mcu: samc21e18a samc21g18a
// PB16 carries TCC0/WO4 under peripheral function F on the J package
// alone. The map is the device header's own PIN_P<pad><fn>_TCC<n>_WO<k>
// symbols, so a pad this package does not bond has no waveform output
// and TccWo<> must refuse it.

#include "samc/pin.hpp"
#include "samc/tcc.hpp"

using namespace brio;

using Wave = TccWo<Pin<'B', 16>, PinFunction::f>;
static_assert(Wave::timer == 0,
              "this assertion is meant to FAIL: PB16 is not bonded to a TCC "
              "output on this package");
