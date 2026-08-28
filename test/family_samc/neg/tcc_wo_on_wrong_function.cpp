// mcu: samc21e18a samc21g18a samc21j18a
// THE FACT THAT SEPARATES THIS MAP FROM THE TC'S: a TCC waveform output
// is keyed by pad AND peripheral function. PA22 carries TCC0/WO4 under
// function F and NOTHING under function E, so asking for the E one must
// not compile - even though the pad is bonded and the instance exists.

#include "samc/pin.hpp"
#include "samc/tcc.hpp"

using namespace brio;

using Wave = TccWo<Pin<'A', 22>, PinFunction::e>;
static_assert(Wave::timer == 0,
              "this assertion is meant to FAIL: PA22 has no TCC output on "
              "function E");
