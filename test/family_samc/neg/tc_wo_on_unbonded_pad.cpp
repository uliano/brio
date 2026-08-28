// mcu: samc21e18a
// PB23 carries TC3/WO1 on the G and J packages (it is the bench board's
// LED) and is not bonded on the E. The pad-to-output map is the device
// header's own PIN_P<pad>E_TC<n>_WO<k> symbols, so a pad this package
// does not bond has no waveform output and TcWo<> must refuse it.

#include "samc/pin.hpp"
#include "samc/tc.hpp"

using namespace brio;

using LedWave = TcWo<Pin<'B', 23>>;
static_assert(LedWave::timer == 3,
              "this assertion is meant to FAIL: PB23 is not bonded on the E "
              "package, so it carries no waveform output");
