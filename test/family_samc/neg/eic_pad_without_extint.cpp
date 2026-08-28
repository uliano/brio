// mcu: samc21e18a
// PB22 carries EXTINT6 on the G and J packages and is not bonded at all
// on the E. The pad-to-line map is the device header's own
// PIN_P<pad>A_EIC_EXTINT_NUM symbols, so a pad this package does not
// bond has no line and ExtInt<> must refuse it - the per-package gate
// with no hand-kept table behind it.

#include "samc/eic.hpp"
#include "samc/pin.hpp"

using namespace brio;

using Button = ExtInt<Pin<'B', 22>>;
static_assert(Button::line == 6,
              "this assertion is meant to FAIL: PB22 is not bonded on the E "
              "package, so it has no external interrupt line");
