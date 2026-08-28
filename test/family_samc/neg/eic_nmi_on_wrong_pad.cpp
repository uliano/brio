// mcu: samc21e18a samc21g18a samc21j18a
// The NMI has exactly one pad on this family, and which one it is comes
// from the device header's PIN_P<pad>A_EIC_NMI symbol. PA09 carries
// EXTINT9 and no NMI, so building an ExtNmi<> on it must not compile:
// an NMI cannot be masked, and a driver that let one be armed on the
// wrong pad would be handing out an unbreakable interrupt.

#include "samc/eic.hpp"
#include "samc/pin.hpp"

using namespace brio;

using WrongNmi = ExtNmi<Pin<'A', 9>>;
static_assert(WrongNmi::pin::pin_number == 9,
              "this assertion is meant to FAIL: PA09 is not the NMI pad");
