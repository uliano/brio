// mcu: samc21e18a samc21g18a samc21j18a
// The same refusal from the resource side: Port<'C'> does not exist.
#include "samc/pin.hpp"
using namespace brio;
void f() { (void)Port<'C'>::in(); }
