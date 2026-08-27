// mcu: samc21e18a
// The E package bonds FOUR SERCOMs (SERCOM0..3); the G and J bond six.
// So this is the family's one genuinely package-dependent refusal, and
// it names only the variant that lacks the instance - on the G and J the
// very same line is legal, which is the point. (The console this driver
// was written for lives on SERCOM5, i.e. on a J.)
#include "samc/sercom.hpp"
using namespace brio;
void f() { (void)Sercom<5>::irq(); }
