// REFUSED: writing LINESTATE_TUNING. 12.7.2 says the register is to be
// left at its reset value and that software should not clear it - every
// device fix in it is already on - so this driver READS it and has no
// verb that stores into it. The absence is asserted here rather than
// promised in a comment.
#include "rp2350/usb.hpp"

using namespace brio;

void retune() { Usb::linestate_tuning(0); }
