// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The pool is handed out 64 bytes a direction, so its size is a whole
// number of them: a pool whose tail could never hold a buffer is refused
// rather than silently rounded.
#include "ch32vx03/usbfs.hpp"

using Ragged = brio::Usbfs<100>;
void f() { Ragged::connect(true); }
