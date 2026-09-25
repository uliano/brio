// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// Endpoint zero sends and receives through ONE 64-byte buffer (RM
// 23.2.2), which init() claims first: a pool smaller than that is
// refused.
#include "ch32vx03/usbfs.hpp"

using Tiny = brio::Usbfs<32>;
void f() { Tiny::connect(true); }
