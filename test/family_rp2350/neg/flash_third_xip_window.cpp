// The XIP cache guards two memory windows, WRITABLE_M0 and WRITABLE_M1.
#include "rp2350/flash.hpp"
void f() { (void)brio::Xip::window_writable<2>(); }
