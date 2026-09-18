// The QMI has two memory windows, one per chip select; there is no third.
#include "rp2350/flash.hpp"
using Bad = brio::QmiWindow<2>;
void f() { (void)Bad::timing(); }
