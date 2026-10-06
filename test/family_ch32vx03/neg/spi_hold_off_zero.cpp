// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A HOLD-OFF OF ZERO is no image's: the tick's handler alone keeps the
// pump's vector waiting, and every critical section does, so a host
// that declared zero would keep two frames in flight at every rate and
// lose answers to the first handler that ran. REFUSED.
#include "ch32vx03/spi.hpp"

using Unheld = brio::SpiHost<1, brio::spi_default_pins<1>, brio::NoDmaEngine, brio::NoDmaEngine, 0>;
void f() { (void)Unheld::status(); }
