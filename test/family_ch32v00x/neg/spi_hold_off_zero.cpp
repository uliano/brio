// mcu: ch32v006k8
// A HOLD-OFF OF ZERO is no image's: the tick's handler alone keeps the
// pump's vector waiting, and every critical section does, so a host
// that declared zero would keep two frames in flight at every rate and
// lose answers to the first handler that ran. REFUSED.
#include "ch32v00x/spi.hpp"

using Unheld = brio::SpiHost<1, brio::spi1_default_pins, brio::NoDmaEngine, brio::NoDmaEngine, 0>;
void f() { (void)Unheld::status(); }
