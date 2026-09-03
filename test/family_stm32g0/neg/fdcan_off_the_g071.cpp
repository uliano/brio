// mcu: stm32g071xx
// Table 1 gives the FDCAN to the G0B1/G0C1 class alone: the G071's
// device header declares no FDCAN1_BASE, no FDCAN_GlobalTypeDef and no
// interrupt enumerator - so the whole register-facing half of the driver
// is not compiled there and `Fdcan<1>` is not a name at all.
#include "stm32g0/fdcan.hpp"
brio::Fdcan<1> absent;
