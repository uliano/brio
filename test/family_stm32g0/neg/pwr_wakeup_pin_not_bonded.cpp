// mcu: stm32g071xx stm32g031xx
// The six wake-up pins are a SPARSE set: only the G0B1/G0C1 bonds
// WKUP3, and the G031 has no WKUP5 either. A driver that assumed six
// would arm a bit that is not there, so the reserve answers per part.
#include "stm32g0/pwr.hpp"
static_assert(brio::Pwr::wakeup_pin_present(3));
