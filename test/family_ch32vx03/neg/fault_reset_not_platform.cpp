// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The fault body writes the breadcrumb through the PLATFORM's own
// storage: a type that is not a Platform must be REFUSED, not accepted
// with some other panic_record().
#include "ch32vx03/reset.hpp"

struct NotAPlatform {};

[[noreturn]] void f() { brio::fault_reset<NotAPlatform>(); }
