// mcu: ch32v203c8
// The fault body writes the breadcrumb through the PLATFORM's own
// storage: a type that is not a Platform must be REFUSED, not accepted
// with some other panic_record().
#include "ch32v203/reset.hpp"

struct NotAPlatform {};

[[noreturn]] void f() { brio::fault_reset<NotAPlatform>(); }
