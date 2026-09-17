// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// IWDG_PSCR is three bits and the driver names the seven dividers it
// encodes (RM 7.3.2); a code past them is refused where the setting is
// a constant.
#include "ch32v203/watchdog.hpp"

void f() {
    (void)brio::Iwdg::configure<brio::IwdgConfig{
        .prescaler = static_cast<brio::IwdgPrescaler>(7), .reload = 100}>();
}
