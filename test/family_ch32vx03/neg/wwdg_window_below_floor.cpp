// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A window below 0x40 has no legal refresh instant in it: the counter
// resets on the step from 0x40 to 0x3F, so a refresh can never happen
// while the counter is at or below such a window (RM 8.2.1).
#include "ch32vx03/watchdog.hpp"

void f() { (void)brio::Wwdg::configure<brio::WwdgConfig{.window = 0x20}>(); }
