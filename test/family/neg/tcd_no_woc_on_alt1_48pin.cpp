// mcu: avr128db48 avr128da48
// The 48-pin headers bond ALT1 as "WOx: PB4, PB5, -, -": WOC and WOD are
// PINLESS there. The route stays usable for the complementary pair; a
// configuration that enables WOC on it is refused.
#include "avrdx/tcd.hpp"
using namespace brio;
void f() {
    (void)Tcd<0>::init<TcdConfig{.route = TcdRoute::alt1, .enable_woa = true,
                                 .enable_woc = true}>();
}
