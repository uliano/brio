// mcu: samc21e18a samc21g18a samc21j18a
// 41.6.8.3: dithering IS the event-driven mode - sixteen START events
// per value, DATABUF reloaded every sixteen. Without EVCTRL.STARTEI the
// sixteen sub-conversions cannot happen at all.

#include "samc/dac.hpp"

using namespace brio;

constexpr DacConfig bad_cfg{
    .dither = true,
};

void use() { (void)Dac::init<bad_cfg>(0); }
