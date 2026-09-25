// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A WAVE GENERATOR WITH NO TRIGGER. The noise and triangle generators step
// three PB1 cycles after each trigger event and on nothing else - 17.2.4's
// note 1 and 17.2.5's note make TENx their precondition - so a generator
// on an untriggered channel would never move: refused.
#include "ch32vx03/dac.hpp"

void f() { brio::Dac::configure<2, brio::DacChannelConfig{.wave = brio::DacWave::noise}>(); }
