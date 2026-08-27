// family_probe - the smallest firmware that is meant to run on EVERY
// package of the family: it toggles PA7 at ~2 Hz on the internal
// oscillator and touches nothing else. PA7 is bonded out on the 28-,
// 32-, 48- and 64-pin parts alike, so the same source builds and blinks
// on all of them.
//
// Two jobs. On the desk it is the carrier of the board matrix (the
// "// build: boards" line below): a build of it for every board TYPE
// proves the matrix, no hardware needed. On the bench it is the first
// thing flashed onto a NEW board: an LED (or a scope) on PA7 says the
// chip runs, the UPDI link works and the fuses are sane, before any
// suite is trusted on it.
//
// Wiring: PA7 -> ~330 ohm -> LED -> GND (or a probe on PA7).

// build: boards = db28,db32,db48

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 4'000'000>;
constexpr SysClock clock;

using Probe = brio::Pin<'A', 7>;

int main() {
    SysClock::init();
    Probe::output();
    for (;;) {
        Probe::toggle();
        brio::delay_us(clock, 250'000);
    }
}
