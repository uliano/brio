// family_probe - the smallest firmware that is meant to run on EVERY
// package of the stratum: it toggles PA7 at ~2 Hz on the internal
// oscillator and touches nothing else. PA7 is bonded out on every
// package of the AVR DA, DB and EA alike, so the same source builds and
// blinks on all of them.
//
// Two jobs. On the desk it is the carrier of the board matrix (the
// "// build: boards" line below): a build of it for every board TYPE
// proves the matrix, no hardware needed. On the bench it is the first
// thing flashed onto a NEW board: an LED (or a scope) on PA7 says the
// chip runs, the UPDI link works and the fuses are sane, before any
// suite is trusted on it.
//
// Wiring: PA7 -> ~330 ohm -> LED -> GND (or a probe on PA7).

// build: boards = db28,db32,db48,ea48

#include "avrxt/clock.hpp"
#include "avrxt/delay.hpp"
#include "avrxt/pin.hpp"

// The internal oscillator at a rate every part reaches: 4 MHz by FRQSEL
// on the AVR DA/DB; on the AVR EA, where the OSCCFG fuse chooses OSCHF's
// rate, the factory 20 MHz over 4 (5 MHz). A part fused otherwise keeps
// its reset clock (init() says false) and blinks at another rate - still
// the sign that it runs.
using SysClock = brio::Clock<brio::ClockSource::internal,
                             brio::has_oschf_frqsel ? 4'000'000 : 20'000'000,
                             brio::has_oschf_frqsel ? brio::ClockDiv::div1 : brio::ClockDiv::div4>;
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
