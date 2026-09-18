// clock_peer - THE FIXED-ROLE FAR END OF A CLOCK LINK, for the clock
// suite of a board on the other side of two wires.
//
// It has no console and needs none: its whole job is to put its own
// crystal on a pin and to count what arrives on another, so that two
// boards measure each OTHER'S crystal - the one external reference this
// bench has, since a chip counting its own oscillators can only ever
// prove their ratios.
//
//   GP21 = GPOUT0 carries this board's crystal divided by 12, 1 MHz
//   GP20 = GPIN0 is counted against this board's crystal, over the
//          longest window the counter has (32 ms), for ever
//
// WHAT IT REPORTS, AND HOW IT IS READ. Two globals, refreshed every lap:
// `peer_gpin_hz`, the last count in hertz (0 when nothing arrives), and
// `peer_laps`, which says the program is alive. A debugger reads them
// out of the ELF while the program runs - no console, no protocol, and
// nothing for the other board to synchronize with. The LED blinks once
// a lap for a human.
//
// WIRING: this board's GP21 to the other board's clock input, the other
// board's clock output to this board's GP20, and a common ground.
//
// build: boards = pico,picow,weact2040

#include <stdint.h>

#include <optional>

#include "rp2040/clock.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/timer.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

using Led = Pin<25>;

}  // namespace

/// The last count of what arrives on GP20, in hertz; 0 when the source
/// is dead. Volatile because a debugger is the reader and nothing here
/// reads it back.
volatile uint32_t peer_gpin_hz = 0;
/// Laps since boot: the proof that the program is running.
volatile uint32_t peer_laps = 0;

int main() {
    (void)SysClock::init();
    (void)Timer::init(clock);
    Led::output();

    // Out: this board's crystal / 12 = 1 MHz on GP21.
    (void)ClockOut<0>::init(GpoutSource::xosc, 12);
    // In: whatever the other board drives onto GP20.
    ClockIn<0>::init();

    uint32_t laps = 0;
    for (;;) {
        const std::optional<uint32_t> hz = SysClock::count_hz(ClockIn<0>::count_source);
        peer_gpin_hz = hz ? *hz : 0u;
        peer_laps = ++laps;
        Led::toggle();
        const uint32_t t0 = Timer::now_low();
        while (Timer::now_low() - t0 < 200'000u) {
        }
    }
}
