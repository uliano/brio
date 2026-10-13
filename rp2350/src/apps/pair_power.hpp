// pair_power.hpp - THE PARTNER'S RAIL, read before a link pad is driven.
//
// The WeAct RP2350B is wired to a WeAct RP2040 (docs/boards/
// weact-rp2350b.md): ten link wires, each board on its own USB-C, no 3V3
// between them. The asymmetry this file exists for is in the two
// datasheets: an RP2350 pad is 3.3 V-failsafe while its chip is
// unpowered (RP2350 datasheet chapter 1), an RP2040 pad is NOT - its
// input may not rise above IOVDD + 0.5 V (RP2040 datasheet 5.5.3.1). So
// an RP2350 image that drives a link pad high - UART1's transmitter
// idling, GPOUT0, a select or a clock - while the RP2040 is unpowered
// feeds that chip through its pad's protection diode. The other way
// round is safe.
//
// THE SENSE. The I2C link's two lines, GP36 (SDA) and GP37 (SCL), carry
// 1.5 kOhm pull-ups to the RP2040's 3V3 pin and to nothing else. With
// that rail up a line reads high; with it down it reads low. Each line is
// read with the pad's own PULL-DOWN on and its input buffer OFF, enabled
// for the read alone (`Pin::read_pulsed()`): erratum RP2350-E9 makes a pad
// whose buffer is on hold history against its pull-down, and the
// errata sheet's workaround is exactly this pair (rp2350/pin.hpp). The
// read drives nothing: an unpowered RP2040 is never fed by it, a powered
// one sees no edge on its bus.
//
// THE SETTLE IS DERIVED, NOT GUESSED. The slow case is the rail DOWN: a
// line pulled to ground only by the pad's pull-down, 113 kOhm at most
// (RP2350 datasheet 14.9's 36..113), discharging the net - two pads, a
// 15 cm wire and the RP2040's pad, some 35 pF - with tau = 113 kOhm x 35
// pF = 4 us. The first sample waits 25 us, six tau: a line that started
// at the rail is then under 10 mV. The rail-up case is the 1.5 kOhm
// against that pull-down - 3.3 V x 113/114.5 - with tau = 53 ns.
//
// ONE LINE HIGH IS ENOUGH, and each is sampled sixteen times 3 us apart:
// both pull-ups hang on the one rail, and a powered RP2040 working its
// bus (a host clocking, a client stretching) can hold either line low at
// a given instant - but not both through 45 us of a 100 kHz bus, whose
// SCL high phase is 5 us against samples 3 us apart. A dead rail never
// reads high: nothing on the net can lift it.
//
// THE RULE, for every RP2350 image of this repository that drives a link
// pad (GP4, GP21, GP28, GP30, GP31, GP36, GP37, GP41): the rail first - a
// letter registered through `guarded`, or `partner_rail()` /
// `partner_rail_up()` called before the pad is handed over - and a rail
// read down leaves the pad at reset and says so. The sense's own two pads
// are left released (isolated, the reset pull-down) whatever it read.

#pragma once

#include <stdint.h>

#include "rp2350/delay.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"

namespace pair {

/// What the sense read: whether each line was ever high over the window.
struct PartnerRail {
    bool sda = false;
    bool scl = false;
    /// The RP2040's 3V3 is up: either pull-up lifted its line.
    constexpr bool up() const { return sda || scl; }
};

/// Read the partner's rail through the I2C link's pull-ups. Costs about
/// 70 us; starts the platform timer if nothing has (rp2350/delay.hpp's
/// one prerequisite). The two pads are released before it returns.
template <typename C>
PartnerRail partner_rail(C clock) {
    using Sda = brio::Pin<36>;
    using Scl = brio::Pin<37>;
    if (!brio::Mtime::running()) {
        (void)brio::Mtime::start(clock);
    }
    (void)Sda::input({.pull = brio::PinPull::down, .input_enable = false});
    (void)Scl::input({.pull = brio::PinPull::down, .input_enable = false});
    PartnerRail r{};
    // Sixteen samples, 3 us apart once the first has waited out the
    // settle's first half: 25 + 15 x 3 = 70 us in all.
    (void)brio::delay_us(clock, 25);
    for (uint8_t k = 0; k < 16u; ++k) {
        r.sda = Sda::read_pulsed() || r.sda;
        r.scl = Scl::read_pulsed() || r.scl;
        (void)brio::delay_us(clock, 3);
    }
    (void)Sda::release();
    (void)Scl::release();
    return r;
}

/// The guard: true when the RP2040's rail is up and a link pad may be
/// driven; false otherwise, the reason printed on `out` through `say`.
template <typename C, typename Say>
bool partner_rail_up(C clock, Say say) {
    const PartnerRail r = partner_rail(clock);
    if (!r.up()) {
        say("  THE PARTNER IS UNPOWERED: GP36/GP37 read low through their pull-ups to the RP2040's 3V3 "
            "- no link pad is driven (an RP2040 pad is not failsafe)");
    }
    return r.up();
}

/// A letter that drives a link pad, registered through the guard: the
/// rail read first, and with it down one failing verdict that says why
/// and the letter itself not run. `bench` is the image's TestBench and
/// `clock` its clock object, both at namespace scope.
template <auto& bench, auto& clock, void (*letter)()>
void guarded() {
    if (partner_rail(clock).up()) {
        letter();
        return;
    }
    bench.verdict("THE PARTNER BOARD IS POWERED (GP36/GP37 read high through their pull-ups to its 3V3) "
                  "- else no link pad is driven, an RP2040 pad not being failsafe",
                  false);
}

}  // namespace pair
