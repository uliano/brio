// Pin family smoke TU: the GPIO nibbles of this series (no speed, no
// open drain), the three configuration registers of a 24-pin port with
// CFGHR configured through its RAM copy, the set/reset halves, the pulls
// with the pull-down that only some pads have, the lock, and the pads a
// part bonds and the ones it shorts together - on every part, each fact
// read from device:: rather than from the part definition. PA0..PA7 and
// the debug port's PC18/PC19 are bonded on every package.
#include "ch32x035/pin.hpp"
#include "util/pwm_channel.hpp"

using namespace brio;

// ---- the nibbles (RM 8.3.1.1) ------------------------------------------------
static_assert(pin_nibble_analog == 0x0 && pin_nibble_floating == 0x4 && pin_nibble_pulled == 0x8);
static_assert(pin_nibble_output == 0x1 && pin_nibble_alternate == 0x9);
static_assert(pin_nibble_drives(pin_nibble_output) && pin_nibble_drives(pin_nibble_alternate));
static_assert(!pin_nibble_drives(pin_nibble_floating) && !pin_nibble_drives(pin_nibble_pulled));

// ---- the pads -----------------------------------------------------------------
static_assert(pad_bonded(Pad{'A', 0}) && pad_bonded(Pad{'C', 19}));
static_assert(!pad_bonded(Pad{}) && !pad_bonded(Pad{'A', 24}) && !pad_bonded(Pad{'D', 0}));
static_assert(pad_has_pull_down(Pad{'A', 15}) && pad_has_pull_down(Pad{'C', 17}));
static_assert(!pad_has_pull_down(Pad{'A', 16}) && !pad_has_pull_down(Pad{'B', 0}) &&
              !pad_has_pull_down(Pad{'C', 18}));
// PC16 shares its pin with PC11 everywhere but on the CH32X035F8U6.
static_assert(pad_twinned(Pad{'C', 16}) == (device::package != Package::qfn20));
static_assert(pad_can_drive(Pad{'C', 16}) == (device::package == Package::qfn20));

using Led = Pin<'A', 0>;
using Probe = Pin<'C', 19>;
static_assert(PwmChannel<Led>);
static_assert(Led::mask == 1u && Led::port_letter == 'A' && Led::pin_number == 0);
static_assert(Led::can_drive && Led::has_pull_down);
static_assert(Probe::can_drive && !Probe::has_pull_down);
static_assert(Port<'A'>::bonded == device::port_pins('A'));

void pin_verbs() {
    Led::output();
    Led::output(true);
    Led::set();
    Led::clear();
    Led::toggle();
    (void)Led::read();
    (void)Led::read_out();
    Led::duty(1);
    (void)Led::input();
    (void)Led::input(PinPull::up);
    (void)Led::input(PinPull::down);
    Led::input<PinPull::down>();
    Led::analog();
    Led::function();
    Led::release();
    (void)Led::nibble();
    (void)Led::lock();
    (void)Led::locked();
    const PinRef r = Led::ref();
    r.set();
    r.clear();
    r.write(true);
    r.toggle();
    (void)r.read();
    (void)r.valid();

    // A pad with no pull-down: the run-time face answers false.
    (void)Probe::input(PinPull::down);
    Probe::input<PinPull::up>();
}

void port_verbs() {
    using A = Port<'A'>;
    A::clock_on();
    (void)A::in();
    (void)A::out();
    A::out_write(0x00000055u);
    A::out_set(0x00000003u);
    A::out_clear(0x00000003u);
    A::out_toggle(0x00000003u);
    A::configure(3, pin_nibble_pulled);
    (void)A::nibble(3);
    (void)A::cfghr_copy();
    (void)A::cfghr_register();
    (void)A::configure_pins(0x000000F0u, pin_nibble_floating);
    (void)A::lock(0x00000001u);
    (void)A::lock<0x00000001u>();
    (void)A::locked();
    (void)A::locked_pins();

    using C = Port<'C'>;
    C::out_set(1UL << 18);      // BSXR's half
    C::out_toggle(3UL << 18);
    C::configure(18, pin_nibble_floating);   // CFGXR
    (void)C::nibble(19);
}

// A pad in port A's upper half (8..15, CFGHR) or high byte (16..23, CFGXR)
// where the package bonds one - a template, so the branch a package has
// not got is never formed.
template <char L, uint8_t N>
void drive_where_bonded() {
    if constexpr (pad_can_drive(Pad{L, N})) {
        using X = Pin<L, N>;
        X::output(false);
        X::toggle();
        (void)X::input(PinPull::up);
        (void)X::nibble();
    }
}

void upper_pads() {
    drive_where_bonded<'A', 10>();   // CFGHR, through its copy
    drive_where_bonded<'A', 20>();   // CFGXR and BSXR
    drive_where_bonded<'B', 12>();
    drive_where_bonded<'C', 16>();   // the USB pad: drivable on the QFN20 alone
}
