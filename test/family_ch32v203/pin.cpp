// Pin family smoke TU: the F1's four-bit nibble over two registers, the
// pull that lives in the output register, every verb of a port and of a
// pin, and the PwmChannel role a bare pin plays.
//
// WHICH PAD a TU may name is the part's, and this is where the nine
// packages differ most: port A is whole on some and full of holes on
// others, port B keeps between two and sixteen pins, and ports C and D
// reach no pad at all on two of them. So no pad is spelled as a literal
// here - the table is asked for the lowest one each port bonds, which is
// also what proves that asking works.
#include "ch32v203/pin.hpp"
#include "util/pwm_channel.hpp"
#include "util/rgb_lamp.hpp"

using namespace brio;

static_assert(gpio_base_for('A') == 0x40010800 && gpio_base_for('D') == 0x40011400);
static_assert(gpio_base_for('Z') == 0);

// RM 10.1.1: CNF above a TWO-BIT MODE, the F1's own nibble - where the
// CH32V00x's MODE is one bit wide.
static_assert(pin_nibble(PinMode::analog, PinDrive::push_pull, PinSpeed::fast) == 0x0);
static_assert(pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast) == 0x4);
static_assert(pin_nibble(PinMode::output, PinDrive::push_pull, PinSpeed::fast) == 0x3);
static_assert(pin_nibble(PinMode::output, PinDrive::open_drain, PinSpeed::fast) == 0x7);
static_assert(pin_nibble(PinMode::alternate, PinDrive::push_pull, PinSpeed::fast) == 0xB);
static_assert(pin_nibble(PinMode::alternate, PinDrive::open_drain, PinSpeed::fast) == 0xF);
static_assert(pin_nibble(PinMode::output, PinDrive::push_pull, PinSpeed::slow) == 0x2);
static_assert(pin_nibble(PinMode::output, PinDrive::push_pull, PinSpeed::medium) == 0x1);
static_assert(pin_mode_code(PinSpeed::fast) == 0x3);

/// The lowest pin of a port this package bonds. Every part bonds
/// something of A and of B; C and D are the packages' own business.
constexpr uint8_t first_pin(char port) {
    for (uint8_t n = 0; n < 16u; ++n) {
        if ((device::port_pins(port) & static_cast<uint16_t>(1u << n)) != 0u) {
            return n;
        }
    }
    return 0;
}

static_assert(device::has_port('A') && device::has_port('B'));

// The bonding as a question about ONE pad, which is what the remap
// columns of afio.hpp are judged by: the lowest pin of a port this part
// bonds is bonded, a port this part has not got bonds nothing, and a
// pad number beyond the sixteen is not a pad.
static_assert(pad_bonded(Pad{'A', first_pin('A')}));
static_assert(!pad_bonded(Pad{}));
static_assert(pad_bonded(Pad{'E', 0}) == device::has_port('E'));
static_assert(pad_bonded(Pad{'C', 13}) == device::has_port('C'));

using Led = Pin<'A', first_pin('A')>;
using Sw = Pin<'B', first_pin('B')>;

static_assert(Led::port_letter == 'A' && Led::pin_number == first_pin('A'));
static_assert(Led::mask == (1UL << first_pin('A')));
static_assert(Led::pad.valid() && Led::pad.port == 'A');
static_assert(!Pad{}.valid());
static_assert(PwmChannel<Led>);
static_assert(Led::max == 1);

using Lamp = RgbLamp<Pin<'A', 0>, Pin<'A', 1>, Pin<'A', 2>>;

/// Every verb of one port and its lowest pin. A template so the pad is
/// formed only for a port this package has - which is what lets the
/// absent ones fall back to A below.
template <char L>
void port_verbs() {
    using P = Port<L>;
    using Pad0 = Pin<L, first_pin(L)>;

    static_assert(P::bonded == device::port_pins(L));

    P::clock_on();
    (void)P::in();
    (void)P::out();
    P::out_set(Pad0::mask);
    P::out_clear(Pad0::mask);
    P::out_toggle(Pad0::mask);
    P::out_write(0);
    P::configure(first_pin(L), pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast));
    P::configure_pins(P::bonded, pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast));
    (void)P::nibble(first_pin(L));
    // The lock, in both faces - the mask checked at compile time and the
    // one checked at run time - and its two read-backs. This TU is
    // compiled and never run, which is the only reason a one-way verb
    // can be named here at all.
    (void)P::template lock<Pad0::mask>();
    (void)P::lock(Pad0::mask);
    (void)P::locked();
    (void)P::locked_pins();
    (void)Pad0::lock();
    (void)Pad0::locked();
    (void)Pad0::nibble();

    Pad0::output();
    Pad0::output(true);
    Pad0::output(false, PinDrive::open_drain, PinSpeed::slow);
    Pad0::set();
    Pad0::clear();
    Pad0::toggle();
    (void)Pad0::read();
    (void)Pad0::read_out();
    Pad0::duty(1);
    Pad0::duty(0);
    Pad0::input();
    Pad0::input(PinPull::up);
    Pad0::input(PinPull::down);
    Pad0::analog();
    Pad0::function();
    Pad0::function(PinDrive::open_drain, PinSpeed::medium);
    Pad0::release();

    const PinRef ref = Pad0::ref();
    ref.set();
    ref.clear();
    ref.write(true);
    ref.toggle();
    (void)ref.read();
    (void)ref.valid();
}

void pin_verbs() {
    port_verbs<'A'>();
    port_verbs<'B'>();
    // The two ports whose presence is the package's: a part without them
    // exercises port A twice rather than naming a pad it has not got.
    port_verbs<device::has_port('C') ? 'C' : 'A'>();
    port_verbs<device::has_port('D') ? 'D' : 'A'>();

    Lamp::show(Rgb{255, 0, 255});
    Lamp::off();

    // A null reference drives nothing and reads low.
    const PinRef none;
    none.set();
    none.clear();
    none.write(true);
    none.toggle();
    (void)none.read();
}
