// Pin family smoke TU: the four ports of the family, the CNF/MODE nibble
// (MODE one bit, the second reserved), the pull through OUTDR and every
// verb - and the PwmChannel role a bare pin plays.
#include "ch32v00x/pin.hpp"
#include "util/pwm_channel.hpp"
#include "util/rgb_lamp.hpp"

using namespace brio;

static_assert(gpio_base_for('A') == 0x40010800 && gpio_base_for('D') == 0x40011400);
static_assert(gpio_base_for('E') == 0);
static_assert(gpio_clock_for('C') == rcc_pb2_gpioc);

// RM 7.3.1.1: CNF above a ONE-BIT MODE.
static_assert(pin_nibble(PinMode::analog, PinDrive::push_pull) == 0x0);
static_assert(pin_nibble(PinMode::input, PinDrive::push_pull) == 0x4);      // floating, the reset state
// The output MODE code is the part's: one bit on the CH32V006, the
// F1's two with the top speed on the CH32V003.
#if defined(CH32V006)
static_assert(pin_nibble(PinMode::output, PinDrive::push_pull) == 0x1);
static_assert(pin_nibble(PinMode::output, PinDrive::open_drain) == 0x5);
static_assert(pin_nibble(PinMode::alternate, PinDrive::push_pull) == 0x9);
static_assert(pin_nibble(PinMode::alternate, PinDrive::open_drain) == 0xD);
#else
static_assert(pin_nibble(PinMode::output, PinDrive::push_pull) == 0x3);
static_assert(pin_nibble(PinMode::output, PinDrive::open_drain) == 0x7);
static_assert(pin_nibble(PinMode::alternate, PinDrive::push_pull) == 0xB);
static_assert(pin_nibble(PinMode::alternate, PinDrive::open_drain) == 0xF);
#endif

using Led = Pin<'C', 0>;
using Sw = Pin<'D', 4>;
using Tx = Pin<'D', 5>;
#if defined(CH32V006)
using Last = Pin<'B', 7>;
#else
using Last = Pin<'C', 7>;   // the CH32V003 has no port B (a neg TU proves the refusal)
#endif

static_assert(Led::mask == 0x01 && Last::mask == 0x80);
static_assert(Led::port_letter == 'C' && Led::pin_number == 0);
static_assert(PwmChannel<Led>);
static_assert(Led::max == 1);

using Lamp = RgbLamp<Pin<'A', 0>, Pin<'A', 1>, Pin<'A', 2>>;

void pin_verbs() {
    Led::output();
    Led::output(true);
    Led::output(false, PinDrive::open_drain);
    Led::set();
    Led::clear();
    Led::toggle();
    (void)Led::read();
    (void)Led::read_out();
    Led::duty(1);
    Sw::input();
    Sw::input(PinPull::up);
    Sw::input(PinPull::down);
    Sw::analog();
    Tx::function();
    Tx::function(PinDrive::open_drain);
    Tx::release();
#if defined(CH32V006)
    Port<'B'>::clock_on();
    Port<'B'>::out_set(0x0F);
    Port<'B'>::out_clear(0x0F);
    Port<'B'>::out_toggle(0x0F);
    (void)Port<'B'>::in();
#endif
    Lamp::show(Rgb{255, 0, 0});
    Lamp::off();
}
