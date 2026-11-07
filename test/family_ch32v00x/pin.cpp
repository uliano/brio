// Pin family smoke TU, every package: the four ports of the family, the
// CNF/MODE nibble (MODE one bit, the second reserved), the pull through
// OUTDR and every verb - and the PwmChannel role a bare pin plays. The
// pads are picked from the package's own table (the first pads it lets
// a program drive, a pad it shares), and each package's table is pinned
// against the datasheet's pin table here.
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

// ---- the package tables, pinned (CH32V006 DS V2.0 table 2-1-1, CH32V003
// DS V1.6 table 2-1): the masks, the shared pads, and the count of pins
// that carry a pad against the model table's GPIO column.
constexpr uint32_t pads_of(uint32_t m) {
    uint32_t n = 0;
    for (; m != 0u; m &= m - 1u) { ++n; }
    return n;
}
constexpr uint32_t bonded_pads() {
    return pads_of(device::port_pins('A')) + pads_of(device::port_pins('B')) + pads_of(device::port_pins('C')) +
           pads_of(device::port_pins('D'));
}
constexpr uint32_t twinned_pads() {
    return pads_of(device::port_twinned('A')) + pads_of(device::port_twinned('B')) +
           pads_of(device::port_twinned('C')) + pads_of(device::port_twinned('D'));
}
// A group of n twinned pads takes one pin: one pair on every package but
// the CH32V003's SOP8, whose pins 1 and 8 carry two and three pads.
constexpr uint32_t pins_saved() {
    return device::package == Ch32Package::pins8 ? 3u : twinned_pads() / 2u;
}
static_assert(bonded_pads() - pins_saved() == device::gpio_count);
static_assert((device::port_twinned('A') & ~device::port_pins('A')) == 0u &&
              (device::port_twinned('D') & ~device::port_pins('D')) == 0u);
#if defined(CH32V006K8)
static_assert(device::port_pins('A') == 0xFF && device::port_pins('B') == 0x7F && device::gpio_count == 31);
static_assert(!pad_bonded(Pad{'B', 7}) && twinned_pads() == 0u);
#elif defined(CH32V006E8)
static_assert(device::port_pins('B') == 0x0B && device::port_pins('C') == 0x37 && device::gpio_count == 22);
static_assert(pad_twinned(Pad{'A', 1}) && pad_twinned(Pad{'A', 6}) && !pad_bonded(Pad{'C', 6}));
#elif defined(CH32V006F8)
static_assert(device::port_pins('A') == 0x16 && device::port_pins('B') == 0 && device::gpio_count == 18);
static_assert(pad_twinned(Pad{'A', 4}) && pad_twinned(Pad{'D', 7}) && !pad_bonded(Pad{'A', 0}) &&
              pad_can_drive(Pad{'C', 0}) && pad_can_drive(Pad{'C', 4}));
#elif defined(CH32V006D8)
static_assert(device::port_pins('C') == 0xD9 && device::port_pins('D') == 0x93 && device::gpio_count == 11);
#elif defined(CH32V003F4)
static_assert(device::port_pins('A') == 0x06 && device::port_pins('C') == 0xFF && device::gpio_count == 18);
#elif defined(CH32V003A4)
static_assert(device::port_pins('C') == 0xDF && device::port_pins('D') == 0xF2 && device::gpio_count == 14);
#elif defined(CH32V003J4)
static_assert(device::port_pins('C') == 0x16 && pad_twinned(Pad{'D', 1}) && pad_twinned(Pad{'A', 1}) &&
              device::gpio_count == 6);
#endif

// ---- the pads this TU drives, from the package's own table ----------------
/// The k-th pad of the package a program may drive, in port then pin order.
constexpr Pad drivable(uint32_t k) {
    for (char port = 'A'; port <= 'D'; ++port) {
        for (uint8_t pin = 0; pin < 8u; ++pin) {
            if (pad_can_drive(Pad{port, pin}) && k-- == 0u) {
                return Pad{port, pin};
            }
        }
    }
    return Pad{};
}
/// The first pad that shares its package pin, or the first drivable pad
/// where the package shares none.
constexpr Pad first_twin() {
    for (char port = 'A'; port <= 'D'; ++port) {
        for (uint8_t pin = 0; pin < 8u; ++pin) {
            if (pad_twinned(Pad{port, pin})) {
                return Pad{port, pin};
            }
        }
    }
    return drivable(0);
}

using Led = Pin<drivable(0).port, drivable(0).pin>;
using Sw = Pin<drivable(1).port, drivable(1).pin>;
using Tx = Pin<drivable(2).port, drivable(2).pin>;
using Twin = Pin<first_twin().port, first_twin().pin>;

static_assert(Led::port_letter == drivable(0).port && Led::mask == (1UL << drivable(0).pin));
static_assert(Led::can_drive && (Twin::can_drive == (twinned_pads() == 0u)));
static_assert(PwmChannel<Led>);
static_assert(Led::max == 1);

using Lamp = RgbLamp<Led, Sw, Tx>;

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
    // A shared pad is an input and an analog pad, never an output.
    Twin::input(PinPull::up);
    Twin::analog();
    Twin::release();
    (void)Twin::read();
#if defined(CH32V006)
    // Port B's registers are the die's, whatever the package bonds of it.
    Port<'B'>::clock_on();
    Port<'B'>::out_set(0x0F);
    Port<'B'>::out_clear(0x0F);
    Port<'B'>::out_toggle(0x0F);
    (void)Port<'B'>::in();
#endif
    Lamp::show(Rgb{255, 0, 0});
    Lamp::off();
}
