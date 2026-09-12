// GPIO family smoke TU: the bank, the pad arithmetic, Pin as a type on
// the first and the last pin of the bank.
#include "rp2040/pin.hpp"

using namespace brio;

static_assert(gpio_count == 30u);
static_assert(pad_value({}) == (PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_SCHMITT_BITS |
                                (1u << PADS_BANK0_GPIO0_DRIVE_LSB)));
static_assert((pad_value({.pull = PinPull::keeper}) &
               (PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS)) ==
              (PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS));
static_assert(PinSel{29, PinFunction::sio}.valid());
static_assert(!PinSel{30, PinFunction::sio}.valid());

using First = Pin<0>;
using Last = Pin<29>;
static_assert(Last::mask == (1u << 29));
static_assert(First::max == 1u);

void pin_verbs() {
    (void)Gpio::ready();
    (void)Gpio::in();
    (void)Gpio::out();
    Gpio::out_set(1u);
    Gpio::out_clear(1u);
    Gpio::out_toggle(1u);
    (void)Gpio::oe();
    Gpio::oe_set(1u);
    Gpio::oe_clear(1u);
    Gpio::function(3, PinFunction::pwm, {.drive = PinDrive::ma12, .slew_fast = true});
    Gpio::release(3);
    Gpio::outputs(0x3ffffffcu);
    First::output();
    First::output(true, {.pull = PinPull::none});
    Last::input(PinPull::up);
    Last::function(PinFunction::uart);
    Last::release();
    First::set();
    First::clear();
    First::toggle();
    (void)First::read();
    (void)First::read_out();
    (void)First::is_output();
    (void)First::function();
    First::pull(PinPull::keeper);
    First::duty(1);
}
