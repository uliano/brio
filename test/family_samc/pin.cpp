// PORT/pin family smoke TU. Both port groups, the pin vocabulary and
// the multi-pin engine must instantiate on every variant - the PORT
// peripheral is the same two groups everywhere in this family, so a
// group letter that compiles here compiles on all of them (which is
// what neg/pin_no_portc.cpp states from the other side).
#include "samc/pin.hpp"

using namespace brio;

static_assert(port_exists('A'));
static_assert(port_exists('B'));
static_assert(!port_exists('C'));
static_assert(!port_exists('a'));
static_assert(Pin<'B', 23>::mask == (1u << 23));
static_assert(PwmChannel<Pin<'B', 23>>);

void pin_common() {
    using Led = Pin<'B', 23>;
    Led::output();
    Led::set();
    Led::clear();
    Led::toggle();
    Led::duty(1);
    (void)Led::read();
    (void)Led::is_output();
    (void)Led::ref();

    using Button = Pin<'B', 22>;
    Button::input(PinPull::up);
    Button::pull(PinPull::down);
    Button::pull(PinPull::none);
    Button::configure({.input_enable = true, .pull = PinPull::up});
    Button::configure({});               // buffer off, no pull, PORT-owned
    Button::input_enable(false);
    Button::strong_drive(true);

    // The peripheral handoff, both PMUX nibbles (even and odd pin).
    Pin<'B', 30>::function(PinFunction::d, {.input_enable = true});
    Pin<'B', 31>::function(PinFunction::d, {.input_enable = true});
    (void)Pin<'B', 30>::has_function();
    Pin<'B', 30>::release();
}

void port_resource() {
    using PA = Port<'A'>;
    PA::dir_set(0x0000'000Cu);
    PA::dir_clear(0x0000'0004u);
    PA::dir_toggle(0x0000'0008u);
    PA::out_set(0x0000'0004u);
    PA::out_clear(0x0000'0004u);
    PA::out_toggle(0x0000'0004u);
    (void)PA::in();
    (void)PA::dir();
    (void)PA::out();

    // A mask spanning both WRCONFIG half-words: two stores, one call.
    PA::configure_mask(0x0001'0001u, {.input_enable = true, .pull = PinPull::up});
    PA::function_mask(0x00C0'0000u, PinFunction::c, {.input_enable = true});
    static_assert(Port<'B'>::group == 1);
}
