// Pin family smoke TU: ports A..D and F exist on every STM32G0 (the
// device header's GPIOx_BASE, read through the reserve), so this TU
// instantiates one pin of each; port E is the G0B1 class's and is
// neg/pin_port_e_off_the_g0b1.cpp's business.
#include "stm32g0/pin.hpp"

using namespace brio;

static_assert(port_exists('A') && port_exists('B') && port_exists('C') &&
              port_exists('D') && port_exists('F'));
static_assert(!port_exists('G') && !port_exists('Z'));
static_assert(gpio_port_clock_mask('A') == RCC_IOPENR_GPIOAEN);
static_assert(gpio_port_clock_mask('Z') == 0);

using Led = Pin<'A', 5>;
using Tx = Pin<'A', 2>;
using B0 = Pin<'B', 0>;
using C13 = Pin<'C', 13>;
using D2 = Pin<'D', 2>;
using F1 = Pin<'F', 1>;

static_assert(Led::mask == (1u << 5));
static_assert(Led::max == 1);
static_assert(PinSel{'A', 2, PinFunction::af1}.valid());
static_assert(!PinSel{'A', 16, PinFunction::af1}.valid());
static_assert(!PinSel{'G', 0, PinFunction::af1}.valid());

void pin_verbs() {
    Led::output();
    Led::output(true);
    Led::set();
    Led::clear();
    Led::toggle();
    (void)Led::read();
    (void)Led::read_out();
    (void)Led::is_output();
    Led::duty(1);
    Led::input(PinPull::up);
    Led::pull(PinPull::down);
    Led::analog();
    Led::release();
    Tx::function(PinFunction::af1, {.speed = PinSpeed::high});
    (void)Tx::has_function();
    B0::output({.open_drain = true});
    C13::input(PinPull::none);
    D2::output();
    F1::output();
    const PinRef r = Led::ref();
    r.set();
    r.clear();
    (void)r.valid();

    Port<'A'>::clock(true);
    (void)Port<'A'>::clock();
    (void)Port<'A'>::in();
    (void)Port<'A'>::out();
    Port<'A'>::out_set(0x3);
    Port<'A'>::out_clear(0x3);
    Port<'A'>::out_toggle(0x3);
    Port<'A'>::configure_mask(0x3, PinMode::alternate, {.pull = PinPull::up}, PinFunction::af1);
    PinSet<Led, B0>::configure(PinMode::input);
}
