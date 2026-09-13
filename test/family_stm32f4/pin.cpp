// Pin family smoke TU: ports A, B, C and H exist on every STM32F4 (the
// device header's GPIOx_BASE, read through the reserve), so this TU
// instantiates one pin of each; D, E, F, G, I, J and K are the bigger
// bondings' and are asked of the header before being touched (the
// negatives hold the refusals).
#include "stm32f4/pin.hpp"

using namespace brio;

static_assert(port_exists('A') && port_exists('B') && port_exists('C') && port_exists('H'));
static_assert(!port_exists('L') && !port_exists('Z'));
static_assert(gpio_port_clock_mask('A') == RCC_AHB1ENR_GPIOAEN);
static_assert(gpio_port_clock_mask('Z') == 0);

using Led = Pin<'A', 5>;
using Tx = Pin<'A', 9>;
using B0 = Pin<'B', 0>;
using C13 = Pin<'C', 13>;
using H1 = Pin<'H', 1>;

static_assert(Led::mask == (1u << 5));
static_assert(Led::max == 1);
static_assert(PinSel{'A', 9, PinFunction::af7}.valid());
static_assert(!PinSel{'A', 16, PinFunction::af7}.valid());
static_assert(!PinSel{'L', 0, PinFunction::af7}.valid());

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
    Tx::function(PinFunction::af7, {.speed = PinSpeed::very_high});
    (void)Tx::has_function();
    B0::input(PinPull::up);
    B0::pull(PinPull::down);
    C13::analog();
    H1::input();
    H1::release();
    const PinRef r = Led::ref();
    r.set();
    r.clear();
    (void)r.valid();
    PinRef none{};
    none.set();
    PinSet<Led, B0, C13>::configure(PinMode::input, {.pull = PinPull::none});
    Port<'A'>::clock(true);
    (void)Port<'A'>::clock();
    (void)Port<'A'>::in();
    (void)Port<'A'>::out();
    Port<'A'>::out_set(0x00FF);
    Port<'A'>::out_clear(0x00FF);
    Port<'A'>::out_toggle(0x0003);
}

// The bigger bondings' ports, where the header has them.
#if defined(GPIOD_BASE)
static_assert(port_exists('D'));
void port_d() { Pin<'D', 2>::release(); }
#else
static_assert(!port_exists('D'));
#endif
#if defined(GPIOE_BASE)
static_assert(port_exists('E'));
void port_e() { Pin<'E', 3>::output({.open_drain = true}); }
#else
static_assert(!port_exists('E'));
#endif
#if defined(GPIOF_BASE)
static_assert(port_exists('F'));
void port_f() { Pin<'F', 0>::output(); }
#else
static_assert(!port_exists('F'));
#endif
#if defined(GPIOK_BASE)
static_assert(port_exists('K'));
void port_k() { Pin<'K', 7>::output(); }
#else
static_assert(!port_exists('K'));
#endif
