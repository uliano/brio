// GPIO family smoke TU: the bank's verbs in both halves, a pin as a
// type, a pin named at run time, and the package facts - compiled for
// both packages, so the same source proves the QFN-60's bank ends at
// GP29 and the QFN-80's at GP47.
#include "rp2350/pin.hpp"

using namespace brio;

static_assert(gpio_count_max == 48u);
static_assert(gpio_count == package_gpio_count(package));
static_assert(package_adc_inputs(Package::qfn60) == 4u);
static_assert(package_adc_inputs(Package::qfn80) == 8u);
static_assert(package_adc_base_pin(Package::qfn60) == 26u);
static_assert(package_adc_base_pin(Package::qfn80) == 40u);
static_assert(pad_value(PinConfig{}) != 0u);
static_assert((pad_value(PinConfig{.pull = PinPull::keeper}) &
               (PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS)) ==
              (PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS));
// The isolation latch is never set by a configuring verb: writing a pad
// IS taking it out of isolation.
static_assert((pad_value(PinConfig{}) & PADS_BANK0_GPIO0_ISO_BITS) == 0u);

// A pin of the low half and one of the high half, which exist on the
// QFN-80 alone - so the pin taken here is the LAST ONE THIS PACKAGE HAS.
using Led = Pin<25>;
using Top = Pin<gpio_count - 1u>;
static_assert(Led::mask == (1UL << 25));
static_assert(!Led::high_half);
static_assert(Top::high_half == (gpio_count > 32u));

constexpr PinSel uart_tx{0, PinFunction::uart};
static_assert(uart_tx.valid());

// The four events as a set, and the composition that names both edges.
static_assert(pin_edges.has(PinEvent::edge_low) && pin_edges.has(PinEvent::edge_high));
static_assert(!pin_edges.has(PinEvent::level_low));
static_assert((PinEvent::level_low | PinEvent::level_high).bits ==
              (static_cast<uint8_t>(PinEvent::level_low) |
               static_cast<uint8_t>(PinEvent::level_high)));
static_assert((pin_events(PinEvent::edge_low) | PinEvent::edge_high) == pin_edges);
static_assert(PinEvents{}.none());

using Button = ExtInt<Pin<23>>;
static_assert(Button::pin == 23u);
static_assert(Button::irq() == IO_IRQ_BANK0_IRQn);

void pin_verbs() {
    (void)Gpio::ready();
    (void)Gpio::bonded(25u);
    (void)Gpio::in();
    (void)Gpio::in_hi();
    (void)Gpio::out();
    (void)Gpio::out_hi();
    Gpio::out_set(1u);
    Gpio::out_set_hi(1u);
    Gpio::out_clear(1u);
    Gpio::out_clear_hi(1u);
    Gpio::out_toggle(1u);
    Gpio::out_toggle_hi(1u);
    (void)Gpio::oe();
    (void)Gpio::oe_hi();
    Gpio::oe_set(1u);
    Gpio::oe_set_hi(1u);
    Gpio::oe_clear(1u);
    Gpio::oe_clear_hi(1u);
    Gpio::oe_toggle(1u);
    Gpio::oe_toggle_hi(1u);
    (void)static_cast<uint32_t>(Gpio::ctrl(2u));
    (void)static_cast<uint32_t>(Gpio::status(2u));
    (void)static_cast<uint32_t>(Gpio::pad(2u));
    (void)Gpio::function(2u, PinFunction::pio2);
    (void)Gpio::analog(gpio_count - 1u, PinPull::none);
    (void)Gpio::release(2u);
    Gpio::outputs(0x0000'000cu);
    Gpio::outputs_hi(0x0000'0001u);

    Gpio::out_override(2u, PinOverride::high);
    (void)Gpio::out_override(2u);
    Gpio::oe_override(2u, PinOeOverride::disable);
    (void)Gpio::oe_override(2u);
    Gpio::in_override(2u, PinOverride::invert);
    (void)Gpio::in_override(2u);
    Gpio::irq_override(2u, PinOverride::pass);
    (void)Gpio::irq_override(2u);
    (void)Gpio::pin_status(2u).in_from_pad;
    Gpio::output_disable(2u, false);
    (void)Gpio::output_disabled(2u);
    Gpio::input_enable(2u, true);
    (void)Gpio::input_enabled(2u);
    Gpio::isolate(2u, false);
    (void)Gpio::isolated(2u);
    (void)(Gpio::voltage() == PadVoltage::v3v3);

    (void)Led::output();
    (void)Led::output(true);
    (void)Led::input(PinPull::up);
    (void)Led::function(PinFunction::sio);
    (void)Led::analog();
    (void)Led::release();
    (void)Led::bonded();
    Led::set();
    Led::clear();
    Led::toggle();
    (void)Led::read();
    (void)Led::read_out();
    (void)Led::is_output();
    (void)Led::function();
    Led::pull(PinPull::down);
    (void)Led::isolated();
    Led::isolate(false);
    (void)Led::input({.pull = PinPull::down, .input_enable = false});
    (void)Led::read_pulsed();
    Led::input_enable(true);
    (void)Led::input_enabled();
    Led::output_disable(false);
    (void)Led::output_disabled();
    Led::out_override(PinOverride::pass);
    (void)Led::out_override();
    Led::oe_override(PinOeOverride::pass);
    (void)Led::oe_override();
    Led::in_override(PinOverride::pass);
    (void)Led::in_override();
    Led::irq_override(PinOverride::pass);
    (void)Led::irq_override();
    (void)Led::status().oe_to_pad;
    Led::duty(1u);
    static_assert(Led::max == 1u);

    // The pin interrupts: the bank's controller, and one pad's face.
    PinIrq::enable(23u, pin_edges);
    (void)PinIrq::enabled(23u);
    (void)PinIrq::status(23u, PinIrqTarget::proc1);
    (void)PinIrq::raw(23u);
    PinIrq::clear(23u, pin_edges);
    PinIrq::force(23u, pin_events(PinEvent::edge_high));
    PinIrq::unforce(23u, pin_events(PinEvent::edge_high));
    (void)PinIrq::summary(PinIrqTarget::proc0, 0u);
    (void)PinIrq::summary(PinIrqTarget::dormant_wake, 1u);
    PinIrq::disable(23u, pin_edges);
    PinIrq::disable_all();
    PinIrq::disable_all(PinIrqTarget::proc1);
    (void)PinIrq::irq();

    (void)Button::arm(pin_events(PinEvent::level_high));
    (void)Button::armed();
    (void)Button::pending();
    (void)Button::raw();
    (void)Button::served();
    Button::clear();
    Button::force(pin_edges);
    Button::unforce(pin_edges);
    Button::disarm();

    (void)Top::output();
    Top::toggle();
    (void)Top::read();

    constexpr PinRef ref = Led::ref();
    ref.set();
    ref.clear();
    ref.toggle();
    (void)ref.read();
    (void)ref.read_out();
    static_assert(ref.valid());
    static_assert(!PinRef{}.valid());
}
