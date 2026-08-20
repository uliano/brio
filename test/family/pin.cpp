// PORT/pin family smoke TU. The Port<L> resource and the pin
// configuration vocabulary must instantiate on every package; INLVL
// is a DB-only field (the DA's PINnCTRL has no such bit).
#include "avrdx/pin.hpp"

using namespace brio;

void pin_common() {
    using P = Pin<'D', 3>;
    P::output();
    P::configure({.pullup = true, .sense = PinSense::falling});
    P::sense(PinSense::level_low);
    (void)P::flag(); P::clear_flag();
    P::configure({.sense = PinSense::input_disable});
    static_assert(Pin<'D', 2>::fully_async);
    static_assert(Pin<'D', 6>::fully_async);
    static_assert(!Pin<'D', 3>::fully_async);

    using PA = Port<'A'>;
    PA::dir_set(0x0C); PA::out_toggle(0x04);
    (void)PA::in(); (void)PA::flags(); PA::clear_flags(0xFF);
    (void)PA::take_flags();
    PA::slew_limit(true); (void)PA::slew_limit();
    PA::configure_mask(0x0C, {.pullup = true});

    using Keys = PinSet<Pin<'A', 2>, Pin<'A', 3>, Pin<'C', 0>, Pin<'D', 5>>;
    Keys::input(true);
    Keys::configure({.pullup = true, .sense = PinSense::both});
    static_assert(Keys::port_mask<'A'>() == 0x0C);
    static_assert(Keys::port_mask<'C'>() == 0x01);
    static_assert(Keys::port_mask<'F'>() == 0);
    (void)Keys::read();

#ifdef PORT_INLVL_bm
    P::configure({.input_level = PinLevel::ttl, .sense = PinSense::rising});   // DB only
#endif
}
