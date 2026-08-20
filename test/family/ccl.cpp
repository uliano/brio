// CCL family smoke TU. LUT counts come from the device header (4 on
// 28/32-pin, 6 on 48/64); a LUT whose PORT the package does not bond
// (LUT5's pins are PORTG, 64-pin only) still works on events - only
// the pin faces are refused.
#include "avrdx/ccl.hpp"
#include "avrdx/evsys.hpp"

using namespace brio;

void ccl_common() {
    Ccl::disable();
    Lut<0>::init<LutConfig{.in0 = LutInput::event_a,
                           .truth = lut_truth([](bool a, bool, bool) { return a; }),
                           .filter = LutFilter::sync, .edge_detect = true}>();
    Lut<0>::event_a_on(EventChannel<1>{});
    Ccl::sequencer<0>(Sequencer::d_flip_flop);
    Ccl::sequencer<1>(Sequencer::rs_latch);
    ToggleFlipFlop<0>::init(EventChannel<1>{});
    Ccl::enable();
    (void)Lut<2>::init({.in0 = LutInput::tca0, .truth = 0xAA, .output_pin = true});
    EventChannel<2>::source(Lut<2>::OutEvent{});
    // a config asking a pin the package lacks is refused at RUN time too
    (void)Lut<0>::init({.in0 = LutInput::mask, .truth = 1, .alt_pin = true});
}

#if defined(__AVR_AVR128DB48__) || defined(__AVR_AVR128DA48__)
// 48-pin: LUT4/5 exist; LUT5 has NO pins (PORTG) but lives on events.
static_assert(Lut<5>::has_pins == false);
static_assert(Lut<4>::has_pins == true);
void ccl_48pin() {
    Lut<5>::init<LutConfig{.in0 = LutInput::event_a, .truth = 0x02}>();
    Lut<5>::event_a_on(EventChannel<3>{});
    EventChannel<4>::source(Lut<5>::OutEvent{});
    Ccl::sequencer<2>(Sequencer::jk_flip_flop);
}
#endif

#if defined(__AVR_AVR128DB64__) || defined(__AVR_AVR128DA64__)
static_assert(Lut<5>::has_pins);            // PORTG bonded
void ccl_64pin() {
    Lut<5>::init<LutConfig{.in0 = LutInput::pin, .truth = 0x02, .output_pin = true}>();
}
#endif

// LUT3 has no ALT1 anywhere (reserved route bit, PF6 = RESET).
static_assert(!Lut<3>::has_alt_pin);
static_assert(Lut<1>::has_alt_pin);
