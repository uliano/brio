// Family smoke TU for samc/ac.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three). The AC block
// is identical across the family at the register level; what differs is
// PAD BONDING - 40.1 gives COMP2/3 only AIN[5:4] on the E and G - and
// that is asserted here per variant, out of the device header's own AIN
// symbols.

#include <stdint.h>

#include "samc/ac.hpp"
#include "samc/clock.hpp"
#include "samc/platform_sam.hpp"

using namespace brio;

static_assert(Ac::comparator_count == 4);
static_assert(Ac::window_count == 2);
static_assert(AcComparator<0>::flag == 0x01);
static_assert(AcComparator<3>::flag == 0x08);
static_assert(AcComparator<2>::sync_mask == AC_SYNCBUSY_COMPCTRL2_Msk);
static_assert(AcWindow<0>::flag == AC_INTFLAG_WIN0_Msk);
static_assert(AcWindow<1>::flag == AC_INTFLAG_WIN1_Msk);

// The EVSYS vocabulary the AC publishes (evsys.hpp owns only the fabric).
static_assert(Ac::comparator_generator(0) == 0x49);
static_assert(Ac::comparator_generator(3) == 0x4C);
static_assert(Ac::window_generator(0) == 0x4D);
static_assert(Ac::window_generator(1) == 0x4E);
static_assert(Ac::start_user(0) == 34);
static_assert(Ac::start_user(3) == 37);
static_assert(AcComparator<2>::event_generator == 0x4B);
static_assert(AcComparator<2>::start_event_user == 36);

// The pair owns the pads, not the comparator (40.6.3).
static_assert(ac_ain_of(0, 2) == 2 && ac_ain_of(2, 2) == 6);
static_assert(ac_pin_index(AcPositive::pin3) == 3);
static_assert(ac_pin_index(AcPositive::vscale) == 0xFF);
static_assert(ac_pin_index(AcNegative::ground) == 0xFF);

// AIN[3:0] are bonded on every variant; AIN[7:6] only on the J, which is
// 40.1's "only two, AIN[5:4], for CMP2 and CMP3 on E and G variants"
// read out of the device header instead of out of the sentence.
static_assert(ac_ain_exists(0) && ac_ain_exists(3));
static_assert(ac_ain_exists(4) && ac_ain_exists(5));
#if defined(__SAMC21J18A__)
static_assert(ac_ain_exists(6) && ac_ain_exists(7));
static_assert(ac_config_valid(2, AcConfig{.positive = AcPositive::pin3}));
#else
static_assert(!ac_ain_exists(6) && !ac_ain_exists(7));
static_assert(!ac_config_valid(2, AcConfig{.positive = AcPositive::pin3}));
// The SAME code on comparator 0 stays legal: PIN3 there is AIN3, bonded
// everywhere - the refusal is about the pair, not about the code.
static_assert(ac_config_valid(0, AcConfig{.positive = AcPositive::pin3}));
#endif

// The two chapter rules that are not about pads.
static_assert(!ac_config_valid(0, AcConfig{.single_shot = true, .hysteresis = true}),
              "40.6.6: hysteresis is available only in continuous mode");
static_assert(!ac_config_valid(
    0, AcConfig{.interrupt_on = AcInterrupt::end_of_comparison}),
              "40.8.12: EOC is single-shot mode only");
static_assert(ac_config_valid(
    0, AcConfig{.single_shot = true,
                .interrupt_on = AcInterrupt::end_of_comparison}));

// INVEIx without COMPEIx inverts an event nothing listens to.
static_assert(ac_event_control_valid(AcEventControl{.start_in = 0x3, .invert_in = 0x1}));
static_assert(!ac_event_control_valid(AcEventControl{.invert_in = 0x1}));

// The config word is constexpr-composable: the fixture pins one shape.
constexpr AcConfig probe_cfg{
    .positive = AcPositive::pin0,
    .negative = AcNegative::vscale,
    .speed = AcSpeed::high,
    .out = AcOut::synchronous,
};
static_assert((ac_compctrl(probe_cfg) & AC_COMPCTRL_OUT_Msk) == AC_COMPCTRL_OUT_SYNC);
static_assert((ac_compctrl(probe_cfg) & AC_COMPCTRL_ENABLE_Msk) == 0u,
              "configure() owns ENABLE; the config word must never carry it");

void block_verbs() {
    (void)Ac::init(4);
    (void)Ac::enabled();
    (void)Ac::flags();
    Ac::clear_flags(0x3F);
    (void)Ac::take_flags();
    (void)Ac::event_config(AcEventControl{.comparator_out = 0x1,
                                          .window_out = 0x1,
                                          .start_in = 0x1,
                                          .invert_in = 0x1});
    (void)Ac::event_config();
    Ac::release();
}

void comparator_verbs() {
    using C = AcComparator<1>;
    (void)C::configure(probe_cfg);
    C::scaler(31);
    (void)C::enable(true);
    (void)C::ready();
    (void)C::state();
    C::start();
    (void)C::flag_set();
    C::clear_flag();
    C::arm(true);
    (void)C::armed();
    (void)C::single_shot();
    (void)C::positive();
    (void)C::enable(false);
}

void window_verbs() {
    using W = AcWindow<0>;
    (void)W::configure(true, AcWindowInterrupt::outside);
    (void)W::enabled();
    (void)W::interrupt_on();
    (void)W::state();
    (void)W::ready();
    (void)W::pair_consistent();
    W::start();
    (void)W::flag_set();
    W::clear_flag();
    W::arm(true);
    (void)W::armed();
    (void)W::configure(false, AcWindowInterrupt::above);
}
