// OPAMP family smoke TU. The peripheral is DB-ONLY: on the DA parts
// avrdx/opamp.hpp must still be INCLUDABLE and compile to nothing, and
// `opamp_count` (evsys.hpp, which every target header pulls in) must
// read 0 there. On DB it is 2 op amps at 28/32 pins and 3 at 48/64 -
// the device header is the authority, and the detection is a `requires`
// on the OP2 registers rather than a device-name list.
#include "avrdx/clock.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/opamp.hpp"

using namespace brio;

#if defined(OPAMP)

static_assert(opamp_count == 2 || opamp_count == 3);

// The ladder's arithmetic.
static_assert(opamp_ladder_r1(OpampWiper::wip0) == 15);
static_assert(opamp_ladder_r2(OpampWiper::wip0) == 1);
static_assert(opamp_ladder_r1(OpampWiper::wip3) == 8);
static_assert(opamp_ladder_r1(OpampWiper::wip7) == 1);
static_assert(opamp_noninverting_gain(OpampWiper::wip3) == OpampGain{2, 1});
static_assert(opamp_noninverting_gain(OpampWiper::wip0) == OpampGain{16, 15});
static_assert(opamp_noninverting_gain(OpampWiper::wip4) == OpampGain{8, 3});
static_assert(opamp_noninverting_gain(OpampWiper::wip7) == OpampGain{16, 1});
static_assert(opamp_inverting_gain(OpampWiper::wip3) == OpampGain{-1, 1});
static_assert(opamp_inverting_gain(OpampWiper::wip0) == OpampGain{-1, 15});
static_assert(opamp_inverting_gain(OpampWiper::wip4) == OpampGain{-5, 3});
static_assert(opamp_inverting_gain(OpampWiper::wip7) == OpampGain{-15, 1});
static_assert(opamp_gain_x1000(OpampGain{16, 15}) == 1067);
static_assert(opamp_gain_x1000(OpampGain{-5, 3}) == -1667);
static_assert(opamp_noninverting_wiper(2, 1) == OpampWiper::wip3);
static_assert(opamp_noninverting_wiper(5, 1) == std::nullopt);   // the ladder has no x5
static_assert(opamp_inverting_wiper(-3, 1) == OpampWiper::wip5);
static_assert(opamp_inverting_wiper(-2, 1) == std::nullopt);

// TIMEBASE is "one less than the CLK_PER cycles that reach 1 us" (35.5.3).
static_assert(opamp_timebase(24'000'000) == 23);
static_assert(opamp_timebase(12'000'000) == 11);
static_assert(opamp_timebase(4'000'000) == 3);
static_assert(opamp_timebase(1'000'000) == 0);
static_assert(opamp_timebase(500'000) == 0);
static_assert(opamp_timebase(3'846'153) == 3);   // the chapter's 260 ns example
static_assert(opamp_timebase_ns(24'000'000) == 1000);
static_assert(opamp_timebase_ns(500'000) == 2000);   // the tick stretches below 1 MHz

void opamp_common() {
    using SysClock = Clock<ClockSource::internal, 4'000'000>;
    constexpr SysClock clock;

    OpampSystem::init(clock);
    OpampSystem::timebase(23);
    (void)OpampSystem::reduced_input_range(true);   // read-only on rev. A4 (errata 2.8.2)
    OpampSystem::debug_run(true);
    OpampSystem::rebase(12'000'000);

    // OP0 and OP1 exist on every DB package.
    Opamp<0>::init<OpampConfig{.positive = OpampPos::inp, .negative = OpampNeg::out,
                               .settle_us = 20}>();
    (void)Opamp<1>::init({.positive = OpampPos::link_out, .negative = OpampNeg::wiper,
                          .top = OpampTop::out, .bottom = OpampBot::gnd,
                          .wiper = OpampWiper::wip3});
    Opamp<0>::mode(OpampMode::software_with_events);
    (void)Opamp<0>::positive(OpampPos::vdd_div2);
    Opamp<0>::negative(OpampNeg::dac);
    Opamp<0>::ladder(OpampTop::vdd, OpampBot::inn, OpampWiper::wip5);
    Opamp<0>::settle_us(10);
    Opamp<0>::cal(Opamp<0>::cal());
    Opamp<0>::run_standby(true);
    Opamp<0>::output(OpampOutput::off);
    (void)Opamp<0>::settled();
    (void)Opamp<0>::mode();
    (void)Opamp<0>::wiper();
    (void)Opamp<0>::output();
    Opamp<0>::release();

    // The event vocabulary: READY out, the four users in.
    EventChannel<0>::source(Opamp<1>::ReadyEvent{});
    Opamp<0>::enable_on(EventChannel<1>{});
    Opamp<0>::disable_on(EventChannel<1>{});
    Opamp<0>::dump_on(EventChannel<0>{});
    Opamp<0>::drive_on(EventChannel<0>{});
    Opamp<0>::dump_off();
    Opamp<0>::drive_off();
    Opamp<0>::enable_off();
    Opamp<0>::disable_off();

    // Tasks.
    (void)OpampFollower<Opamp<0>>::init();
    (void)OpampPga<Opamp<1>>::init(OpampWiper::wip3);
    OpampPga<Opamp<1>>::set(OpampWiper::wip5);
    (void)OpampInvertingPga<Opamp<0>>::init(OpampWiper::wip4, OpampBot::dac);
    static_assert(OpampPga<Opamp<1>>::gain_of(OpampWiper::wip3) == OpampGain{2, 1});
    static_assert(OpampPga<Opamp<1>>::for_gain(4) == OpampWiper::wip5);
    static_assert(OpampInvertingPga<Opamp<0>>::gain_of(OpampWiper::wip7) == OpampGain{-15, 1});
    OpampSystem::disable();
}

// The pads: OP0/OP1 are PORTD everywhere, OP2's are PORTE and exist
// exactly where OP2 does.
static_assert(opamp_inp_pin(0).port == 'D' && opamp_inp_pin(0).pin == 1);
static_assert(opamp_out_pin(0).port == 'D' && opamp_out_pin(0).pin == 2);
static_assert(opamp_inn_pin(0).port == 'D' && opamp_inn_pin(0).pin == 3);
static_assert(opamp_inp_pin(1).port == 'D' && opamp_inp_pin(1).pin == 4);
static_assert(opamp_out_pin(1).port == 'D' && opamp_out_pin(1).pin == 5);
static_assert(opamp_inn_pin(1).port == 'D' && opamp_inn_pin(1).pin == 7);

// MUXPOS's link codes are per instance on every package.
static_assert(!opamp_config_valid(0, OpampConfig{.positive = OpampPos::link_out}));
static_assert(opamp_config_valid(1, OpampConfig{.positive = OpampPos::link_out}));
static_assert(!opamp_config_valid(0, OpampConfig{.positive = OpampPos::link_wip}));
static_assert(!opamp_config_valid(1, OpampConfig{.positive = OpampPos::link_wip}));
static_assert(!opamp_config_valid(0, OpampConfig{.settle_us = 128}));

#if defined(__AVR_AVR128DB28__) || defined(__AVR_AVR128DB32__)
// 28/32-pin: no OP2 at all, so its pads are gone and OP0's MUXBOT
// LINKOUT - which is OP2's output, 35.5.7 note 1 - has no source.
static_assert(opamp_count == 2);
static_assert(opamp_inp_pin(2).port == 0);
static_assert(!opamp_config_valid(2, OpampConfig{}));
static_assert(!opamp_config_valid(0, OpampConfig{.bottom = OpampBot::link_out}));
static_assert(opamp_config_valid(1, OpampConfig{.bottom = OpampBot::link_out}));
bool opamp_small_package() {
    return Opamp<1>::init({.positive = OpampPos::link_wip});   // -> false here
}
#else
// 48/64-pin: three op amps, PORTE pads, LINKWIP and the whole
// instrumentation recipe.
static_assert(opamp_count == 3);
static_assert(opamp_inp_pin(2).port == 'E' && opamp_inp_pin(2).pin == 1);
static_assert(opamp_out_pin(2).port == 'E' && opamp_out_pin(2).pin == 2);
static_assert(opamp_inn_pin(2).port == 'E' && opamp_inn_pin(2).pin == 3);
static_assert(opamp_config_valid(2, OpampConfig{.positive = OpampPos::link_wip}));
static_assert(opamp_config_valid(0, OpampConfig{.bottom = OpampBot::link_out}));

static_assert(instrumentation_gain(InstrumentationGain::unity) == OpampGain{1, 1});
static_assert(instrumentation_gain(InstrumentationGain::div15) == OpampGain{1, 15});
static_assert(instrumentation_wiper_op0(InstrumentationGain::x15) == OpampWiper::wip0);
static_assert(instrumentation_wiper_op2(InstrumentationGain::x15) == OpampWiper::wip7);

void opamp_three() {
    Opamp<2>::init<OpampConfig{.positive = OpampPos::link_wip, .negative = OpampNeg::wiper,
                               .top = OpampTop::out, .bottom = OpampBot::link_out,
                               .wiper = OpampWiper::wip3}>();
    EventChannel<2>::source(Opamp<2>::ReadyEvent{});
    Opamp<2>::dump_on(EventChannel<2>{});
    (void)InstrumentationAmp<>::init(InstrumentationGain::x7);
    InstrumentationAmp<>::set(InstrumentationGain::unity);
    (void)InstrumentationAmp<>::settled();
    InstrumentationAmp<>::release();
}
#endif

#else   // no OPAMP: the DA family

// The header is includable and defines nothing; the count says so, and
// so does the absence of every OPAMP event vocabulary in evsys.hpp.
static_assert(opamp_count == 0);

#endif
