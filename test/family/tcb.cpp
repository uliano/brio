// TCB family smoke TU: every package must compile this (instantiation
// only). TCB0..TCB2 exist everywhere; TCB2's default pin (PC0) must
// work even where its ALT1 (PB4) is not bonded; TCB4 is 64-pin only.
#include "avrdx/clock.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/evsys.hpp"

using namespace brio;

// The clock contract: the microsecond-speaking tasks are ClockUsers -
// a DynamicClock can list them (which requires their rebase), and
// their init(clock) asserts clock_follows.
using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot, PeriodicTick<Tcb<0>>, Timeout<Tcb<1>>,
                         OneShotPulse<Tcb<2>>, FrequencyMeter<Tcb<0>>,
                         PulseWidthMeter<Tcb<1>>, DutyMeter<Tcb<2>>>;

void tcb_clock_users() {
    (void)PeriodicTick<Tcb<0>>::init(Dyn{}, 1000);
    (void)Timeout<Tcb<1>>::init(Dyn{}, 1000, EventChannel<2>{});
    (void)OneShotPulse<Tcb<2>>::init(Dyn{}, 100, EventChannel<1>{});
    FrequencyMeter<Tcb<0>>::init(Dyn{}, EventChannel<2>{});
    PulseWidthMeter<Tcb<1>>::init(Dyn{}, EventChannel<2>{});
    DutyMeter<Tcb<2>>::init(Dyn{}, EventChannel<2>{});
    FrequencyMeter<Tcb<0>>::rebase(12'000'000);
    PulseCounter<Tcb<1>>::init(EventChannel<2>{});
    PulseCounter<Tcb<1>>::snapshot_on(EventChannel<3>{}, true, true);
}

void tcb_common() {
    Tcb<0>::init<TcbConfig{.mode = TcbMode::periodic, .compare = 1000}>();
    Tcb<1>::init({.mode = TcbMode::capture, .clock = TcbClock::event});
    Tcb<2>::init<TcbConfig{.mode = TcbMode::pwm8, .compare = 0x80FF, .output = true}>();
    (void)Tcb<2>::init({.mode = TcbMode::single_shot, .compare = 240,
                        .event_input = true, .output = true});
    Tcb<2>::capture_on(EventChannel<0>{});
    Tcb<2>::count_on(EventChannel<1>{});
    (void)Pwm8<Tcb<2>>::init();
    Pwm8<Tcb<2>>::duty(255);        // the >= max path drives the pin from PORT
    EvTcbCaptIn<2>::listen(EventChannel<0>{});
    (void)EvTcbCapt<2>::code;
    // the freshly filled EVSYS vocabulary instantiates everywhere
    (void)EvUpdiSynch::code;
    (void)EvZcdOut<2>::code;
    (void)EvUsartXck<0>::code;
    (void)EvSpiSck<1>::code;
    EvUsartIrda<0>::listen(EventChannel<0>{});
    EvUsartIrda<0>::unlisten();
    EvOut<Pin<'A', 7>>::listen(EventChannel<0>{});
    EvOut<Pin<'A', 7>>::unlisten();               // pin back to input, ALT1 bit cleared
#ifdef MVIO
    (void)EvMvioOk::code;
#endif
#ifdef OPAMP
    // The last op amp of THIS package: OP1 at 28/32 pins, OP2 at 48/64.
    // Its four EVSYS user registers are the ones that do not exist on
    // the smaller parts (the struct simply ends earlier).
    (void)EvOpampReady<opamp_count - 1>::code;
    EvOpampCtl<opamp_count - 1, OpampAction::drive>::listen(EventChannel<1>{});
#endif
#ifdef TCD0
    (void)EvTcdProgEv::code;
    EvTcdInputA::listen(EventChannel<1>{});
#endif
#ifdef PORTG
    (void)EvPin<Pin<'G', 2>>::code;
    EvOut<Pin<'G', 7>>::listen(EventChannel<6>{});
#endif
}

#if defined(__AVR_AVR128DB28__) || defined(__AVR_AVR128DB32__) || \
    defined(__AVR_AVR128DA28__) || defined(__AVR_AVR128DA32__)
// 28/32-pin: the instance is usable, only the ALT1 position is refused.
static_assert(!Tcb<2>::has_alt_pin);
static_assert(Tcb<2>::has_default_pin);
bool tcb_small_package() {
    // alt_pin at run time: refused with false, nothing programmed.
    return Tcb<2>::init({.mode = TcbMode::pwm8, .compare = 0x80FF,
                         .output = true, .alt_pin = true});   // -> false
}
#endif

#if defined(__AVR_AVR128DB64__) || defined(__AVR_AVR128DA64__)
// 64-pin: TCB4 end-to-end (resource, pins, event vocabulary, task).
static_assert(Tcb<4>::has_default_pin);   // PG3
static_assert(Tcb<4>::has_alt_pin);       // PC6 (bonded; dead per errata 2.13.2)
void tcb_big_package() {
    Tcb<4>::init<TcbConfig{.mode = TcbMode::periodic, .compare = 1000, .output = true}>();
    Tcb<4>::capture_on(EventChannel<3>{});
    Tcb<4>::count_on(EventChannel<4>{});
    EvTcbCaptIn<4>::listen(EventChannel<3>{});
    EvTcbCountIn<4>::listen(EventChannel<4>{});
    (void)EvTcbCapt<4>::code;
    (void)EvTcbOvf<4>::code;
    (void)Pwm8<Tcb<4>>::init();
    (void)Tcb<4>::regs().CNT;
}
#endif
