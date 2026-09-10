// TIM family smoke TU: ch32v00x/tim.hpp's two resources, Tim3, and every
// task instantiated on both timers - instantiation only, no main(), no
// hardware. The chapters' arithmetic is pinned in the header; what this
// fixture adds is the util contracts: TimPwm and TimPairPwm are
// PwmChannels on both instances, and the meters feed a MeterLatch.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/tim.hpp"
#include "util/meter_sampler.hpp"
#include "util/pwm_channel.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

static_assert(tim_base_for(1) == 0x40012C00 && tim_base_for(2) == 0x40000000 && tim3_base == 0x40000800);
static_assert(tim_clock_hz(SysClock{}) == 48'000'000UL);
static_assert(Tim<1>::has_break && !Tim<2>::has_break);
static_assert(!Tim<1>::has_dead_time_pairs && Tim<2>::has_dead_time_pairs);
static_assert(Tim<1>::update_irq() == Irq::tim1_up && Tim<2>::update_irq() == Irq::tim2 &&
              Tim<2>::cc_irq() == Irq::tim2);

static_assert(PwmChannel<TimPwm<Tim<1>, 0, 1000>> && PwmChannel<TimPwm<Tim<2>, 3, 255>>);
static_assert(PwmChannel<TimPairPwm<Tim<1>, 2, 1000>> && PwmChannel<TimPairPwm<Tim<2>, 1, 1000>>);

using Pad1 = TimPad<tim_default_pads::tim1_ch1>;
using Pad2 = TimPad<Pad{'D', 4}>;

template <uint8_t n>
void resource_verbs() {
    using T = Tim<n>;
    T::init();
    (void)T::bus_clock();
    (void)T::configure({.prescaler = 47, .period = 999, .direction = TimDirection::down,
                        .alignment = TimAlignment::center_both, .clock_division = TimClockDivision::div4,
                        .auto_reload_preload = true, .one_pulse = true, .update_on_overflow_only = true,
                        .capture_level = true, .capture_overflow = true, .outputs_float_when_stopped = true});
    T::enable(true); (void)T::enabled();
    (void)T::count(); T::set_count(5);
    (void)T::prescaler(); T::set_prescaler(1);
    (void)T::period(); (void)T::set_period(100);
    (void)T::auto_reload_preload();
    (void)T::repetition(); (void)T::set_repetition(3);
    T::update(); (void)T::capture_compare_event(0); T::trigger_event();
    (void)T::commutation_event(); (void)T::break_event();
    (void)T::flags(); (void)T::flag(T::update_flag); T::clear_flags(T::compare_flag(2) | T::overcapture_flag(2));
    T::interrupts(T::update_interrupt | T::compare_interrupt(1) | T::trigger_interrupt | T::break_interrupt |
                  T::commutation_interrupt | T::update_dma | T::compare_dma(0) | T::trigger_dma | T::commutation_dma, true);
    (void)T::interrupts();
    (void)T::isr();
    (void)T::ccr_address(0);
    (void)T::compare(0); (void)T::capture_raw(1); (void)T::captured_level(1);
    (void)T::set_compare(3, 10);
    (void)T::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 100, .preload = true, .fast = true,
                                .clear_on_etrf = true, .active_low = true, .complementary_active_low = true,
                                .enable = true, .complementary_enable = true, .idle_high = true,
                                .complementary_idle_high = true});
    (void)T::capture_channel(1, {.select = TimChannelSelect::indirect, .polarity = TimCapturePolarity::falling,
                                 .prescaler = TimCapturePrescaler::every4, .filter = 7});
    (void)T::channel_enable(2, true); (void)T::channel_enabled(2);
    (void)T::complementary_enable(0, false);
    T::ti1_xor(true);
    (void)T::slave({.mode = TimSlaveMode::reset, .trigger = TimTrigger::ti1, .master_slave = true});
    (void)T::slave_mode(); (void)T::slave_trigger();
    (void)T::master(TimMasterMode::update); (void)T::master();
    (void)T::external_trigger({.inverted = true, .prescaler = 2, .filter = 3, .clock_mode2 = true});
    (void)T::break_dead_time({.dead_time = 0x81, .break_enable = true, .lock = 1});
    (void)T::main_output(true); (void)T::main_output(); (void)T::dead_time_code();
    (void)T::pair_dead_time(0, {.ticks = 4, .active_low = true}); (void)T::pair_dead_time_ticks(0);
    T::release();
}

void all_resources() {
    resource_verbs<1>();
    resource_verbs<2>();
    Tim3::init();
    (void)Tim3::configure({.period = 999, .direction = TimDirection::up, .alignment = TimAlignment::edge,
                           .auto_reload_preload = true, .update_disable = false, .clocked_by_tim1 = true});
    Tim3::enable(true); (void)Tim3::enabled(); (void)Tim3::count(); Tim3::set_count(0); (void)Tim3::period();
    (void)Tim3::compare(3); (void)Tim3::set_compare(0, 500); (void)Tim3::compare_preload(0, true);
    (void)Tim3::dma_request(2, true); (void)Tim3::dma_request(0, true);
    Tim3::reset();
}

using Latch = MeterLatch<uint16_t, P, 0>;

template <class T>
void tasks() {
    (void)TimPwm<T, 0, 1000>::setup(47);
    TimPwm<T, 0, 1000>::duty(250); (void)TimPwm<T, 0, 1000>::duty();
    (void)TimPairPwm<T, 1, 1000>::setup(0, 4);
    TimPairPwm<T, 1, 1000>::duty(500); (void)TimPairPwm<T, 1, 1000>::dead_time_ticks();
    (void)TimPeriodMeter<T>::setup(0, 3, true);
    (void)TimPeriodMeter<T>::period_ticks(); (void)TimPeriodMeter<T>::width_ticks();
    (void)TimIntervalMeter<T, 2>::setup(0, 0, TimCapturePolarity::falling, TimCapturePrescaler::every8);
    if (auto d = TimIntervalMeter<T, 2>::interval()) { Latch::store(*d); }
    TimIntervalMeter<T, 2>::restart();
    (void)TimEventCounter<T>::setup(T::instance == 1 ? TimTrigger::itr1 : TimTrigger::itr0);
    (void)TimEventCounter<T>::count(); TimEventCounter<T>::restart();
    (void)TimGatedCounter<T>::setup(T::instance == 1 ? TimTrigger::itr1 : TimTrigger::itr0, 0, 1000);
    (void)TimGatedCounter<T>::count();
    (void)TimPeriodicTick<T>::setup(47, 999); TimPeriodicTick<T>::stop();
    (void)TimOnePulse<T, 0>::setup(0, 10, 100); TimOnePulse<T, 0>::fire(); (void)TimOnePulse<T, 0>::busy();
}

void all_tasks() {
    tasks<Tim<1>>();
    tasks<Tim<2>>();
    Pad1::claim(); Pad1::claim_input(PinPull::up); Pad1::release();
    Pad2::claim(PinDrive::open_drain); Pad2::release();
}
