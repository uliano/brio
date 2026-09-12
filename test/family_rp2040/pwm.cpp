// PWM family smoke TU: the vocabulary and its arithmetic, the pin
// table, the block's and the slice's verbs, every task.
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/pwm.hpp"
#include "util/rgb_lamp.hpp"

using namespace brio;

static_assert(pwm_slice_count == 8u);
static_assert(pwm_slice_of(9) == 4u && pwm_pin_is_b(9) && pwm_slice_of(17) == 0u);
static_assert(PwmDivider{}.valid() && !PwmDivider{1, 16}.valid());
static_assert(pwm_period_sixteenths({.divider = {2, 0}, .top = 999}) == 32'000u);
static_assert(pwm_output_hz(125'000'000, {.divider = {2, 0}, .top = 999}) == 62'500u);
static_assert(pwm_config_for(125'000'000, 100'000, 1249)->divider == PwmDivider{1, 0});
static_assert(!pwm_config_for(125'000'000, 100'000'000, 1249).has_value());
static_assert(PwmOutput<12, 999>::max == 1000u && PwmOutput<12, 999>::channel == 0u && PwmOutput<13, 999>::channel == 1u);
static_assert(PwmChannel<PwmOutput<12, 999>> && PwmChannel<PwmPair<12, 13, 999>>);
static_assert(PwmEdgeCounter<15>::Slice::index == 7u);

using SysClock = Clock<ClockSource::pll, 125'000'000>;
using OutA = PwmOutput<12, 999>;
using OutB = PwmOutput<13, 999>;
using OutC = PwmOutput<17, 999>;
using Lamp = RgbLamp<OutA, OutB, OutC>;
using Pair = PwmPair<12, 13, 999>;
using Edges = PwmEdgeCounter<15>;
using Level = PwmLevelCounter<9>;
using Tick = PwmPeriodicTick<3>;
using Stream = DmaTxEngine<6, uint32_t>;

uint32_t table[8];

void pwm_verbs() {
    constexpr SysClock clock;
    (void)Pwm::reset();
    (void)Pwm::released();
    Pwm::hold();
    Pwm::start(0x03);
    Pwm::stop(0x03);
    (void)Pwm::running();
    Pwm::interrupts(0x01, true);
    (void)Pwm::interrupts();
    (void)Pwm::raw_pending();
    (void)Pwm::pending();
    Pwm::clear_pending(0xFF);
    Pwm::force(0x01, false);
    (void)Pwm::isr();

    using S = PwmSlice<6>;
    (void)S::configure({.mode = PwmDivMode::free_running, .divider = {3, 4}, .top = 999, .phase_correct = true});
    (void)S::config();
    S::enable(true);
    (void)S::enabled();
    S::level(0, 100);
    S::levels(100, 200);
    (void)S::level(1);
    S::top(999);
    (void)S::top();
    (void)S::counter();
    S::counter(0);
    (void)S::advance_phase();
    (void)S::retard_phase();
    S::interrupt(true);
    (void)S::raw_pending();
    (void)S::pending();
    S::clear_pending();
    (void)S::cc_address();
    (void)S::top_address();

    (void)OutA::setup({2, 0});
    (void)OutB::setup_hz(clock, 1000);
    OutC::attach();
    OutA::duty(500);
    (void)OutA::duty();
    Lamp::show({10, 20, 30});
    Lamp::off();
    OutA::release();
    (void)Pair::setup({1, 0}, 10, true);
    Pair::duty(400);
    (void)Pair::dead_time();
    Pair::release();
    (void)Edges::setup(false, {1, 0}, 0xFFFF, PinPull::down);
    Edges::run(true);
    Edges::restart();
    (void)Edges::count();
    Edges::release();
    (void)Level::setup({4, 0});
    Level::run(true);
    Level::restart();
    (void)Level::count();
    Level::release();
    (void)Tick::setup({1, 0}, 124);
    (void)Tick::setup_hz(clock, 1000);
    Tick::stop();
    Stream::arm(S::cc_address(), S::dreq);
    (void)Stream::start(table, 8);
}
