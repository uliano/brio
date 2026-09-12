// Platform family smoke TU: the concept, the ticker, the resets and
// the chip id - every verb instantiated once.
#include "rp2040/delay.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/resets.hpp"
#include "rp2040/sysinfo.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/clock.hpp"

using namespace brio;

using P = Rp2040Platform<>;
static_assert(Platform<P>);
static_assert(P::ticks_per_second == 1000u);
static_assert(P::atomic_width == 4u);
static_assert(irq_priority_levels == 4u);

using SysClock = Clock<ClockSource::pll, 125'000'000>;

void platform_verbs() {
    constexpr SysClock clock;
    { P::CriticalSection cs; }
    P::idle();
    (void)P::interrupts_enabled();
    (void)P::now();
    (void)P::core_id();
    (void)P::panic_record();
    (void)Ticker::init(clock);
    Ticker::tick();
    (void)Ticker::ticks();
    (void)Ticker::millis();
    Ticker::rebase(12'000'000);
    Ticker::pause();
    Ticker::resume();
    (void)SysTickCounter::start(clock);
    (void)delay_us(clock, 10u);
    Nvic::enable(UART0_IRQ_IRQn);
    Nvic::set_pending(UART1_IRQ_IRQn);
    (void)Nvic::enabled(TIMER_IRQ_0_IRQn);
    (void)Resets::release(ResetBlock::uart0 | ResetBlock::pads_bank0);
    Resets::hold(ResetBlock::pio0);
    (void)Resets::cycle(ResetBlock::adc);
    (void)Resets::released(ResetBlock::all);
    (void)Resets::held(ResetBlock::dma);
    Resets::watchdog_resets(ResetBlock::uart0);
    (void)Resets::watchdog_resets();
    (void)ChipId::read();
    (void)ChipId::gitref();
    hw_set(RESETS->WDSEL, 1u);
    hw_clear(RESETS->WDSEL, 1u);
    hw_xor(RESETS->WDSEL, 1u);
    hw_write_masked(RESETS->WDSEL, 0u, 1u);
}
