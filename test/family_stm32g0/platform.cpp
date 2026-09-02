// Platform family smoke TU: the STM32G0 realization of the kernel's
// Platform concept (Stm32Platform), the SysTick timebase above it and
// the interrupt-control verbs below it. The core is the same Cortex-M0+
// on every variant; IRQn_Type is where the differences show, so this TU
// names a line every G0 has (USART1_IRQn).
#include "kernel/panic.hpp"
#include "kernel/time.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/ticker.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;

static_assert(Platform<Stm32Platform>);
static_assert(Stm32Platform::atomic_width == 4);
static_assert(Stm32Platform::ticks_per_second == 1000);
static_assert(Ticker::ticks_per_second == 1000);
static_assert(irq_priority_levels == 4);

// The kernel's tick conversions fold to the identity at 1000 Hz.
static_assert(ticks_from_ms<Stm32Platform>(500) == 500);
static_assert(ticks_from_secs<Stm32Platform>(3) == 3000);

void platform_verbs() {
    Stm32Platform::CriticalSection cs;
    (void)Stm32Platform::now();
    (void)Stm32Platform::interrupts_enabled();
    Stm32Platform::idle();
    (void)take_panic_record<Stm32Platform>();
}

void ticker_verbs() {
    constexpr SysClock clock;
    (void)Ticker::init(clock);
    Ticker::tick();
    (void)Ticker::ticks();
    (void)Ticker::millis();
    (void)Ticker::secs();
    TimeStamp stamp{};
    Ticker::now(stamp);
    Ticker::advance(3);
    Ticker::pause();
    Ticker::resume();

    using SlowTicker = BasicTicker<125>;
    (void)SlowTicker::init(clock);
    (void)SlowTicker::millis();
}

void interrupt_verbs() {
    enable_interrupts();
    disable_interrupts();
    (void)interrupts_enabled();

    Nvic::enable(USART1_IRQn);
    (void)Nvic::enabled(USART1_IRQn);
    Nvic::set_pending(USART1_IRQn);
    (void)Nvic::pending(USART1_IRQn);
    Nvic::clear_pending(USART1_IRQn);
    (void)Nvic::priority(USART1_IRQn, 1);
    (void)Nvic::priority(USART1_IRQn, 4);    // refused: this core has four levels
    (void)Nvic::priority(SysTick_IRQn, 0);   // a core exception: priority only
    (void)Nvic::priority(USART1_IRQn);
    Nvic::disable(USART1_IRQn);
}
