// Platform family smoke TU: the SAM realization of the kernel's
// Platform concept (SamPlatform), the SysTick timebase above it and the
// interrupt-control verbs below it. The core is the same Cortex-M0+ on
// every variant, and IRQn_Type is where the differences would show - so
// this TU names a line that exists on all of them (SERCOM0) and leaves
// the per-instance existence question to the SERCOM driver's own
// negatives, when that driver arrives.
#include "kernel/panic.hpp"
#include "kernel/time.hpp"
#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/platform_sam.hpp"
#include "samc/ticker.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 48'000'000>;

static_assert(Platform<SamPlatform>);
static_assert(SamPlatform::atomic_width == 4);
static_assert(SamPlatform::ticks_per_second == 1000);
static_assert(Ticker::ticks_per_second == 1000);
static_assert(irq_priority_levels == 4);

// The kernel's tick conversions fold to the identity at 1000 Hz - the
// property the AVR side cannot have at 1024.
static_assert(ticks_from_ms<SamPlatform>(500) == 500);
static_assert(ticks_from_secs<SamPlatform>(3) == 3000);

void platform_verbs() {
    SamPlatform::CriticalSection cs;
    (void)SamPlatform::now();
    (void)SamPlatform::interrupts_enabled();
    SamPlatform::idle();
    (void)take_panic_record<SamPlatform>();
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
    Ticker::pause();
    Ticker::resume();

    // A slower rate that still divides 1000 exactly.
    using SlowTicker = BasicTicker<125>;
    (void)SlowTicker::init(clock);
    (void)SlowTicker::millis();
}

void interrupt_verbs() {
    enable_interrupts();
    disable_interrupts();
    (void)interrupts_enabled();

    Nvic::enable(SERCOM0_IRQn);
    (void)Nvic::enabled(SERCOM0_IRQn);
    Nvic::set_pending(SERCOM0_IRQn);
    (void)Nvic::pending(SERCOM0_IRQn);
    Nvic::clear_pending(SERCOM0_IRQn);
    (void)Nvic::priority(SERCOM0_IRQn, 1);
    (void)Nvic::priority(SERCOM0_IRQn, 4);   // refused: this core has four levels
    (void)Nvic::priority(SysTick_IRQn, 0);   // a core exception: priority only
    (void)Nvic::priority(SERCOM0_IRQn);
    Nvic::disable(SERCOM0_IRQn);
}
