// Platform family smoke TU: the STM32G0 realization of the kernel's
// Platform concept (Stm32g0Platform<TB>), the SysTick timebase it takes
// by default and the interrupt-control verbs below it - and the
// kernel's optional idle_until hook, which the SysTick platform must NOT
// have and a platform on a Tickless timebase must (stm32g0/lptim_ticker.hpp's
// LptimTicker is the real one and has its own TU; here a local fake
// with the concept's three members proves the platform's constrained
// member and the kernel's detection of it on every header, LPTIM or
// not). The core is the same Cortex-M0+ on every variant; IRQn_Type is
// where the differences show, so this TU names a line every G0 has
// (USART1_IRQn).
#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/kernel.hpp"
#include "kernel/panic.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/ticker.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;

static_assert(Platform<Stm32g0Platform<>>);
static_assert(Stm32g0Platform<>::atomic_width == 4);
static_assert(Stm32g0Platform<>::ticks_per_second == 1000);
static_assert(Ticker::ticks_per_second == 1000);
static_assert(irq_priority_levels == 4);

// The kernel's tick conversions fold to the identity at 1000 Hz.
static_assert(ticks_from_ms<Stm32g0Platform<>>(500) == 500);
static_assert(ticks_from_secs<Stm32g0Platform<>>(3) == 3000);

// The SysTick ticker is not Tickless, and the platform on it has no
// idle_until: the kernel then compiles idle() and nothing else. (The
// probe must be dependent - a requires-expression on a fixed type is
// ill-formed rather than false when the call does not exist.)
template <class P>
constexpr bool has_idle_until = requires { P::idle_until(std::optional<uint32_t>{}); };
static_assert(!Tickless<Ticker>);
static_assert(!has_idle_until<Stm32g0Platform<>>);

// A timebase with the concept's three members - a stand-in for
// LptimTicker (its own TU compiles the real thing where LPTIM exists).
struct FakeTickless {
    static inline uint32_t count = 0;
    static uint32_t ticks() { return count; }
    static constexpr uint32_t ticks_per_second = 1024;
    static bool arm_wake(uint32_t, uint32_t) { return true; }
    static bool park() { return true; }
};
static_assert(Tickless<FakeTickless>);
using TicklessPlatform = Stm32g0Platform<FakeTickless>;
static_assert(Platform<TicklessPlatform>);
static_assert(TicklessPlatform::ticks_per_second == 1024);
static_assert(std::same_as<TicklessPlatform::Timebase, FakeTickless>);
static_assert(has_idle_until<TicklessPlatform>);
static_assert(ticks_from_ms<TicklessPlatform>(1000) == 1024);   // ceil, no longer the identity
static_assert(ticks_from_ms<TicklessPlatform>(1) == 2);         // at least, never early

struct Ao {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, TicklessPlatform> queue;
    static inline TimeEvent<TicklessPlatform, Ao, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) {}
};

void tickless_kernel() {
    // The kernel's idle_if_empty takes the idle_until branch here.
    Kernel<TicklessPlatform, Ao>::init_all();
    Ao::alarm.arm(10);
    Kernel<TicklessPlatform, Ao>::idle_if_empty();
    (void)TimeEvents<TicklessPlatform>::next_deadline();
    TicklessPlatform::idle_until(std::nullopt);
    TicklessPlatform::idle_until(std::optional<uint32_t>{42});
}

void platform_verbs() {
    Stm32g0Platform<>::CriticalSection cs;
    (void)Stm32g0Platform<>::now();
    (void)Stm32g0Platform<>::interrupts_enabled();
    Stm32g0Platform<>::idle();
    (void)take_panic_record<Stm32g0Platform<>>();
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
