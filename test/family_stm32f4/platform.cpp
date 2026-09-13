// Platform family smoke TU: the STM32F4 realization of the kernel's
// Platform concept (Stm32f4Platform<TB>), the SysTick timebase it takes
// by default and the interrupt-control verbs below it - the armv6m core
// files included by a Cortex-M4 header, which is what this TU proves
// compiles on every header of the pack. IRQn_Type is where the headers
// differ, so this TU names a line every F4 has (USART1_IRQn).
#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/panic.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"

using namespace brio;

using P = Stm32f4Platform<>;

static_assert(Platform<P>);
static_assert(P::atomic_width == 4);
static_assert(P::ticks_per_second == 1000);
static_assert(Ticker::ticks_per_second == 1000);
static_assert(irq_priority_levels == 16);   // __NVIC_PRIO_BITS is 4 on every F4

// The kernel's tick conversions fold to the identity at 1000 Hz.
static_assert(ticks_from_ms<P>(500) == 500);
static_assert(ticks_from_secs<P>(3) == 3000);

// No Tickless timebase on this family yet: the platform offers idle()
// and no idle_until(). (The probe must be dependent - a requires-
// expression on a fixed type is ill-formed rather than false when the
// call does not exist.)
template <class Q>
constexpr bool has_idle_until = requires { Q::idle_until(std::optional<uint32_t>{}); };
static_assert(!has_idle_until<P>);

struct Ao {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Ao, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) {}
};

void kernel_verbs() {
    Tenuto<P, Ao>::init_all();
    Ao::alarm.arm(10);
    Tenuto<P, Ao>::idle_if_empty();
    P::idle();
    (void)P::interrupts_enabled();
    (void)P::now();
    (void)P::panic_record();
    (void)take_panic_record<P>();
}

void nvic_verbs() {
    {
        InterruptGuard g;
        (void)interrupts_enabled();
    }
    enable_interrupts();
    disable_interrupts();
    Nvic::enable(USART1_IRQn);
    Nvic::disable(USART1_IRQn);
    (void)Nvic::enabled(USART1_IRQn);
    Nvic::set_pending(USART1_IRQn);
    Nvic::clear_pending(USART1_IRQn);
    (void)Nvic::pending(USART1_IRQn);
    (void)Nvic::priority(USART1_IRQn, 3);
    (void)Nvic::priority(USART1_IRQn);
}

void ticker_verbs() {
    using SysClock = Clock<ClockSource::hsi, 16'000'000>;
    (void)Ticker::init(SysClock{});
    Ticker::tick();
    (void)Ticker::ticks();
    (void)Ticker::millis();
    (void)Ticker::secs();
    TimeStamp ts{};
    Ticker::now(ts);
    Ticker::rebase(16'000'000u);
}
