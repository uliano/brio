// Platform family smoke TU: the CH32V00x realization of the kernel's
// Platform concept (Ch32v00xPlatform<TB>), the STK timebase it takes by
// default and the interrupt-control verbs below it. The platform is
// NOT tickless (the STK stops with the core), so the kernel's optional
// idle_until must be absent and the loop must compile idle() alone.
#include <optional>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/kernel.hpp"
#include "kernel/panic.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

static_assert(Platform<P>);
static_assert(P::atomic_width == 4);
static_assert(P::ticks_per_second == 1000);
static_assert(Ticker::ticks_per_second == 1000);
static_assert(std::same_as<P::Timebase, Ticker>);

// The kernel's tick conversions fold to the identity at 1000 Hz.
static_assert(ticks_from_ms<P>(500) == 500);
static_assert(ticks_from_secs<P>(3) == 3000);

// No idle_until: the probe must be dependent (a requires-expression on
// a fixed type is ill-formed rather than false when the call does not
// exist).
template <class Pl>
constexpr bool has_idle_until = requires { Pl::idle_until(std::optional<uint32_t>{}); };
static_assert(!has_idle_until<P>);

// The interrupt numbers ARE the vector table's word indices.
static_assert(static_cast<uint8_t>(Irq::systick) == 12);
static_assert(static_cast<uint8_t>(Irq::usart1) == 32);

struct Ao {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Ao, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) {}
};

void kernel_paths() {
    Kernel<P, Ao>::init_all();
    Ao::alarm.arm(10);
    (void)Kernel<P, Ao>::step();
    Kernel<P, Ao>::idle_if_empty();
}

void platform_verbs() {
    P::CriticalSection cs;
    (void)P::now();
    (void)P::interrupts_enabled();
    P::idle();
    (void)take_panic_record<P>();
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
    Ticker::rebase(24'000'000);

    using SlowTicker = BasicTicker<125>;
    (void)SlowTicker::init(clock);
    (void)SlowTicker::millis();
}

void interrupt_verbs() {
    enable_interrupts();
    disable_interrupts();
    (void)interrupts_enabled();
    {
        InterruptGuard guard;
        InterruptGuard nested;   // nests: the inner restores nothing
    }
    Pfic::enable(Irq::usart1);
    (void)Pfic::enabled(Irq::usart1);
    (void)Pfic::pending(Irq::usart1);
    Pfic::disable(Irq::usart1);
}
