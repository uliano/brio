// Platform family smoke TU: the CH32V203 realization of the kernel's
// Platform concept (Ch32vx03Platform<TB>), the 64-bit STK timebase it
// takes by default, the microsecond wait on that counter and the
// interrupt-control verbs below them. The platform is NOT tickless (the
// STK stops with the core), so the kernel's optional idle_until must be
// absent and the loop must compile idle() alone.
#include <optional>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/panic.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 144'000'000>;

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

// The interrupt numbers ARE the vector table's word indices, and the
// tail from 58 up is the DEVICE CLASS's.
static_assert(static_cast<uint8_t>(Irq::breakpoint) == 9);
static_assert(static_cast<uint8_t>(Irq::systick) == 12);
static_assert(static_cast<uint8_t>(Irq::software) == 14);
static_assert(static_cast<uint8_t>(Irq::usart1) == 53);
static_assert(device::device_class != DeviceClass::v20x_d6 ||
                  static_cast<uint8_t>(Irq::dma1_channel8) + 1u == device::vector_count,
              "the CH32V20x_D6 table ends at the eighth DMA channel");
static_assert(device::device_class != DeviceClass::v30x_d8 ||
                  static_cast<uint8_t>(Irq::dma2_channel11) + 1u == device::vector_count,
              "the CH32V30x_D8 table ends at DMA2's eleventh channel");

// The wait's factor is a ceiling: late, never early.
static_assert(delay_rate(144'000'000).cycles_per_us == 144);
static_assert(delay_rate(8'000'000).cycles_per_us == 8);
static_assert(delay_rate(144'000'001).cycles_per_us == 145);
static_assert(delay_rate(0).cycles_per_us == 0);

struct Ao {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Ao, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) {}
};

void kernel_paths() {
    Tenuto<P, Ao>::init_all();
    Ao::alarm.arm(10);
    (void)Tenuto<P, Ao>::step();
    Tenuto<P, Ao>::idle_if_empty();
}

void platform_verbs() {
    P::CriticalSection cs;
    (void)P::now();
    (void)P::interrupts_enabled();
    P::idle();
    (void)take_panic_record<P>();
    (void)stack_untouched();
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
    Ticker::advance(4);
    Ticker::pause();
    Ticker::resume();
    Ticker::rebase(72'000'000);

    using SlowTicker = BasicTicker<125>;
    (void)SlowTicker::init(clock);
    (void)SlowTicker::millis();
}

void delay_verbs() {
    constexpr SysClock clock;
    (void)delay_us(clock, 100);
    (void)delay_us(delay_rate(72'000'000), 10);
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
    Pfic::set_pending(Irq::software);
    Pfic::clear_pending(Irq::software);
    (void)Pfic::active(Irq::software);
    Pfic::disable(Irq::usart1);
}

// The handler attribute is one spelling, whichever way the image is built.
extern "C" BRIO_CH32_INTERRUPT void software_handler() { Pfic::clear_pending(Irq::software); }
