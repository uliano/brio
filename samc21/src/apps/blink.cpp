// blink - the first brio-kernel firmware on the SAM C21: two active
// objects talking.
//
// Blinker toggles the LED on PB23, driven by its own periodic time
// event; Supervisor cycles the blink period (500 -> 250 -> 100 ms)
// every 3 seconds by POSTING a SetPeriod command to the Blinker - the
// canonical AO-to-AO addressed message. No delay loops anywhere: between
// events the CPU is in WFI sleep, woken by the SysTick tick.
//
// Nothing above the target glue knows which silicon this is: the two
// AOs, their events, their queues and every kernel and util header
// below them are target-independent. What this file supplies is the
// clock type, the pin and the vector binding (SysTick_Handler). The
// tick rate is 1000 Hz here and nothing above notices it - the
// kernel's tick opacity, exercised for real.
//
// Wiring: none - the LED on PB23 is on the board.
//
// build: boards = c21j

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "samc21/clock.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/platform.hpp"
#include "samc21/ticker.hpp"

using P = brio::SamPlatform;

// The clock: the ONE truth about CLK_CPU for every driver of this target
// (samc21/clock.hpp). OSC48M undivided; `clock` is an empty tag passed to
// driver inits.
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<'B', 23>;  // PB23

// ---- events -----------------------------------------------------------------
struct Toggle {};                      // Blinker's own heartbeat
struct SetPeriod { uint16_t ticks; };  // command: change the blink period
struct Cycle {};                       // Supervisor's own heartbeat

// ---- the blinker ------------------------------------------------------------
struct Blinker : brio::Fsm<Blinker, Toggle, SetPeriod> {
    static inline brio::EventQueue<Event, 4, P> queue;
    static inline brio::TimeEvent<P, Blinker, Toggle> heartbeat{Toggle{}};

    static void init() {
        Led::output();
        start(&running);
    }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                heartbeat.arm_every(brio::ticks_from_ms<P>(500));
                return handled();
            },
            [](Toggle) {
                Led::toggle();
                return handled();
            },
            [](SetPeriod p) {
                heartbeat.arm_every(p.ticks);  // restart with the new cadence
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }
};

// ---- the supervisor ---------------------------------------------------------
struct Supervisor : brio::Fsm<Supervisor, Cycle> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Supervisor, Cycle> cadence{Cycle{}};

    // periods precomputed to ticks at compile time
    static constexpr uint16_t periods[] = {
        static_cast<uint16_t>(brio::ticks_from_ms<P>(500)),
        static_cast<uint16_t>(brio::ticks_from_ms<P>(250)),
        static_cast<uint16_t>(brio::ticks_from_ms<P>(100)),
    };
    static inline uint8_t index = 0;

    static void init() { start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                cadence.arm_every(brio::ticks_from_secs<P>(3));
                return handled();
            },
            [](Cycle) {
                index = static_cast<uint8_t>((index + 1) % 3);
                brio::post<Blinker>(SetPeriod{periods[index]});
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }
};

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }  // tick + idle wakeup

int main()
{
    SysClock::init();             // OSC48M -> GCLK0 -> CLK_CPU = 48 MHz
    brio::Ticker::init(clock);    // SysTick timebase (runs through WFI sleep)
    brio::enable_interrupts();

    brio::Kernel<P, Blinker, Supervisor>::run();  // never returns
}
