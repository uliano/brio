// blink - two active objects talking.
//
// Blinker toggles the LED, driven by its own periodic time event;
// Supervisor cycles the blink period (500 -> 250 -> 100 ms) every 3
// seconds by POSTING a SetPeriod command to the Blinker - the canonical
// AO-to-AO addressed message. No delay loops anywhere: between events
// the CPU sleeps, woken by the system counter's tick.
//
// Everything above the target glue is portable: the two AOs, their
// events, their queues and every kernel and util header below them.
// Only the glue lines are this target's - the clock type, the pin, the
// platform, the vector binding.
//
// Wiring: the module's LED has no pin of its own - it is jumpered to
// one. PC0 is the pin every app of this project drives: bonded on all
// five packages of the family, no bus and no analog function on it,
// TIM2_CH3 as its alternate function (a dimmable LED, one day).
//
// build: boards = v006k8

#include <stdint.h>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"

using P = brio::Ch32v00xPlatform<>;

// The clock: the ONE truth about HCLK for every driver of this target
// (ch32v00x/clock.hpp). HSI doubled by the PLL to the part's 48 MHz
// ceiling; `clock` is an empty tag passed to driver inits.
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<'C', 0>;   // the jumpered LED (see the header)

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
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

int main()
{
    SysClock::init();             // HSI x2 -> 48 MHz
    brio::Ticker::init(clock);    // the STK timebase
    brio::enable_interrupts();

    brio::Kernel<P, Blinker, Supervisor>::run();  // never returns
}
