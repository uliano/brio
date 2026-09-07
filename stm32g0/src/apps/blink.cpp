// blink - the first brio-kernel firmware on the STM32G0: two active
// objects talking.
//
// Blinker toggles LD4 on PA5, driven by its own periodic time event;
// Supervisor cycles the blink period (500 -> 250 -> 100 ms) every 3
// seconds by POSTING a SetPeriod command to the Blinker - the canonical
// AO-to-AO addressed message. No delay loops anywhere: between events
// the CPU is in WFI sleep, woken by the SysTick tick.
//
// This app is a PORT of the AVR and SAM projects' blink.cpp and its
// point is what did NOT change, for the THIRD time: the two AOs, their
// events, their queues and every kernel and util header below them
// compile untouched. Only the target-glue lines differ - the clock type
// (the PLL at 64 MHz here), the pin, the platform, the vector binding.
//
// Wiring: none - the user LED is on the board: LD4 on PA5 on the
// Nucleo-64s (G0B1RE, G071RB), LD3 on PC6 on the Nucleo-32 (G031K8). The
// part selects the pin because on this desk the part IS the board (one
// Nucleo per part - stm32g0/CMakeLists.txt); a board file would own this
// line on a desk with two boards of one part.
//
// build: boards = g0b1re,g071rb,g031k8

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/ticker.hpp"

using P = brio::Stm32g0Platform<>;

// The clock: the ONE truth about SYSCLK for every driver of this target
// (stm32g0/clock.hpp). HSI16 through the PLL to the part's 64 MHz
// ceiling; `clock` is an empty tag passed to driver inits.
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

#if defined(STM32G031xx)
using Led = brio::Pin<'C', 6>;  // PC6 = LD3 on the Nucleo-32
#else
using Led = brio::Pin<'A', 5>;  // PA5 = LD4 on the Nucleo-64s
#endif

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
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }  // tick + idle wakeup

int main()
{
    SysClock::init();             // HSI16 -> PLL -> SYSCLK = 64 MHz
    brio::Ticker::init(clock);    // SysTick timebase (runs through WFI sleep)
    brio::enable_interrupts();

    brio::Kernel<P, Blinker, Supervisor>::run();  // never returns
}
