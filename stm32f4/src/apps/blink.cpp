// blink - two active objects talking.
//
// Blinker toggles the user LED, driven by its own periodic time event;
// Supervisor cycles the blink period (500 -> 250 -> 100 ms) every 3
// seconds by POSTING a SetPeriod command to the Blinker - the canonical
// AO-to-AO addressed message. No delay loops anywhere: between events
// the CPU is in WFI sleep, woken by the SysTick tick.
//
// Everything above the target glue is portable: the two AOs, their
// events, their queues and every kernel and util header below them.
// Only the glue lines are this target's - the clock type (the PLL at
// the part's ceiling here), the pin, the platform, the vector binding.
//
// Wiring: none - the user LED is on the board: LD3 on PG13 on the
// STM32F429I-DISC1, LD2 on PA5 on the Nucleo-F446RE, the black pill's
// LED on PC13 (which LIGHTS WHEN LOW: the LED hangs from 3.3 V and the
// pin sinks it). The part selects the pin because one board carries one
// part (stm32f4/CMakeLists.txt); a board file would own this line where
// two boards carry the same part.
//
// build: boards = f429zi,f446re,f411ce,f469ni

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"

using P = brio::Stm32f4Platform<>;

// The clock: the ONE truth about SYSCLK for every driver of this target
// (stm32f4/clock.hpp). The Nucleo-F446RE feeds 8 MHz from its ST-LINK's
// MCO into HSE in bypass, the STM32F429I-DISC1 has an 8 MHz crystal X3 on
// HSE (its MCO route is a solder bridge left open) and the
// 32F469IDISCOVERY an 8 MHz crystal X2 (UM1932 4.3.1); all three run the
// PLL to 180 MHz in over-drive. The black pill has a 25 MHz crystal and
// the F411's 100 MHz ceiling.
#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx) || defined(STM32F469xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

#if defined(STM32F429xx)
using Led = brio::Pin<'G', 13>;  // PG13 = LD3 on the STM32F429I-DISC1
#elif defined(STM32F469xx)
using Led = brio::Pin<'G', 6>;   // PG6 = LD1 on the 32F469IDISCOVERY, lit when low
#elif defined(STM32F411xE)
using Led = brio::Pin<'C', 13>;  // PC13 = the black pill's LED, lit when low
#else
using Led = brio::Pin<'A', 5>;   // PA5 = LD2 on the Nucleo-64
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
    SysClock::init();             // HSE -> PLL -> the part's ceiling
    brio::Ticker::init(clock);    // SysTick timebase (runs through WFI sleep)
    brio::enable_interrupts();

    brio::Tenuto<P, Blinker, Supervisor>::run();  // never returns
}
