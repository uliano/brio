// events0 - the event system on the bench: generators routed to EVOUT
// pins, watched with a logic analyzer (and one with the naked eye).
// The driver it exercises is documented in docs/avrxt/evsys.md.
//
// What it shows, without any peripheral other than the EVSYS itself:
//   - a static route, made once at init and never touched again:
//     channel 0 <- PIT / 8192 (an EVEN channel, as the table demands)
//     -> EVOUTF = PF2, the on-board LED. It blinks at 32768 / 8192 =
//     4 Hz with ZERO CPU involvement, also while the kernel sleeps;
//   - a route that an active object REWIRES from its states, every 10 s:
//     channel 1 (ODD: PIT / 64 is legal here, PIT / 8192 is not - try
//     it, it does not compile) -> EVOUTD = PD2 for the analyzer:
//       phase 1: PIT / 64        -> a 512 Hz square wave on PD2
//       phase 2: pin PA2 (button 0, pull-up) -> PD2 follows the button
//                (level: high idle, low while pressed; PORTA pins are
//                legal on channels 0-1 only)
//       phase 3: channel off     -> PD2 flat
//     Entry of each state routes, Exit of the pin state disconnects
//     nothing (the next Entry re-sources the same channel: the
//     generator changes under the same user, which is the point);
//   - a software event: at every phase change one pulse() on channel 9
//     -> EVOUTC = PC2. One CLK_PER cycle wide (42 ns at 24 MHz): the
//     analyzer may or may not catch it - documented, not relied upon;
//     the real test of software events is a user that latches them
//     (an ADC start, for one).
// The console prints each phase with the uptime so the trace can be
// aligned, plus the silicon revision (SYSCFG.REVID: MAJOR<<4 | MINOR,
// 0x1x = A, 0x2x = B) to know which errata items apply.
//
// Wiring: analyzer on PD2 (and PC2 if you want the pulses); LED on PF2;
// button 0 on PA2 to GND (traffic bench). Console 460800.
//
// On the AVR64EA48 board (the AVR EA: six channels, every generator on
// any of them, pins and PIT dividers through two SLOTS per port and two
// in the RTC - evsys.md) the same three routes, placed for that board:
// the static PIT / 8192 on EVOUTB = PB2 (a free pad - never the board's
// LED: no app blinks a LED by itself), the probe channel's PIT / 64
// through the PIT's SECOND slot (the first carries the static route's
// divider: two routes of the PIT at once are two slots), the button the
// board's own on PF3, the software pulses on EVOUTC's ALT1 PC7 (PC2 is
// the I2C pair's dual pad on that board) and on channel 5. With no
// analyzer on that desk, each phase also prints what the chip sees on
// its own EVOUT pads: the edges counted on PD2 and PB2 over 300 ms.

// build: boards = db48,ea48

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrxt/clock.hpp"
#include "avrxt/delay.hpp"
#include "avrxt/evsys.hpp"
#include "avrxt/pin.hpp"
#include "avrxt/platform.hpp"
#include "avrxt/ticker.hpp"
#include "avrxt/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"

#include "avr_board.hpp"

using P = brio::AvrPlatform;

using SysClock = BoardClock;   // the board's HF source (avr_board.hpp)
constexpr SysClock clock;

namespace {

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

// ---- the routes, named ----------------------------------------------------
#if defined(BRIO_BOARD_ea48)
using LedPin = brio::Pin<'B', 2>;                 // EVOUTB: a free pad, not the LED
using ProbePin = brio::Pin<'D', 2>;               // EVOUTD, the analyzer
using PulsePin = brio::Pin<'C', 7>;               // EVOUTC ALT1, software events
using Button0 = brio::Pin<'F', 3>;                // the board's button: pin level

using LedChannel = brio::EventChannel<0>;         // any channel takes any generator
using ProbeChannel = brio::EventChannel<1>;
using PulseChannel = brio::EventChannel<5>;       // software events
using ProbePit = brio::EvPitDiv<64, 1>;           // the PIT's second slot
#else
using LedPin = brio::Pin<'F', 2>;                 // EVOUTF
using ProbePin = brio::Pin<'D', 2>;               // EVOUTD, the analyzer
using PulsePin = brio::Pin<'C', 2>;               // EVOUTC, software events
using Button0 = brio::Pin<'A', 2>;                // generator: pin level

using LedChannel = brio::EventChannel<0>;         // even: PIT/8192 legal
using ProbeChannel = brio::EventChannel<1>;       // odd: PIT/64 legal; PORTA pins legal
using PulseChannel = brio::EventChannel<9>;       // software events, SWEVENTB bit 1
using ProbePit = brio::EvPitDiv<64>;
#endif

// ---- the AO that rewires the probe channel from its states ---------------
struct Next {};

struct Cycler : brio::Fsm<Cycler, Next> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Cycler, Next> phase{Next{}};

    static void init() {
        Button0::input(brio::PinPull::up);
        // The probe user listens to channel 1 for the whole run; only
        // the channel's GENERATOR changes with the state.
        brio::EvOut<ProbePin>::listen(ProbeChannel{});
        brio::EvOut<PulsePin>::listen(PulseChannel{});
        start(&pit64);
    }

    static Status pit64(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                ProbeChannel::source(ProbePit{});               // 512 Hz on PD2
                announce("phase 1: PIT/64 -> PD2 (512 Hz)");
                return handled();
            },
            [](Next) { return transition(&button); },
            [](auto) { return unhandled(); });
    }

    static Status button(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                ProbeChannel::source(brio::EvPin<Button0>{});  // PA2 level on PD2
#if defined(BRIO_BOARD_ea48)
                announce("phase 2: PF3 level -> PD2 (press the button)");
#else
                announce("phase 2: PA2 level -> PD2 (press button 0)");
#endif
                return handled();
            },
            [](Next) { return transition(&quiet); },
            [](auto) { return unhandled(); });
    }

    static Status quiet(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                ProbeChannel::off();                            // PD2 flat
                announce("phase 3: channel off -> PD2 flat");
                return handled();
            },
            [](Next) { return transition(&pit64); },
            [](auto) { return unhandled(); });
    }

private:
    static void announce(const char* what) {
        PulseChannel::pulse();                                  // one CLK_PER on PC2
        brio::TimeStamp ts;
        brio::Ticker::now(ts);
        brio::print(serial, ts, " ", what, brio::crlf);
#if defined(BRIO_BOARD_ea48)
        observe();
#endif
        phase.arm(brio::ticks_from_secs<P>(10));
    }
#if defined(BRIO_BOARD_ea48)
    /// The EVOUT pads read back through their own input buffers: edges
    /// on PD2 (the probe) and PB2 (the static route) over 300 ms, sampled
    /// every 10 us (the loop's reads stretch the window past 300 ms) -
    /// 512 Hz is some 300 to 340 edges, 4 Hz two or three, a level none.
    static void observe() {
        uint16_t probe_edges = 0, led_edges = 0;
        bool probe = ProbePin::read(), led = LedPin::read();
        for (uint16_t i = 0; i < 30000; ++i) {
            brio::delay_us(clock, 10);
            const bool pv = ProbePin::read(), lv = LedPin::read();
            if (pv != probe) ++probe_edges;
            if (lv != led) ++led_edges;
            probe = pv;
            led = lv;
        }
        brio::print(serial, "  seen in 300 ms: PD2 ", probe_edges, " edges (now ", probe ? 1 : 0,
                    "), PB2 ", led_edges, " edges", brio::crlf);
    }
#endif
};

} // namespace

// ---- target glue ------------------------------------------------------------
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    brio::Ticker::init();          // RTC clock chosen here; the PIT events derive from it

    // The static route: no AO, no CPU - made once, lives forever.
    LedChannel::source(brio::EvPitDiv<8192>{});
    brio::EvOut<LedPin>::listen(LedChannel{});
    sei();

#if defined(BRIO_BOARD_ea48)
    brio::print(serial, brio::crlf, "events0 (clk=", xtal ? "XTAL" : "OSCHF",
                ", silicon rev ", brio::hex(SYSCFG.REVID), ")", brio::crlf,
                "PB2 <- PIT/8192 (4 Hz, no CPU, PIT slot 0); PD2 <- channel 1, rewired every 10 s",
                brio::crlf);
#else
    brio::print(serial, brio::crlf, "events0 (clk=", xtal ? "XTAL" : "OSCHF",
                ", silicon rev ", brio::hex(SYSCFG.REVID), ")", brio::crlf,
                "LED PF2 <- PIT/8192 (4 Hz, no CPU); PD2 <- channel 1, rewired every 10 s",
                brio::crlf);
#endif

    brio::Tenuto<P, Cycler>::run();
}
