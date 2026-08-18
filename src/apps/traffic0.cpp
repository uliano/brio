// traffic0 - a pedestrian-call traffic light on 4 RGB LEDs and 4 buttons,
// built step by step as the LEARNING testbed for the brio AO/FSM model.
// This file is deliberately over-commented: it explains the framework
// idioms where they appear, so it can be read top to bottom.
//
// STEP 0 (this file; later steps are traffic1, traffic2, ... - every
// step stays as its own app): the plumbing only. Buttons samples the 4 keys
// every 10 ms, debounces them and PUBLISHES ButtonPressed{id}; Demo
// subscribes and, on button n, steps the colour of LED n (off -> red ->
// green -> blue) and traces the event on the console. Nothing waits,
// nothing polls in a loop: a timer posts Tick, Tick becomes an event,
// the event becomes a published fact, the fact reaches whoever cares.
//
// Wiring (common-cathode RGB: a colour is ON with the pin HIGH; buttons
// to GND with the internal pull-up: pressed = LOW):
//   LED1 R/G/B = PA2/PA3/PA4      LED3 R/G/B = PC0/PC1/PC2
//   LED2 R/G/B = PA5/PA6/PA7      LED4 R/G/B = PC3/PC4/PC5
//   buttons 0..3 = PB0..PB3
// Console @ 460800 on USART2 ALT1 (PF4/PF5). Crystal on PA0/PA1.
//
// THE MODEL IN ONE PARAGRAPH. An active object (AO) is a class with an
// event QUEUE and a dispatch() function; the kernel loop pops one event
// from the highest-priority non-empty queue and calls that AO's
// dispatch(), which runs to completion (no preemption between AOs, no
// blocking inside). Events are small values copied into the queue.
// Every AO here is written as a flat state machine (Fsm): the current
// state IS a function, dispatch calls it, and it answers with a verdict
// (handled / unhandled / transition to another state). Timers do not
// call code: they post an event to their owner AO when they expire.
// The kernel sleeps when every queue is empty.

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/uart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"

// The Platform: everything the kernel needs from the machine (critical
// section, idle sleep, the tick clock, ticks_per_second...). Named ONCE
// here; every kernel template below takes it as its first argument.
using P = brio::AvrPlatform;

// Anonymous namespace: everything in it is private to this file (the
// C++ way of saying "static" for types too). Apps keep their AOs here.
namespace {

// The console. Uart<2, alt1> is a MONOSTATE driver: no object, all
// static; `constexpr Serial serial;` is an empty tag object so that
// print(serial, ...) reads naturally and costs nothing.
using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

// ---- an RGB lamp: three pins, one colour ------------------------------------
// Not an AO: a plain static actuator. It has no state and no events, it
// just turns three pins into a colour. AOs call it from their handlers.
enum class Colour : uint8_t { off, red, green, blue, yellow, white };

template <typename R, typename G, typename B>     // three brio::Pin<> types
struct Lamp {
    static void init() {
        R::clear(); G::clear(); B::clear();       // colours off (common cathode)
        R::output(); G::output(); B::output();
    }
    static void show(Colour c) {
        const bool r = (c == Colour::red || c == Colour::yellow || c == Colour::white);
        const bool g = (c == Colour::green || c == Colour::yellow || c == Colour::white);
        const bool b = (c == Colour::blue || c == Colour::white);
        if (r) R::set(); else R::clear();
        if (g) G::set(); else G::clear();
        if (b) B::set(); else B::clear();
    }
};

// Pin<'A', 2> is a compile-time pin: port letter and number are template
// arguments, set()/clear() compile to single-instruction VPORT accesses.
using Lamp1 = Lamp<brio::Pin<'A', 2>, brio::Pin<'A', 3>, brio::Pin<'A', 4>>;
using Lamp2 = Lamp<brio::Pin<'A', 5>, brio::Pin<'A', 6>, brio::Pin<'A', 7>>;
using Lamp3 = Lamp<brio::Pin<'C', 0>, brio::Pin<'C', 1>, brio::Pin<'C', 2>>;
using Lamp4 = Lamp<brio::Pin<'C', 3>, brio::Pin<'C', 4>, brio::Pin<'C', 5>>;

// ---- the shared fact: somebody pressed a button ----------------------------
// EVENTS ARE PLAIN STRUCTS. This one is the lingua franca between the
// producer (Buttons) and any consumer: no global enum of signals, no
// base class - a struct that both sides can name. Keep events small:
// they are copied into queues.
struct ButtonPressed { uint8_t id; };     // 0..3

// ---- Buttons: samples, debounces, publishes ---------------------------------
// A private event: Buttons' own heartbeat. Empty struct = zero payload;
// its TYPE is the information.
struct Tick {};

// publish() fans a fact out to a compile-time list of subscribers, so
// Buttons must know the subscriber's TYPE (not an object - there are no
// objects). Demo is defined below, hence this forward declaration.
struct Demo;

// THE AO. `struct Buttons : brio::Fsm<Buttons, Tick>` reads: Buttons is
// a flat state machine whose events are Entry, Exit (added by Fsm) and
// Tick. The first template argument is Buttons itself (CRTP): a type
// label that gives this AO its own private state variable inside Fsm,
// distinct from any other AO with the same event list.
struct Buttons : brio::Fsm<Buttons, Tick> {
    // The AO contract with the kernel needs a member named `queue`:
    // EventQueue<Event, depth, P>. Event is the variant Fsm built for us
    // (std::variant<Entry, Exit, Tick>). Depth 2: at most one Tick can
    // be pending while another is being handled - sized on the real
    // burst of THIS AO. `static inline` = one instance in .bss, no
    // constructor at runtime, no object to pass around.
    static inline brio::EventQueue<Event, 2, P> queue;

    // A time event: a timer OWNED by Buttons which, on expiry, POSTS the
    // payload given in braces (Tick{}) into Buttons' queue. It is not a
    // callback: expiry becomes an event served in dispatch like any
    // other. Constructed at static init, disarmed until armed below.
    static inline brio::TimeEvent<P, Buttons, Tick> sampler{Tick{}};

    static constexpr uint8_t count = 4;
    static constexpr uint8_t stable_samples = 3;    // 3 x 10 ms = 30 ms

    // init(): the second thing the contract requires. The kernel calls
    // it once for every AO before entering its loop. Hardware setup of
    // what this AO owns, then start(&initial_state): arm the machine
    // and deliver its first Entry - synchronously, right here.
    static void init() {
        for (uint8_t i = 0; i < count; ++i) {
            PORTB.DIRCLR = static_cast<uint8_t>(1u << i);   // inputs
        }
        PORTB.PIN0CTRL = PORT_PULLUPEN_bm;                  // pull-ups: idle high
        PORTB.PIN1CTRL = PORT_PULLUPEN_bm;
        PORTB.PIN2CTRL = PORT_PULLUPEN_bm;
        PORTB.PIN3CTRL = PORT_PULLUPEN_bm;
        start(&sampling);
    }

    // A STATE IS A FUNCTION: static, takes the event, returns a Status
    // (the verdict). Fsm keeps a pointer to the current one and calls it
    // from dispatch(). This AO has a single state; a one-state machine
    // still earns its Entry (below) for free.
    static Status sampling(const Event& e) {
        // brio::match(e, lambdas...) looks at which alternative `e`
        // currently holds and calls the lambda whose parameter type
        // matches (it is std::visit under the hood - see kernel/fsm.hpp).
        // Every lambda must return the same type (Status). The [](auto)
        // at the end catches every alternative not listed (here Exit) -
        // without it the code would not compile: variant dispatch is
        // exhaustive.
        return brio::match(e,
            // Entry: delivered once, when this state is entered (here:
            // by start() in init). The natural place to arm timers -
            // "the action of a state lives in its Entry".
            [](brio::Entry) {
                sampler.arm_every(brio::ticks_from_ms<P>(10));   // periodic, drift-free
                return handled();
            },
            // Tick: the sampler expired and posted this; we are now
            // inside Buttons' own dispatch, main context, no ISR rules.
            [](Tick) {
                // VPORTB.IN read once; buttons are active-low, invert.
                const uint8_t raw = static_cast<uint8_t>(~VPORTB.IN & 0x0F);  // 1 = pressed
                for (uint8_t i = 0; i < count; ++i) {
                    const bool now = raw & (1u << i);
                    if (now == pressed[i]) {
                        run[i] = 0;                       // no change: nothing to debounce
                        continue;
                    }
                    if (++run[i] < stable_samples) {
                        continue;                         // not stable yet
                    }
                    run[i] = 0;
                    pressed[i] = now;                     // accept the new level
                    if (now) {
                        // THE FACT GOES OUT. One copy per subscriber
                        // lands in each subscriber's queue; the kernel
                        // will dispatch it to them later, in priority
                        // order. Buttons neither waits nor knows what
                        // they do with it. Would not compile if Demo's
                        // Event could not hold a ButtonPressed.
                        brio::publish(brio::Subscribers<Demo>{}, ButtonPressed{i});
                    }
                }
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

private:
    // AO state variables: static inline like everything else. They
    // persist across dispatches; only this AO touches them, and every
    // dispatch runs to completion, so no locking is ever needed.
    static inline bool pressed[count]{};
    static inline uint8_t run[count]{};
};

// ---- Demo: one lamp per button, colour steps on each press ------------------
// The subscriber. Its event list holds ButtonPressed - that is what
// makes publish(Subscribers<Demo>{}, ButtonPressed{...}) legal above.
struct Demo : brio::Fsm<Demo, ButtonPressed> {
    // Depth 4: four buttons could all be published in one Buttons tick.
    static inline brio::EventQueue<Event, 4, P> queue;

    static void init() {
        Lamp1::init(); Lamp2::init(); Lamp3::init(); Lamp4::init();
        start(&running);
    }

    static Status running(const Event& e) {
        return brio::match(e,
            // The lambda receives the event BY VALUE (it is 1 byte); a
            // larger event would be taken as `const T&`.
            [](ButtonPressed b) {
                if (b.id < 4) {
                    step[b.id] = static_cast<uint8_t>((step[b.id] + 1) % 4);
                    show(b.id, static_cast<Colour>(step[b.id]));   // off,red,green,blue
                    // print blocks only if the TX ring is full (256
                    // bytes); a short line returns in microseconds and
                    // the UART ISR drains it while we go on.
                    brio::print(serial, "button ", b.id, " -> lamp ", b.id + 1,
                                " colour ", step[b.id], brio::crlf);
                }
                return handled();
            },
            [](auto) { return unhandled(); }     // Entry, Exit: nothing to do
        );
    }

private:
    static void show(uint8_t lamp, Colour c) {
        switch (lamp) {
            case 0: Lamp1::show(c); break;
            case 1: Lamp2::show(c); break;
            case 2: Lamp3::show(c); break;
            default: Lamp4::show(c); break;
        }
    }
    static inline uint8_t step[4]{};
};

} // namespace

// ---- target glue ------------------------------------------------------------
// Interrupt vectors are irreducibly target-specific, so they live in the
// app: each one just calls the driver's handler body. The PIT tick is
// the kernel's timebase (1024 Hz): it wakes the CPU from idle sleep,
// and the kernel loop then checks which time events matured.
ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    brio::init_clock_24mhz();      // 24 MHz crystal (fallback: internal)
    Serial::init(460800);
    brio::Ticker::init();          // RTC/PIT timebase, before sei()
    sei();

    brio::print(serial, brio::crlf, "traffic0: buttons -> lamps", brio::crlf);

    // Kernel<P, Aos...>::run(): calls init() on every AO (in order),
    // then loops forever: pop the highest-priority non-empty queue,
    // dispatch one event, rescan from the top; all empty -> idle sleep
    // until the next interrupt. Priority IS the order written here:
    // Demo before Buttons means a pending ButtonPressed is served before
    // the next sampling Tick. Never returns.
    brio::Kernel<P, Demo, Buttons>::run();
}
