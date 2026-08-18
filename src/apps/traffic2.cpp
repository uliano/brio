// traffic2 - traffic1 with PWM lamps: yellow that is really yellow.
// Read traffic0 and traffic1 first; the state machine here is traffic1's
// UNCHANGED (same states, same events, same transitions). What is NEW:
//   - avrdx/pwm.hpp: TcaPwm<n, port> drives a TCA timer in split mode
//     as six 8-bit PWM channels on pins 0..5 of one port. Two timers,
//     twelve channels, four RGB lamps: TCA1 -> PORTB (LED1, LED2),
//     TCA0 -> PORTC (LED3, LED4);
//   - Lamp is now three PWM channels and a colour TABLE (an Rgb triple
//     per Colour) instead of three on/off pins - the mixed colours get
//     the per-channel levels they need (a green LED is far brighter than
//     the red one; "yellow" wants a lot less green than red). Tune the
//     table on the bench, nothing else moves;
//   - the AOs do not know: they still call LampN::show(Colour). The
//     actuator changed underneath, the logic did not - the whole point
//     of keeping "what to show" and "how to show it" apart.
//
// Behaviour as traffic1: N-S green (LED1) 5 s -> yellow 1.5 s -> all
// red 1 s -> E-W green (LED2) 5 s -> yellow -> all red -> ...
// Pedestrian lamps (LED3 = N-S crossing, LED4 = E-W crossing) show
// red; button 0 or 1 = pedestrian call served at the next all-red
// (walk 5 s, flashing walk 3 s). Every transition is traced on the
// console with the uptime.
//
// Wiring (rewired for the PWM step; traffic0/traffic1 follow it too):
//   LED1 R/G/B  PB0/PB1/PB2   TCA1 WO0-2
//   LED2 R/G/B  PB3/PB4/PB5   TCA1 WO3-5
//   LED3 R/G/B  PC0/PC1/PC2   TCA0 WO0-2
//   LED4 R/G/B  PC3/PC4/PC5   TCA0 WO3-5
//   buttons 0..3 PA2..PA5 to GND (internal pull-ups; PA0/PA1 = crystal)
// LEDs common cathode with series resistors.

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrdx/clock.hpp"
#include "avrdx/pwm.hpp"
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
#include "util/rgb_lamp.hpp"

// The Platform: everything the kernel needs from the machine (critical
// section, idle sleep, the tick clock, ticks_per_second...). Named ONCE
// here; every kernel template below takes it as its first argument.
using P = brio::AvrPlatform;

// The clock: the ONE truth about CLK_PER for every driver of this
// target (avrdx/clock.hpp). 24 MHz crystal on PA0/PA1, OSCHF fallback at
// the same rate; `clock` is an empty tag passed to driver inits.
using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

// Anonymous namespace: everything in it is private to this file (the
// C++ way of saying "static" for types too). Apps keep their AOs here.
namespace {

// The console. Uart<2, alt1> is a MONOSTATE driver: no object, all
// static; `constexpr Serial serial;` is an empty tag object so that
// print(serial, ...) reads naturally and costs nothing.
using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

// ---- an RGB lamp: three PWM channels, one colour --------------------------
// Not an AO: a plain static actuator, exactly as in traffic1 - only the
// "how" changed. brio::RgbLamp<R, G, B> (util/rgb_lamp.hpp) takes three
// PwmChannel types and writes three duties; here the channels are TCA
// split-mode outputs (TcaPwm::Channel<ch>, max 255), in traffic1 they
// are bare pins (max 1) - the SAME lamp code drives both. The colour
// vocabulary (Colour, the palette) stays in the app, above the lamp.
enum class Colour : uint8_t { off, red, green, blue, yellow, white };

// The palette. Levels are per-channel brightness (0 = off, 255 = full)
// and are the ONLY thing to tune on the bench: green and blue dies are
// much more efficient than red at equal current, so a balanced yellow
// takes far less green than red, and white takes less green/blue too.
constexpr brio::Rgb palette[] = {
    /* off    */ {0, 0, 0},
    /* red    */ {255, 0, 0},
    /* green  */ {0, 90, 0},
    /* blue   */ {0, 0, 255},
    /* yellow */ {90, 60, 0},
    /* white  */ {255, 90, 60},
};

template <typename R, typename G, typename B>     // three PwmChannel types
struct Lamp : brio::RgbLamp<R, G, B> {
    static void init() {}                    // the timer is initialized once, below
    static void show(Colour c) {
        brio::RgbLamp<R, G, B>::show(palette[static_cast<uint8_t>(c)]);
    }
};

// Two timers, six channels each. TcaPwm<1, 'B'> = TCA1 routed to PORTB
// (PB0..PB5), TcaPwm<0, 'C'> = TCA0 routed to PORTC (PC0..PC5). Which
// timer reaches which port is a fact of the chip; an impossible route
// is a compile error inside pwm.hpp. Channel<n> is pin n of that port.
using PwmB = brio::TcaPwm<1, 'B'>;
using PwmC = brio::TcaPwm<0, 'C'>;
using Lamp1 = Lamp<PwmB::Channel<0>, PwmB::Channel<1>, PwmB::Channel<2>>;
using Lamp2 = Lamp<PwmB::Channel<3>, PwmB::Channel<4>, PwmB::Channel<5>>;
using Lamp3 = Lamp<PwmC::Channel<0>, PwmC::Channel<1>, PwmC::Channel<2>>;
using Lamp4 = Lamp<PwmC::Channel<3>, PwmC::Channel<4>, PwmC::Channel<5>>;

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
// objects). Intersection is defined below, hence this forward
// declaration.
struct Intersection;

// THE AO. `struct Buttons : brio::Fsm<Buttons, Tick>` reads: Buttons is
// a flat state machine whose events are Entry, Exit (added by Fsm) and
// Tick. The first template argument is Buttons itself (CRTP): a type
// label that gives this AO its own private state variable inside Fsm,
// distinct from any other AO with the same event list.
struct Buttons : brio::Fsm<Buttons, Tick> {
    // The AO contract with the kernel (the ActiveObject concept in
    // kernel/active_object.hpp) needs a member named `queue`:
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
        PORTA.DIRCLR = 0x3C;                                // PA2..PA5 inputs
        PORTA.PIN2CTRL = PORT_PULLUPEN_bm;                  // pull-ups: idle high
        PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
        PORTA.PIN4CTRL = PORT_PULLUPEN_bm;
        PORTA.PIN5CTRL = PORT_PULLUPEN_bm;
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
                // VPORTA.IN read once; buttons on PA2..PA5, active-low.
                const uint8_t raw = static_cast<uint8_t>((~VPORTA.IN >> 2) & 0x0F);  // 1 = pressed
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
                        brio::publish(brio::Subscribers<Intersection>{}, ButtonPressed{i});
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

// ---- Intersection: the traffic light FSM ----------------------------------
// Two private events: PhaseOver (the phase timer expired) and Blink (the
// walk-flash timer). Both are empty structs: their type is the message.
struct PhaseOver {};
struct Blink {};

// Event = variant<Entry, Exit, ButtonPressed, PhaseOver, Blink>.
struct Intersection : brio::Fsm<Intersection, ButtonPressed, PhaseOver, Blink> {
    // Depth 4: a Blink, a PhaseOver and a couple of ButtonPressed could
    // all be pending in the same kernel turn.
    static inline brio::EventQueue<Event, 4, P> queue;

    // ONE one-shot timer for every phase: each state's Entry re-arms it
    // with its own duration. Re-arming a timer simply replaces its
    // deadline. And one periodic timer for the flashing walk.
    static inline brio::TimeEvent<P, Intersection, PhaseOver> phase{PhaseOver{}};
    static inline brio::TimeEvent<P, Intersection, Blink> blink{Blink{}};

    // The pedestrian call, remembered until served. This is the whole
    // point of "taking note": the button event is consumed by whatever
    // state is running, but its EFFECT is deferred to all_red.
    static inline bool ped_call = false;

    // Where the cycle resumes after a walk phase: which vehicle green
    // comes next. Set by the yellow states.
    static inline bool next_is_ns = true;

    static void init() {
        PwmB::init(); PwmC::init();               // both timers: split mode, all dark
        Lamp1::init(); Lamp2::init(); Lamp3::init(); Lamp4::init();
        start(&all_red);         // the initial state: safe by construction
    }

    // ---- vehicle phases ------------------------------------------------------
    // Every phase state has the same shape: Entry paints the lamps, arms
    // the phase timer and traces; PhaseOver moves on; ButtonPressed just
    // takes note. The [](auto) swallows Exit and Blink.
    static Status ns_green(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                paint(Colour::green, Colour::red);
                arm_phase(5000, "ns_green");
                return handled();
            },
            [](PhaseOver) { return transition(&ns_yellow); },
            [](ButtonPressed b) { return note_call(b); },
            [](auto) { return unhandled(); }
        );
    }

    static Status ns_yellow(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                paint(Colour::yellow, Colour::red);
                arm_phase(1500, "ns_yellow");
                next_is_ns = false;              // after this, E-W's turn
                return handled();
            },
            [](PhaseOver) { return transition(&all_red); },
            [](ButtonPressed b) { return note_call(b); },
            [](auto) { return unhandled(); }
        );
    }

    static Status ew_green(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                paint(Colour::red, Colour::green);
                arm_phase(5000, "ew_green");
                return handled();
            },
            [](PhaseOver) { return transition(&ew_yellow); },
            [](ButtonPressed b) { return note_call(b); },
            [](auto) { return unhandled(); }
        );
    }

    static Status ew_yellow(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                paint(Colour::red, Colour::yellow);
                arm_phase(1500, "ew_yellow");
                next_is_ns = true;
                return handled();
            },
            [](PhaseOver) { return transition(&all_red); },
            [](ButtonPressed b) { return note_call(b); },
            [](auto) { return unhandled(); }
        );
    }

    // all_red is the hub: the safety gap between any two phases, and
    // the ONLY place where a pedestrian call is served. When its timer
    // expires it decides where to go - the decision is made when it is
    // due, on the state of the flag at that moment.
    static Status all_red(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                paint(Colour::red, Colour::red);
                arm_phase(1000, "all_red");
                return handled();
            },
            [](PhaseOver) {
                if (ped_call) {
                    ped_call = false;            // consumed: served once
                    return transition(&walk);
                }
                return transition(next_is_ns ? &ns_green : &ew_green);
            },
            [](ButtonPressed b) { return note_call(b); },
            [](auto) { return unhandled(); }
        );
    }

    // ---- pedestrian phases ---------------------------------------------------
    static Status walk(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                Lamp3::show(Colour::green);
                Lamp4::show(Colour::green);
                arm_phase(5000, "walk");
                return handled();
            },
            [](PhaseOver) { return transition(&walk_flash); },
            // A call during the walk: nothing to note, it is being served.
            [](ButtonPressed) { return handled(); },
            [](auto) { return unhandled(); }
        );
    }

    // The flashing phase: Entry arms the periodic blink timer, Exit
    // disarms it. Whatever state comes next, no Blink will ever reach it
    // - Exit is where a state cleans up after itself, guaranteed to run
    // on every way out (the transition machinery calls it, not us).
    static Status walk_flash(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                lit = true;
                blink.arm_every(brio::ticks_from_ms<P>(250));
                arm_phase(3000, "walk_flash");
                return handled();
            },
            [](Blink) {
                lit = !lit;
                Lamp3::show(lit ? Colour::green : Colour::off);
                Lamp4::show(lit ? Colour::green : Colour::off);
                return handled();
            },
            [](brio::Exit) {
                blink.disarm();
                Lamp3::show(Colour::red);
                Lamp4::show(Colour::red);
                return handled();                // Exit's verdict is ignored anyway
            },
            [](PhaseOver) { return transition(&all_red); },
            [](ButtonPressed) { return handled(); },
            [](auto) { return unhandled(); }
        );
    }

private:
    // Helpers are ordinary static functions: states share them freely.
    static void paint(Colour ns, Colour ew) {
        Lamp1::show(ns);
        Lamp2::show(ew);
    }

    static void arm_phase(uint16_t ms, const char* name) {
        phase.arm(brio::ticks_from_ms<P>(ms));
        trace(name);
    }

    // Uptime-stamped console line: "12.345s all_red".
    static void trace(const char* what) {
        brio::TimeStamp ts;
        brio::Ticker::now(ts);
        brio::print(serial, ts, " ", what, brio::crlf);
    }

    // The "take note" handler shared by every vehicle state: buttons 0
    // and 1 are the two crossings' calls (served together), 2 and 3 are
    // free for later steps.
    static Status note_call(ButtonPressed b) {
        if (b.id <= 1 && !ped_call) {
            ped_call = true;
            trace("pedestrian call noted");
        }
        return handled();
    }

    static inline bool lit = false;
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
    SysClock::init();      // 24 MHz crystal (fallback: internal)
    Serial::init(clock, 460800);
    brio::Ticker::init();          // RTC/PIT timebase, before sei()
    sei();

    brio::print(serial, brio::crlf, "traffic2: the light, with pedestrian call", brio::crlf);

    brio::Kernel<P, Intersection, Buttons>::run();
}
