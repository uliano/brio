// mcu: avr128db48
// NEGATIVE: an input that cannot be read is not a ScannedInput.
//
// The scanner's whole contract is one verb - read() returning true for
// ACTIVE - and a type that carries a level under any other name (a pin
// wrapper that forgot to expose it, a driver whose readback is called
// state()) must be refused where it is listed, not silently sampled as
// something else.
#include <stdint.h>

#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "util/input_scanner.hpp"

using namespace brio;

struct Good {
    static bool read() { return Pin<'A', 2>::read(); }
};

/// The level is there, but not behind read().
struct Mute {
    static bool state() { return Pin<'A', 3>::read(); }
};

struct Watcher : Fsm<Watcher, InputEdge> {
    static inline EventQueue<Event, 4, AvrPlatform> queue;
    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](InputEdge) { return handled(); });
    }
};

using Scanner = InputScanner<AvrPlatform, Subscribers<Watcher>, ScanConfig{}, Good, Mute>;

void refused() { Scanner::init(4); }
