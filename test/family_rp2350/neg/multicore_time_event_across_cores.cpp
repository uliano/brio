// A timer posts LOCALLY: a time event on core 1's timebase may not name
// an AO whose queue is core 0's (kernel/time_event.hpp). Two tickers have
// two phases, and a tick count means nothing across the bridge.
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/platform.hpp"

struct Tick {};

struct Local : brio::Fsm<Local, Tick> {
    static inline brio::EventQueue<Event, 4, brio::Rp2350Platform<0>> queue;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};

brio::TimeEvent<brio::Rp2350Platform<1>, Local, Tick> metronome{Tick{}};
