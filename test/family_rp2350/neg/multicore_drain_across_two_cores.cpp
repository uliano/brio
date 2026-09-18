// One drain serves ONE core: Inboxes<...> over AOs whose queues sit on
// two different platforms is refused (util/inbox.hpp), because the bell
// it pops and the posts it makes belong to one core.
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "rp2350/platform.hpp"
#include "util/inbox.hpp"

struct Tick {};

struct Here : brio::Fsm<Here, Tick> {
    static inline brio::EventQueue<Event, 4, brio::Rp2350Platform<0>> queue;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};

struct There : brio::Fsm<There, Tick> {
    static inline brio::EventQueue<Event, 4, brio::Rp2350Platform<1>> queue;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};

void one_drain_two_cores() { brio::Inboxes<Here, There>::isr(); }
