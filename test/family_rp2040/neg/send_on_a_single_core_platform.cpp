// Must FAIL: a send to an AO whose platform names no Doorbell - here a
// platform of the host's shape, with no second core - is refused: that
// event is post<Ao>()'s (util/inbox.hpp's static_assert).
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "util/inbox.hpp"

struct OneCore {
    struct CriticalSection {};
    static void idle() {}
    static void break_here() {}
    static uint32_t now() { return 0; }
    static constexpr uint32_t ticks_per_second = 1000;
    static constexpr unsigned atomic_width = 4;
    static brio::PanicRecord& panic_record() { static brio::PanicRecord r; return r; }
};

struct Tick {};
struct Lonely : brio::Fsm<Lonely, Tick> {
    static inline brio::EventQueue<Event, 4, OneCore> queue;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};

void crossing() { brio::send<Lonely>(Tick{}); }
