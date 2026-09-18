// Multicore family smoke TU: the doorbell of both cores, the mailbox
// FIFO, the launch of core 1 with its entry shim, the platform's
// per-core members and the whole of util/inbox.hpp over them - every
// verb instantiated once, for both architectures and both packages.
#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "rp2350/multicore.hpp"
#include "rp2350/platform.hpp"
#include "util/inbox.hpp"

using namespace brio;

using P0 = Rp2350Platform<0>;
using P1 = Rp2350Platform<1>;

// A platform that is one core of several names a Doorbell, which is what
// util/inbox.hpp requires of it.
static_assert(std::same_as<P0::Doorbell, SioDoorbell<0>>);
static_assert(std::same_as<P1::Doorbell, SioDoorbell<1>>);
static_assert(SioDoorbell<0>::irq() == SioDoorbell<1>::irq());
static_assert(SioDoorbell<0>::flags == 0xFFu);
static_assert((SioDoorbell<0>::bell & SioDoorbell<0>::flags) != 0u);
static_assert(SioMailbox::irq() != SioDoorbell<0>::irq());
static_assert(sizeof(Core1Frame) == 16u);

struct Ping {
    uint16_t n;
};
struct Pong {
    uint16_t n;
};

/// An AO of core 0 that receives from core 1.
struct Origin : Fsm<Origin, Pong> {
    static inline EventQueue<Event, 8, P0> queue;
    static constexpr uint8_t inbox_depth = 4;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(
            e, [](Entry) { return handled(); }, [](Pong) { return handled(); },
            [](auto) { return unhandled(); });
    }
};

/// An AO of core 1 that receives from core 0.
struct Echo : Fsm<Echo, Ping> {
    static inline EventQueue<Event, 8, P1> queue;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(
            e, [](Entry) { return handled(); },
            [](Ping p) {
                send<Origin>(Pong{p.n});
                return handled();
            },
            [](auto) { return unhandled(); });
    }
};

using K0 = Tenuto<P0, Origin>;
using K1 = Tenuto<P1, Echo>;
using Drain0 = Inboxes<Origin>;
using Drain1 = Inboxes<Echo>;

static_assert(Inbox<Echo>::capacity() == 8u);    // the default depth
static_assert(Inbox<Origin>::capacity() == 4u);  // the AO's own

[[noreturn]] void core1_entry() {
    Drain1::enable();
    enable_interrupts();
    K1::run();
}

void doorbell_verbs() {
    (void)SioDoorbell<0>::irq();
    SioDoorbell<0>::ring();
    SioDoorbell<1>::ring();
    (void)SioDoorbell<0>::pop_all();
    (void)SioDoorbell<0>::pending();
    (void)SioDoorbell<1>::pending();
    (void)SioDoorbell<0>::enable();
    (void)SioDoorbell<0>::disable();
    core_send_event();
}

void mailbox_verbs() {
    (void)SioMailbox::writable();
    (void)SioMailbox::readable();
    (void)SioMailbox::push(1u);
    uint32_t word = 0;
    (void)SioMailbox::pop(word);
    (void)SioMailbox::drain();
    (void)SioMailbox::read_on_empty();
    (void)SioMailbox::write_on_full();
    SioMailbox::clear_errors();
}

void launch_verbs() {
    (void)Core1::default_stack_top();
    (void)Core1::launch(&core1_entry);
    (void)Core1::launch(&core1_entry, Core1::default_stack_top() - 1024u);
    (void)Core1::protocol(&core1_entry);
    (void)Core1::reset();
    (void)Core1::entered();
    (void)Core1::trace[0];
    (void)Core1::trace_count;
    (void)core_global_pointer();
    (void)core_vector_base();
    void (*shim)() = &core1_trampoline;
    (void)shim;
}

void bridge_verbs() {
    K0::init_all();
    (void)K0::step();
    send<Echo>(Ping{1});
    (void)Inbox<Echo>::send(Ping{2});
    (void)Inbox<Echo>::drain();
    (void)Inbox<Echo>::pending();
    (void)Inbox<Echo>::overflows();
    Inbox<Echo>::clear();
    Drain0::isr();
    Drain0::enable();
    (void)Drain0::idle();
    (void)Origin::queue.misposts();
    constexpr auto reply = send_reply_to<Origin, Pong>();
    (void)reply;
}
