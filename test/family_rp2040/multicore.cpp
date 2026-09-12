// Multicore family smoke TU: the platform per core, the doorbell, the
// launch of core 1, and util/inbox.hpp instantiated over two RP2040
// platforms - a send, a drain, a crossing reply.
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/platform.hpp"
#include "util/inbox.hpp"

using namespace brio;

using P0 = Rp2040Platform<0>;
using P1 = Rp2040Platform<1>;
static_assert(P0::core == 0 && P1::core == 1);
static_assert(std::same_as<P0::Doorbell, SioDoorbell<0>> && std::same_as<P1::Doorbell, SioDoorbell<1>>);
static_assert(std::same_as<P1::Timebase, CoreTicker<1>>);
static_assert(!std::same_as<CoreTicker<0>, CoreTicker<1>>);
static_assert(CoreAware<P0> && CoreAware<P1>);
static_assert(SioDoorbell<1>::irq() == SIO_IRQ_PROC1_IRQn);

struct Ping { uint8_t n; };
struct Pong { uint8_t n; };

struct Remote : Fsm<Remote, Ping> {
    static inline EventQueue<Event, 4, P1> queue;
    static constexpr uint8_t inbox_depth = 16;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};
struct Local : Fsm<Local, Pong> {
    static inline EventQueue<Event, 4, P0> queue;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};
static_assert(Inbox<Remote>::capacity() == 16 && Inbox<Local>::capacity() == 8);
static_assert(queue_on<Remote, P1>() && !queue_on<Remote, P0>());

using K0 = Kernel<P0, Local>;
using K1 = Kernel<P1, Remote>;

[[noreturn]] void core1_entry() {
    Inboxes<Remote>::enable();
    K1::run();
}

void multicore_verbs() {
    (void)Core1::launch(&core1_entry);
    (void)Core1::launch(&core1_entry, Core1::default_stack_top());
    (void)Core1::reset();
    SioDoorbell<1>::ring();
    SioDoorbell<0>::pop_all();
    (void)SioDoorbell<0>::pending();
    SioDoorbell<0>::enable();
    SioDoorbell<0>::disable();
    send<Remote>(Ping{1});
    send<Local>(Pong{2});
    (void)Inbox<Remote>::overflows();
    (void)Inbox<Remote>::pending();
    Inbox<Remote>::clear();
    Inboxes<Local>::isr();
    (void)Inboxes<Local>::idle();
    (void)send_reply_to<Local, Pong>();
    (void)Remote::queue.misposts();
    (void)P0::on_own_core();
    K0::init_all();
    (void)K0::step();
}
