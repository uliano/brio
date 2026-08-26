// Power-management family smoke TU: every package must compile the
// util/power.hpp model, the AVR sleep site that realizes it, and a
// manager with real voters on it (instantiation only).
//
// util/ is target-independent, so nothing here is package-dependent by
// itself; what this TU proves is that the whole composition instantiates
// on every DA/DB - the AVR site over SLPCTRL, a BusMaster voting as the
// arbiter of a bus, and the kernel pack that holds them - on the parts
// with two TWI instances and on the parts with one, and that the pieces
// still agree once avrdx/sleep.hpp includes a util/ header.
#include <stdint.h>

#include "avrdx/platform_avr.hpp"
#include "avrdx/sleep.hpp"
#include "kernel/kernel.hpp"
#include "util/bus_master.hpp"
#include "util/power.hpp"

using namespace brio;

using P = AvrPlatform;

// The ladder is ordered, and the helpers over it fold.
static_assert(rung(SleepDepth::none) == 0);
static_assert(rung(SleepDepth::deep) == 3);
static_assert(shallower(SleepDepth::deep, SleepDepth::light) == SleepDepth::light);
static_assert(!is_deep_mode(SleepDepth::light));
static_assert(is_deep_mode(SleepDepth::standby));
static_assert(SleepSite<AvrSleepSite>);

/// A bus engine reduced to the contract's minimum: the point is the
/// arbiter above it, which is the voter.
struct FakeBus {
    struct Request {
        uint8_t what;
        ReplyTo<BusDone> reply;
    };
    static bool start(const Request&) { return false; }
};

using Bus = BusMaster<FakeBus, P>;

/// A stakeholder that both votes and listens.
struct Watcher : Fsm<Watcher, PrepareSleep, WakeReport> {
    static inline EventQueue<Event, 4, P> queue;
    static inline SleepDepth last = SleepDepth::none;

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](const PrepareSleep& p) { p.reply.send(SleepVote{true}); return handled(); },
            [](WakeReport w) { last = w.was; return handled(); });
    }
};

/// The default configuration and a hand-set one: the knob is an NTTP,
/// so a wrong type would not survive this line.
using Manager = PowerManager<P, AvrSleepSite, PowerConfig{}, Bus, Watcher>;
using Impatient = PowerManager<P, AvrSleepSite, PowerConfig{.min_deep_ticks = 64}, Watcher>;

static_assert(ActiveObject<Manager>);
static_assert(ActiveObject<Impatient>);

using System = Kernel<P, Bus, Watcher, Manager>;

void power_verbs() {
    System::init_all();
    (void)System::step();

    // The site, both ways round.
    for (uint8_t i = 0; i < 4; ++i) {
        const SleepDepth d = static_cast<SleepDepth>(i);
        if (AvrSleepSite::arm(d) && AvrSleepSite::armed() != d) {
            AvrSleepSite::disarm();
        }
    }
    AvrSleepSite::disarm();

    // A request, a standing restriction, and the readbacks.
    post<Manager>(SleepRequested{SleepDepth::standby, {}});
    PowerLock lock = Manager::restrict(SleepDepth::light);
    if (Manager::ceiling() == SleepDepth::light) {
        lock.release();
    }
    PowerLock moved = std::move(lock);
    (void)Manager::armed_depth();
    (void)Impatient::ceiling();

    // The question the deadline guard asks.
    (void)TimeEvents<P>::ticks_to_next();
}
