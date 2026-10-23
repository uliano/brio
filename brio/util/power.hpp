/*
 * power.hpp
 *
 * The power model: who decides that the program may stop its clocks,
 * and how everyone with something at stake gets a say. A program that
 * wants to sleep deeper than the kernel's own idle lists a
 * PowerManager among its active objects (objects with their own event
 * queues, dispatched by the kernel loop), names the stakeholders that
 * vote and the target's sleep site, and posts a SleepRequested when it
 * knows it is idle; the manager asks every voter, and on a unanimous ok
 * arms the site, and the kernel's next idle is the sleep. The smallest
 * use, as the AVR's power suite (test_avr_power) spells it:
 *
 *     using Pm = brio::PowerManager<P, brio::AvrSleepSite, brio::PowerConfig{}, Bus, Probe>;
 *     brio::Tenuto<P, Probe, Bus, Pm>::run();
 *
 *     // whoever decides the program is done for now:
 *     brio::post<Pm>(brio::SleepRequested{d, brio::reply_to<Probe, brio::SleepVote>()});
 *     // every voter, on PrepareSleep: answer exactly once
 *     p.reply.send(brio::SleepVote{ok});
 *
 * and the wake path that resumes the program posts the same request at
 * depth none, SleepRequested{SleepDepth::none, reply_to<...>()} with the
 * reply going to whoever wants the ok, "awake, no new request" (the
 * prerequisite docs/design/power.md states first, and why it matters on
 * a target whose tick stops). The page this model belongs to is
 * docs/design/power.md, "The power model".
 *
 * Who calls what, and from where:
 *  - SleepRequested and every SleepVote reach the manager through
 *    post(), legal from any context (kernel/post.hpp); the manager
 *    dispatches them in the kernel loop, in main context, and posts
 *    PrepareSleep to each voter, which answers in its own dispatch;
 *  - nothing in the manager waits: a round is a sequence of
 *    dispatches, and the sleep itself is the platform's idle() when
 *    every queue is empty after the site was armed;
 *  - restrict(), ceiling() and a PowerLock's release() take the
 *    platform's critical section for a counter update, so an interrupt
 *    handler may take and give back a lock; armed_depth() is a plain
 *    read for diagnostics.
 *
 * The mechanism - which sleep mode exists, what it gates, what wakes it
 * - belongs to the target (its own sleep.hpp). What belongs here is the
 * policy shape, and it is target-independent: an ordered ladder of
 * depths, a site that arms one, a round of votes among the
 * stakeholders, and standing restrictions for the stakeholders that
 * cannot vote because they live in an interrupt.
 *
 * Why a round of votes (docs/design/power.md, "The round: unanimity
 * among stakeholders"). The kernel's idle hook sleeps by itself, but
 * only in the shallowest mode a target has: there the wake-up list is
 * complete, so "no events queued" is the whole of what it must know.
 * Deeper modes gate clock domains and shrink that list - a bus engine
 * mid-transfer, a converter mid-conversion, a driver whose oscillator
 * is about to stop all have facts the requester does not have. So the
 * manager asks them, one PrepareSleep each, and a single not-ok ends
 * the round. Unanimity is required because the cost of being wrong is
 * asymmetric: a refused sleep wastes power, an accepted one loses data.
 *
 * Why the site only arms (docs/design/power.md, "The site: arming and
 * sleeping are two different acts"). Arming a sleep mode and executing
 * the sleep instruction are two different acts on every machine brio
 * targets: the mode sits in a register, and the CPU stops when the idle
 * path takes it. That split is what lets this whole model exist
 * without a new kernel hook - the manager arms, its dispatch returns,
 * the kernel loop finds every queue empty and calls the platform's
 * idle(), and that is the sleep. A target whose idle path would
 * otherwise impose its own shallow mode must let an already-armed
 * deeper one stand; each target's platform.hpp says how it does.
 *
 * The deadline guard (docs/design/power.md, "The deadline guard").
 * Leaving a deep mode is not free: it costs the oscillator's restart,
 * and on some silicon a separate voltage-regulator bill on top
 * (on the AVR128DB48 bench board: ~290 us of regulator, which
 * docs/avrxt/platform.md measures, and 1.77 ms for the whole wake
 * beside a crystal main clock, this header's own figure - the platform
 * page gives about 1.5 ms for that crystal's restart alone). Stopping
 * for less than that is a losing
 * trade, so a request for standby or deeper is refused when the
 * nearest armed time event is nearer than `min_deep_ticks`
 * (kernel/time_event.hpp's ticks_to_next() answers exactly this
 * question). The default, two ticks, is that worst case expressed in
 * the coarsest timebase brio runs on; an app with a faster tick or a
 * cheaper wake sets its own.
 *
 * The first event after a wake disarms (docs/design/power.md, "The
 * first event after a wake disarms"). The manager does not sleep and
 * does not wake anyone: the CPU comes back on an interrupt, that
 * interrupt's handler posts what it always posts, and the manager's
 * next dispatch - of any event posted to it - first disarms the site
 * and sends a WakeReport to the stakeholders that declare one. Nothing
 * polls, and the round ends where the program resumes. A wake path that has
 * nothing to say to the manager says it with a SleepRequested{none}:
 * "awake, no new request" is a no-op that still replies ok.
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"

namespace brio {

/**
 * The ladder, shallowest first (docs/design/power.md, "The ladder"; a
 * rung is one depth of it). `none` is not a sleep mode: it is the
 * absence of a request - the kernel's plain idle hook stays exactly as
 * it is, which is what every brio program does when it has nothing to
 * do. The deeper rungs are the target's gated modes.
 *
 * A target need not have all four. What it does not have it maps to the
 * nearest shallower mode - never deeper than asked - so a portable app
 * asking for `deep` gets the deepest stop that target really offers, and
 * never a stop that loses more than the app agreed to lose.
 */
enum class SleepDepth : uint8_t {
    none = 0,
    light = 1,
    standby = 2,
    deep = 3,
};

/// Depth as its rung number: the ladder is ordered and code says so
/// once, here, instead of casting at every comparison.
constexpr uint8_t rung(SleepDepth d) { return static_cast<uint8_t>(d); }

/// The shallower of two depths (the clamp a restriction applies).
constexpr SleepDepth shallower(SleepDepth a, SleepDepth b) {
    return rung(a) < rung(b) ? a : b;
}

/// Standby and below: the modes that gate clock domains, shrink the
/// wake-up list and cost real time to leave.
constexpr bool is_deep_mode(SleepDepth d) { return rung(d) >= rung(SleepDepth::standby); }

/**
 * What a target must provide for a manager to run on it: arming, and
 * saying what is armed. Not the sleep instruction - see the file header.
 * The manager calls all three in main context: from its own dispatch,
 * and disarm() also from its init(), which Tenuto calls before the
 * first event.
 *
 *  - `arm(d)`: put the machine in a state where the next idle path stops
 *    at depth d. False = this site cannot or will not do it (a target
 *    lacking the rung maps it shallower instead and returns true; false
 *    is for a real refusal, e.g. an interlock the silicon imposes).
 *  - `disarm()`: back to the kernel's own idle behaviour.
 *  - `armed()`: what a following idle path would take, `none` when
 *    nothing was armed.
 */
template <typename S>
concept SleepSite = requires(SleepDepth d) {
    { S::arm(d) } -> std::same_as<bool>;
    { S::disarm() } -> std::same_as<void>;
    { S::armed() } -> std::same_as<SleepDepth>;
};

// ---- the vocabulary ----------------------------------------------------------

/// A stakeholder's answer. One not-ok ends the round.
struct SleepVote {
    bool ok;
};

/// Asked of every voter before a sleep round: "may the program stop at
/// this depth right now?" Answer through `reply` - always, exactly once,
/// from the voter's own dispatch (reply.send() posts, it does not
/// wait): a voter that never replies stalls the round (the manager
/// waits for unanimity, it does not time out - a missing reply is a
/// bug, not a condition to recover from). A voter's queue therefore
/// needs a slot for the PrepareSleep.
struct PrepareSleep {
    SleepDepth depth;
    ReplyTo<SleepVote> reply;
};

/// The request itself, posted to the manager by whoever decides the
/// program is done for now (an idle-detecting active object, a console
/// command, a supervisor). `reply` learns the outcome the same way a
/// voter speaks: ok = the site is armed at some depth (or, for `none`,
/// nothing was asked of anyone), not-ok = the round was refused - by a
/// voter, by the site, by the deadline guard, or because another round
/// was already running. `depth` is what the requester wants; standing
/// restrictions may clamp it and the target may map it shallower.
struct SleepRequested {
    SleepDepth depth;
    ReplyTo<SleepVote> reply;
};

/// Sent to the stakeholders after a completed round is disarmed: `was`
/// is the depth the site had been armed at. A stakeholder that declares
/// WakeReport in its event variant receives it, posted; one that does
/// not costs nothing (it is a notification, not a subscription).
struct WakeReport {
    SleepDepth was;
};

// ---- standing restrictions ---------------------------------------------------

/// The manager's compile-time knobs. Declared before the lock because
/// the manager's own declaration - which the lock befriends - names it.
struct PowerConfig {
    /// A request for standby or deeper is refused when the nearest armed
    /// time event is nearer than this. Default: two ticks - the worst
    /// wake-up bill brio has measured (a crystal's 1.77 ms restart) in
    /// the coarsest timebase it runs on (1024 Hz).
    uint32_t min_deep_ticks = 2;
};

template <Platform P, SleepSite Site, PowerConfig config, typename... Voters>
class PowerManager;

/**
 * A standing ceiling on how deeply the program may sleep, held as an
 * object: while it lives, no round judged after it was taken may arm
 * deeper than the depth it names (docs/design/power.md, "Standing
 * restrictions: the
 * stakeholders that cannot be asked"). A request deeper than the
 * ceiling is clamped, not refused.
 *
 * The vote is for stakeholders the manager can ask - active objects,
 * dispatched in the loop. A stakeholder that lives in an interrupt (an
 * ISR pump mid-burst, a pin edge that must not be missed) cannot answer
 * a PrepareSleep, and asking it later would be asking too late. It takes
 * a lock instead: `auto lock = Pm::restrict(SleepDepth::light);` at the
 * start of the burst, `lock.release()` or scope exit at its end. The
 * counters are guarded by the platform's critical section, so both ends
 * are safe from an ISR.
 *
 * Non-copyable, movable (a lock is an owned right, and moving it is how
 * it is handed to the object that will end the burst). A moved-from or
 * released lock holds nothing and its destructor does nothing.
 *
 * The ceiling is read when a request is judged, at the start of its
 * round. A lock taken after that - while the voters are asked, or once
 * the site is armed - does not touch that round: it applies from the
 * next request on. Where the lock is taken by the interrupt that woke
 * the CPU from the armed sleep, that is no gap: the first event the
 * wake path posts to the manager disarms. A lock taken while the CPU is
 * still awake, between the arming and the idle, does not stop that
 * sleep: the site stays armed at the depth the round chose, and nothing
 * here checks for it.
 */
class PowerLock {
public:
    using Release = void (*)(SleepDepth);

    constexpr PowerLock() = default;   ///< holds nothing

    PowerLock(const PowerLock&) = delete;
    PowerLock& operator=(const PowerLock&) = delete;

    constexpr PowerLock(PowerLock&& other)
        : release_(other.release_), level_(other.level_) {
        other.release_ = nullptr;
    }

    PowerLock& operator=(PowerLock&& other) {
        if (this != &other) {
            release();
            release_ = other.release_;
            level_ = other.level_;
            other.release_ = nullptr;
        }
        return *this;
    }

    ~PowerLock() { release(); }

    /// Give the right back early; idempotent.
    void release() {
        if (release_ != nullptr) {
            Release r = release_;
            release_ = nullptr;
            r(level_);
        }
    }

    /// The ceiling this lock imposes while it lives.
    constexpr SleepDepth level() const { return level_; }

    /// True while it holds a restriction.
    constexpr explicit operator bool() const { return release_ != nullptr; }

private:
    template <Platform P, SleepSite Site, PowerConfig config, typename... Voters>
    friend class PowerManager;

    constexpr PowerLock(Release r, SleepDepth l) : release_(r), level_(l) {}

    Release release_ = nullptr;
    SleepDepth level_ = SleepDepth::deep;
};

/**
 * The power-management active object.
 *
 * `Voters` are the stakeholders: each is asked before every round and
 * each hears the WakeReport that ends it (if its variant declares one).
 * A voter's event variant must accept PrepareSleep - a stakeholder that
 * cannot be asked does not compile into the list.
 *
 * The manager itself never sleeps and never wakes anyone (see the file
 * header). Its whole surface is: take a request, clamp it, guard it,
 * collect the votes, arm - and disarm again on the first event after the
 * machine comes back.
 */
template <Platform P, SleepSite Site, PowerConfig config = PowerConfig{},
          typename... Voters>
class PowerManager : public Fsm<PowerManager<P, Site, config, Voters...>,
                                SleepRequested, SleepVote> {
    using Self = PowerManager<P, Site, config, Voters...>;
    using Base = Fsm<Self, SleepRequested, SleepVote>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    /// sizeof...(Voters) + 3 events deep (kernel/event_queue.hpp's
    /// depth: the events that can WAIT while one is being dispatched).
    /// What can wait during a round while a vote is dispatched: the other
    /// voters' answers (sizeof...(Voters) - 1), the request the wake path
    /// re-posts, and a second requester's request (refused when served,
    /// collecting() below, but a queued event until then): that is
    /// sizeof...(Voters) + 1, and the request that opened the round is no
    /// longer in the queue. The depth keeps two places of margin, one for
    /// each further hand on the manager that nothing here forbids (a third
    /// requester, a lock's event); a program that counts its requesters
    /// could shrink it, and no suite has needed the margin.
    static inline EventQueue<Event, sizeof...(Voters) + 3, P> queue;

    /// Disarms the site and starts between rounds, every lock count
    /// left as it is. Called by Tenuto, in main context, before the
    /// first event.
    static void init() {
        Site::disarm();
        armed_depth_ = SleepDepth::none;
        outstanding_ = 0;
        refused_ = false;
        Base::start(&idle_state);
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /**
     * Take a standing restriction: while the returned lock lives, no
     * round arms deeper than `max_allowed`. Safe from an interrupt
     * handler (a counter update under the critical section, no wait).
     * It does not disarm a round already armed: it applies from the
     * next request on.
     *
     *   static brio::PowerLock burst;            // released: holds nothing
     *   burst = Pm::restrict(brio::SleepDepth::light);
     *   ...
     *   burst.release();
     */
    static PowerLock restrict(SleepDepth max_allowed) {
        typename P::CriticalSection cs;
        uint8_t& n = locks_[rung(max_allowed)];
        if (n != UINT8_MAX) {
            ++n;
        }
        return PowerLock{&release_lock, max_allowed};
    }

    /// The shallowest active restriction, `deep` when there is none.
    static SleepDepth ceiling() {
        typename P::CriticalSection cs;
        for (uint8_t i = 0; i < 4; ++i) {
            if (locks_[i] != 0) {
                return static_cast<SleepDepth>(i);
            }
        }
        return SleepDepth::deep;
    }

    /// What the site was armed at by the round in force (what the target
    /// really took, which may be shallower than asked), `none` between
    /// rounds. Diagnostics and tests; a plain read, main context.
    static SleepDepth armed_depth() { return armed_depth_; }

private:
    // ---- states -------------------------------------------------------------

    /// Between rounds: a request starts one, a stray vote from a round
    /// that already concluded is dropped.
    static Status idle_state(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](const SleepRequested& r) { return begin_round(r); },
            [](SleepVote) { return Base::handled(); });
    }

    /// Waiting for the voters. A second request while a round runs is
    /// refused rather than queued: the answer would be about a machine
    /// state that no longer exists by the time it is served.
    static Status collecting(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](const SleepRequested& r) {
                r.reply.send(SleepVote{false});
                return Base::handled();
            },
            [](SleepVote v) {
                if (!v.ok) {
                    refused_ = true;
                }
                if (--outstanding_ != 0) {
                    return Base::handled();
                }
                return conclude() ? Base::transition(&armed_state)
                                  : Base::transition(&idle_state);
            });
    }

    /// The site is armed, and any event reaching here is taken as the
    /// wake: normally the CPU has stopped at least once by then, and an
    /// event posted to the manager between the arming and the idle ends
    /// the round before it slept. Disarm, tell
    /// the stakeholders, and let a fresh request start a fresh round -
    /// re-posted to self so it is judged against the machine as it is
    /// now, not as it was before the sleep.
    static Status armed_state(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](const SleepRequested& r) {
                wake();
                post<Self>(r);
                return Base::transition(&idle_state);
            },
            [](SleepVote) {
                wake();
                return Base::transition(&idle_state);
            });
    }

    // ---- the round ----------------------------------------------------------

    static Status begin_round(const SleepRequested& r) {
        const SleepDepth d = shallower(r.depth, ceiling());

        // "Do not sleep" is a legitimate request and the wake path's own
        // acknowledgement: nothing to ask anyone, nothing to arm.
        if (d == SleepDepth::none) {
            Site::disarm();
            r.reply.send(SleepVote{true});
            return Base::handled();
        }
        if (is_deep_mode(d) && !deadline_allows()) {
            r.reply.send(SleepVote{false});
            return Base::handled();
        }

        requested_ = d;
        requester_ = r.reply;
        refused_ = false;

        if constexpr (sizeof...(Voters) == 0) {
            return conclude() ? Base::transition(&armed_state) : Base::handled();
        } else {
            outstanding_ = sizeof...(Voters);
            const PrepareSleep p{d, reply_to<Self, SleepVote>()};
            (post<Voters>(p), ...);
            return Base::transition(&collecting);
        }
    }

    /// Would a deep stop be over before the next thing the program has
    /// to do? No armed time event = nothing says no.
    static bool deadline_allows() {
        const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
        return !next.has_value() || *next >= config.min_deep_ticks;
    }

    /// Arm if the vote was unanimous. True = armed (the caller moves to
    /// armed_state); false = the round is over and nothing is armed.
    static bool conclude() {
        if (refused_ || !Site::arm(requested_)) {
            Site::disarm();
            requester_.send(SleepVote{false});
            return false;
        }
        armed_depth_ = Site::armed();   // what the target really took
        requester_.send(SleepVote{true});
        return true;
    }

    /// End the round the machine has just come back from.
    static void wake() {
        Site::disarm();
        const SleepDepth was = armed_depth_;
        armed_depth_ = SleepDepth::none;
        if constexpr (sizeof...(Voters) > 0) {
            const WakeReport w{was};
            (tell<Voters>(w), ...);
        } else {
            (void)was;   // no stakeholders: the round simply ends
        }
    }

    /// A WakeReport reaches the stakeholders that declare it. Deliberately
    /// not publish(): a voter is listed because it has a say in the
    /// decision, which is not the same as wanting the news afterwards, and
    /// forcing the alternative into every voter's variant would make a
    /// bus engine pay queue slots for a fact it has no use for.
    template <typename Ao>
    static void tell(const WakeReport& w) {
        if constexpr (std::constructible_from<typename Ao::Event, WakeReport>) {
            post<Ao>(w);
        }
    }

    static void release_lock(SleepDepth d) {
        typename P::CriticalSection cs;
        uint8_t& n = locks_[rung(d)];
        if (n != 0) {
            --n;
        }
    }

    /// One counter per rung, indexed by the ceiling a lock imposes; the
    /// effective ceiling is the shallowest rung with a live lock. The
    /// increment saturates at 255 and the release stops at zero: 255
    /// simultaneous locks at one rung is a design error, and past it the
    /// count falls to zero while locks still live (the restriction lifts
    /// with as many locks alive as went uncounted).
    static inline uint8_t locks_[4]{};

    static inline SleepDepth requested_ = SleepDepth::none;
    static inline SleepDepth armed_depth_ = SleepDepth::none;
    static inline ReplyTo<SleepVote> requester_{};
    static inline uint8_t outstanding_ = 0;
    static inline bool refused_ = false;
};

} // namespace brio
