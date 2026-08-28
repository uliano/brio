/*
 * power.hpp
 *
 * The power model: WHO decides that the program may stop its clocks,
 * and how everyone with something at stake gets a say.
 *
 * The mechanism - which sleep mode exists, what it gates, what wakes it
 * - belongs to the target (avrdx/sleep.hpp on AVR DA/DB). What belongs
 * here is the POLICY shape, and it is target-independent: an ordered
 * ladder of depths, a site that arms one, a round of votes among the
 * stakeholders, and standing restrictions for the stakeholders that
 * cannot vote because they live in an interrupt.
 *
 * WHY A ROUND OF VOTES. The kernel's idle hook sleeps by itself, but
 * only in the shallowest mode a target has: there the wake-up list is
 * complete, so "no events queued" is the whole of what it must know.
 * Deeper modes gate clock domains and SHRINK that list - a bus engine
 * mid-transfer, a converter mid-conversion, a driver whose oscillator
 * is about to stop all have facts the requester does not have. So the
 * manager asks them, one PrepareSleep each, and a single not-ok ends
 * the round. Unanimity is required because the cost of being wrong is
 * asymmetric: a refused sleep wastes power, an accepted one loses data.
 *
 * WHY THE SITE ONLY ARMS. Arming a sleep mode and executing the sleep
 * instruction are two different acts on every machine brio targets: the
 * mode sits in a register, and the CPU stops when the idle path takes
 * it. That split is what lets this whole model exist without a new
 * kernel hook - the manager arms, its dispatch returns, the kernel loop
 * finds every queue empty and calls the platform's idle(), and THAT is
 * the sleep. A target whose idle path would otherwise impose its own
 * shallow mode must let an already-armed deeper one stand (avrdx's does,
 * see avrdx/platform_avr.hpp).
 *
 * THE DEADLINE GUARD. Leaving a deep mode is not free - on AVR DA/DB it
 * costs the oscillator's restart plus a separate ~290 us of voltage
 * regulator, and beside a crystal 1.77 ms. Stopping for less than that
 * is a losing trade, so a request for standby or deeper is REFUSED when
 * the nearest armed time event is nearer than `min_deep_ticks`
 * (kernel/time_event.hpp gained ticks_to_next() for exactly this
 * question). The default, two ticks, is that worst case expressed in the
 * coarsest timebase brio runs on; an app with a faster tick or a
 * cheaper wake sets its own.
 *
 * THE FIRST EVENT AFTER A WAKE DISARMS. The manager does not sleep and
 * does not wake anyone: the CPU comes back on an interrupt, that
 * interrupt's handler posts what it always posts, and the manager's next
 * dispatch - of ANY event - first disarms the site and publishes a
 * WakeReport to the stakeholders that declare one. Nothing polls, and
 * the round ends where the program resumes. A wake path that has nothing
 * to say to the manager says it with a SleepRequested{none}: "awake, no
 * new request" is a no-op that still replies ok.
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
 * The ladder, shallowest first. `none` is not a sleep mode: it is the
 * absence of a request - the kernel's plain idle hook stays exactly as
 * it is, which is what every brio program does when it has nothing to
 * do. The deeper rungs are the target's gated modes.
 *
 * A target need not have all four. What it does NOT have it maps to the
 * nearest SHALLOWER mode - never deeper than asked - so a portable app
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
 * saying what is armed. NOT the sleep instruction - see the file header.
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
/// this depth right now?" Answer through `reply` - always, exactly once:
/// a voter that never replies stalls the round (the manager waits for
/// unanimity, it does not time out - a missing reply is a bug, not a
/// condition to recover from).
struct PrepareSleep {
    SleepDepth depth;
    ReplyTo<SleepVote> reply;
};

/// The request itself, posted to the manager by whoever decides the
/// program is done for now (an idle-detecting AO, a console command, a
/// supervisor). `reply` learns the outcome the same way a voter speaks:
/// ok = the site is armed at some depth, not-ok = the round was refused.
/// `depth` is what the requester WANTS; standing restrictions may clamp
/// it and the target may map it shallower.
struct SleepRequested {
    SleepDepth depth;
    ReplyTo<SleepVote> reply;
};

/// Published to the stakeholders after a completed round is disarmed:
/// `was` is the depth the site had been armed at. A stakeholder that
/// declares WakeReport in its event variant receives it; one that does
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
 * object: while it lives, no round may arm deeper than the depth it
 * names.
 *
 * The vote is for stakeholders the manager can ASK - active objects,
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
 * A lock taken while a round is already armed does NOT retroactively
 * disarm it: it applies from the next request on. That is not a gap -
 * the interrupt that takes the lock is itself a wake, and the first
 * event after a wake disarms.
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
 * A voter's event variant MUST accept PrepareSleep - a stakeholder that
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

    /// One request, every voter's answer, and one request re-posted by
    /// the wake path - the deepest this queue can legitimately get.
    static inline EventQueue<Event, sizeof...(Voters) + 3, P> queue;

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
     * round arms deeper than `max_allowed`. Safe from an ISR.
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

    /// What the site was armed at by the round in force, `none` between
    /// rounds. Diagnostics and tests.
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

    /// The site is armed and the CPU has stopped at least once by the
    /// time anything reaches here: ANY event is the wake. Disarm, tell
    /// the stakeholders, and let a fresh request start a fresh round -
    /// re-posted to self so it is judged against the machine as it is
    /// NOW, not as it was before the sleep.
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
    /// NOT publish(): a voter is listed because it has a say in the
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
    /// increment saturates: 255 simultaneous locks at one rung is a
    /// design error, and erring toward keeping the restriction is the
    /// safe half of it.
    static inline uint8_t locks_[4]{};

    static inline SleepDepth requested_ = SleepDepth::none;
    static inline SleepDepth armed_depth_ = SleepDepth::none;
    static inline ReplyTo<SleepVote> requester_{};
    static inline uint8_t outstanding_ = 0;
    static inline bool refused_ = false;
};

} // namespace brio
