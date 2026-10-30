/*
 * platform.hpp
 *
 * The host's implementation of the brio Platform concept
 * (kernel/platform.hpp: the set of requirements a platform type meets -
 * a critical section, a clock, idle(), break_here(), the panic record,
 * the atomic width), for a native test process. A host test names
 * `HostPlatform` wherever a target program names its family's platform,
 * and the kernel and util/ run over it unchanged (docs/host/README.md, "The host platform").
 *
 * The smallest use, as test/test_time_event spells it: the platform as
 * the template argument, its state put back at the start of each case,
 * and time advanced by hand:
 *
 *     using TE = brio::TimeEvents<brio::HostPlatform>;
 *     static inline brio::EventQueue<Event, 8, brio::HostPlatform> queue;
 *
 *     brio::HostPlatform::reset();      // the start of every test case
 *     ++brio::HostPlatform::ticks;      // one tick of the virtual clock
 *     TE::process();                    // then assert what was posted
 *
 * What each piece does: the critical section only tracks its nesting
 * depth and counts its entries (tests are single threaded, so there is
 * nothing to mask); the clock is a plain counter the test advances by
 * hand, so time becomes deterministic arithmetic; and idle() and
 * break_here() record their calls so tests can assert on them - no
 * sleep, no trap. All state is static inline so tests reach it without
 * plumbing; call reset() at the start of each test case. HostCore<n>
 * below is the two-core host of the inbox's suite (test_inbox), and
 * HostDoorbell<core> its counting doorbell; RubatoHostPlatform is the
 * same platform made preemptive, with the scheduler's entry and exit a
 * target core keeps, for the Rubato kernel's tests.
 */

#pragma once

#include <stdint.h>

#include "kernel/platform.hpp"

namespace brio {

struct HostPlatform {
    class CriticalSection {
    public:
        CriticalSection() {
            ++depth;
            ++entries;
        }
        ~CriticalSection() { --depth; }
        CriticalSection(const CriticalSection&) = delete;
        CriticalSection& operator=(const CriticalSection&) = delete;

        /// Current nesting depth; must be back to 0 when no guard is alive.
        static inline uint8_t depth = 0;
        /// How many times a guard was taken: a path that claims to need
        /// none is checked by this staying still.
        static inline uint32_t entries = 0;
    };

    static inline uint32_t ticks = 0;       ///< advanced by the test
    static inline uint8_t current_core = 0; ///< which core "runs" (HostCore below)
    static inline uint32_t idle_calls = 0;  ///< how many times idle() ran
    static inline uint32_t break_calls = 0; ///< how many times break_here() ran
    static inline bool in_isr = false;       ///< raised by a test's harness inside a simulated interrupt

    static void idle() { ++idle_calls; }
    static void break_here() { ++break_calls; }
    static uint32_t now() { return ticks; }

    /// The optional member of kernel/platform.hpp: on the host, whether a
    /// test's harness has raised `in_isr` around the call.
    static bool in_handler() { return in_isr; }

    /// 1000 Hz: the identity case for ms conversions. Tests exercising a
    /// non-dividing rate define their own local platform (e.g. tps=1024).
    static constexpr uint32_t ticks_per_second = 1000;

    /// A 32-bit-class value: on the host every index a brio ring can
    /// derive is lock-free. Tests of the guarded path define a local
    /// platform with atomic_width = 1 (see test_ring).
    static constexpr unsigned atomic_width = 4;

    /// On the host a plain static plays the reset-surviving storage;
    /// reset() clears it like a cold boot would leave it invalid.
    static PanicRecord& panic_record() {
        static PanicRecord rec{};
        return rec;
    }

    static void reset() {
        ticks = 0;
        current_core = 0;
        idle_calls = 0;
        break_calls = 0;
        in_isr = false;
        CriticalSection::depth = 0;
        CriticalSection::entries = 0;
        panic_record() = PanicRecord{};
    }
};

static_assert(Platform<HostPlatform>);

/// The host platform of a Rubato kernel (kernel/rubato.hpp): HostPlatform
/// with the three members of a preemptive platform (kernel/platform.hpp),
/// the entry and the exit that contract asks of the platform's scheduler,
/// and what a test needs to simulate an interrupt over a dispatch or
/// inside the scheduler itself.
///  - pend_schedule() records the request in `pending` and, unless the
///    test has raised the base's `in_isr`, takes it at once: the host's
///    image of a scheduling interrupt taken at once. Inside a simulated
///    interrupt the request waits, and the test's harness calls
///    interrupt_exit() at the interrupt's exit, which takes it.
///  - take_request() is the scheduling interrupt itself, with the target
///    cores' entry and exit: while a request is pending it lowers it,
///    runs `entry_hook` (an interrupt landing after the request was taken
///    and before the entry's store, the window a Cortex-M or a QingKe V4
///    has and Hazard3 has not; a request it makes is taken right after
///    the store, as a run over the entry), reads the ceiling and stores 0
///    in its place, calls `scheduler` with what it read, then puts that
///    back and compares the count of quiet posts with the one the
///    scheduler handed back - with no landing place inside either step,
///    as a core makes each whole - raising the request again when the
///    count moved, which the same loop then takes in place of the run
///    that just ended. `requests` counts the posts that asked, `retakes` the exits
///    that asked again.
///  - `scheduler` is the kernel's activate(), the one indirect call the
///    host pays: the test sets it. Null, a request is recorded and nothing
///    runs (a scheduling interrupt not yet enabled).
///  - preempt_point() is where a test object's handler, or the time-event
///    walk (kernel/time_event.hpp), lets a simulated interrupt land: it
///    runs the test's `preempt_hook`, if any. scan_point() is the same
///    inside the scheduler (kernel/platform.hpp's SchedulerPoint): it runs
///    `scan_hook`, if any.
/// The counters, the clock and the critical section are the base's: one
/// set, shared. reset() clears all of it.
struct RubatoHostPlatform : HostPlatform {
    static constexpr bool preemptive = true;
    static inline uint8_t ceiling = 255;     ///< the index dispatched or just served; 0 choosing; 255 at the base

    static inline bool pending = false;      ///< a request not yet taken
    static inline uint32_t requests = 0;     ///< how many times pend_schedule() ran
    static inline uint32_t retakes = 0;      ///< how many exits found the count moved
    static inline Activation (*scheduler)(uint8_t) = nullptr;
    static inline void (*preempt_hook)() = nullptr;
    static inline void (*entry_hook)() = nullptr;
    static inline void (*scan_hook)(SchedulerPoint, uint8_t) = nullptr;
    static inline bool in_entry = false;     ///< `entry_hook` running

    static void pend_schedule() {
        ++requests;
        pending = true;
        if (!in_isr) {
            take_request();
        }
    }

    static void take_request() {
        while (pending && scheduler != nullptr) {
            pending = false;
            if (entry_hook != nullptr) {
                in_entry = true;
                entry_hook();
                in_entry = false;
            }
            const uint8_t base = ceiling;      // the entry: read, 0 stored
            ceiling = 0;
            if (pending) {
                // A request made in the entry's window, which the core
                // takes at the entry's unmask: a run over this one's entry,
                // at the ceiling 0 just stored.
                take_request();
            }
            const Activation a = scheduler(base);
            ceiling = base;                    // the exit: base back, the count compared
            if (quiet_posts<RubatoHostPlatform> != a.count) {
                ++retakes;
                pending = true;
            }
        }
    }

    /// What a simulated interrupt's exit does with a request its posts
    /// made: takes it - unless that interrupt landed in the entry's window
    /// (`entry_hook`), where the scheduling interrupt is already in
    /// progress and the core takes the request after the entry's store.
    static void interrupt_exit() {
        if (!in_entry) {
            take_request();
        }
    }

    static void preempt_point() {
        if (preempt_hook != nullptr) {
            preempt_hook();
        }
    }

    static void scan_point(SchedulerPoint where, uint8_t index) {
        if (scan_hook != nullptr) {
            scan_hook(where, index);
        }
    }

    static void reset() {
        HostPlatform::reset();
        ceiling = 255;
        pending = false;
        requests = 0;
        retakes = 0;
        scheduler = nullptr;
        preempt_hook = nullptr;
        entry_hook = nullptr;
        scan_hook = nullptr;
        in_entry = false;
        quiet_posts<RubatoHostPlatform> = 0;
    }
};

static_assert(Platform<RubatoHostPlatform>);
static_assert(Preemptive<RubatoHostPlatform>);
static_assert(!Preemptive<HostPlatform>);

/// A doorbell on the host: the rings counted, the pending bells a
/// number a test reads and pops. `pending` saturates at 8 the way a
/// hardware FIFO would stop taking bells.
template <uint8_t core>
struct HostDoorbell {
    static inline uint32_t rings = 0;
    static inline uint8_t pending = 0;
    static inline uint32_t pops = 0;
    static inline bool enabled = false;

    static void ring() {
        ++rings;
        if (pending < 8u) {
            ++pending;
        }
    }
    static void pop_all() {
        pending = 0;
        ++pops;
    }
    static void enable() { enabled = true; }
    static void reset() {
        rings = 0;
        pending = 0;
        pops = 0;
        enabled = false;
    }
};

/// One core of a two-core host: the same platform, told apart by type
/// (the kernel statics are keyed by it) and carrying the two optional
/// members a multi-core platform has (kernel/platform.hpp) - `core`,
/// `on_own_core()` against the test-set `HostPlatform::current_core`,
/// and a `Doorbell`. The counters and the breadcrumb are the base's:
/// one set, shared, as a test wants them.
template <uint8_t n>
struct HostCore : HostPlatform {
    static constexpr uint8_t core = n;
    using Doorbell = HostDoorbell<n>;
    static bool on_own_core() { return current_core == n; }
};

} // namespace brio
