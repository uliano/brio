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
 * same platform made preemptive, for the Rubato kernel's tests.
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

    static void idle() { ++idle_calls; }
    static void break_here() { ++break_calls; }
    static uint32_t now() { return ticks; }

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
        CriticalSection::depth = 0;
        CriticalSection::entries = 0;
        panic_record() = PanicRecord{};
    }
};

static_assert(Platform<HostPlatform>);

/// The host platform of a Rubato kernel (kernel/rubato.hpp): HostPlatform
/// with the three members of a preemptive platform (kernel/platform.hpp)
/// and what a test needs to simulate an interrupt over a dispatch.
///  - pend_schedule() records the request in `pending` and, unless the
///    test has raised `in_isr`, runs the scheduler at once and clears it:
///    the host's image of a scheduling interrupt taken at once. Inside a
///    simulated interrupt the request waits, and the test's harness runs
///    the scheduler at the interrupt's exit when `pending` is set.
///  - `scheduler` is that scheduler, the one indirect call the host pays:
///    the test sets it to its kernel's activate(). Null, a request is
///    recorded and nothing runs (a scheduling interrupt not yet enabled).
///  - preempt_point() is where a test object's handler, or the time-event
///    walk (kernel/time_event.hpp), lets a simulated interrupt land: it
///    runs the test's `preempt_hook`, if any.
/// The counters, the clock and the critical section are the base's: one
/// set, shared. reset() clears all of it.
struct RubatoHostPlatform : HostPlatform {
    static constexpr bool preemptive = true;
    static inline uint8_t ceiling = 255;     ///< the index being dispatched; 255 at the base

    static inline bool in_isr = false;       ///< raised by the harness inside a simulated interrupt
    static inline bool pending = false;      ///< a request not yet served
    static inline uint32_t requests = 0;     ///< how many times pend_schedule() ran
    static inline bool (*scheduler)() = nullptr;
    static inline void (*preempt_hook)() = nullptr;

    static void pend_schedule() {
        ++requests;
        pending = true;
        if (!in_isr && scheduler != nullptr) {
            pending = false;
            (void)scheduler();
        }
    }

    static void preempt_point() {
        if (preempt_hook != nullptr) {
            preempt_hook();
        }
    }

    static void reset() {
        HostPlatform::reset();
        ceiling = 255;
        in_isr = false;
        pending = false;
        requests = 0;
        scheduler = nullptr;
        preempt_hook = nullptr;
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
