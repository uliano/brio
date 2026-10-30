/*
 * platform.hpp
 *
 * The Platform concept: everything the kernel needs from the chip it
 * runs on, as a list of requirements on one type.
 *
 * An application names its platform once, as a type alias, and passes it
 * to the kernel, to every queue and to every time event:
 *
 *     using P = brio::Stm32g0Platform<>;
 *
 * Each target stratum provides that type in its own platform.hpp
 * (avrxt/, samc21/, stm32g0/, stm32f4/, ch32v00x/, ch32vx03/, ch32x035/,
 * rp2040/, rp2350/, and host/ for the native test build). Kernel code is
 * templated on the platform and never includes a hardware header; a
 * type that does not meet the concept is refused where it is used, with
 * the requirement named. There is no default platform and no #ifdef.
 *
 * The contract, member by member:
 *  - CriticalSection: RAII guard. The constructor masks interrupts, the
 *    destructor restores the PREVIOUS state (so guards nest). Entering
 *    and leaving must also act as compiler memory barriers: data shared
 *    with ISRs then needs no volatile, exactly like the cli/sei barriers
 *    of the classic ATOMIC_BLOCK pattern. Both halves are forced inline:
 *    a half the compiler outlines puts a call and a return inside the
 *    window it opens.
 *  - idle(): called with interrupts MASKED when there is no work. Must
 *    suspend until the next interrupt and return with interrupts
 *    enabled, with no lost-wakeup window: an interrupt that becomes
 *    pending between the kernel's empty check and the sleep must still
 *    end the sleep. How is the core's business (an enable whose effect
 *    is deferred by one instruction followed by the sleep instruction on
 *    the AVR; a wait-for-interrupt that a pending interrupt ends even
 *    while masked on the Cortex-M, unmasked afterwards), and each
 *    platform header says which.
 *    One more promise the kernel relies on and no member states: an
 *    interrupt handler is not interrupted by another interrupt handler
 *    (docs/design/kernel.md, "The model in one page"). Each platform
 *    keeps it its own way, and says how.
 *  - break_here(): a debug trap. With a debugger halted on it, the
 *    program stops there. What the instruction does with NO debugger
 *    attached is the core's business and is not promised here: some
 *    cores make it a no-op, others escalate it to a fault - so a
 *    reporter that must survive without a debugger cannot assume this
 *    call returns, and each platform's own header says which it is.
 *  - now(): current tick count of the system timebase.
 *  - ticks_per_second: the tick rate, a compile-time constant OF THE
 *    TARGET (commonly 1000 or 1024, whatever its timebase divides to).
 *    The kernel reasons in opaque ticks and assumes NOTHING about the
 *    rate - no power-of-two, no "1 tick = 1 ms"; conversions live in
 *    kernel/time.hpp parameterized on this constant.
 *  - atomic_width: the widest naturally aligned load/store the CPU
 *    performs as ONE uninterruptible access, in bytes (1 on an 8-bit
 *    core, 4 on a 32-bit one). Lock-free SPSC code (util/ring.hpp) uses
 *    it to decide whether an index can be shared with an ISR bare or
 *    needs a CriticalSection; the platform states the fact, generic
 *    code chooses the path with if constexpr - no #ifdef, no per-use
 *    knob.
 *  - panic_record(): reference to a PanicRecord in storage that
 *    SURVIVES a reset without being zeroed by startup (a .noinit
 *    section or the target's equivalent), so a panic breadcrumb written
 *    just before a watchdog reset can be reported at the next boot (see
 *    kernel/panic.hpp).
 *
 * Optional member, NOT part of the concept - a platform whose timebase
 * keeps counting while the core sleeps (a low-power timer, not a tick
 * interrupt) may provide it, and the kernel loop detects it by requires
 * (tenuto.hpp's idle_if_empty; a platform without it gets idle() exactly
 * as before):
 *  - idle_until(std::optional<uint32_t> deadline): called INSTEAD of
 *    idle(), with interrupts MASKED, with the ABSOLUTE tick of the
 *    nearest armed time event (empty = nothing armed). It must return
 *    with interrupts enabled whether or not it slept; it may return
 *    without sleeping (a deadline already due, a wake it could not
 *    place); and it must never sleep PAST a deadline it was given -
 *    waking at or after it is the contract, late is legal, early is not
 *    (kernel/time.hpp's "at least"). How the wake is placed is the
 *    platform's business; the kernel only relies on the contract.
 *
 * Optional member a service that may be called from an interrupt handler
 * asks for (the HandlerAware concept below; a platform without it is
 * refused by such a service and by nothing else):
 *  - in_handler() -> bool: whether the caller runs inside an interrupt
 *    handler. The kernel keeps which active object is being dispatched
 *    (dispatching<P>, kernel/active_object.hpp), and a handler sees the
 *    index of the dispatch it interrupted, so this is how a service
 *    tells a handler's call from that object's. How the core knows is
 *    the platform's business (an exception number register on the
 *    Cortex-M, where IPSR is zero in thread mode and the exception's
 *    number in a handler), and each platform header says which.
 *
 * Optional members of a platform that is ONE CORE OF SEVERAL (a chip
 * running a kernel per core, one platform type per core - the kernel
 * statics are keyed by P, so the type IS the core), detected the same
 * way; a single-core platform has neither and compiles nothing for them:
 *  - on_own_core() -> bool: whether the calling core is this platform's.
 *    EventQueue::push refuses a copy from another core with it (a
 *    queue's critical section guards one core), counting the mispost.
 *  - Doorbell: the type util/inbox.hpp rings when an event is sent to
 *    an AO of this platform from another core - `ring()` from the
 *    sending core, `pop_all()` and `enable()` on this one - over
 *    whatever the chip has between its cores (a hardware FIFO with an
 *    interrupt line per core on the RP2040, a counter on the host).
 *
 * Optional members of a PREEMPTIVE platform, the one a Rubato kernel
 * runs on (kernel/rubato.hpp), detected together by the Preemptive
 * concept below; a platform with none of them is a Tenuto platform, and
 * every header that asks (kernel/post.hpp, kernel/event_queue.hpp,
 * kernel/time_event.hpp, util/ring.hpp) compiles for it exactly what it
 * compiles without the question:
 *  - preemptive: a static constexpr bool, true;
 *  - ceiling: a static uint8_t, 255 at the loop's base, which the kernel
 *    keeps: the pack index of the active object being dispatched or just
 *    served, and 0 while it chooses the next one. A post to an object
 *    whose index is below it asks for the scheduler; every other post on
 *    the platform adds one to the kernel's count of quiet posts,
 *    quiet_posts<P> below;
 *  - pend_schedule(): that request. The scheduler (Rubato's activate())
 *    runs when no interrupt handler is running: at once from a
 *    dispatch, at the handler's exit from an interrupt handler.
 *
 * The half of the preemptive contract no member states is how the
 * platform runs the scheduler once the request is taken - its ENTRY and
 * its EXIT - and it is the half that bounds the stack:
 *  - the entry reads the ceiling and stores 0 in its place before any
 *    interrupt handler can run after the request was taken, and calls
 *    activate(base) with the value it read. A handler that runs after
 *    that store finds the ceiling at 0: its post asks for nothing, moves
 *    the count, and the choice under way sees it;
 *  - activate(base) returns with the ceiling still at 0, handing back the
 *    count its last choice read (Activation below);
 *  - the exit stores `base` back in the ceiling and compares the count
 *    with quiet_posts<P> in one step no interrupt handler can enter, and
 *    when the count moved it makes the request again before it returns,
 *    so that the scheduler starts again over the code the request
 *    interrupted, in place of the run that just ended, instead of
 *    nesting over it.
 * Kept, every run of the scheduler that starts over a dispatch starts at
 * that dispatch's index, below the ceiling of the run it interrupts, so a
 * pack of n objects holds at most n runs (kernel/rubato.hpp, "The
 * stack"). How each core keeps the two steps whole is its scheduler
 * file's (cortexm/schedule.hpp, qingke/schedule.hpp,
 * rp2350/schedule_hazard3.hpp), with the one request a core may still find
 * pending at its entry; the host's simulation is RubatoHostPlatform's
 * (host/platform.hpp).
 *
 * Optional members of a HOST platform only, detected the same way:
 *  - interleave_point(): called by EventQueue's take() and release() at
 *    every boundary between two of their shared accesses - each place
 *    an interrupt could land - so a host suite can push from there and
 *    check every interleaving (test/test_event_queue). No target
 *    platform has it, and no code is compiled for it there;
 *  - preempt_point(): called by TimeEvents::process() after each fire
 *    on a preemptive platform, the place a host suite lands a simulated
 *    interrupt inside the walk (test/test_rubato); a test object's
 *    handler may call it too. No target platform has it;
 *  - scan_point(SchedulerPoint where, uint8_t index): called by Rubato's
 *    activate() at each place of its choice and of its gap between two
 *    dispatches where an interrupt could land (SchedulerPoint below), so
 *    a host suite can land one there (test/test_rubato). No target
 *    platform has it, and no code is compiled for it there.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <concepts>
#include <type_traits>

namespace brio {

/// Panic breadcrumb: written by panic() BEFORE any reporter runs, read
/// (and cleared) at the next boot via take_panic_record<P>(). Lives in
/// platform-provided reset-surviving storage.
///
/// Defined HERE and not in kernel/panic.hpp on purpose: it is the one
/// kernel data type a Platform must host, so the concept below has to
/// name it, and panic.hpp (which owns the semantics: codes, magic,
/// panic(), take_panic_record()) sits above the concept in the include
/// graph. Layout only - the meaning of code/context is panic.hpp's.
struct PanicRecord {
    uint16_t magic;    ///< panic_magic when the record is valid
    uint8_t code;      ///< PanicCode of the failure
    uint8_t context;   ///< producer-defined detail (queue id, AO id, ...)
};

template <typename P>
concept Platform =
    std::is_default_constructible_v<typename P::CriticalSection> &&
    requires {
        { P::idle() } -> std::same_as<void>;
        { P::break_here() } -> std::same_as<void>;
        { P::now() } -> std::same_as<uint32_t>;
        { P::panic_record() } -> std::same_as<PanicRecord&>;
        // must be a positive compile-time constant (usable in constexpr)
        requires P::ticks_per_second > 0u;
        requires P::atomic_width > 0u;
    };

/// Whether a platform can tell a call from an interrupt handler apart
/// from one made by the object being dispatched (the optional member
/// above).
template <typename P>
concept HandlerAware = requires {
    { P::in_handler() } -> std::same_as<bool>;
};

/// Whether a platform is preemptive: it says so and carries the ceiling
/// and the scheduling request (the optional members above). A platform
/// that says so and lacks either is not one, and Rubato refuses it.
template <typename P>
concept Preemptive = requires {
    requires P::preemptive;
    { P::ceiling } -> std::same_as<uint8_t&>;
    { P::pend_schedule() } -> std::same_as<void>;
};

/// The count of quiet posts on preemptive platform P: one added by every
/// post() that asked for no scheduler (kernel/post.hpp), modulo 256.
/// Rubato's choice reads it before its first scan and again once the
/// ceiling is raised to the object chosen, and the platform's exit
/// compares it with the value activate() hands back
/// (the entry and the exit above): a post that landed behind a look is
/// seen by the count having moved. A count is all it is - which object,
/// and how many, the next choice finds - so a lost update between an
/// interrupt handler and a dispatch, which still moves it, is harmless.
/// Two misses it cannot see, both stated here and not guarded: 256 quiet
/// posts inside one choice, and on a chip with two cores a post from the
/// other core (a mispost the queue refuses) whose read and write of the
/// byte straddle this core's own increment. A Tenuto platform never
/// instantiates it.
template <typename P>
inline uint8_t quiet_posts = 0;

/// What Rubato's activate() hands back to the platform's entry that
/// called it: `count`, quiet_posts<P> as the last choice read it, which
/// the exit compares; and `served`, whether this run dispatched anything,
/// which the kernel's own loop asks. The scheduler bodies written in
/// assembly read `count` as the low byte of the register the result
/// comes back in (r0 under the AAPCS, a0 under the RISC-V calling
/// convention: a structure no wider than a register returns in it, laid
/// out as in memory, and both targets are little-endian), which the
/// layout asserted below makes true.
struct Activation {
    uint8_t count;
    bool served;
};

static_assert(sizeof(Activation) == 2 && offsetof(Activation, count) == 0,
              "the scheduler bodies read Activation::count as the result's low byte");

/// The places of Rubato's activate() a host platform's scan_point() is
/// called at (the optional member above): the ceiling is 0 at `look`,
/// `empty` and `found` in a choice's first scan (the held object's index
/// in the scans after it), at `raised` the chosen object's index, in the
/// `gap` after a dispatch the index of the object just served, and at
/// `exit` 0 again, the run's last choice having found nothing.
enum class SchedulerPoint : uint8_t {
    look,     ///< before one queue's look
    empty,    ///< after a look that found the queue empty
    found,    ///< after the look that chose an object, before the raise
    raised,   ///< after the raise, before the count is read again
    gap,      ///< after a dispatch, before the next choice
    exit,     ///< after the last choice, before activate() returns
};

} // namespace brio
