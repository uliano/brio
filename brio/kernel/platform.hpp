/*
 * platform.hpp
 *
 * The Platform concept: the complete list of what the brio kernel needs
 * from the machine underneath. Kernel code is templated on a Platform and
 * NEVER includes a hardware header; each target implements the concept in
 * its own <stratum>/platform.hpp (avrdx/, samc21/, stm32g0/, and host/
 * for the native test build) and the app names its platform once. No
 * #ifdef: the door to the next target stays open by construction.
 *
 * Contract:
 *  - CriticalSection: RAII guard. The constructor masks interrupts, the
 *    destructor restores the PREVIOUS state (so guards nest). Entering
 *    and leaving must also act as compiler memory barriers: data shared
 *    with ISRs then needs no volatile, exactly like the cli/sei barriers
 *    of the classic ATOMIC_BLOCK pattern.
 *  - idle(): called with interrupts MASKED when there is no work. Must
 *    re-enable interrupts and suspend until the next interrupt with no
 *    lost-wakeup window (where a core defers an interrupt enable by one
 *    instruction, that means the enable immediately followed by the
 *    sleep instruction).
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
 * (kernel.hpp's idle_if_empty; a platform without it gets idle() exactly
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
 */

#pragma once

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

} // namespace brio
