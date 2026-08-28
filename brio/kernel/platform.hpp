/*
 * platform.hpp
 *
 * The Platform concept: the complete list of what the brio kernel needs
 * from the machine underneath. Kernel code is templated on a Platform and
 * NEVER includes a hardware header; each target implements the concept in
 * its own header (platform_avr.hpp for AVR Dx, platform_host.hpp for the
 * native test build) and the app names its platform once. No #ifdef: the
 * door to targets other than AVR stays open by construction.
 *
 * Contract:
 *  - CriticalSection: RAII guard. The constructor masks interrupts, the
 *    destructor restores the PREVIOUS state (so guards nest). Entering
 *    and leaving must also act as compiler memory barriers: data shared
 *    with ISRs then needs no volatile, exactly like the cli/sei barriers
 *    of the classic ATOMIC_BLOCK pattern.
 *  - idle(): called with interrupts MASKED when there is no work. Must
 *    re-enable interrupts and suspend until the next interrupt with no
 *    lost-wakeup window (on AVR: sei immediately followed by sleep - sei
 *    takes effect after the following instruction).
 *  - break_here(): drop into the debugger when one is attached, do
 *    nothing otherwise (AVR BREAK is a NOP without an active OCD).
 *  - now(): current tick count of the system timebase.
 *  - ticks_per_second: the tick rate, a compile-time constant OF THE
 *    TARGET (1024 on AVR Dx from the 32k PIT dividers, typically 1000
 *    on SysTick-based targets). The kernel reasons in opaque ticks and
 *    assumes NOTHING about the rate - no power-of-two, no "1 tick =
 *    1 ms"; conversions live in kernel/time.hpp parameterized on this
 *    constant.
 *  - atomic_width: the widest naturally aligned load/store the CPU
 *    performs as ONE uninterruptible access, in bytes (1 on AVR, 4 on
 *    32-bit cores). Lock-free SPSC code (util/ring.hpp) uses it to
 *    decide whether an index can be shared with an ISR bare or needs a
 *    CriticalSection; the platform states the fact, generic code
 *    chooses the path with if constexpr - no #ifdef, no per-use knob.
 *  - panic_record(): reference to a PanicRecord in storage that
 *    SURVIVES a reset without being zeroed by startup (.noinit on AVR),
 *    so a panic breadcrumb written just before a watchdog reset can be
 *    reported at the next boot (see kernel/panic.hpp).
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
