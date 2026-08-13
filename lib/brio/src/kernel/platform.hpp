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
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <type_traits>

namespace brio {

template <typename P>
concept Platform =
    std::is_default_constructible_v<typename P::CriticalSection> &&
    requires {
        { P::idle() } -> std::same_as<void>;
        { P::break_here() } -> std::same_as<void>;
        { P::now() } -> std::same_as<uint32_t>;
        // must be a positive compile-time constant (usable in constexpr)
        requires P::ticks_per_second > 0u;
    };

} // namespace brio
