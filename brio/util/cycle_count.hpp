/*
 * cycle_count.hpp
 *
 * The cycle count of a periodic timebase at one instant, composed from
 * what a ticker can read: the tick count its handler keeps, the
 * position of the hardware counter in the current period, and the
 * counter's pending flag. Pure arithmetic, host-tested (test_cycle_count);
 * each ticker that offers cycles() reads its registers and hands the
 * pieces to ticker_cycles() - the one place the composition is written.
 *
 * THE WINDOW. The hardware counter restarts its period on its own, and
 * the handler counts the tick some cycles later - more if interrupts
 * are masked. In between, the position has already gone back to the
 * start while the tick count has not moved: the naive `ticks * period +
 * position` goes BACKWARDS by a whole period there, and a wait that
 * subtracts two such readings takes a huge difference for an elapsed
 * timeout (measured on a CH32V006: about one sample in a million fell in
 * the window, and a timed conversion run that began there read zero
 * cycles in two runs of seven). The counter's pending flag is what says
 * the restart is in: set, the period the handler has not counted yet is
 * added.
 *
 * THE EDGE. A counter may raise its flag one step BEFORE it restarts (a
 * SysTick counting down shows VAL = 0 for one clock with the exception
 * already pending, and reloads on the next), so a pending flag with the
 * position at the period's last step is the end of the OLD period, not
 * the start of the new one: the period is added only below that step.
 *
 * THE READS. The caller reads, in this order, the tick count, the flag,
 * the position, the flag again and the tick count again. A handler
 * running between them moves the tick count, a restart between the two
 * flag reads changes the flag: either way the pieces do not belong to
 * one instant and the answer is nothing - read again. With interrupts
 * masked for more than one period the periods beyond the first are
 * lost, as the tick itself loses them (the flag is one bit).
 */

#pragma once

#include <stdint.h>

#include <optional>

namespace brio {

/// The cycles of a timebase at the instant of a consistent set of reads,
/// or nothing when the reads straddle the handler or a restart. `elapsed`
/// is the position in the current period, 0 at its start and period - 1
/// at its last step; the result wraps at 2^32, so a difference of two
/// results is exact under 2^32 cycles.
constexpr std::optional<uint32_t> ticker_cycles(uint32_t ticks_before, bool pending_before,
                                                uint32_t elapsed, bool pending_after,
                                                uint32_t ticks_after, uint32_t period) {
    if (ticks_before != ticks_after || pending_before != pending_after) {
        return std::nullopt;
    }
    const bool uncounted = pending_after && elapsed + 1u < period;
    return static_cast<uint32_t>((ticks_before + (uncounted ? 1u : 0u)) * period + elapsed);
}

}  // namespace brio
