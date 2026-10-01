/*
 * cycle_count.hpp
 *
 * The cycle count of a periodic timebase at one instant, composed from
 * what a ticker can read: the tick count its handler keeps, the
 * position of the hardware counter in the current period, and the
 * counter's pending flag. Pure arithmetic, host-tested (test_cycle_count);
 * each ticker that offers cycles() reads its registers into a
 * TickerSample, asks ticker_consistent() whether the reads belong to one
 * instant and hands a consistent sample to ticker_compose() - the one
 * place the test and the composition are written.
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
 * one instant, ticker_consistent() says so, and the caller reads again.
 * With interrupts masked for more than one period the periods beyond the
 * first are lost, as the tick itself loses them (the flag is one bit).
 *
 * THE SHAPE: a test and a composition, the loop in the ticker. A read is
 * on every hot path that stamps time, so what it costs in instructions
 * is part of it. Every piece crosses the two functions as a plain bool or
 * word, both always inlined, so the sample lives in registers: nothing
 * of it is an aggregate the compiler must keep whole, where a single
 * function answering "a count, or nothing" would hand back a
 * std::optional whose flag gcc stores and loads back through the stack
 * on the RISC-V cores on every read, a consistent one included. The
 * retry loop stays in each ticker's cycles(), four lines over its own
 * registers, and no lambda carries the reads into a looping helper.
 */

#pragma once

#include <stdint.h>

namespace brio {

/// One set of reads of a periodic timebase, in the order they are taken
/// (the file header, THE READS): the tick count, the counter's pending
/// flag, the counter's position in its period - 0 at its start and
/// period - 1 at its last step -, the flag again, the tick count again.
struct TickerSample {
    uint32_t t0;         ///< the tick count, read first
    bool p0;             ///< the pending flag, read second
    uint32_t position;   ///< the counter's position in its period, third
    bool p1;             ///< the pending flag, read fourth
    uint32_t t1;         ///< the tick count, read last
};

/// Whether the reads of `s` belong to one instant: no handler counted a
/// tick between them and no restart raised or cleared the flag between
/// the two flag reads. False means read again.
[[gnu::always_inline]] constexpr bool ticker_consistent(TickerSample s) {
    return s.t0 == s.t1 && s.p0 == s.p1;
}

/// The cycles of the timebase at the instant of a CONSISTENT sample (a
/// sample ticker_consistent() refuses composes to no meaningful count):
/// the counted ticks, plus the period the handler has not counted yet
/// when the flag stands below the period's last step (THE WINDOW, THE
/// EDGE), times the period, plus the position. Wraps at 2^32, so a
/// difference of two results is exact under 2^32 cycles.
[[gnu::always_inline]] constexpr uint32_t ticker_compose(TickerSample s, uint32_t period) {
    const bool uncounted = s.p1 && s.position + 1u < period;
    return static_cast<uint32_t>((s.t0 + (uncounted ? 1u : 0u)) * period + s.position);
}

}  // namespace brio
