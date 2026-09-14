/*
 * counting.hpp (gfx)
 *
 * WHAT AN UPDATE ACTUALLY COST. A surface that forwards everything to
 * another and counts what went through it: how many filled rectangles,
 * how many runs, how many pixels in them.
 *
 * It exists because the whole economy of a write-only panel is "touch
 * only what changed", and that is a claim until something counts. On a
 * panel behind a bus each of those calls is a window command and a burst
 * of bytes, so the difference between redrawing a field and redrawing
 * one cell of it is the difference between an interface that answers and
 * one that crawls - and the numbers here are what makes that visible
 * instead of asserted. The accounting is the same idea as a flash
 * simulation's wear counters: the cost of a decision, told plainly.
 *
 * It is a Surface, so it goes anywhere a surface goes and a primitive
 * cannot tell. It costs nothing where it is not used, and where it is
 * used it costs three additions.
 */

#pragma once

#include <stdint.h>

#include <span>

#include "gfx/surface.hpp"

namespace brio {

/// What went through, since the last reset.
struct DrawTally {
    uint32_t rects;  ///< filled rectangles asked for
    uint32_t runs;   ///< runs of caller-given pixels asked for
    uint32_t pixels; ///< pixels in all of them, clipping not deducted

    /// Calls a panel behind a bus would turn into window commands.
    uint32_t windows() const { return rects + runs; }
};

/**
 * A surface that forwards to another and counts. The one underneath does
 * the work and the clipping; these numbers are what the LIBRARY asked
 * for, which is what a bus would have carried.
 */
template <Surface S>
class Counting {
public:
    using Format = typename S::Format;
    using Color = typename S::Color;

    explicit Counting(S& under) : under_(&under) {}

    Extent width() const { return under_->width(); }
    Extent height() const { return under_->height(); }

    void fill_rect(Coord x, Coord y, Extent w, Extent h, Color c) {
        ++tally_.rects;
        tally_.pixels += static_cast<uint32_t>(w) * h;
        under_->fill_rect(x, y, w, h, c);
    }

    void write_run(Coord x, Coord y, std::span<const Color> run) {
        ++tally_.runs;
        tally_.pixels += static_cast<uint32_t>(run.size());
        under_->write_run(x, y, run);
    }

    const DrawTally& tally() const { return tally_; }
    void reset() { tally_ = DrawTally{}; }

    /// What was drawn since the last call, and start again. The verb an
    /// interaction wraps itself in to say what it cost.
    DrawTally take() {
        const DrawTally t = tally_;
        tally_ = DrawTally{};
        return t;
    }

private:
    S* under_;
    DrawTally tally_{};
};

} // namespace brio
