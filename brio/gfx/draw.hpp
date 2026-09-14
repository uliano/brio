/*
 * draw.hpp (gfx)
 *
 * The primitives. Every one of them is a FREE FUNCTION over the
 * write-only Surface concept, so it works on any surface - a
 * framebuffer, a window onto one, a panel that answers no questions -
 * without that surface implementing anything beyond the two verbs.
 *
 * AXIS-ALIGNED WORK TAKES THE FAST PATH. An interface is mostly frames,
 * separators and bars, so a horizontal or vertical run is not a
 * convenience spelling of line(): it is a one-pixel-thick fill_rect, the
 * shape a panel does in a single window, and line() routes the
 * axis-aligned cases into it rather than stepping them.
 *
 * NOTHING HERE READS. Not one primitive takes a ReadableSurface, which
 * is what makes them all correct on a write-only panel; erasing is
 * drawing again in the background colour, and shapes do not overlap
 * (there is no compositing in this library, by decision - see the design
 * page).
 *
 * (docs/design/gfx.md.)
 */

#pragma once

#include <stdint.h>

#include "gfx/surface.hpp"

namespace brio {

/// The whole surface, one colour.
template <Surface S>
void clear(S& s, typename S::Color c) {
    s.fill_rect(0, 0, s.width(), s.height(), c);
}

/// One pixel: the degenerate rectangle. Derived on purpose - see the
/// header comment of gfx/surface.hpp for why it is not the foundation.
template <Surface S>
void set_pixel(S& s, Coord x, Coord y, typename S::Color c) {
    s.fill_rect(x, y, 1, 1, c);
}

/// A filled rectangle. Forwards to the surface's own verb; it exists so
/// that every primitive is spelled the same way at a call site.
template <Surface S>
void fill_rect(S& s, Coord x, Coord y, Extent w, Extent h,
               typename S::Color c) {
    s.fill_rect(x, y, w, h, c);
}

/// `len` pixels rightwards from (x, y), both ends included.
template <Surface S>
void hline(S& s, Coord x, Coord y, Extent len, typename S::Color c) {
    s.fill_rect(x, y, len, 1, c);
}

/// `len` pixels downwards from (x, y), both ends included.
template <Surface S>
void vline(S& s, Coord x, Coord y, Extent len, typename S::Color c) {
    s.fill_rect(x, y, 1, len, c);
}

/// The OUTLINE of a rectangle, one pixel thick, drawn inside the given
/// rectangle (an outline never grows the shape). Four fills; the two
/// horizontals already cover a rectangle two pixels tall or less, so the
/// verticals are only asked for what remains.
template <Surface S>
void rect(S& s, Coord x, Coord y, Extent w, Extent h, typename S::Color c) {
    if (w == 0 || h == 0) {
        return;
    }
    s.fill_rect(x, y, w, 1, c);
    if (h > 1) {
        s.fill_rect(x, Coord(int32_t(y) + int32_t(h) - 1), w, 1, c);
    }
    if (h > 2) {
        const Extent mid = Extent(h - 2);
        s.fill_rect(x, Coord(int32_t(y) + 1), 1, mid, c);
        if (w > 1) {
            s.fill_rect(Coord(int32_t(x) + int32_t(w) - 1), Coord(int32_t(y) + 1),
                        1, mid, c);
        }
    }
}

/**
 * A segment, both endpoints drawn.
 *
 * THE SPECIFICATION, because this is the first primitive where there is
 * one to state: the segment is walked along its MAJOR axis - the one it
 * spans further - and at each step exactly ONE pixel is lit, the one
 * whose centre is nearest the ideal segment at that step. So a line is
 * everywhere one pixel thick, it is never thicker where it runs shallow,
 * and it has exactly max(|dx|, |dy|) + 1 pixels. The reference renderer
 * in host/gfx_reference.hpp computes that set from the definition, by
 * brute force over every pixel, and the two are compared exhaustively.
 *
 * Where the ideal segment passes exactly half way between two pixel
 * centres the tie is broken AWAY FROM ZERO in the minor axis, which is
 * what the integer form below does and what the reference is written to
 * match.
 *
 * Axis-aligned segments are handed to the run verb instead of stepped.
 */
template <Surface S>
void line(S& s, Coord x0, Coord y0, Coord x1, Coord y1, typename S::Color c) {
    if (y0 == y1) {
        const Coord from = x0 < x1 ? x0 : x1;
        const int32_t len = (x0 < x1 ? int32_t(x1) - x0 : int32_t(x0) - x1) + 1;
        s.fill_rect(from, y0, Extent(len), 1, c);
        return;
    }
    if (x0 == x1) {
        const Coord from = y0 < y1 ? y0 : y1;
        const int32_t len = (y0 < y1 ? int32_t(y1) - y0 : int32_t(y0) - y1) + 1;
        s.fill_rect(x0, from, 1, Extent(len), c);
        return;
    }

    int32_t x = x0;
    int32_t y = y0;
    const int32_t dx = x1 > x0 ? int32_t(x1) - x0 : int32_t(x0) - x1;
    const int32_t dy = -(y1 > y0 ? int32_t(y1) - y0 : int32_t(y0) - y1);
    const int32_t sx = x0 < x1 ? 1 : -1;
    const int32_t sy = y0 < y1 ? 1 : -1;
    int32_t err = dx + dy;
    for (;;) {
        s.fill_rect(Coord(x), Coord(y), 1, 1, c);
        if (x == int32_t(x1) && y == int32_t(y1)) {
            return;
        }
        const int32_t e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
}

} // namespace brio
