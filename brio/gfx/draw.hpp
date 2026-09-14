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

/// Exact integer square root, digit by digit - no floating point
/// anywhere in a primitive, since the smallest targets have none and
/// the answer has to be the same on all of them. floor(sqrt(n)).
constexpr uint16_t isqrt(uint32_t n) {
    uint32_t res = 0;
    uint32_t bit = static_cast<uint32_t>(1) << 30;
    while (bit > n) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (n >= res + bit) {
            n -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<uint16_t>(res);
}

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
        s.fill_rect(
            x, static_cast<Coord>(static_cast<int32_t>(y) + static_cast<int32_t>(h) - 1),
            w, 1, c);
    }
    if (h > 2) {
        const Extent mid = static_cast<Extent>(h - 2);
        s.fill_rect(x, static_cast<Coord>(static_cast<int32_t>(y) + 1), 1, mid,
                    c);
        if (w > 1) {
            s.fill_rect(
                static_cast<Coord>(static_cast<int32_t>(x) + static_cast<int32_t>(w) - 1),
                static_cast<Coord>(static_cast<int32_t>(y) + 1), 1, mid, c);
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
        const int32_t len =
            (x0 < x1 ? static_cast<int32_t>(x1) - x0
                     : static_cast<int32_t>(x0) - x1) + 1;
        s.fill_rect(from, y0, static_cast<Extent>(len), 1, c);
        return;
    }
    if (x0 == x1) {
        const Coord from = y0 < y1 ? y0 : y1;
        const int32_t len =
            (y0 < y1 ? static_cast<int32_t>(y1) - y0
                     : static_cast<int32_t>(y0) - y1) + 1;
        s.fill_rect(x0, from, 1, static_cast<Extent>(len), c);
        return;
    }

    int32_t x = x0;
    int32_t y = y0;
    const int32_t dx =
        x1 > x0 ? static_cast<int32_t>(x1) - x0 : static_cast<int32_t>(x0) - x1;
    const int32_t dy =
        -(y1 > y0 ? static_cast<int32_t>(y1) - y0 : static_cast<int32_t>(y0) - y1);
    const int32_t sx = x0 < x1 ? 1 : -1;
    const int32_t sy = y0 < y1 ? 1 : -1;
    int32_t err = dx + dy;
    for (;;) {
        s.fill_rect(static_cast<Coord>(x),
                static_cast<Coord>(y), 1, 1, c);
        if (x == static_cast<int32_t>(x1) && y == static_cast<int32_t>(y1)) {
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

/// The eight mirrors of one octant point about a centre. A circle is
/// computed for an eighth of its circumference and reflected; this is
/// where the reflection lives, so the two round primitives share it
/// without sharing anything else.
template <Surface S>
void octant_points(S& s, Coord cx, Coord cy, int32_t x, int32_t y,
                   typename S::Color c) {
    const int32_t ox = static_cast<int32_t>(cx);
    const int32_t oy = static_cast<int32_t>(cy);
    s.fill_rect(static_cast<Coord>(ox + x),
                static_cast<Coord>(oy + y), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ox - x),
                static_cast<Coord>(oy + y), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ox + x),
                static_cast<Coord>(oy - y), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ox - x),
                static_cast<Coord>(oy - y), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ox + y),
                static_cast<Coord>(oy + x), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ox - y),
                static_cast<Coord>(oy + x), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ox + y),
                static_cast<Coord>(oy - x), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ox - y),
                static_cast<Coord>(oy - x), 1, 1, c);
}

/// The same reflection for a rounded rectangle, where the four arcs have
/// four different centres and each keeps only its outward quadrant.
template <Surface S>
void corner_points(S& s, int32_t ax0, int32_t ay0, int32_t ax1, int32_t ay1,
                   int32_t px, int32_t py, typename S::Color c) {
    s.fill_rect(static_cast<Coord>(ax0 - px),
                static_cast<Coord>(ay0 - py), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ax1 + px),
                static_cast<Coord>(ay0 - py), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ax0 - px),
                static_cast<Coord>(ay1 + py), 1, 1, c);
    s.fill_rect(static_cast<Coord>(ax1 + px),
                static_cast<Coord>(ay1 + py), 1, 1, c);
}

/**
 * A circle's OUTLINE, one pixel thick.
 *
 * THE SPECIFICATION: the ring holds, for every column within the
 * radius, the row nearest the ideal circle there - and, for every row,
 * the column nearest it. The UNION of those two families is what makes
 * the ring connected: stepping columns alone leaves gaps where the
 * circle runs steep, stepping rows alone leaves them where it runs
 * shallow. A radius of zero is the single pixel at the centre, the
 * circle of no extent being the point.
 *
 * Below is the integer midpoint form, which carries no square root; the
 * reference computes the same set from the definition and the two are
 * compared over every radius a test surface holds.
 */
template <Surface S>
void circle(S& s, Coord cx, Coord cy, Extent r, typename S::Color c) {
    if (r == 0) {
        s.fill_rect(cx, cy, 1, 1, c);
        return;
    }
    int32_t x = static_cast<int32_t>(r);
    int32_t y = 0;
    int32_t err = 1 - static_cast<int32_t>(r);
    while (x >= y) {
        octant_points(s, cx, cy, x, y, c);
        ++y;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            --x;
            err += 2 * (y - x) + 1;
        }
    }
}

/**
 * A filled disc: every pixel whose centre lies within the radius, which
 * is a DIFFERENT SET from circle()'s ring and deliberately so - the ring
 * is the nearest pixels to a curve, the disc is an inequality. One run
 * per row, because a row of one colour is what a panel is fast at.
 */
template <Surface S>
void fill_circle(S& s, Coord cx, Coord cy, Extent r, typename S::Color c) {
    const int32_t rr = static_cast<int32_t>(r) * static_cast<int32_t>(r);
    for (int32_t dy = -static_cast<int32_t>(r); dy <= static_cast<int32_t>(r);
         ++dy) {
        const int32_t half =
            static_cast<int32_t>(isqrt(static_cast<uint32_t>(rr - dy * dy)));
        s.fill_rect(static_cast<Coord>(static_cast<int32_t>(cx) - half),
                    static_cast<Coord>(static_cast<int32_t>(cy) + dy),
                    static_cast<Extent>(2 * half + 1), 1, c);
    }
}

/**
 * A rectangle with quarter-circle corners, outline only.
 *
 * THE SPECIFICATION: the shape is the set of points within `r` of the
 * rectangle inset by `r` on every side - a Minkowski sum, which is what
 * makes "rounded rectangle" mean one thing and not four. Its outline is
 * then four straight runs and four quarter rings, each ring obeying
 * circle()'s rule above.
 *
 * THE RADIUS IS CLAMPED to (min(w, h) - 1) / 2, the largest that leaves
 * the inset rectangle non-empty; at exactly that value on a square the
 * shape IS a circle. A radius of zero is a plain rectangle, with no
 * special case needed anywhere else.
 */
template <Surface S>
void round_rect(S& s, Coord x, Coord y, Extent w, Extent h, Extent r,
                typename S::Color c) {
    if (w == 0 || h == 0) {
        return;
    }
    const Extent limit = static_cast<Extent>(((w < h ? w : h) - 1) / 2);
    if (r > limit) {
        r = limit;
    }
    if (r == 0) {
        rect(s, x, y, w, h, c);
        return;
    }

    const int32_t x0 = static_cast<int32_t>(x);
    const int32_t y0 = static_cast<int32_t>(y);
    const int32_t x1 = x0 + static_cast<int32_t>(w) - 1;
    const int32_t y1 = y0 + static_cast<int32_t>(h) - 1;
    const Extent straight_w =
        static_cast<Extent>(static_cast<int32_t>(w) - 2 * static_cast<int32_t>(r));
    const Extent straight_h =
        static_cast<Extent>(static_cast<int32_t>(h) - 2 * static_cast<int32_t>(r));

    s.fill_rect(static_cast<Coord>(x0 + r), static_cast<Coord>(y0), straight_w, 1,
                c);
    s.fill_rect(static_cast<Coord>(x0 + r), static_cast<Coord>(y1), straight_w, 1,
                c);
    s.fill_rect(static_cast<Coord>(x0), static_cast<Coord>(y0 + r), 1, straight_h,
                c);
    s.fill_rect(static_cast<Coord>(x1), static_cast<Coord>(y0 + r), 1, straight_h,
                c);

    // The four arc centres, each inset by the radius.
    const int32_t ax0 = x0 + static_cast<int32_t>(r);
    const int32_t ax1 = x1 - static_cast<int32_t>(r);
    const int32_t ay0 = y0 + static_cast<int32_t>(r);
    const int32_t ay1 = y1 - static_cast<int32_t>(r);

    int32_t px = static_cast<int32_t>(r);
    int32_t py = 0;
    int32_t err = 1 - static_cast<int32_t>(r);
    while (px >= py) {
        corner_points(s, ax0, ay0, ax1, ay1, px, py, c);
        corner_points(s, ax0, ay0, ax1, ay1, py, px, c);
        ++py;
        if (err < 0) {
            err += 2 * py + 1;
        } else {
            --px;
            err += 2 * (py - px) + 1;
        }
    }
}

/**
 * The filled rounded rectangle: every pixel within `r` of the inset
 * rectangle, one run per row - the same set the outline bounds, by the
 * same definition, with the same clamp.
 */
template <Surface S>
void fill_round_rect(S& s, Coord x, Coord y, Extent w, Extent h, Extent r,
                     typename S::Color c) {
    if (w == 0 || h == 0) {
        return;
    }
    const Extent limit = static_cast<Extent>(((w < h ? w : h) - 1) / 2);
    if (r > limit) {
        r = limit;
    }
    if (r == 0) {
        s.fill_rect(x, y, w, h, c);
        return;
    }

    const int32_t y0i = static_cast<int32_t>(y) + static_cast<int32_t>(r);
    const int32_t y1i =
        static_cast<int32_t>(y) + static_cast<int32_t>(h) - 1 - static_cast<int32_t>(r);
    const int32_t rr = static_cast<int32_t>(r) * static_cast<int32_t>(r);

    for (int32_t py = static_cast<int32_t>(y);
         py <= static_cast<int32_t>(y) + static_cast<int32_t>(h) - 1; ++py) {
        int32_t dy = 0;
        if (py < y0i) {
            dy = y0i - py;
        } else if (py > y1i) {
            dy = py - y1i;
        }
        const int32_t grow =
            static_cast<int32_t>(isqrt(static_cast<uint32_t>(rr - dy * dy)));
        const int32_t from =
            static_cast<int32_t>(x) + static_cast<int32_t>(r) - grow;
        const int32_t span =
            static_cast<int32_t>(w) - 2 * static_cast<int32_t>(r) + 2 * grow;
        s.fill_rect(static_cast<Coord>(from), static_cast<Coord>(py),
                    static_cast<Extent>(span), 1, c);
    }
}

} // namespace brio
