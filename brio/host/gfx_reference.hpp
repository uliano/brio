/*
 * gfx_reference.hpp (host)
 *
 * THE JUDGE OF THE PRIMITIVES, and nothing else. For every shape
 * gfx/draw.hpp knows how to draw fast, this file computes the same set
 * of pixels SLOWLY AND FROM THE DEFINITION - by asking, of every pixel
 * on the canvas independently, whether it belongs to the shape. No
 * incremental error term, no octant symmetry, no shared arithmetic with
 * the thing it judges: an oracle is worth exactly its independence, and
 * a reference that borrowed the algorithm would prove f(x) == f(x).
 *
 * It lives in the host stratum because from here it cannot be linked
 * into a target image even by accident, and because down here the whole
 * standard library is allowed - this code exists to be obviously right,
 * never to be quick (docs/host/README.md).
 *
 * WHAT IT CANNOT JUDGE. Whether a specification is the RIGHT one: when
 * this file and a primitive disagree, the usual finding is that the
 * specification was never decided, not that either side is broken. And
 * whether a result is legible - that is a golden image looked at once by
 * a human. Glyph shapes in particular can never be judged here: a
 * reference reading the same font data would agree with the same
 * mistake.
 */

#pragma once

#include <stdint.h>

#include <cmath>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include "gfx/surface.hpp"

namespace brio {

/**
 * A canvas of one byte per pixel, whatever the format under test - this
 * is the observer's copy, not a surface. It is deliberately NOT a
 * Surface: nothing must be able to draw into it through the library.
 */
class RefCanvas {
public:
    RefCanvas(Extent w, Extent h) : w_(w), h_(h), px_(static_cast<size_t>(w) * h, 0) {}

    Extent width() const { return w_; }
    Extent height() const { return h_; }

    uint8_t at(Coord x, Coord y) const {
        if (!inside(x, y)) {
            return 0;
        }
        return px_[index(x, y)];
    }

    void clear(uint8_t c) {
        for (auto& p : px_) {
            p = c;
        }
    }

    /// Every pixel asked, independently, whether it is in the rectangle.
    void fill_rect(Coord x, Coord y, Extent w, Extent h, uint8_t c) {
        each([&](int32_t px, int32_t py) {
            return in_rect(px, py, x, y, w, h);
        }, c);
    }

    /// In the rectangle AND on one of its four edges.
    void rect(Coord x, Coord y, Extent w, Extent h, uint8_t c) {
        if (w == 0 || h == 0) {
            return;
        }
        const int32_t x0 = x;
        const int32_t y0 = y;
        const int32_t x1 = x0 + w - 1;
        const int32_t y1 = y0 + h - 1;
        each([&](int32_t px, int32_t py) {
            return in_rect(px, py, x, y, w, h) &&
                   (px == x0 || px == x1 || py == y0 || py == y1);
        }, c);
    }

    void hline(Coord x, Coord y, Extent len, uint8_t c) {
        fill_rect(x, y, len, 1, c);
    }

    void vline(Coord x, Coord y, Extent len, uint8_t c) {
        fill_rect(x, y, 1, len, c);
    }

    /// In the rectangle AND NOT in the rectangle shrunk by `t` on every
    /// side. When the shrinking leaves nothing the outline is the whole
    /// rectangle - the definition says so on its own, with no clamp.
    void rect(Coord x, Coord y, Extent w, Extent h, Extent t, uint8_t c) {
        if (w == 0 || h == 0 || t == 0) {
            return;
        }
        const int32_t ix0 = static_cast<int32_t>(x) + static_cast<int32_t>(t);
        const int32_t iy0 = static_cast<int32_t>(y) + static_cast<int32_t>(t);
        const int32_t ix1 = static_cast<int32_t>(x) + static_cast<int32_t>(w) -
                            static_cast<int32_t>(t); // exclusive
        const int32_t iy1 = static_cast<int32_t>(y) + static_cast<int32_t>(h) -
                            static_cast<int32_t>(t);
        each([&](int32_t px, int32_t py) {
            if (!in_rect(px, py, x, y, w, h)) {
                return false;
            }
            const bool inner = px >= ix0 && px < ix1 && py >= iy0 && py < iy1;
            return !inner;
        }, c);
    }

    /// A thick line is the rectangle it covers: `t` rows down from `y`,
    /// or `t` columns right from `x`.
    void hline(Coord x, Coord y, Extent len, Extent t, uint8_t c) {
        fill_rect(x, y, len, t, c);
    }

    void vline(Coord x, Coord y, Extent len, Extent t, uint8_t c) {
        fill_rect(x, y, t, len, c);
    }

    /**
     * The segment, from the definition in gfx/draw.hpp's comment: walk
     * the MAJOR axis, and at each step the pixel whose centre is nearest
     * the ideal segment. Asked here of every pixel at once - is this
     * pixel's major coordinate within the span, and is its minor
     * coordinate the nearest one there? - with no stepping and no error
     * term anywhere.
     *
     * The tie, when the segment passes exactly half way between two
     * centres, is broken away from zero on the offset travelled, which
     * is what llround does and what the integer primitive is claimed to
     * do. If that claim is false the exhaustive comparison says so.
     */
    void line(Coord x0, Coord y0, Coord x1, Coord y1, uint8_t c) {
        const int32_t dx = static_cast<int32_t>(x1) - static_cast<int32_t>(x0);
        const int32_t dy = static_cast<int32_t>(y1) - static_cast<int32_t>(y0);
        const bool major_x = std::abs(dx) >= std::abs(dy);
        each([&](int32_t px, int32_t py) {
            const int32_t major = major_x ? px : py;
            const int32_t a0 = major_x ?
                               static_cast<int32_t>(x0)
                               : static_cast<int32_t>(y0);
            const int32_t a1 = major_x ?
                               static_cast<int32_t>(x1)
                               : static_cast<int32_t>(y1);
            const int32_t b0 = major_x ?
                               static_cast<int32_t>(y0)
                               : static_cast<int32_t>(x0);
            const int32_t minor = major_x ? py : px;
            const int32_t lo = a0 < a1 ? a0 : a1;
            const int32_t hi = a0 < a1 ? a1 : a0;
            if (major < lo || major > hi) {
                return false;
            }
            if (a1 == a0) { // a point: both spans zero
                return minor == b0;
            }
            const double t = double(major - a0) / double(a1 - a0);
            const int32_t span = major_x ? dy : dx;
            const int32_t want =
                b0 + static_cast<int32_t>(std::llround(t * double(span)));
            return minor == want;
        }, c);
    }

    /**
     * The circle's ring, from the definition in gfx/draw.hpp: for every
     * column within the radius the nearest row, and for every row the
     * nearest column, UNIONED. Asked of each pixel at once - is my row
     * the nearest one for my column, or my column the nearest for my
     * row? - with no octant, no reflection and no error term.
     */
    void circle(Coord cx, Coord cy, Extent r, uint8_t c) {
        if (r == 0) {
            each([&](int32_t px, int32_t py) {
                return px == static_cast<int32_t>(cx) && py == static_cast<int32_t>(cy);
            }, c);
            return;
        }
        each([&](int32_t px, int32_t py) {
            const int32_t dx = std::abs(px - static_cast<int32_t>(cx));
            const int32_t dy = std::abs(py - static_cast<int32_t>(cy));
            if (dx > static_cast<int32_t>(r) || dy > static_cast<int32_t>(r)) {
                return false;
            }
            return dy == nearest_on_circle(dx, r) ||
                   dx == nearest_on_circle(dy, r);
        }, c);
    }

    /// The disc: an inequality, and nothing more.
    void fill_circle(Coord cx, Coord cy, Extent r, uint8_t c) {
        const int32_t rr = static_cast<int32_t>(r) * static_cast<int32_t>(r);
        each([&](int32_t px, int32_t py) {
            const int32_t dx = px - static_cast<int32_t>(cx);
            const int32_t dy = py - static_cast<int32_t>(cy);
            return dx * dx + dy * dy <= rr;
        }, c);
    }

    /**
     * The rounded rectangle's outline: a pixel belongs if it is on one
     * of the four straight runs, or on one of the four quarter rings -
     * each ring judged by the circle rule above, about its own corner
     * centre, and only in the quadrant that faces outwards.
     *
     * This oracle tests the ARITHMETIC and not the decomposition into
     * runs and arcs, because that decomposition IS the specification.
     */
    void round_rect(Coord x, Coord y, Extent w, Extent h, Extent r, uint8_t c) {
        if (w == 0 || h == 0) {
            return;
        }
        const Extent limit = static_cast<Extent>(((w < h ? w : h) - 1) / 2);
        if (r > limit) {
            r = limit;
        }
        if (r == 0) {
            rect(x, y, w, h, c);
            return;
        }
        const int32_t x0 = static_cast<int32_t>(x);
        const int32_t y0 = static_cast<int32_t>(y);
        const int32_t x1 = x0 + static_cast<int32_t>(w) - 1;
        const int32_t y1 = y0 + static_cast<int32_t>(h) - 1;
        const int32_t ax0 = x0 + static_cast<int32_t>(r);
        const int32_t ax1 = x1 - static_cast<int32_t>(r);
        const int32_t ay0 = y0 + static_cast<int32_t>(r);
        const int32_t ay1 = y1 - static_cast<int32_t>(r);

        each([&](int32_t px, int32_t py) {
            const bool in_cols = px >= ax0 && px <= ax1;
            const bool in_rows = py >= ay0 && py <= ay1;
            if (in_cols && (py == y0 || py == y1)) {
                return true;
            }
            if (in_rows && (px == x0 || px == x1)) {
                return true;
            }
            // Outside both straight bands: the quadrant of the nearest
            // corner, judged as a ring about that corner's centre.
            if (in_cols || in_rows) {
                return false;
            }
            const int32_t cx = px < ax0 ? ax0 : ax1;
            const int32_t cy = py < ay0 ? ay0 : ay1;
            const int32_t dx = std::abs(px - cx);
            const int32_t dy = std::abs(py - cy);
            if (dx > static_cast<int32_t>(r) || dy > static_cast<int32_t>(r)) {
                return false;
            }
            return dy == nearest_on_circle(dx, r) ||
                   dx == nearest_on_circle(dy, r);
        }, c);
    }

    /// The filled rounded rectangle: within `r` of the inset rectangle -
    /// the Minkowski sum written as the distance it is.
    void fill_round_rect(Coord x, Coord y, Extent w, Extent h, Extent r,
                         uint8_t c) {
        if (w == 0 || h == 0) {
            return;
        }
        const Extent limit = static_cast<Extent>(((w < h ? w : h) - 1) / 2);
        if (r > limit) {
            r = limit;
        }
        const int32_t x0i = static_cast<int32_t>(x) + static_cast<int32_t>(r);
        const int32_t x1i = int32_t{x} + w - 1 - r;
        const int32_t y0i = static_cast<int32_t>(y) + static_cast<int32_t>(r);
        const int32_t y1i = int32_t{y} + h - 1 - r;
        const int32_t rr = static_cast<int32_t>(r) * static_cast<int32_t>(r);
        each([&](int32_t px, int32_t py) {
            int32_t dx = 0;
            int32_t dy = 0;
            if (px < x0i) {
                dx = x0i - px;
            } else if (px > x1i) {
                dx = px - x1i;
            }
            if (py < y0i) {
                dy = y0i - py;
            } else if (py > y1i) {
                dy = py - y1i;
            }
            return dx * dx + dy * dy <= rr;
        }, c);
    }

    /**
     * A line of text, by asking of every pixel which cell it falls in,
     * which row and column of that cell it is, and whether the font
     * lights that bit. No buffer, no run, no chunking, no advance
     * carried from one character to the next.
     *
     * This judges the GEOMETRY - where cells land, how they tile, what
     * the background covers, where the edge cuts - and not the glyphs:
     * a reference reading the same font data agrees with the same
     * mistake. Letter shapes are a golden image looked at once.
     */
    template <typename F>
    void text(Coord x, Coord y, std::string_view str, uint8_t fg, uint8_t bg) {
        if (str.empty()) {
            return;
        }
        const int32_t cells = static_cast<int32_t>(str.size());
        each([&](int32_t px, int32_t py) {
            const int32_t dx = px - static_cast<int32_t>(x);
            const int32_t dy = py - static_cast<int32_t>(y);
            if (dx < 0 || dy < 0 || dy >= static_cast<int32_t>(F::cell_h)) {
                return false;
            }
            const int32_t cell = dx / static_cast<int32_t>(F::cell_w);
            if (cell >= cells) {
                return false;
            }
            return true;
        }, bg);
        each([&](int32_t px, int32_t py) {
            const int32_t dx = px - static_cast<int32_t>(x);
            const int32_t dy = py - static_cast<int32_t>(y);
            if (dx < 0 || dy < 0 || dy >= static_cast<int32_t>(F::cell_h)) {
                return false;
            }
            const int32_t cell = dx / static_cast<int32_t>(F::cell_w);
            if (cell >= cells) {
                return false;
            }
            const int32_t col = dx % static_cast<int32_t>(F::cell_w);
            const typename F::Row bits =
                F::row_bits(
                            static_cast<uint8_t>(str[static_cast<size_t>(cell)]),
                            static_cast<Extent>(dy));
            return ((bits >> (static_cast<int32_t>(F::cell_w) - 1 - col)) & 1u) != 0;
        }, fg);
    }

    /// '#' where lit, '.' where not; one line per row, a header naming
    /// the geometry. ASCII because this is what gets checked in beside a
    /// test and read by a human when one fails.
    std::string ascii() const {
        std::string out = "gfx " + std::to_string(w_) + " " + std::to_string(h_) + "\n";
        for (Extent y = 0; y < h_; ++y) {
            for (Extent x = 0; x < w_; ++x) {
                out += px_[index(static_cast<Coord>(x), static_cast<Coord>(y))] ?
                       '#'
                       : '.';
            }
            out += '\n';
        }
        return out;
    }

private:
    bool inside(Coord x, Coord y) const {
        const int32_t px = x;
        const int32_t py = y;
        return px >= 0 && px < w_ && py >= 0 && py < h_;
    }

    size_t index(Coord x, Coord y) const {
        return static_cast<size_t>(static_cast<uint16_t>(y)) * w_ +
               static_cast<size_t>(static_cast<uint16_t>(x));
    }

    /// How far from the centre line the ideal circle stands at offset
    /// `a`: the definition itself, in floating point, rounded to the
    /// nearest whole pixel and away from zero at a tie. This file is
    /// allowed to be slow and is required to be obvious.
    static int32_t nearest_on_circle(int32_t a, Extent r) {
        const double rr =
            double(r) * double(r) - double(a) * double(a);
        return static_cast<int32_t>(std::llround(std::sqrt(rr < 0.0 ? 0.0 : rr)));
    }

    static bool in_rect(int32_t px, int32_t py, Coord x, Coord y, Extent w,
                        Extent h) {
        const int32_t x0 = x;
        const int32_t y0 = y;
        return px >= x0 && px < x0 + w && py >= y0 && py < y0 + h;
    }

    /// Ask the predicate of every pixel on the canvas, independently.
    template <typename F>
    void each(F&& belongs, uint8_t c) {
        for (Extent y = 0; y < h_; ++y) {
            for (Extent x = 0; x < w_; ++x) {
                if (belongs(static_cast<int32_t>(x), static_cast<int32_t>(y))) {
                    px_[index(static_cast<Coord>(x), static_cast<Coord>(y))] = c;
                }
            }
        }
    }

    Extent w_;
    Extent h_;
    std::vector<uint8_t> px_;
};

/// What a comparison found. `map` is the picture a failure prints:
/// '#' both agree lit, 'A' only the surface, 'R' only the reference,
/// '.' both agree dark.
struct GfxDiff {
    uint32_t only_surface{};
    uint32_t only_reference{};
    std::string map;

    bool agree() const { return only_surface == 0 && only_reference == 0; }
};

/// Compare a drawn surface against the reference, pixel by pixel. Takes
/// a ReadableSurface because READING IS THE OBSERVER'S PRIVILEGE - the
/// library's primitives never get this concept.
template <ReadableSurface S>
GfxDiff compare(const S& s, const RefCanvas& ref) {
    GfxDiff d;
    d.map = "gfx " + std::to_string(s.width()) + " " + std::to_string(s.height()) + "\n";
    for (Extent y = 0; y < s.height(); ++y) {
        for (Extent x = 0; x < s.width(); ++x) {
            const bool a =
                s.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) != 0;
            const bool r = ref.at(static_cast<Coord>(x), static_cast<Coord>(y)) != 0;
            if (a && r) {
                d.map += '#';
            } else if (a) {
                d.map += 'A';
                ++d.only_surface;
            } else if (r) {
                d.map += 'R';
                ++d.only_reference;
            } else {
                d.map += '.';
            }
        }
        d.map += '\n';
    }
    return d;
}

/// The same picture for a surface alone, for a golden file or an eye.
template <ReadableSurface S>
std::string ascii(const S& s) {
    std::string out =
        "gfx " + std::to_string(s.width()) + " " + std::to_string(s.height()) + "\n";
    for (Extent y = 0; y < s.height(); ++y) {
        for (Extent x = 0; x < s.width(); ++x) {
            out += s.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) ?
                   '#'
                   : '.';
        }
        out += '\n';
    }
    return out;
}

} // namespace brio
