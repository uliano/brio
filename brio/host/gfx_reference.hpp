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
    RefCanvas(Extent w, Extent h) : w_(w), h_(h), px_(size_t(w) * h, 0) {}

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
        const int32_t x1 = int32_t(x) + int32_t(w) - 1;
        const int32_t y1 = int32_t(y) + int32_t(h) - 1;
        each([&](int32_t px, int32_t py) {
            return in_rect(px, py, x, y, w, h) &&
                   (px == int32_t(x) || px == x1 || py == int32_t(y) || py == y1);
        }, c);
    }

    void hline(Coord x, Coord y, Extent len, uint8_t c) {
        fill_rect(x, y, len, 1, c);
    }

    void vline(Coord x, Coord y, Extent len, uint8_t c) {
        fill_rect(x, y, 1, len, c);
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
        const int32_t dx = int32_t(x1) - int32_t(x0);
        const int32_t dy = int32_t(y1) - int32_t(y0);
        const bool major_x = std::abs(dx) >= std::abs(dy);
        each([&](int32_t px, int32_t py) {
            const int32_t major = major_x ? px : py;
            const int32_t a0 = major_x ? int32_t(x0) : int32_t(y0);
            const int32_t a1 = major_x ? int32_t(x1) : int32_t(y1);
            const int32_t b0 = major_x ? int32_t(y0) : int32_t(x0);
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
            const int32_t want = b0 + int32_t(std::llround(t * double(span)));
            return minor == want;
        }, c);
    }

    /// '#' where lit, '.' where not; one line per row, a header naming
    /// the geometry. ASCII because this is what gets checked in beside a
    /// test and read by a human when one fails.
    std::string ascii() const {
        std::string out = "gfx " + std::to_string(w_) + " " + std::to_string(h_) + "\n";
        for (Extent y = 0; y < h_; ++y) {
            for (Extent x = 0; x < w_; ++x) {
                out += px_[index(Coord(x), Coord(y))] ? '#' : '.';
            }
            out += '\n';
        }
        return out;
    }

private:
    bool inside(Coord x, Coord y) const {
        return int32_t(x) >= 0 && int32_t(x) < int32_t(w_) && int32_t(y) >= 0 &&
               int32_t(y) < int32_t(h_);
    }

    size_t index(Coord x, Coord y) const {
        return size_t(uint16_t(y)) * w_ + size_t(uint16_t(x));
    }

    static bool in_rect(int32_t px, int32_t py, Coord x, Coord y, Extent w,
                        Extent h) {
        return px >= int32_t(x) && px < int32_t(x) + int32_t(w) &&
               py >= int32_t(y) && py < int32_t(y) + int32_t(h);
    }

    /// Ask the predicate of every pixel on the canvas, independently.
    template <typename F>
    void each(F&& belongs, uint8_t c) {
        for (Extent y = 0; y < h_; ++y) {
            for (Extent x = 0; x < w_; ++x) {
                if (belongs(int32_t(x), int32_t(y))) {
                    px_[index(Coord(x), Coord(y))] = c;
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
            const bool a = s.get_pixel(Coord(x), Coord(y)) != 0;
            const bool r = ref.at(Coord(x), Coord(y)) != 0;
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
    std::string out = "gfx " + std::to_string(s.width()) + " " + std::to_string(s.height()) + "\n";
    for (Extent y = 0; y < s.height(); ++y) {
        for (Extent x = 0; x < s.width(); ++x) {
            out += s.get_pixel(Coord(x), Coord(y)) ? '#' : '.';
        }
        out += '\n';
    }
    return out;
}

} // namespace brio
