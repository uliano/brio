/*
 * surface.hpp (gfx)
 *
 * WHERE PIXELS GO. The Surface concept is the whole of what a drawing
 * primitive is allowed to assume, and it is deliberately the WRITE-ONLY
 * shape: two verbs, no way to read anything back. A panel that holds its
 * own pixels and answers no questions satisfies it; so does a
 * framebuffer in the program's memory, which can do far more and is not
 * asked to. Everything a primitive needs it gets from those two verbs.
 *
 * WHY THE VERBS ARE THESE TWO. A panel is addressed as a window and then
 * streamed into; that is what it is fast at. So the contract asks for a
 * rectangle filled with one colour and a horizontal run of caller-given
 * pixels - and a single pixel is the degenerate rectangle, derived, not
 * the foundation. A library built upward from set-a-pixel crawls on real
 * hardware because every pixel costs its own window command, and a
 * framebuffer hides that completely: the direction is fixed here so the
 * easy case cannot choose it later.
 *
 * THE READ VERB IS A REFINEMENT, AND PRIMITIVES MAY NOT USE IT.
 * ReadableSurface adds get_pixel for whoever owns the memory - a test,
 * an observer, an application in the memory-mapped tier. No primitive in
 * this library takes it as a parameter, and the base concept having no
 * such verb is the enforcement: a primitive that reached for it would
 * not compile against a write-only panel, which is the point.
 *
 * CLIPPING IS THE SURFACE'S JOB AND IT IS SILENT. On a microcontroller a
 * pixel written outside the buffer corrupts whatever lies beside it, so
 * every verb here trims to its own bounds. Silently, because an
 * interface that trims a shape at the edge of the screen is right and
 * one that refuses to draw it is not - and because a primitive that had
 * to ask first would pay for the question on every call.
 *
 * COORDINATES ARE SIGNED, EXTENTS ARE NOT. A shape may begin off the
 * left or top edge and still be partly visible, so a coordinate is
 * signed; a width is not. Their sum exceeds 16 bits, so the clipping
 * arithmetic names int32_t and the API never adds the two.
 *
 * (docs/design/gfx.md.)
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include <concepts>
#include <optional>
#include <span>

namespace brio {

/// A position. Signed: a shape may start off-surface and still show.
using Coord = int16_t;

/// A width or a height. Never negative; never added to a Coord except
/// through the int32_t arithmetic in clip().
using Extent = uint16_t;

/// An axis-aligned rectangle: an ORIGIN AND AN EXTENT, never two
/// corners. Whether a far corner is inside or outside is the commonest
/// off-by-one in drawing code; an extent has no such question, and a
/// zero extent means "nothing" with no special case anywhere.
struct Rect {
    Coord x{};
    Coord y{};
    Extent w{};
    Extent h{};
};

/// The part of `r` that lies within a `bw` x `bh` surface, or nothing at
/// all. The one place the coordinate and the extent meet, so the one
/// place that names its width.
constexpr std::optional<Rect> clip(Rect r, Extent bw, Extent bh) {
    int32_t x0 = r.x;
    int32_t y0 = r.y;
    int32_t x1 = x0 + int32_t(r.w); // exclusive
    int32_t y1 = y0 + int32_t(r.h);
    if (x0 < 0) {
        x0 = 0;
    }
    if (y0 < 0) {
        y0 = 0;
    }
    if (x1 > int32_t(bw)) {
        x1 = int32_t(bw);
    }
    if (y1 > int32_t(bh)) {
        y1 = int32_t(bh);
    }
    if (x1 <= x0 || y1 <= y0) {
        return {};
    }
    return Rect{Coord(x0), Coord(y0), Extent(x1 - x0), Extent(y1 - y0)};
}

/*
 * The pixel formats. A format says how a colour sits in a row of bytes
 * and nothing else - it is the "element type is the beat" rule of the
 * transfer engines, applied to pixels.
 */

/// One bit per pixel: row-major, the MOST SIGNIFICANT BIT LEFTMOST, each
/// row a whole number of bytes. This is a framebuffer's layout in
/// memory, and it is deliberately not any panel's native order - a panel
/// that stores columns of pages converts inside its own driver, where
/// the conversion can be judged against the silicon.
struct Mono {
    using Color = uint8_t; ///< 0 or 1
    static constexpr unsigned bits = 1;
    static constexpr Color max_color = 1;

    static constexpr uint16_t stride_for(Extent w) {
        return uint16_t((uint32_t(w) + 7u) / 8u);
    }

    static constexpr Color get(const uint8_t* row, Extent x) {
        return Color((row[x >> 3] >> (7u - (x & 7u))) & 1u);
    }

    static constexpr void put(uint8_t* row, Extent x, Color c) {
        const uint8_t mask = uint8_t(0x80u >> (x & 7u));
        row[x >> 3] = c ? uint8_t(row[x >> 3] | mask)
                        : uint8_t(row[x >> 3] & uint8_t(~mask));
    }

    /// `n` pixels from `x`, one colour. Head bits, whole bytes, tail
    /// bits - the reason fill_rect is the base verb and not set_pixel.
    static constexpr void fill(uint8_t* row, Extent x, Extent n, Color c) {
        if (n == 0) {
            return;
        }
        const uint32_t first = x;
        const uint32_t last = uint32_t(x) + n - 1u; // inclusive
        const uint32_t fb = first >> 3;
        const uint32_t lb = last >> 3;
        const uint8_t head = uint8_t(0xFFu >> (first & 7u));
        const uint8_t tail = uint8_t(0xFFu << (7u - (last & 7u)));
        if (fb == lb) {
            const uint8_t m = uint8_t(head & tail);
            row[fb] = c ? uint8_t(row[fb] | m) : uint8_t(row[fb] & uint8_t(~m));
            return;
        }
        row[fb] = c ? uint8_t(row[fb] | head) : uint8_t(row[fb] & uint8_t(~head));
        for (uint32_t b = fb + 1u; b < lb; ++b) {
            row[b] = c ? uint8_t(0xFFu) : uint8_t(0x00u);
        }
        row[lb] = c ? uint8_t(row[lb] | tail) : uint8_t(row[lb] & uint8_t(~tail));
    }
};

/// Eight bits per pixel. The byte is a grey level or an index into a
/// table of 256 colours, and THIS LIBRARY DOES NOT CARE WHICH: the
/// palette travels with the surface, for whoever displays it. A grey
/// ramp is the identity palette.
struct Indexed8 {
    using Color = uint8_t;
    static constexpr unsigned bits = 8;
    static constexpr Color max_color = 255;

    static constexpr uint16_t stride_for(Extent w) { return w; }

    static constexpr Color get(const uint8_t* row, Extent x) { return row[x]; }
    static constexpr void put(uint8_t* row, Extent x, Color c) { row[x] = c; }

    static constexpr void fill(uint8_t* row, Extent x, Extent n, Color c) {
        for (Extent i = 0; i < n; ++i) {
            row[x + i] = c;
        }
    }
};

/**
 * What a drawing primitive may assume. Two verbs and the bounds - and
 * no way to read a pixel back, which is what keeps every primitive
 * correct on a panel that answers nothing.
 */
template <typename S>
concept Surface = requires(S& s, const S& cs, Coord p, Extent e,
                           typename S::Color c,
                           std::span<const typename S::Color> run) {
    typename S::Format;
    typename S::Color;
    { cs.width() } -> std::same_as<Extent>;
    { cs.height() } -> std::same_as<Extent>;
    s.fill_rect(p, p, e, e, c);
    s.write_run(p, p, run);
};

/**
 * A surface whose pixels the program owns and may read back. The
 * memory-mapped tier, an observer, a test. NO PRIMITIVE IN THIS LIBRARY
 * REQUIRES IT - that is the whole point of it being a separate concept.
 */
template <typename S>
concept ReadableSurface = Surface<S> && requires(const S& cs, Coord p) {
    { cs.get_pixel(p, p) } -> std::same_as<typename S::Color>;
};

/**
 * Pixels in the program's own memory, over storage the CALLER owns -
 * nothing here allocates. The geometry is compile-time so the byte count
 * is a constant a caller can declare an array of; the bounds are still
 * read through width()/height() so that a Viewport, whose rectangle is a
 * run-time thing, satisfies the same concept.
 *
 * This is the memory-mapped tier's real implementation and not a
 * simulation of one: on a part with a display controller scanning RAM,
 * this type over that RAM is the whole driver.
 */
template <typename Fmt, Extent W, Extent H>
class Framebuffer {
public:
    using Format = Fmt;
    using Color = typename Fmt::Color;

    static constexpr uint16_t stride = Fmt::stride_for(W);
    static constexpr size_t bytes = size_t(stride) * H;

    explicit Framebuffer(std::span<uint8_t, bytes> storage)
        : px_(storage.data()) {}

    static constexpr Extent width() { return W; }
    static constexpr Extent height() { return H; }

    /// The raw bytes, for whoever publishes them to an observer.
    std::span<uint8_t, bytes> bits() const { return std::span<uint8_t, bytes>(px_, bytes); }

    void fill_rect(Coord x, Coord y, Extent w, Extent h, Color c) {
        const auto r = clip(Rect{x, y, w, h}, W, H);
        if (!r) {
            return;
        }
        for (Extent row = 0; row < r->h; ++row) {
            Fmt::fill(row_at(Extent(r->y + row)), Extent(r->x), r->w, c);
        }
    }

    void write_run(Coord x, Coord y, std::span<const Color> run) {
        if (run.empty() || int32_t(y) < 0 || int32_t(y) >= int32_t(H)) {
            return;
        }
        int32_t x0 = x;
        int32_t x1 = x0 + int32_t(run.size());
        int32_t skip = 0;
        if (x0 < 0) {
            skip = -x0;
            x0 = 0;
        }
        if (x1 > int32_t(W)) {
            x1 = int32_t(W);
        }
        if (x1 <= x0) {
            return;
        }
        uint8_t* row = row_at(Extent(y));
        for (int32_t i = 0; i < x1 - x0; ++i) {
            Fmt::put(row, Extent(x0 + i), run[size_t(skip + i)]);
        }
    }

    /// Out of bounds reads as 0: a reader outside the surface asked
    /// about a pixel that does not exist, and there is no colour to
    /// invent for it.
    Color get_pixel(Coord x, Coord y) const {
        if (int32_t(x) < 0 || int32_t(x) >= int32_t(W) || int32_t(y) < 0 ||
            int32_t(y) >= int32_t(H)) {
            return Color{};
        }
        return Fmt::get(row_at(Extent(y)), Extent(x));
    }

private:
    uint8_t* row_at(Extent y) const { return px_ + size_t(y) * stride; }

    uint8_t* px_;
};

/**
 * A window onto another surface: a translation and a clip, and NOT a
 * mode. It is a surface in its own right, so a primitive drawing into it
 * knows nothing of the one underneath and cannot escape the rectangle -
 * which is how an element of an interface gets its sandbox without every
 * primitive having to consult a clip rectangle on every call.
 *
 * Composes with itself. Costs nothing where it is not used.
 */
template <Surface S>
class Viewport {
public:
    using Format = typename S::Format;
    using Color = typename S::Color;

    Viewport(S& base, Coord x, Coord y, Extent w, Extent h)
        : base_(&base), x_(x), y_(y), w_(w), h_(h) {}

    Extent width() const { return w_; }
    Extent height() const { return h_; }

    void fill_rect(Coord x, Coord y, Extent w, Extent h, Color c) {
        const auto r = clip(Rect{x, y, w, h}, w_, h_);
        if (!r) {
            return;
        }
        base_->fill_rect(Coord(r->x + x_), Coord(r->y + y_), r->w, r->h, c);
    }

    void write_run(Coord x, Coord y, std::span<const Color> run) {
        if (run.empty() || int32_t(y) < 0 || int32_t(y) >= int32_t(h_)) {
            return;
        }
        int32_t x0 = x;
        int32_t x1 = x0 + int32_t(run.size());
        int32_t skip = 0;
        if (x0 < 0) {
            skip = -x0;
            x0 = 0;
        }
        if (x1 > int32_t(w_)) {
            x1 = int32_t(w_);
        }
        if (x1 <= x0) {
            return;
        }
        base_->write_run(Coord(x0 + x_), Coord(y + y_),
                         run.subspan(size_t(skip), size_t(x1 - x0)));
    }

private:
    S* base_;
    Coord x_;
    Coord y_;
    Extent w_;
    Extent h_;
};

} // namespace brio
