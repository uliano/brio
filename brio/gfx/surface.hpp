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
 * signed; a width is not. Their SUM would not fit either of them, which
 * is why clip() below never forms it - and why nothing here needs to be
 * wider than sixteen bits. On an eight-bit part that is the difference
 * between one instruction and two on the hottest path in the library.
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

/// THE COORDINATE DOMAIN. A position, and a surface's own dimensions,
/// stay within this: a panel an MCU drives is at most some hundreds of
/// pixels a side, and this leaves room to place a shape several screens
/// outside one and let the clipping deal with it.
///
/// It exists because the segment's error term is tested at twice its
/// value and reaches three times the segment's span - measured - so a
/// domain is what lets that stepping stay sixteen bits wide instead of
/// thirty-two. At 4096 the widest intermediate is 24574, a quarter of
/// the type still spare.
///
/// A surface's size is checked against it; a coordinate a caller
/// computes cannot be, which is why the number is generous enough that
/// reaching it means something has already gone wrong.
inline constexpr Coord coord_max = 4096;

/// A width or a height. Never negative, and never added to a Coord:
/// clip() reaches the same answer without forming that sum.
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

/// How far a span starting at a negative `p` lies off the near edge.
/// Written so that the most negative coordinate there is does not
/// overflow on its way to becoming a magnitude: -(p + 1) is always
/// representable, and the missing one is added afterwards.
constexpr uint_fast16_t off_near_edge(Coord p) {
    return static_cast<uint_fast16_t>(static_cast<uint_fast16_t>(-(p + 1)) + 1u);
}

/**
 * The part of `r` that lies within a `bw` x `bh` surface, or nothing at
 * all.
 *
 * THIS RUNS ON EVERY PRIMITIVE CALL - on every pixel of a segment, since
 * a pixel is a one-by-one rectangle - so it is the one piece of
 * arithmetic in the library whose width is worth arguing about. It stays
 * in SIXTEEN BITS throughout, and does so without restricting what a
 * caller may ask: the trick is that the far edge `p + n` is never
 * formed. Trimming the near edge shortens the span, and trimming the far
 * one compares against the room that is left - `bound - p`, with `p`
 * already known non-negative and less than `bound`. Neither can leave
 * the range of the types.
 *
 * Forming `p + n` was what forced a wider type, and on an eight-bit part
 * that made the hottest path in the library twice the work for nothing.
 */
constexpr std::optional<Rect> clip(Rect r, Extent bw, Extent bh) {
    int_fast16_t x = r.x;
    int_fast16_t y = r.y;
    uint_fast16_t w = r.w;
    uint_fast16_t h = r.h;

    if (x < 0) {
        const uint_fast16_t skip = off_near_edge(x);
        if (skip >= w) {
            return {};
        }
        w -= skip;
        x = 0;
    }
    if (y < 0) {
        const uint_fast16_t skip = off_near_edge(y);
        if (skip >= h) {
            return {};
        }
        h -= skip;
        y = 0;
    }

    // Both non-negative now, so the comparisons and the subtractions
    // below are exact in the unsigned type.
    const uint_fast16_t px = static_cast<uint_fast16_t>(x);
    const uint_fast16_t py = static_cast<uint_fast16_t>(y);
    if (px >= bw || py >= bh) {
        return {};
    }
    const uint_fast16_t room_w = bw - px;
    const uint_fast16_t room_h = bh - py;
    if (w > room_w) {
        w = room_w;
    }
    if (h > room_h) {
        h = room_h;
    }
    if (w == 0 || h == 0) {
        return {};
    }
    return Rect{static_cast<Coord>(x), static_cast<Coord>(y),
                static_cast<Extent>(w), static_cast<Extent>(h)};
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
    static constexpr uint8_t bits = 1;
    static constexpr Color max_color = 1;

    static constexpr uint16_t stride_for(Extent w) {
        return static_cast<uint16_t>((static_cast<uint32_t>(w) + 7u) / 8u);
    }

    static constexpr Color get(const uint8_t* row, Extent x) {
        return static_cast<Color>((row[x >> 3] >> (7u - (x & 7u))) & 1u);
    }

    static constexpr void put(uint8_t* row, Extent x, Color c) {
        const uint8_t mask = static_cast<uint8_t>(0x80u >> (x & 7u));
        row[x >> 3] = c ? static_cast<uint8_t>(row[x >> 3] | mask)
                        : static_cast<uint8_t>(row[x >> 3] & static_cast<uint8_t>(~mask));
    }

    /// `n` pixels from `x`, one colour. Head bits, whole bytes, tail
    /// bits - the reason fill_rect is the base verb and not set_pixel.
    static constexpr void fill(uint8_t* row, Extent x, Extent n, Color c) {
        if (n == 0) {
            return;
        }
        const uint32_t first = x;
        const uint32_t last = static_cast<uint32_t>(x) + n - 1u; // inclusive
        const uint32_t fb = first >> 3;
        const uint32_t lb = last >> 3;
        const uint8_t head = static_cast<uint8_t>(0xFFu >> (first & 7u));
        const uint8_t tail = static_cast<uint8_t>(0xFFu << (7u - (last & 7u)));
        if (fb == lb) {
            const uint8_t m = static_cast<uint8_t>(head & tail);
            row[fb] = c ?
                      static_cast<uint8_t>(row[fb] | m)
                      : static_cast<uint8_t>(row[fb] & static_cast<uint8_t>(~m));
            return;
        }
        row[fb] = c ?
                  static_cast<uint8_t>(row[fb] | head)
                  : static_cast<uint8_t>(row[fb] & static_cast<uint8_t>(~head));
        for (uint32_t b = fb + 1u; b < lb; ++b) {
            row[b] = c ? static_cast<uint8_t>(0xFFu) : static_cast<uint8_t>(0x00u);
        }
        row[lb] = c ?
                  static_cast<uint8_t>(row[lb] | tail)
                  : static_cast<uint8_t>(row[lb] & static_cast<uint8_t>(~tail));
    }
};

/// Eight bits per pixel. The byte is a grey level or an index into a
/// table of 256 colours, and THIS LIBRARY DOES NOT CARE WHICH: the
/// palette travels with the surface, for whoever displays it. A grey
/// ramp is the identity palette.
struct Indexed8 {
    using Color = uint8_t;
    static constexpr uint8_t bits = 8;
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

    static_assert(W <= coord_max && H <= coord_max,
                  "a surface larger than the coordinate domain");

    static constexpr uint16_t stride = Fmt::stride_for(W);
    static constexpr size_t bytes = static_cast<size_t>(stride) * H;

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
            Fmt::fill(row_at(static_cast<Extent>(r->y + row)),
                      static_cast<Extent>(r->x), r->w, c);
        }
    }

    void write_run(Coord x, Coord y, std::span<const Color> run) {
        const uint_fast16_t py = static_cast<uint_fast16_t>(y);
        if (run.empty() || py >= H) {
            return;
        }
        int32_t x0 = x;
        int32_t x1 = x0 + static_cast<int32_t>(run.size());
        int32_t skip = 0;
        if (x0 < 0) {
            skip = -x0;
            x0 = 0;
        }
        if (x1 > static_cast<int32_t>(W)) {
            x1 = static_cast<int32_t>(W);
        }
        if (x1 <= x0) {
            return;
        }
        uint8_t* row = row_at(static_cast<Extent>(py));
        for (int32_t i = 0; i < x1 - x0; ++i) {
            Fmt::put(row, static_cast<Extent>(x0 + i),
                     run[static_cast<size_t>(skip + i)]);
        }
    }

    /// Out of bounds reads as 0: a reader outside the surface asked
    /// about a pixel that does not exist, and there is no colour to
    /// invent for it.
    Color get_pixel(Coord x, Coord y) const {
        // ONE comparison each: read as unsigned, a negative coordinate
        // becomes a very large one and fails the same bound test. Half
        // the work of asking twice, and it says the same thing.
        const uint_fast16_t px = static_cast<uint_fast16_t>(x);
        const uint_fast16_t py = static_cast<uint_fast16_t>(y);
        if (px >= W || py >= H) {
            return Color{};
        }
        return Fmt::get(row_at(static_cast<Extent>(py)),
                        static_cast<Extent>(px));
    }

private:
    uint8_t* row_at(Extent y) const { return px_ + static_cast<size_t>(y) * stride; }

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
        base_->fill_rect(static_cast<Coord>(r->x + x_),
                         static_cast<Coord>(r->y + y_), r->w, r->h, c);
    }

    void write_run(Coord x, Coord y, std::span<const Color> run) {
        const uint_fast16_t py = static_cast<uint_fast16_t>(y);
        if (run.empty() || py >= h_) {
            return;
        }
        int32_t x0 = x;
        int32_t x1 = x0 + static_cast<int32_t>(run.size());
        int32_t skip = 0;
        if (x0 < 0) {
            skip = -x0;
            x0 = 0;
        }
        if (x1 > static_cast<int32_t>(w_)) {
            x1 = static_cast<int32_t>(w_);
        }
        if (x1 <= x0) {
            return;
        }
        base_->write_run(static_cast<Coord>(x0 + x_), static_cast<Coord>(y + y_),
                         run.subspan(static_cast<size_t>(skip),
                                     static_cast<size_t>(x1 - x0)));
    }

private:
    S* base_;
    Coord x_;
    Coord y_;
    Extent w_;
    Extent h_;
};

} // namespace brio
