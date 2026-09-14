/*
 * text.hpp (gfx)
 *
 * TEXT IS OPAQUE, ALWAYS. A cell is drawn foreground AND background
 * together, so writing a character replaces whatever stood there without
 * reading it - which is the only way a field can be updated on a panel
 * that answers no questions. There is therefore no transparent variant,
 * and its absence is a property of the tier and not a missing feature.
 *
 * A FIELD IS THE VERB THAT MATTERS. text() draws what it is given;
 * text_field() pads to a fixed number of cells, which turns "the
 * previous value is gone" from something a caller must remember into
 * something the call does. A value that shrinks from "100" to "99"
 * leaves no 0 behind.
 *
 * GENERATED ONE ROW AT A TIME AND STREAMED. A line of text is written as
 * horizontal runs, not pixel by pixel: for each row of the cell the
 * glyph bits become colours in a small buffer which is flushed to the
 * surface's run verb. The pixels exist only in flight, the buffer is a
 * fixed size that does not grow with the string, and a panel sees a
 * handful of windows instead of one per pixel.
 *
 * (docs/design/gfx.md.)
 */

#pragma once

#include <stdint.h>

#include <string_view>

#include "gfx/font.hpp"
#include "gfx/surface.hpp"

namespace brio {

/// How many pixels a text row buffers before it is flushed to the
/// surface. Bounded on purpose: the cost of a long string is more runs,
/// never more stack. Sized so a short field crosses in one window.
inline constexpr unsigned text_run_max = 32;

/**
 * One line of text at (x, y), the top-left of the first cell. Returns
 * the x where the next cell would start, so callers can chain without
 * repeating the arithmetic.
 *
 * `fg` and `bg` are both required: see the header comment.
 */
template <typename F, Surface S>
    requires Font<F>
Coord text(S& s, Coord x, Coord y, std::string_view str, typename S::Color fg,
           typename S::Color bg) {
    using Color = typename S::Color;
    const int32_t end_x = int32_t(x) + int32_t(F::cell_w) * int32_t(str.size());
    if (str.empty()) {
        return Coord(end_x);
    }

    for (Extent row = 0; row < F::cell_h; ++row) {
        Color run[text_run_max];
        unsigned held = 0;
        int32_t run_x = int32_t(x);

        for (char ch : str) {
            const uint8_t bits = F::row_bits(uint8_t(ch), row);
            for (Extent col = 0; col < F::cell_w; ++col) {
                const bool on =
                    (bits >> (F::cell_w - 1 - col)) & 1u;
                run[held++] = on ? fg : bg;
                if (held == text_run_max) {
                    s.write_run(Coord(run_x), Coord(int32_t(y) + row),
                                std::span<const Color>(run, held));
                    run_x += int32_t(held);
                    held = 0;
                }
            }
        }
        if (held != 0) {
            s.write_run(Coord(run_x), Coord(int32_t(y) + row),
                        std::span<const Color>(run, held));
        }
    }
    return Coord(end_x);
}

/**
 * A field of exactly `cells` characters: the string, then background to
 * the end. Whatever the field held before is gone, without anyone having
 * had to know what it was - the erase-by-rewriting discipline as a verb.
 *
 * A string longer than the field is cut, never allowed to overrun the
 * space the layout gave it.
 */
template <typename F, Surface S>
    requires Font<F>
Coord text_field(S& s, Coord x, Coord y, std::string_view str, Extent cells,
                 typename S::Color fg, typename S::Color bg) {
    if (str.size() > cells) {
        str = str.substr(0, cells);
    }
    const Coord after = text<F>(s, x, y, str, fg, bg);
    const Extent blank = Extent(cells - Extent(str.size()));
    if (blank != 0) {
        s.fill_rect(after, y, Extent(int32_t(blank) * int32_t(F::cell_w)),
                    F::cell_h, bg);
    }
    return Coord(int32_t(x) + int32_t(cells) * int32_t(F::cell_w));
}

} // namespace brio
