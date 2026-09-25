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

#include <array>
#include <string_view>

#include "gfx/font.hpp"
#include "gfx/surface.hpp"

namespace brio {

/// How many pixels a text row buffers before it is flushed to the
/// surface. Bounded on purpose: the cost of a long string is more runs,
/// never more stack. Sized so a short field crosses in one window; a
/// scaled cell wider than this is split across runs, so a large readout
/// costs several windows a row on a panel.
inline constexpr uint16_t text_run_max = 32;

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
    const int32_t cell_w = F::cell_w;   // a constant; folded away
    const int32_t end_x = x + cell_w * static_cast<int32_t>(str.size());
    if (str.empty()) {
        return static_cast<Coord>(end_x);
    }

    for (Extent row = 0; row < F::cell_h; ++row) {
        std::array<Color, text_run_max> run{};
        uint16_t held = 0;
        int32_t run_x = static_cast<int32_t>(x);

        for (char ch : str) {
            const typename F::Row bits = F::row_bits(static_cast<uint8_t>(ch), row);
            for (Extent col = 0; col < F::cell_w; ++col) {
                const bool on =
                    (bits >> (F::cell_w - 1 - col)) & 1u;
                run[held++] = on ? fg : bg;
                if (held == text_run_max) {
                    s.write_run(static_cast<Coord>(run_x),
                                static_cast<Coord>(static_cast<int32_t>(y) + row),
                                std::span<const Color>(run.data(), held));
                    run_x += static_cast<int32_t>(held);
                    held = 0;
                }
            }
        }
        if (held != 0) {
            s.write_run(static_cast<Coord>(run_x),
                        static_cast<Coord>(static_cast<int32_t>(y) + row),
                        std::span<const Color>(run.data(), held));
        }
    }
    return static_cast<Coord>(end_x);
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
    const int32_t cell_w = F::cell_w;
    const Coord after = text<F>(s, x, y, str, fg, bg);
    const Extent blank = static_cast<Extent>(cells - static_cast<Extent>(str.size()));
    if (blank != 0) {
        s.fill_rect(after, y,
                    static_cast<Extent>(static_cast<int32_t>(blank) * cell_w),
                    F::cell_h, bg);
    }
    return static_cast<Coord>(x + cells * cell_w);
}

} // namespace brio
