/*
 * font.hpp (gfx)
 *
 * What a font is to the text renderer (gfx/text.hpp): the Font concept a
 * font type meets, the narrowest row type for a cell, and Scaled<F, n>,
 * any font enlarged by a whole number. A program names a font type as a
 * template argument of the text verbs; the one font the library carries
 * is gfx/font_5x7.hpp's. The smallest use, a caption at twice the table's
 * size and a readout at four times:
 *
 *     using Caption = brio::Scaled<brio::Font5x7, 2>;
 *     using Readout = brio::Scaled<brio::Font5x7, 4>;
 *     brio::text<Caption>(s, label_x, label_y, "V", col_ink, col_bg);
 *
 * A font is constexpr data and constexpr functions: nothing to
 * initialize, nothing owned at run time, callable from any context. Its
 * glyph table is static data of the image, there for as long as the
 * program is.
 *
 * What a font is here: a type, not a pointer to data. The cell is then a
 * compile-time constant, so laying a field out is arithmetic the
 * compiler does, and the linker drops every font a program does not
 * name - on a part with sixteen kilobytes of flash the fonts are the
 * largest data in the image, so this is not a micro-optimization
 * (docs/design/gfx.md, "Text").
 *
 * Monospaced, and that is a consequence. A fixed cell is what lets a
 * field erase itself by being written over, which is the only erasing a
 * write-only surface has. A proportional font would need a per-glyph
 * advance and would break that guarantee, so there is none.
 *
 * The cell includes its advance - a six-by-eight cell carries a
 * five-by-seven glyph - so consecutive cells tile with no gap and the
 * opaque background covers the whole run. A glyph's row is the font's
 * own unsigned Row type - a byte for a cell up to eight wide, wider for
 * a scaled one - whose bit (cell_w - 1) is its leftmost pixel, the same
 * direction Mono packs a framebuffer in, so no renderer ever has to
 * reverse anything.
 *
 * Rows and not columns, against the tradition. Column-major glyphs exist
 * because a page-oriented panel stores columns; but a panel's memory
 * layout is its own driver's business (gfx/surface.hpp), and up here
 * text is drawn as horizontal runs. Storing rows is what lets a line of
 * text be generated one row at a time and streamed, with no transpose
 * anywhere.
 *
 * A larger size is an adapter, not a second table. Scaled<F, n> below is
 * a font like any other - a type, with a cell n times the size and every
 * pixel an n by n block - and it adds no data at all, which on the
 * smallest part is the whole reason a readout can be three times the
 * height of a caption. A font drawn for a size is the other road, and
 * costs its table; none exists, because no display has yet asked for
 * more legibility than a block gives.
 */

#pragma once

#include <stdint.h>

#include <concepts>
#include <type_traits>

#include "gfx/surface.hpp"

namespace brio {

/**
 * What a renderer needs of a font: the cell it tiles with, the type one
 * row of a glyph comes as, and one row of one glyph as a bit pattern in
 * that type.
 *
 * `row_bits` is total: a row past the glyph's height, a character
 * outside the range - it answers for everything, so no caller checks.
 */
template <typename F>
concept Font = requires(uint8_t ch, Extent row) {
    typename F::Row;
    requires std::unsigned_integral<typename F::Row>;
    requires F::cell_w > 0;
    requires F::cell_h > 0;
    requires F::cell_w <= 8 * sizeof(typename F::Row);
    { F::row_bits(ch, row) } -> std::same_as<typename F::Row>;
};

/// The narrowest unsigned type that holds a row of `bits` pixels, for a
/// font to name as its Row: a scaled cell pays a wider shift only where
/// it is wider.
template <Extent bits>
using FontRow = std::conditional_t<(bits <= 8), uint8_t,
                                   std::conditional_t<(bits <= 16), uint16_t, uint32_t>>;

/**
 * A font enlarged by a whole number: every pixel of `F` becomes an n by
 * n block. A type, like the font it wraps, so it satisfies Font and no
 * renderer learns it is not a table of its own - and it carries no data.
 * A row is asked of the base font once per n rows, and each of its bits
 * is repeated n times. The scaled cell is at most 32 pixels wide (the
 * requires clause).
 */
template <Font F, uint8_t n>
    requires (n >= 1 && F::cell_w * n <= 32)
struct Scaled {
    static constexpr Extent cell_w = static_cast<Extent>(F::cell_w * n);
    static constexpr Extent cell_h = static_cast<Extent>(F::cell_h * n);
    using Row = FontRow<cell_w>;

    static constexpr Row row_bits(uint8_t ch, Extent row) {
        constexpr Row block = static_cast<Row>((uint32_t{1} << n) - 1u);
        const typename F::Row base = F::row_bits(ch, static_cast<Extent>(row / n));
        Row out = 0;
        for (Extent col = 0; col < F::cell_w; ++col) {
            const bool on = ((base >> (F::cell_w - 1 - col)) & 1u) != 0;
            out = static_cast<Row>((out << n) | (on ? block : Row{0}));
        }
        return out;
    }
};

} // namespace brio
