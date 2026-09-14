/*
 * font.hpp (gfx)
 *
 * WHAT A FONT IS HERE: a TYPE, not a pointer to data. The cell is then a
 * compile-time constant, so laying a field out is arithmetic the
 * compiler does, and the linker drops every font a program does not
 * name - on a part with sixteen kilobytes of flash the fonts are the
 * largest data in the image, so this is not a micro-optimization.
 *
 * MONOSPACED, AND THAT IS A CONSEQUENCE. A fixed cell is what lets a
 * field erase itself by being written over, which is the only erasing a
 * write-only surface has. A proportional font would need a per-glyph
 * advance and would break that guarantee, so there is none.
 *
 * THE CELL INCLUDES ITS ADVANCE - a six-by-eight cell carries a
 * five-by-seven glyph - so consecutive cells tile with no gap and the
 * opaque background covers the whole run. A glyph's row is a BYTE whose
 * bit (cell_w - 1) is its leftmost pixel, the same direction Mono packs
 * a framebuffer in, so no renderer ever has to reverse anything.
 *
 * ROWS AND NOT COLUMNS, against the tradition. Column-major glyphs exist
 * because a page-oriented panel stores columns; but a panel's memory
 * layout is its own driver's business (gfx/surface.hpp), and up here
 * text is drawn as horizontal runs. Storing rows is what lets a line of
 * text be generated one row at a time and streamed, with no transpose
 * anywhere.
 *
 * (docs/design/gfx.md.)
 */

#pragma once

#include <stdint.h>

#include <concepts>

#include "gfx/surface.hpp"

namespace brio {

/**
 * What a renderer needs of a font: the cell it tiles with, the range of
 * characters it draws, and one row of one glyph as a bit pattern.
 *
 * `row_bits` is total: a row past the glyph's height, a character
 * outside the range - it answers for everything, so no caller checks.
 */
template <typename F>
concept Font = requires(uint8_t ch, Extent row) {
    requires F::cell_w > 0;
    requires F::cell_h > 0;
    { F::row_bits(ch, row) } -> std::same_as<uint8_t>;
};

} // namespace brio
