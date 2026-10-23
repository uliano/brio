/*
 * pen.hpp (gfx)
 *
 * The convenience, kept out of the contract. A pen holds what a caller
 * would otherwise repeat - a colour, a background, and a current point -
 * and forwards to the stateless primitives of gfx/draw.hpp and
 * gfx/text.hpp. The smallest use, a polyline and a line of text (the
 * host suite, test/test_gfx, spells the pen so):
 *
 *     brio::Pen<Panel> p(panel, 1, 0);       // ink 1 on background 0
 *     p.move_to(2, 3);
 *     p.line_to(20, 15);                     // from the cursor, which moves on
 *     p.line_to(25, 2);
 *     p.text<brio::Font5x7>("brio");         // in the pen's two colours
 *
 * A pen is an ordinary object the caller owns, over a surface it points
 * to (which must outlive it); it runs in the caller's context, as the
 * primitives do, and adds nothing to what they cost but its own few
 * stores. It is the only drawing state in this library (the counting
 * surface's tally, gfx/counting.hpp, is a measurement), and it is
 * deliberately above the surface rather than in it, because state below
 * would make primitives order-dependent and would let a golden image
 * turn on something nobody can see (docs/design/gfx.md, "Conventions":
 * drawing state lives above the base).
 *
 * The cursor is for tracing, and for nothing else. Sequential work - a
 * polyline across a plot, a run of text - is where each step begins
 * where the last one ended and repeating the coordinate would be noise.
 * The shapes are placed rather than continued, so they take their own
 * coordinates and the pen supplies only their colours.
 *
 * Nothing in gfx/ uses a pen internally. Every primitive keeps taking
 * all of its arguments, so a program that wants none of this pays for
 * none of it, and the two surfaces of the library never disagree.
 *
 * A width belongs here too, the day an oblique stroke wider than one
 * pixel earns its specification (docs/design/gfx.md, "What is
 * deliberately absent", says what that costs). It would be a fourth
 * member and a set of overloads, not a change to anything that exists.
 * The axis-aligned thick forms take their thickness as an argument of
 * the shape (gfx/draw.hpp), because it is a property of one rectangle
 * and not of a stroke.
 */

#pragma once

#include <stdint.h>

#include <string_view>

#include "gfx/draw.hpp"
#include "gfx/font.hpp"
#include "gfx/surface.hpp"
#include "gfx/text.hpp"

namespace brio {

/**
 * A drawing cursor over any surface. `move_to` places it, `line_to`
 * draws from it and leaves it at the far end - a polyline is then a
 * move and a run of line_to, with no coordinate written twice.
 */
template <Surface S>
class Pen {
public:
    using Color = typename S::Color;

    /// A pen over `surface` (kept by pointer), ink `fg`, background `bg`,
    /// the cursor at (0, 0).
    explicit Pen(S& surface, Color fg = Color{1}, Color bg = Color{})
        : s_(&surface), fg_(fg), bg_(bg) {}

    void set_color(Color c) { fg_ = c; }
    void set_bg(Color c) { bg_ = c; }
    Color color() const { return fg_; }
    Color bg() const { return bg_; }

    void move_to(Coord x, Coord y) {
        x_ = x;
        y_ = y;
    }

    Coord x() const { return x_; }
    Coord y() const { return y_; }

    /// Draws from the current point to (x, y) and ends there. This and
    /// the text verbs are the whole of what the cursor is for.
    void line_to(Coord x, Coord y) {
        line(*s_, x_, y_, x, y, fg_);
        x_ = x;
        y_ = y;
    }

    /// The whole surface in the background colour.
    void clear() { brio::clear(*s_, bg_); }

    /*
     * The shapes take their own coordinates and leave the cursor alone.
     *
     * The cursor exists for work that is sequential - a polyline, a run
     * of text - where each step begins where the last one ended and
     * repeating the coordinate would be noise. Nothing about a rectangle
     * or a circle is sequential: they are placed, not continued. Letting
     * them read the cursor would also force a silent choice between an
     * origin and a centre, which are not the same point, and a reader
     * would have to remember which verb meant which.
     *
     * So here the pen supplies the colours and nothing else.
     */
    void set_pixel(Coord x, Coord y) { brio::set_pixel(*s_, x, y, fg_); }

    void rect(Coord x, Coord y, Extent w, Extent h) {
        brio::rect(*s_, x, y, w, h, fg_);
    }
    void fill_rect(Coord x, Coord y, Extent w, Extent h) {
        brio::fill_rect(*s_, x, y, w, h, fg_);
    }
    void round_rect(Coord x, Coord y, Extent w, Extent h, Extent r) {
        brio::round_rect(*s_, x, y, w, h, r, fg_);
    }
    void fill_round_rect(Coord x, Coord y, Extent w, Extent h, Extent r) {
        brio::fill_round_rect(*s_, x, y, w, h, r, fg_);
    }
    void circle(Coord cx, Coord cy, Extent r) {
        brio::circle(*s_, cx, cy, r, fg_);
    }
    void fill_circle(Coord cx, Coord cy, Extent r) {
        brio::fill_circle(*s_, cx, cy, r, fg_);
    }

    /// Text in the pen's two colours at the cursor (the top-left of the
    /// first cell), the cursor left after the last cell so successive
    /// calls run on.
    template <typename F>
        requires Font<F>
    void text(std::string_view str) {
        x_ = brio::text<F>(*s_, x_, y_, str, fg_, bg_);
    }

    /// A field of `cells` characters at the cursor (gfx/text.hpp's
    /// text_field()), the cursor left after the field.
    template <typename F>
        requires Font<F>
    void text_field(std::string_view str, Extent cells) {
        x_ = brio::text_field<F>(*s_, x_, y_, str, cells, fg_, bg_);
    }

    /// Down one line of the given font, back to the starting column.
    template <typename F>
        requires Font<F>
    void new_line(Coord left) {
        x_ = left;
        y_ = static_cast<Coord>(static_cast<int32_t>(y_) + static_cast<int32_t>(F::cell_h));
    }

private:
    S* s_;
    Color fg_;
    Color bg_;
    Coord x_{};
    Coord y_{};
};

} // namespace brio
