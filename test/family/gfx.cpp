// Family smoke TU: the graphics primitives on an 8-bit target.
//
// gfx/ is target-independent - it includes nothing of any stratum - so
// what this TU proves is not package variability but the one axis that
// really differs here: on this family `int` is SIXTEEN BITS, so every
// place where a coordinate meets an extent (a clip, a far edge, a run
// longer than the surface) has to name its own width or wrap silently.
// It compiles for every package because the fixture asks that of every
// TU, and because a surface costs only RAM.

#include <stdint.h>

#include <array>

#include "gfx/draw.hpp"
#include "gfx/font_5x7.hpp"
#include "gfx/pen.hpp"
#include "gfx/text.hpp"

using namespace brio;

namespace {

// A mono panel of the size the cheap I2C parts carry: 1 KB of RAM.
using Panel = Framebuffer<Mono, 128, 64>;
std::array<uint8_t, Panel::bytes> panel_bits;
Panel panel{panel_bits};

// And a small indexed surface, to instantiate the other format.
using Indexed = Framebuffer<Indexed8, 64, 32>;
std::array<uint8_t, Indexed::bytes> indexed_bits;
Indexed indexed{indexed_bits};

static_assert(Surface<Panel>);
static_assert(ReadableSurface<Panel>);
static_assert(Surface<Indexed>);
static_assert(Surface<Viewport<Panel>>);
static_assert(Surface<Viewport<Viewport<Panel>>>);

// The base must NOT offer a read: a primitive written against it has to
// stay correct on a panel that answers nothing.
static_assert(!ReadableSurface<Viewport<Panel>>);

} // namespace

void gfx_verbs() {
    clear(panel, 0);
    set_pixel(panel, 3, 4, 1);
    fill_rect(panel, 10, 10, 40, 20, 1);
    rect(panel, 0, 0, 128, 64, 1);
    hline(panel, -5, 32, 200, 1);
    vline(panel, 64, -5, 200, 1);
    line(panel, 0, 0, 127, 63, 1);

    circle(panel, 64, 32, 30, 1);
    fill_circle(panel, 64, 32, 20, 1);
    round_rect(panel, 2, 2, 124, 60, 12, 1);
    fill_round_rect(panel, 20, 20, 40, 24, 200, 1); // radius past the clamp

    // Coordinates far outside a 16-bit sum, which must clip and not wrap.
    fill_rect(panel, -30000, -30000, 60000, 60000, 1);
    line(panel, -20000, -20000, 20000, 20000, 1);
    circle(panel, -300, -300, 600, 1);

    // The square root the filled round shapes lean on, at the largest
    // argument it can be given. The literals carry their width: on this
    // family `unsigned int` is SIXTEEN BITS, so 65535u * 65535u is 1.
    static_assert(isqrt(65535UL * 65535UL) == 65535u);
    static_assert(isqrt(0u) == 0u);

    // Text: the font's table is the largest datum gfx puts in an image,
    // so it is compiled here as well as instantiated.
    text<Font5x7>(panel, 2, 2, "brio", 1, 0);
    text_field<Font5x7>(panel, 2, 12, "3.30", 8, 1, 0);
    text<Font5x7>(panel, -20, 40, "clipped at both ends of the panel", 1, 0);

    Pen<Panel> pen(panel, 1, 0);
    pen.move_to(4, 30);
    pen.line_to(60, 44);
    pen.circle(6);
    pen.move_to(4, 50);
    pen.text<Font5x7>("V=");
    pen.text_field<Font5x7>("12.5", 6);
    pen.new_line<Font5x7>(4);

    Viewport<Panel> window(panel, 8, 8, 48, 24);
    clear(window, 0);
    rect(window, 0, 0, 48, 24, 1);
    line(window, -10, -10, 100, 100, 1);

    const std::array<Panel::Color, 8> run{1, 0, 1, 0, 1, 0, 1, 0};
    panel.write_run(-2, 5, run);
    window.write_run(44, 5, run);

    clear(indexed, 0x20);
    fill_rect(indexed, 4, 4, 8, 8, 0xC0);
    line(indexed, 0, 31, 63, 0, 0xFF);
    (void)panel.get_pixel(0, 0);
    (void)indexed.get_pixel(0, 0);
}
