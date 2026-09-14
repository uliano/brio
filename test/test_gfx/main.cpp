// Host tests for the graphics primitives (gfx/surface.hpp, gfx/draw.hpp).
// Every shape is drawn twice - once by the primitive, once by the
// reference renderer that computes the same set of pixels from the
// definition (host/gfx_reference.hpp) - and compared pixel by pixel.
// Run with: ctest --preset host (or ctest --preset host -R test_gfx)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <array>
#include <string>
#include <vector>

#include "gfx/draw.hpp"
#include "gfx/surface.hpp"
#include "host/gfx_reference.hpp"

using brio::clip;
using brio::Coord;
using brio::Extent;
using brio::Framebuffer;
using brio::GfxDiff;
using brio::Indexed8;
using brio::Mono;
using brio::Rect;
using brio::RefCanvas;
using brio::Viewport;

namespace {

// The two shapes under test, both small enough that an exhaustive sweep
// over every pair of endpoints stays instant.
template <Extent W, Extent H>
using MonoFb = Framebuffer<Mono, W, H>;

// A framebuffer and its storage in one object, so a test case can make
// one on the stack without repeating the array declaration.
template <typename Fmt, Extent W, Extent H>
struct Canvas {
    using Fb = Framebuffer<Fmt, W, H>;
    std::array<uint8_t, Fb::bytes> bits{};
    Fb fb{bits};

    void wipe() {
        bits.fill(0);
    }
};

} // namespace

TEST_CASE("the concepts are satisfied by what must satisfy them") {
    using Fb = MonoFb<16, 8>;
    static_assert(brio::Surface<Fb>);
    static_assert(brio::ReadableSurface<Fb>);
    static_assert(brio::Surface<Framebuffer<Indexed8, 16, 8>>);
    static_assert(brio::ReadableSurface<Framebuffer<Indexed8, 16, 8>>);

    // A window onto a surface IS a surface, which is what lets a
    // primitive draw into it knowing nothing of what lies beneath.
    static_assert(brio::Surface<Viewport<Fb>>);

    // And it is deliberately NOT readable: a viewport is a place to
    // draw, not a place to ask questions.
    static_assert(!brio::ReadableSurface<Viewport<Fb>>);
}

TEST_CASE("Mono packs row-major with the most significant bit leftmost") {
    CHECK(Mono::stride_for(1) == 1);
    CHECK(Mono::stride_for(8) == 1);
    CHECK(Mono::stride_for(9) == 2);
    CHECK(Mono::stride_for(128) == 16);

    uint8_t row[4] = {0, 0, 0, 0};
    Mono::put(row, 0, 1);
    CHECK(row[0] == 0x80); // pixel 0 is the most significant bit
    Mono::put(row, 7, 1);
    CHECK(row[0] == 0x81);
    Mono::put(row, 8, 1);
    CHECK(row[1] == 0x80);
    Mono::put(row, 0, 0);
    CHECK(row[0] == 0x01);

    for (Extent x = 0; x < 32; ++x) {
        CHECK(Mono::get(row, x) == ((x == 7 || x == 8) ? 1 : 0));
    }
}

TEST_CASE("Mono fill lights exactly the asked run, at every alignment") {
    // The head/body/tail path is where a bit-packed fill goes wrong, so
    // every start and every length on a four-byte row is tried.
    for (Extent x = 0; x < 32; ++x) {
        for (Extent n = 1; x + n <= 32; ++n) {
            uint8_t row[4] = {0, 0, 0, 0};
            Mono::fill(row, x, n, 1);
            for (Extent i = 0; i < 32; ++i) {
                const bool want = (i >= x && i < Extent(x + n));
                REQUIRE(Mono::get(row, i) == (want ? 1 : 0));
            }
            // And clearing the same run puts it back exactly.
            Mono::fill(row, x, n, 0);
            for (Extent i = 0; i < 32; ++i) {
                REQUIRE(Mono::get(row, i) == 0);
            }
        }
    }
}

TEST_CASE("Mono fill of zero pixels touches nothing") {
    uint8_t row[2] = {0xAA, 0x55};
    Mono::fill(row, 3, 0, 1);
    CHECK(row[0] == 0xAA);
    CHECK(row[1] == 0x55);
}

TEST_CASE("Indexed8 is one byte a pixel") {
    CHECK(Indexed8::stride_for(37) == 37);
    uint8_t row[4] = {0, 0, 0, 0};
    Indexed8::put(row, 2, 0xC3);
    CHECK(Indexed8::get(row, 2) == 0xC3);
    Indexed8::fill(row, 1, 2, 0x7F);
    CHECK(row[0] == 0x00);
    CHECK(row[1] == 0x7F);
    CHECK(row[2] == 0x7F);
    CHECK(row[3] == 0x00);
}

TEST_CASE("clip keeps what is inside and nothing else") {
    SUBCASE("wholly inside is unchanged") {
        const auto r = clip(Rect{2, 3, 4, 5}, 20, 20);
        REQUIRE(r.has_value());
        CHECK(r->x == 2);
        CHECK(r->y == 3);
        CHECK(r->w == 4);
        CHECK(r->h == 5);
    }
    SUBCASE("off the left and top is trimmed, not moved") {
        const auto r = clip(Rect{-3, -4, 10, 10}, 20, 20);
        REQUIRE(r.has_value());
        CHECK(r->x == 0);
        CHECK(r->y == 0);
        CHECK(r->w == 7);
        CHECK(r->h == 6);
    }
    SUBCASE("off the right and bottom is trimmed") {
        const auto r = clip(Rect{15, 16, 10, 10}, 20, 20);
        REQUIRE(r.has_value());
        CHECK(r->w == 5);
        CHECK(r->h == 4);
    }
    SUBCASE("wholly outside is nothing, on every side") {
        CHECK(!clip(Rect{-10, 0, 5, 5}, 20, 20).has_value());
        CHECK(!clip(Rect{20, 0, 5, 5}, 20, 20).has_value());
        CHECK(!clip(Rect{0, -10, 5, 5}, 20, 20).has_value());
        CHECK(!clip(Rect{0, 20, 5, 5}, 20, 20).has_value());
    }
    SUBCASE("a zero extent is nothing, with no special case") {
        CHECK(!clip(Rect{5, 5, 0, 9}, 20, 20).has_value());
        CHECK(!clip(Rect{5, 5, 9, 0}, 20, 20).has_value());
    }
    SUBCASE("the far edge does not overflow a coordinate") {
        // x + w exceeds what an int16_t holds; the arithmetic names
        // int32_t so the rectangle is trimmed and not wrapped.
        const auto r = clip(Rect{32000, 0, 60000, 4}, 100, 100);
        CHECK(!r.has_value());
        const auto r2 = clip(Rect{-32000, 0, 64000, 4}, 100, 100);
        REQUIRE(r2.has_value());
        CHECK(r2->x == 0);
        CHECK(r2->w == 100);
    }
}

// ---------------------------------------------------------------------
// The primitives against the reference renderer.
// ---------------------------------------------------------------------

TEST_CASE("fill_rect matches the definition, everywhere and half off") {
    constexpr Extent W = 12;
    constexpr Extent H = 9;
    Canvas<Mono, W, H> c;

    for (Coord x = -4; x <= Coord(W + 2); ++x) {
        for (Coord y = -4; y <= Coord(H + 2); ++y) {
            for (Extent w = 0; w <= 6; ++w) {
                for (Extent h = 0; h <= 6; ++h) {
                    c.wipe();
                    brio::fill_rect(c.fb, x, y, w, h, 1);

                    RefCanvas ref(W, H);
                    ref.fill_rect(x, y, w, h, 1);

                    const GfxDiff d = brio::compare(c.fb, ref);
                    INFO("rect x=" << x << " y=" << y << " w=" << w << " h=" << h
                                   << "\n"
                                   << d.map);
                    REQUIRE(d.agree());
                }
            }
        }
    }
}

TEST_CASE("the rectangle outline matches the definition") {
    constexpr Extent W = 12;
    constexpr Extent H = 9;
    Canvas<Mono, W, H> c;

    for (Coord x = -3; x <= Coord(W + 1); ++x) {
        for (Coord y = -3; y <= Coord(H + 1); ++y) {
            for (Extent w = 0; w <= 8; ++w) {
                for (Extent h = 0; h <= 8; ++h) {
                    c.wipe();
                    brio::rect(c.fb, x, y, w, h, 1);

                    RefCanvas ref(W, H);
                    ref.rect(x, y, w, h, 1);

                    const GfxDiff d = brio::compare(c.fb, ref);
                    INFO("outline x=" << x << " y=" << y << " w=" << w
                                      << " h=" << h << "\n"
                                      << d.map);
                    REQUIRE(d.agree());
                }
            }
        }
    }
}

TEST_CASE("horizontal and vertical runs match the definition") {
    constexpr Extent W = 10;
    constexpr Extent H = 10;
    Canvas<Mono, W, H> c;

    for (Coord x = -3; x <= Coord(W + 1); ++x) {
        for (Coord y = -3; y <= Coord(H + 1); ++y) {
            for (Extent n = 0; n <= 12; ++n) {
                c.wipe();
                brio::hline(c.fb, x, y, n, 1);
                RefCanvas rh(W, H);
                rh.hline(x, y, n, 1);
                REQUIRE(brio::compare(c.fb, rh).agree());

                c.wipe();
                brio::vline(c.fb, x, y, n, 1);
                RefCanvas rv(W, H);
                rv.vline(x, y, n, 1);
                REQUIRE(brio::compare(c.fb, rv).agree());
            }
        }
    }
}

TEST_CASE("every segment on a small canvas matches the definition") {
    // Exhaustive over EVERY pair of endpoints, including endpoints that
    // lie outside the surface, because clipping and stepping have to
    // agree there too.
    constexpr Extent W = 9;
    constexpr Extent H = 9;
    Canvas<Mono, W, H> c;

    uint32_t compared = 0;
    for (Coord x0 = -3; x0 <= Coord(W + 2); ++x0) {
        for (Coord y0 = -3; y0 <= Coord(H + 2); ++y0) {
            for (Coord x1 = -3; x1 <= Coord(W + 2); ++x1) {
                for (Coord y1 = -3; y1 <= Coord(H + 2); ++y1) {
                    c.wipe();
                    brio::line(c.fb, x0, y0, x1, y1, 1);

                    RefCanvas ref(W, H);
                    ref.line(x0, y0, x1, y1, 1);

                    const GfxDiff d = brio::compare(c.fb, ref);
                    INFO("line (" << x0 << "," << y0 << ") -> (" << x1 << ","
                                  << y1 << ")\n"
                                  << d.map);
                    REQUIRE(d.agree());
                    ++compared;
                }
            }
        }
    }
    MESSAGE("segments compared: " << compared);
}

TEST_CASE("a segment is one pixel thick and as long as its major axis") {
    constexpr Extent W = 32;
    constexpr Extent H = 32;
    Canvas<Mono, W, H> c;

    for (Coord x1 = 0; x1 < Coord(W); ++x1) {
        for (Coord y1 = 0; y1 < Coord(H); ++y1) {
            c.wipe();
            brio::line(c.fb, 0, 0, x1, y1, 1);
            uint32_t lit = 0;
            for (Extent y = 0; y < H; ++y) {
                for (Extent x = 0; x < W; ++x) {
                    lit += c.fb.get_pixel(Coord(x), Coord(y)) ? 1u : 0u;
                }
            }
            const int32_t major = x1 > y1 ? x1 : y1;
            INFO("to (" << x1 << "," << y1 << ")");
            REQUIRE(lit == uint32_t(major + 1));
        }
    }
}

// ---------------------------------------------------------------------
// The surface verbs themselves.
// ---------------------------------------------------------------------

TEST_CASE("write_run places pixels and trims what falls outside") {
    Canvas<Indexed8, 8, 3> c;
    const std::array<uint8_t, 4> run{1, 2, 3, 4};

    SUBCASE("wholly inside") {
        c.fb.write_run(2, 1, run);
        CHECK(c.fb.get_pixel(1, 1) == 0);
        CHECK(c.fb.get_pixel(2, 1) == 1);
        CHECK(c.fb.get_pixel(5, 1) == 4);
        CHECK(c.fb.get_pixel(6, 1) == 0);
    }
    SUBCASE("off the left: the head of the run is dropped, not shifted") {
        c.fb.write_run(-2, 0, run);
        CHECK(c.fb.get_pixel(0, 0) == 3);
        CHECK(c.fb.get_pixel(1, 0) == 4);
        CHECK(c.fb.get_pixel(2, 0) == 0);
    }
    SUBCASE("off the right: the tail is dropped") {
        c.fb.write_run(6, 2, run);
        CHECK(c.fb.get_pixel(6, 2) == 1);
        CHECK(c.fb.get_pixel(7, 2) == 2);
    }
    SUBCASE("off the surface entirely writes nothing") {
        c.fb.write_run(-9, 0, run);
        c.fb.write_run(9, 0, run);
        c.fb.write_run(0, -1, run);
        c.fb.write_run(0, 3, run);
        for (Extent y = 0; y < 3; ++y) {
            for (Extent x = 0; x < 8; ++x) {
                REQUIRE(c.fb.get_pixel(Coord(x), Coord(y)) == 0);
            }
        }
    }
}

TEST_CASE("clear paints the whole surface and nothing beyond it") {
    Canvas<Mono, 13, 5> c;
    brio::clear(c.fb, 1);
    for (Extent y = 0; y < 5; ++y) {
        for (Extent x = 0; x < 13; ++x) {
            REQUIRE(c.fb.get_pixel(Coord(x), Coord(y)) == 1);
        }
    }
    // The padding bits of the last byte of a row are not the surface's
    // and a reader must never see them as pixels.
    CHECK(c.fb.bits().size() == size_t(2) * 5);
}

// ---------------------------------------------------------------------
// The viewport.
// ---------------------------------------------------------------------

TEST_CASE("a viewport translates and cannot be escaped") {
    Canvas<Indexed8, 16, 10> c;
    Viewport<Canvas<Indexed8, 16, 10>::Fb> v(c.fb, 4, 3, 6, 4);

    CHECK(v.width() == 6);
    CHECK(v.height() == 4);

    SUBCASE("the origin is the viewport's, not the surface's") {
        brio::set_pixel(v, 0, 0, 9);
        CHECK(c.fb.get_pixel(4, 3) == 9);
        CHECK(c.fb.get_pixel(0, 0) == 0);
    }

    SUBCASE("a shape larger than the window is trimmed to it") {
        brio::fill_rect(v, -5, -5, 100, 100, 7);
        for (Extent y = 0; y < 10; ++y) {
            for (Extent x = 0; x < 16; ++x) {
                const bool in = (x >= 4 && x < 10 && y >= 3 && y < 7);
                INFO("x=" << x << " y=" << y);
                REQUIRE(c.fb.get_pixel(Coord(x), Coord(y)) == (in ? 7 : 0));
            }
        }
    }

    SUBCASE("a run is trimmed at both ends of the window") {
        const std::array<uint8_t, 10> run{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        v.write_run(-2, 1, run);
        // Window columns 0..5 take run elements 2..7.
        CHECK(c.fb.get_pixel(3, 4) == 0);
        CHECK(c.fb.get_pixel(4, 4) == 3);
        CHECK(c.fb.get_pixel(9, 4) == 8);
        CHECK(c.fb.get_pixel(10, 4) == 0);
    }

    SUBCASE("a viewport onto a viewport composes") {
        Viewport<Viewport<Canvas<Indexed8, 16, 10>::Fb>> inner(v, 1, 1, 2, 2);
        brio::clear(inner, 5);
        for (Extent y = 0; y < 10; ++y) {
            for (Extent x = 0; x < 16; ++x) {
                const bool in = (x >= 5 && x < 7 && y >= 4 && y < 6);
                INFO("x=" << x << " y=" << y);
                REQUIRE(c.fb.get_pixel(Coord(x), Coord(y)) == (in ? 5 : 0));
            }
        }
    }
}

TEST_CASE("a primitive drawn through a viewport matches the definition") {
    // The same segment drawn into a window and into the reference's own
    // translated coordinates: a viewport must be a pure change of
    // origin, not a different renderer.
    constexpr Extent W = 20;
    constexpr Extent H = 14;
    Canvas<Mono, W, H> c;
    Viewport<Canvas<Mono, W, H>::Fb> v(c.fb, 3, 2, 10, 8);

    for (Coord x1 = -2; x1 <= 12; ++x1) {
        for (Coord y1 = -2; y1 <= 10; ++y1) {
            c.wipe();
            brio::line(v, 0, 0, x1, y1, 1);

            RefCanvas ref(W, H);
            ref.line(3, 2, Coord(x1 + 3), Coord(y1 + 2), 1);
            // The reference draws on the whole surface; the viewport
            // trims to its window, so compare only inside the window.
            uint32_t disagreements = 0;
            for (Extent y = 0; y < H; ++y) {
                for (Extent x = 0; x < W; ++x) {
                    const bool inside =
                        (x >= 3 && x < 13 && y >= 2 && y < 10);
                    const bool a = c.fb.get_pixel(Coord(x), Coord(y)) != 0;
                    const bool r = inside && ref.at(Coord(x), Coord(y)) != 0;
                    if (a != r) {
                        ++disagreements;
                    }
                }
            }
            INFO("segment to (" << x1 << "," << y1 << ")");
            REQUIRE(disagreements == 0);
        }
    }
}
