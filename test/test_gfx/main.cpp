// Host tests for the graphics primitives (gfx/surface.hpp, gfx/draw.hpp).
// Every shape is drawn twice - once by the primitive, once by the
// reference renderer that computes the same set of pixels from the
// definition (host/gfx_reference.hpp) - and compared pixel by pixel.
// Run with: ctest --preset host (or ctest --preset host -R test_gfx)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <fstream>
#include <iterator>
#include <string>
#include <type_traits>
#include <vector>

#include "gfx/counting.hpp"
#include "gfx/draw.hpp"
#include "gfx/font_5x7.hpp"
#include "gfx/pen.hpp"
#include "gfx/surface.hpp"
#include "gfx/text.hpp"
#include "host/gfx_reference.hpp"
#include "host/sim_display.hpp"

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
                const bool want = (i >= x && i < static_cast<Extent>(x + n));
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

    for (Coord x = -4; x <= static_cast<Coord>(W + 2); ++x) {
        for (Coord y = -4; y <= static_cast<Coord>(H + 2); ++y) {
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

    for (Coord x = -3; x <= static_cast<Coord>(W + 1); ++x) {
        for (Coord y = -3; y <= static_cast<Coord>(H + 1); ++y) {
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

    for (Coord x = -3; x <= static_cast<Coord>(W + 1); ++x) {
        for (Coord y = -3; y <= static_cast<Coord>(H + 1); ++y) {
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

TEST_CASE("the thick outline matches the definition, and is the thin one at t = 1") {
    constexpr Extent W = 12;
    constexpr Extent H = 9;
    Canvas<Mono, W, H> c;
    Canvas<Mono, W, H> thin;

    for (Coord x = -3; x <= static_cast<Coord>(W + 1); ++x) {
        for (Coord y = -3; y <= static_cast<Coord>(H + 1); ++y) {
            for (Extent w = 0; w <= 8; ++w) {
                for (Extent h = 0; h <= 8; ++h) {
                    for (Extent t = 0; t <= 5; ++t) {
                        c.wipe();
                        brio::rect(c.fb, x, y, w, h, t, 1);

                        RefCanvas ref(W, H);
                        ref.rect(x, y, w, h, t, 1);

                        const GfxDiff d = brio::compare(c.fb, ref);
                        INFO("thick outline x=" << x << " y=" << y << " w=" << w
                                                << " h=" << h << " t=" << t << "\n"
                                                << d.map);
                        REQUIRE(d.agree());

                        // The thin outline is the case t = 1, pixel for pixel.
                        if (t == 1) {
                            thin.wipe();
                            brio::rect(thin.fb, x, y, w, h, 1);
                            REQUIRE(c.bits == thin.bits);
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("thick runs match the definition") {
    constexpr Extent W = 10;
    constexpr Extent H = 10;
    Canvas<Mono, W, H> c;

    for (Coord x = -3; x <= static_cast<Coord>(W + 1); ++x) {
        for (Coord y = -3; y <= static_cast<Coord>(H + 1); ++y) {
            for (Extent n = 0; n <= 12; ++n) {
                for (Extent t = 0; t <= 4; ++t) {
                    c.wipe();
                    brio::hline(c.fb, x, y, n, t, 1);
                    RefCanvas rh(W, H);
                    rh.hline(x, y, n, t, 1);
                    REQUIRE(brio::compare(c.fb, rh).agree());

                    c.wipe();
                    brio::vline(c.fb, x, y, n, t, 1);
                    RefCanvas rv(W, H);
                    rv.vline(x, y, n, t, 1);
                    REQUIRE(brio::compare(c.fb, rv).agree());
                }
            }
        }
    }
}

TEST_CASE("a thick outline costs four windows, or one when nothing is inside") {
    Canvas<Mono, 40, 30> c;
    brio::Counting<Canvas<Mono, 40, 30>::Fb> counted(c.fb);

    brio::rect(counted, 2, 2, 20, 10, 3, 1);
    CHECK(counted.take().rects == 4);
    brio::rect(counted, 2, 2, 20, 6, 3, 1); // twice the thickness reaches the height
    CHECK(counted.take().rects == 1);
    brio::rect(counted, 2, 2, 20, 10, 0, 1);
    CHECK(counted.take().rects == 0);
    brio::hline(counted, 0, 0, 10, 2, 1);
    CHECK(counted.take().rects == 1);
}

TEST_CASE("every segment on a small canvas matches the definition") {
    // Exhaustive over EVERY pair of endpoints, including endpoints that
    // lie outside the surface, because clipping and stepping have to
    // agree there too.
    constexpr Extent W = 9;
    constexpr Extent H = 9;
    Canvas<Mono, W, H> c;

    uint32_t compared = 0;
    for (Coord x0 = -3; x0 <= static_cast<Coord>(W + 2); ++x0) {
        for (Coord y0 = -3; y0 <= static_cast<Coord>(H + 2); ++y0) {
            for (Coord x1 = -3; x1 <= static_cast<Coord>(W + 2); ++x1) {
                for (Coord y1 = -3; y1 <= static_cast<Coord>(H + 2); ++y1) {
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

    for (Coord x1 = 0; x1 < static_cast<Coord>(W); ++x1) {
        for (Coord y1 = 0; y1 < static_cast<Coord>(H); ++y1) {
            c.wipe();
            brio::line(c.fb, 0, 0, x1, y1, 1);
            uint32_t lit = 0;
            for (Extent y = 0; y < H; ++y) {
                for (Extent x = 0; x < W; ++x) {
                    lit += c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) ? 1u : 0u;
                }
            }
            const int32_t major = x1 > y1 ? x1 : y1;
            INFO("to (" << x1 << "," << y1 << ")");
            REQUIRE(lit == static_cast<uint32_t>(major + 1));
        }
    }
}

TEST_CASE("isqrt is exact where a primitive uses it") {
    for (uint32_t n = 0; n < 4096; ++n) {
        const uint32_t r = brio::isqrt(n);
        REQUIRE(r * r <= n);
        REQUIRE((r + 1) * (r + 1) > n);
    }
    // The largest argument isqrt can be given. The literals carry their
    // width on purpose: where `unsigned int` is sixteen bits - it is, on
    // the AVR strata - 65535u * 65535u is 1, and this reads as passing
    // while testing nothing.
    CHECK(brio::isqrt(65535UL * 65535UL) == 65535u);
}

TEST_CASE("the circle's ring matches the definition at every radius") {
    constexpr Extent W = 45;
    constexpr Extent H = 45;
    Canvas<Mono, W, H> c;

    for (Extent r = 0; r <= 22; ++r) {
        c.wipe();
        brio::circle(c.fb, 22, 22, r, 1);

        RefCanvas ref(W, H);
        ref.circle(22, 22, r, 1);

        const GfxDiff d = brio::compare(c.fb, ref);
        INFO("radius " << r << "\n" << d.map);
        REQUIRE(d.agree());
    }
}

TEST_CASE("a circle half off the surface is trimmed, not deformed") {
    constexpr Extent W = 16;
    constexpr Extent H = 16;
    Canvas<Mono, W, H> c;

    for (Coord cx = -6; cx <= static_cast<Coord>(W + 5); cx += 3) {
        for (Coord cy = -6; cy <= static_cast<Coord>(H + 5); cy += 3) {
            for (Extent r = 0; r <= 9; ++r) {
                c.wipe();
                brio::circle(c.fb, cx, cy, r, 1);

                RefCanvas ref(W, H);
                ref.circle(cx, cy, r, 1);

                const GfxDiff d = brio::compare(c.fb, ref);
                INFO("centre (" << cx << "," << cy << ") r=" << r << "\n"
                                << d.map);
                REQUIRE(d.agree());
            }
        }
    }
}

TEST_CASE("the disc matches the inequality that defines it") {
    constexpr Extent W = 45;
    constexpr Extent H = 45;
    Canvas<Mono, W, H> c;

    for (Extent r = 0; r <= 22; ++r) {
        c.wipe();
        brio::fill_circle(c.fb, 22, 22, r, 1);

        RefCanvas ref(W, H);
        ref.fill_circle(22, 22, r, 1);

        const GfxDiff d = brio::compare(c.fb, ref);
        INFO("radius " << r << "\n" << d.map);
        REQUIRE(d.agree());
    }
}

TEST_CASE("a disc half off the surface is trimmed, not deformed") {
    constexpr Extent W = 16;
    constexpr Extent H = 16;
    Canvas<Mono, W, H> c;

    for (Coord cx = -6; cx <= static_cast<Coord>(W + 5); cx += 3) {
        for (Coord cy = -6; cy <= static_cast<Coord>(H + 5); cy += 3) {
            for (Extent r = 0; r <= 9; ++r) {
                c.wipe();
                brio::fill_circle(c.fb, cx, cy, r, 1);

                RefCanvas ref(W, H);
                ref.fill_circle(cx, cy, r, 1);

                INFO("centre (" << cx << "," << cy << ") r=" << r);
                REQUIRE(brio::compare(c.fb, ref).agree());
            }
        }
    }
}

TEST_CASE("the rounded rectangle's outline matches the definition") {
    constexpr Extent W = 24;
    constexpr Extent H = 24;
    Canvas<Mono, W, H> c;

    for (Extent w = 1; w <= 18; ++w) {
        for (Extent h = 1; h <= 18; ++h) {
            for (Extent r = 0; r <= 10; ++r) {
                c.wipe();
                brio::round_rect(c.fb, 3, 3, w, h, r, 1);

                RefCanvas ref(W, H);
                ref.round_rect(3, 3, w, h, r, 1);

                const GfxDiff d = brio::compare(c.fb, ref);
                INFO("w=" << w << " h=" << h << " r=" << r << "\n" << d.map);
                REQUIRE(d.agree());
            }
        }
    }
}

TEST_CASE("the filled rounded rectangle matches the definition") {
    constexpr Extent W = 24;
    constexpr Extent H = 24;
    Canvas<Mono, W, H> c;

    for (Extent w = 1; w <= 18; ++w) {
        for (Extent h = 1; h <= 18; ++h) {
            for (Extent r = 0; r <= 10; ++r) {
                c.wipe();
                brio::fill_round_rect(c.fb, 3, 3, w, h, r, 1);

                RefCanvas ref(W, H);
                ref.fill_round_rect(3, 3, w, h, r, 1);

                const GfxDiff d = brio::compare(c.fb, ref);
                INFO("w=" << w << " h=" << h << " r=" << r << "\n" << d.map);
                REQUIRE(d.agree());
            }
        }
    }
}

TEST_CASE("a rounded rectangle clipped at the surface edge still agrees") {
    constexpr Extent W = 14;
    constexpr Extent H = 14;
    Canvas<Mono, W, H> c;

    for (Coord x = -5; x <= static_cast<Coord>(W + 2); x += 2) {
        for (Coord y = -5; y <= static_cast<Coord>(H + 2); y += 2) {
            for (Extent e = 1; e <= 14; e += 2) {
                for (Extent r = 0; r <= 6; ++r) {
                    c.wipe();
                    brio::round_rect(c.fb, x, y, e, e, r, 1);
                    RefCanvas ro(W, H);
                    ro.round_rect(x, y, e, e, r, 1);
                    INFO("outline at (" << x << "," << y << ") e=" << e
                                        << " r=" << r);
                    REQUIRE(brio::compare(c.fb, ro).agree());

                    c.wipe();
                    brio::fill_round_rect(c.fb, x, y, e, e, r, 1);
                    RefCanvas rf(W, H);
                    rf.fill_round_rect(x, y, e, e, r, 1);
                    INFO("fill at (" << x << "," << y << ") e=" << e
                                     << " r=" << r);
                    REQUIRE(brio::compare(c.fb, rf).agree());
                }
            }
        }
    }
}

TEST_CASE("the radius clamp makes a square rounded rectangle a circle") {
    constexpr Extent W = 21;
    constexpr Extent H = 21;
    Canvas<Mono, W, H> rounded;
    Canvas<Mono, W, H> round_one;

    for (Extent side = 1; side <= 21; side += 2) { // odd: a true centre
        const Extent r = static_cast<Extent>((side - 1) / 2);
        rounded.wipe();
        round_one.wipe();
        // Asking for a radius far beyond the clamp must give the same
        // shape as asking for exactly the clamp.
        brio::round_rect(rounded.fb, 0, 0, side, side, 200, 1);
        brio::circle(round_one.fb, static_cast<Coord>(r),
                     static_cast<Coord>(r), r, 1);

        for (Extent y = 0; y < H; ++y) {
            for (Extent x = 0; x < W; ++x) {
                INFO("side=" << side << " at (" << x << "," << y << ")");
                REQUIRE(rounded.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) ==
                        round_one.fb.get_pixel(static_cast<Coord>(x),
                                               static_cast<Coord>(y)));
            }
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
                REQUIRE(c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) == 0);
            }
        }
    }
}

TEST_CASE("clear paints the whole surface and nothing beyond it") {
    Canvas<Mono, 13, 5> c;
    brio::clear(c.fb, 1);
    for (Extent y = 0; y < 5; ++y) {
        for (Extent x = 0; x < 13; ++x) {
            REQUIRE(c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) == 1);
        }
    }
    // The padding bits of the last byte of a row are not the surface's
    // and a reader must never see them as pixels.
    CHECK(c.fb.bits().size() == static_cast<size_t>(2) * 5);
}

// ---------------------------------------------------------------------
// Text: the geometry against the reference, the glyphs against a golden.
// ---------------------------------------------------------------------

TEST_CASE("the font answers for everything it is asked") {
    static_assert(brio::Font<brio::Font5x7>);

    // Past the glyph is the line gap, on every character.
    for (int ch = 0; ch < 256; ++ch) {
        for (Extent row = brio::Font5x7::glyph_h; row < brio::Font5x7::cell_h;
             ++row) {
            REQUIRE(brio::Font5x7::row_bits(static_cast<uint8_t>(ch), row) == 0);
        }
    }

    // The advance column is always background, so cells tile with no
    // seam and an opaque write covers the whole of what stood there.
    for (int ch = 0; ch < 256; ++ch) {
        for (Extent row = 0; row < brio::Font5x7::cell_h; ++row) {
            INFO("ch=" << ch << " row=" << row);
            REQUIRE((brio::Font5x7::row_bits(static_cast<uint8_t>(ch), row) & 1u) == 0);
        }
    }

    // Outside the range every character draws the same visible box.
    const uint8_t box_top = brio::Font5x7::row_bits(0x1F, 0);
    CHECK(box_top == brio::Font5x7::row_bits(0x7F, 0));
    CHECK(box_top == brio::Font5x7::row_bits(0xFF, 0));
    CHECK(box_top != 0);
    // A space is inside the range and draws nothing at all.
    for (Extent row = 0; row < brio::Font5x7::cell_h; ++row) {
        REQUIRE(brio::Font5x7::row_bits(' ', row) == 0);
    }
}

TEST_CASE("a scaled font is the base font with every pixel an n by n block") {
    using brio::Font5x7;
    using Two = brio::Scaled<Font5x7, 2>;
    using Three = brio::Scaled<Font5x7, 3>;
    using Four = brio::Scaled<Font5x7, 4>;
    static_assert(brio::Font<Two> && brio::Font<Three> && brio::Font<Four>);
    // The row widens only where the cell does: six, twelve, eighteen
    // and twenty-four pixels wide.
    static_assert(std::is_same_v<brio::Scaled<Font5x7, 1>::Row, uint8_t>);
    static_assert(std::is_same_v<Two::Row, uint16_t>);
    static_assert(std::is_same_v<Three::Row, uint32_t>);
    static_assert(Two::cell_w == 12 && Two::cell_h == 16);
    static_assert(Four::cell_w == 24 && Four::cell_h == 32);

    // Scale one IS the font.
    for (int ch = 0; ch < 256; ++ch) {
        for (Extent row = 0; row < Font5x7::cell_h; ++row) {
            REQUIRE(brio::Scaled<Font5x7, 1>::row_bits(static_cast<uint8_t>(ch), row) ==
                    Font5x7::row_bits(static_cast<uint8_t>(ch), row));
        }
    }

    // And the DEFINITION of scaling, on rendered text, sharing nothing
    // with the adapter's bit arithmetic: the scaled picture at (px, py)
    // is the plain one at (px / n, py / n).
    const std::string_view word = "Ag7.";
    Canvas<Mono, 4 * 6, 8> plain;
    brio::text<Font5x7>(plain.fb, 0, 0, word, 1, 0);

    auto check = [&](auto font_tag, Extent n) {
        using F = decltype(font_tag);
        Canvas<Mono, 4 * 6 * 4, 8 * 4> big;
        brio::text<F>(big.fb, 0, 0, word, 1, 0);
        for (Extent py = 0; py < F::cell_h; ++py) {
            for (Extent px = 0; px < 4 * F::cell_w; ++px) {
                INFO("n=" << n << " at (" << px << "," << py << ")");
                REQUIRE(big.fb.get_pixel(static_cast<Coord>(px), static_cast<Coord>(py)) ==
                        plain.fb.get_pixel(static_cast<Coord>(px / n),
                                           static_cast<Coord>(py / n)));
            }
        }
    };
    check(Two{}, 2);
    check(Three{}, 3);
    check(Four{}, 4);
}

TEST_CASE("scaled text lands where the reference says") {
    using Big = brio::Scaled<brio::Font5x7, 3>;
    constexpr Extent W = 70;
    constexpr Extent H = 40;
    Canvas<Mono, W, H> c;

    for (Coord x = -20; x <= static_cast<Coord>(W + 2); x += 7) {
        for (Coord y = -13; y <= static_cast<Coord>(H + 1); y += 5) {
            c.wipe();
            brio::text<Big>(c.fb, x, y, "1.5V", 1, 0);

            RefCanvas ref(W, H);
            ref.text<Big>(x, y, "1.5V", 1, 0);

            const GfxDiff d = brio::compare(c.fb, ref);
            INFO("scaled at (" << x << "," << y << ")\n" << d.map);
            REQUIRE(d.agree());
        }
    }
}

TEST_CASE("text lands where the reference says, wherever it is put") {
    constexpr Extent W = 40;
    constexpr Extent H = 20;
    Canvas<Mono, W, H> c;
    const char* words[] = {"", "i", "Hg", "brio", "0123456789"};

    for (const char* w : words) {
        for (Coord x = -8; x <= static_cast<Coord>(W + 2); x += 3) {
            for (Coord y = -5; y <= static_cast<Coord>(H + 1); y += 2) {
                c.wipe();
                brio::text<brio::Font5x7>(c.fb, x, y, w, 1, 0);

                RefCanvas ref(W, H);
                ref.text<brio::Font5x7>(x, y, w, 1, 0);

                const GfxDiff d = brio::compare(c.fb, ref);
                INFO("\"" << w << "\" at (" << x << "," << y << ")\n" << d.map);
                REQUIRE(d.agree());
            }
        }
    }
}

TEST_CASE("text returns where the next cell would start") {
    Canvas<Mono, 60, 10> c;
    const Coord after = brio::text<brio::Font5x7>(c.fb, 4, 1, "abc", 1, 0);
    CHECK(after == static_cast<Coord>(4 + 3 * brio::Font5x7::cell_w));
    // An empty string draws nothing and does not move.
    CHECK(brio::text<brio::Font5x7>(c.fb, 7, 1, "", 1, 0) == 7);
}

TEST_CASE("a field erases what it no longer holds") {
    constexpr Extent W = 60;
    constexpr Extent H = 10;
    Canvas<Mono, W, H> c;
    constexpr Extent cells = 5;

    // A long value, then a short one over it: nothing of the first may
    // survive - the write-only erase discipline, as a verb.
    brio::text_field<brio::Font5x7>(c.fb, 2, 1, "88888", cells, 1, 0);
    brio::text_field<brio::Font5x7>(c.fb, 2, 1, "7", cells, 1, 0);

    Canvas<Mono, W, H> fresh;
    brio::text_field<brio::Font5x7>(fresh.fb, 2, 1, "7", cells, 1, 0);

    for (Extent y = 0; y < H; ++y) {
        for (Extent x = 0; x < W; ++x) {
            INFO("at (" << x << "," << y << ")");
            REQUIRE(c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) ==
                    fresh.fb.get_pixel(static_cast<Coord>(x),
                                       static_cast<Coord>(y)));
        }
    }
}

TEST_CASE("a field is exactly its width, and cuts what will not fit") {
    constexpr Extent W = 60;
    constexpr Extent H = 10;
    Canvas<Mono, W, H> c;
    constexpr Extent cells = 3;

    const Coord after =
        brio::text_field<brio::Font5x7>(c.fb, 2, 1, "abcdef", cells, 1, 0);
    CHECK(after == static_cast<Coord>(2 + cells * brio::Font5x7::cell_w));

    // Nothing past the field's last column was touched.
    for (Extent y = 0; y < H; ++y) {
        for (Extent x = static_cast<Extent>(2 + cells * brio::Font5x7::cell_w); x < W; ++x) {
            INFO("at (" << x << "," << y << ")");
            REQUIRE(c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) == 0);
        }
    }

    // And the first three cells are the first three characters.
    Canvas<Mono, W, H> want;
    brio::text<brio::Font5x7>(want.fb, 2, 1, "abc", 1, 0);
    for (Extent y = 0; y < H; ++y) {
        for (Extent x = 0; x < W; ++x) {
            REQUIRE(c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) ==
                    want.fb.get_pixel(static_cast<Coord>(x),
                                      static_cast<Coord>(y)));
        }
    }
}

TEST_CASE("the glyphs are the ones that were looked at") {
    // What no reference can judge: the SHAPE of a letter. The whole
    // printable set is rendered and frozen; a diff against this file is
    // a human's business to approve, not a test's to explain.
    constexpr Extent W = 16 * brio::Font5x7::cell_w;
    constexpr Extent H = 6 * brio::Font5x7::cell_h;
    Canvas<Mono, W, H> c;
    for (int row = 0; row < 6; ++row) {
        std::string line;
        for (int i = 0; i < 16; ++i) {
            const int ch = 0x20 + row * 16 + i;
            line += (ch <= 0x7E) ? char(ch) : ' ';
        }
        brio::text<brio::Font5x7>(c.fb, 0,
                                  static_cast<Coord>(row * brio::Font5x7::cell_h),
                                  line, 1, 0);
    }

    std::ifstream in(BRIO_SUITE_DIR "/golden/font_5x7.txt");
    REQUIRE(in.good());
    const std::string want((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    const std::string got = brio::ascii(c.fb);
    INFO("rendered:\n" << got);
    CHECK(got == want);
}

// ---------------------------------------------------------------------
// The pen.
// ---------------------------------------------------------------------

TEST_CASE("a pen is the stateless primitives with the arguments remembered") {
    constexpr Extent W = 30;
    constexpr Extent H = 20;
    Canvas<Mono, W, H> pen_side;
    Canvas<Mono, W, H> plain_side;

    brio::Pen<Canvas<Mono, W, H>::Fb> p(pen_side.fb, 1, 0);
    p.move_to(2, 3);
    p.line_to(20, 15);
    p.line_to(25, 2);
    p.circle(10, 10, 4);

    brio::line(plain_side.fb, 2, 3, 20, 15, 1);
    brio::line(plain_side.fb, 20, 15, 25, 2, 1);
    brio::circle(plain_side.fb, 10, 10, 4, 1);

    for (Extent y = 0; y < H; ++y) {
        for (Extent x = 0; x < W; ++x) {
            INFO("at (" << x << "," << y << ")");
            REQUIRE(pen_side.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) ==
                    plain_side.fb.get_pixel(static_cast<Coord>(x),
                                            static_cast<Coord>(y)));
        }
    }
    // A shape does not move the cursor: it is still where the last
    // segment ended.
    CHECK(p.x() == 25);
    CHECK(p.y() == 2);
}

TEST_CASE("a pen carries the text cursor across calls") {
    constexpr Extent W = 60;
    constexpr Extent H = 20;
    Canvas<Mono, W, H> by_pen;
    Canvas<Mono, W, H> by_hand;

    brio::Pen<Canvas<Mono, W, H>::Fb> p(by_pen.fb, 1, 0);
    p.move_to(1, 2);
    p.text<brio::Font5x7>("ab");
    p.text<brio::Font5x7>("cd");

    brio::text<brio::Font5x7>(by_hand.fb, 1, 2, "abcd", 1, 0);

    for (Extent y = 0; y < H; ++y) {
        for (Extent x = 0; x < W; ++x) {
            INFO("at (" << x << "," << y << ")");
            REQUIRE(by_pen.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) ==
                    by_hand.fb.get_pixel(static_cast<Coord>(x),
                                         static_cast<Coord>(y)));
        }
    }
}

// ---------------------------------------------------------------------
// Publishing a framebuffer to another process.
// ---------------------------------------------------------------------

namespace {

// What a viewer does: attach BY NAME - never by a path, which macOS does
// not have - read only, map what the object holds, and take the byte
// count from the header the program wrote: the object's own size is
// rounded up to a page on macOS (host/shared_segment.hpp).
struct Attached {
    uint8_t* base{nullptr};
    size_t bytes{0};   // what the header declares
    size_t mapped{0};  // what the object holds: no less, a page's rounding more on macOS

    explicit Attached(const std::string& name) {
        const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
        if (fd < 0) {
            return;
        }
        struct stat st {};
        if (::fstat(fd, &st) == 0 &&
            st.st_size >= static_cast<off_t>(brio::sim_display_header_bytes)) {
            mapped = static_cast<size_t>(st.st_size);
            void* p = ::mmap(nullptr, mapped, PROT_READ, MAP_SHARED, fd, 0);
            base = (p == MAP_FAILED) ? nullptr : static_cast<uint8_t*>(p);
        }
        ::close(fd);
        if (base != nullptr) {
            bytes = brio::sim_display_bytes(*header());
            if (bytes > mapped) {  // a header of another layout: not ours to read
                ::munmap(base, mapped);
                base = nullptr;
            }
        }
    }
    ~Attached() {
        if (base != nullptr) {
            ::munmap(base, mapped);
        }
    }
    Attached(const Attached&) = delete;
    Attached& operator=(const Attached&) = delete;

    bool ok() const { return base != nullptr; }
    const brio::SimDisplayHeader* header() const {
        return reinterpret_cast<const brio::SimDisplayHeader*>(base);
    }
    const uint8_t* pixels() const { return base + brio::sim_display_header_bytes; }
};

std::string unique_name(const char* tag) {
    return std::string(tag) + std::to_string(::getpid() % 100000);
}

} // namespace

TEST_CASE("a published framebuffer describes itself to whoever attaches") {
    const std::string n = unique_name("hdr");
    brio::SimDisplay<Mono, 128, 64> d(n);

    Attached v(d.name());
    REQUIRE(v.ok());
    const auto* h = v.header();
    CHECK(std::string(h->magic, 4) == "BRGX");
    CHECK(h->version == 1);
    CHECK(h->header_bytes == brio::sim_display_header_bytes);
    CHECK(h->width == 128);
    CHECK(h->height == 64);
    CHECK(h->stride == 16);
    CHECK(h->format == 1);
    CHECK(h->buffers == 1);
    CHECK(h->front == 0);
    CHECK(h->palette_used == 2);
    // A viewer needs no arguments: the size comes from the object.
    CHECK(v.bytes == brio::SimDisplay<Mono, 128, 64>::total_bytes);
}

TEST_CASE("what is drawn is what another mapping sees, with no copy") {
    const std::string n = unique_name("px");
    brio::SimDisplay<Mono, 64, 32> d(n);
    Attached v(d.name());
    REQUIRE(v.ok());

    auto fb = d.surface();
    brio::clear(fb, 0);
    brio::rect(fb, 0, 0, 64, 32, 1);
    brio::line(fb, 0, 0, 63, 31, 1);
    brio::text<brio::Font5x7>(fb, 4, 4, "hi", 1, 0);

    // Read the SAME pages back the way a viewer would, and compare
    // against the surface itself pixel by pixel.
    for (Extent y = 0; y < 32; ++y) {
        for (Extent x = 0; x < 64; ++x) {
            const uint8_t* row = v.pixels() + static_cast<size_t>(y) * 8;
            const uint8_t got = Mono::get(row, x);
            INFO("at (" << x << "," << y << ")");
            REQUIRE(got == fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)));
        }
    }

    // And a later write is seen through the mapping already held.
    brio::fill_rect(fb, 10, 10, 4, 4, 1);
    CHECK(Mono::get(v.pixels() + static_cast<size_t>(11) * 8, 11) == 1);
}

TEST_CASE("publish marks the picture, for a viewer that repaints on change") {
    const std::string n = unique_name("pub");
    brio::SimDisplay<Indexed8, 16, 8> d(n);
    Attached v(d.name());
    REQUIRE(v.ok());

    CHECK(v.header()->frame == 0);
    d.publish();
    d.publish();
    CHECK(v.header()->frame == 2);
    CHECK(d.frame() == 2);
}

TEST_CASE("the palette travels, because a byte alone shows nothing") {
    const std::string n = unique_name("pal");
    brio::SimDisplay<Indexed8, 8, 8> d(n);
    Attached v(d.name());
    REQUIRE(v.ok());

    // An 8-bit surface defaults to a grey ramp: the identity palette.
    CHECK(v.header()->palette_used == 256);
    CHECK(v.header()->palette[128][0] == 128);
    CHECK(v.header()->palette[128][1] == 128);

    d.set_palette(1, 0x00, 0x40, 0xFF);
    CHECK(v.header()->palette[1][0] == 0x00);
    CHECK(v.header()->palette[1][1] == 0x40);
    CHECK(v.header()->palette[1][2] == 0xFF);
}

TEST_CASE("the boot id is why a viewer cannot trust a counter inside") {
    const std::string n = unique_name("boot");

    uint64_t first_id = 0;
    {
        brio::SimDisplay<Mono, 16, 8> d(n);
        first_id = d.boot_id();
        Attached held(d.name());
        REQUIRE(held.ok());
        CHECK(held.header()->boot_id == first_id);

        // A second display under the same name unlinks the first and
        // makes a new object. The mapping above still points at the OLD
        // one - alive because it is still referenced - so nothing
        // written inside that object could ever tell the viewer.
        brio::SimDisplay<Mono, 16, 8> again(n);
        CHECK(again.boot_id() != first_id);
        CHECK(held.header()->boot_id == first_id); // the stale view

        // Re-opening BY NAME is what finds the change.
        Attached fresh(again.name());
        REQUIRE(fresh.ok());
        CHECK(fresh.header()->boot_id == again.boot_id());
    }
}

TEST_CASE("a name too long is refused rather than truncated") {
    // macOS caps a shared object's name at 31 characters, so a name that
    // works here must work there.
    CHECK_THROWS_AS(
        (brio::SimDisplay<Mono, 8, 8>(std::string(40, 'x'))),
        std::runtime_error);
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
                REQUIRE(c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) == (in ? 7 : 0));
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
                REQUIRE(c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) == (in ? 5 : 0));
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
            ref.line(3, 2, static_cast<Coord>(x1 + 3),
                     static_cast<Coord>(y1 + 2), 1);
            // The reference draws on the whole surface; the viewport
            // trims to its window, so compare only inside the window.
            uint32_t disagreements = 0;
            for (Extent y = 0; y < H; ++y) {
                for (Extent x = 0; x < W; ++x) {
                    const bool inside =
                        (x >= 3 && x < 13 && y >= 2 && y < 10);
                    const bool a = c.fb.get_pixel(static_cast<Coord>(x), static_cast<Coord>(y)) != 0;
                    const bool r = inside && ref.at(static_cast<Coord>(x), static_cast<Coord>(y)) != 0;
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
