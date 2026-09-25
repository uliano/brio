// Host tests for the DCS vocabulary (brio/devices/dcs.hpp): the command
// codes, the address mode's bits, the pixel format's fields, the window
// packer and its inverse, and the three pixel packers.
//
// What is judged here is ARITHMETIC AND NOTHING ELSE. There is no panel
// in this suite, no link and no state: this file is target-free,
// controller-free and link-free, and the whole of it is constexpr, so
// most of what follows could be a static_assert and some of it is. What
// a controller makes of these words is test_ili9481's question.
//
// Run with: ctest --preset host (or ctest --preset host -R test_dcs)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <array>
#include <span>

#include "devices/dcs.hpp"

using namespace brio;

namespace {

/// A colour sampler with no library behind it: xorshift, so the sweep is
/// the same on every machine and every run.
uint32_t next(uint32_t& state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

constexpr uint32_t mask666 = 0x00FCFCFCu;
constexpr uint32_t mask565 = 0x00F8FCF8u;
constexpr uint32_t mask888 = 0x00FFFFFFu;

}   // namespace

TEST_CASE("the command codes are the ones a panel answers to") {
    CHECK(Dcs::nop == 0x00);
    CHECK(Dcs::swreset == 0x01);
    CHECK(Dcs::rddid == 0x04);
    CHECK(Dcs::rddst == 0x09);
    CHECK(Dcs::rddpm == 0x0A);
    CHECK(Dcs::rddmadctl == 0x0B);
    CHECK(Dcs::rddcolmod == 0x0C);
    CHECK(Dcs::rddim == 0x0D);
    CHECK(Dcs::rddsm == 0x0E);
    CHECK(Dcs::rddsdr == 0x0F);
    CHECK(Dcs::slpin == 0x10);
    CHECK(Dcs::slpout == 0x11);
    CHECK(Dcs::ptlon == 0x12);
    CHECK(Dcs::noron == 0x13);
    CHECK(Dcs::invoff == 0x20);
    CHECK(Dcs::invon == 0x21);
    CHECK(Dcs::gamset == 0x26);
    CHECK(Dcs::dispoff == 0x28);
    CHECK(Dcs::dispon == 0x29);
    CHECK(Dcs::caset == 0x2A);
    CHECK(Dcs::paset == 0x2B);
    CHECK(Dcs::ramwr == 0x2C);
    CHECK(Dcs::ramrd == 0x2E);
    CHECK(Dcs::ptlar == 0x30);
    CHECK(Dcs::teoff == 0x34);
    CHECK(Dcs::teon == 0x35);
    CHECK(Dcs::madctl == 0x36);
    CHECK(Dcs::idmoff == 0x38);
    CHECK(Dcs::idmon == 0x39);
    CHECK(Dcs::colmod == 0x3A);
    CHECK(Dcs::ramwr_continue == 0x3C);
    CHECK(Dcs::ramrd_continue == 0x3E);
    CHECK(Dcs::wrdisbv == 0x51);
    CHECK(Dcs::rddisbv == 0x52);
    CHECK(Dcs::wrctrld == 0x53);
    CHECK(Dcs::rdctrld == 0x54);
}

TEST_CASE("the address mode's bits are the standard's bit numbers") {
    CHECK(DcsAddressMode::page_order == 0x80);        // B7
    CHECK(DcsAddressMode::column_order == 0x40);      // B6
    CHECK(DcsAddressMode::exchange == 0x20);          // B5
    CHECK(DcsAddressMode::line_order == 0x10);        // B4
    CHECK(DcsAddressMode::bgr == 0x08);               // B3
    CHECK(DcsAddressMode::horizontal_flip == 0x02);   // B1
    CHECK(DcsAddressMode::vertical_flip == 0x01);     // B0

    // The three that reorder the walk, and no other.
    CHECK(DcsAddressMode::walk_bits == 0xE0);
    static_assert((DcsAddressMode::walk_bits & DcsAddressMode::bgr) == 0);

    // Every bit is a different bit: the codes the bench walked are these
    // three in every combination.
    CHECK((DcsAddressMode::page_order | DcsAddressMode::column_order) == 0xC0);
    CHECK((DcsAddressMode::exchange | DcsAddressMode::column_order) == 0x60);
    CHECK((DcsAddressMode::exchange | DcsAddressMode::page_order) == 0xA0);
    CHECK((DcsAddressMode::exchange | DcsAddressMode::column_order |
           DcsAddressMode::page_order) == 0xE0);
}

TEST_CASE("the pixel format is two three-bit fields") {
    CHECK(DcsPixelFormat::dbi_mask == 0x07);
    CHECK(DcsPixelFormat::dpi_mask == 0x70);
    CHECK(DcsPixelFormat::bpp3 == 1);
    CHECK(DcsPixelFormat::bpp16 == 5);
    CHECK(DcsPixelFormat::bpp18 == 6);
    CHECK(DcsPixelFormat::bpp24 == 7);

    // The two fields do not reach each other, and bits 3 and 7 are
    // nobody's.
    CHECK(dcs_colmod(DcsPixelFormat::bpp18, DcsPixelFormat::bpp18) == 0x66);
    CHECK(dcs_colmod(DcsPixelFormat::bpp16, DcsPixelFormat::bpp24) == 0x75);
    CHECK(dcs_colmod(DcsPixelFormat::bpp24, 0) == 0x07);
    CHECK(dcs_colmod(0, DcsPixelFormat::bpp24) == 0x70);
    CHECK(dcs_colmod(0xFF, 0xFF) == 0x77);

    static_assert(dcs_colmod_18bpp == 0x66);
    static_assert(dcs_colmod_dbi(dcs_colmod_18bpp) == DcsPixelFormat::bpp18);
    static_assert(dcs_colmod_dpi(dcs_colmod_18bpp) == DcsPixelFormat::bpp18);

    for (uint8_t dbi = 0; dbi < 8u; ++dbi) {
        for (uint8_t dpi = 0; dpi < 8u; ++dpi) {
            const uint8_t code = dcs_colmod(dbi, dpi);
            CHECK(dcs_colmod_dbi(code) == dbi);
            CHECK(dcs_colmod_dpi(code) == dpi);
        }
    }
}

TEST_CASE("the window packer is big-endian, and its inverse is exact over the whole range") {
    const std::array<uint8_t, 4> w = dcs_window(0x0123, 0x4567);
    CHECK(w[0] == 0x01);
    CHECK(w[1] == 0x23);
    CHECK(w[2] == 0x45);
    CHECK(w[3] == 0x67);

    static_assert(dcs_window(0, 319)[3] == 0x3F);
    static_assert(dcs_window(479, 479)[0] == 0x01);
    static_assert(dcs_window_range(dcs_window(300, 303)).start == 300);
    static_assert(dcs_window_range(dcs_window(300, 303)).end == 303);

    // Every sixteen-bit address, as a start and as an end.
    for (uint32_t v = 0; v <= 0xFFFFu; ++v) {
        const uint16_t start = static_cast<uint16_t>(v);
        const uint16_t end = static_cast<uint16_t>(0xFFFFu - v);
        const std::array<uint8_t, 4> p = dcs_window(start, end);
        const DcsWindowRange back = dcs_window_range(p);
        REQUIRE(back.start == start);
        REQUIRE(back.end == end);
    }
}

TEST_CASE("the 18-bit packer keeps six bits a channel and nothing else") {
    // The low two bits of every byte are zero, and the six that remain
    // sit in the byte's most significant bits.
    const std::array<uint8_t, 3> p = dcs_rgb666_pack(0x00FF8001u);
    CHECK(p[0] == 0xFC);
    CHECK(p[1] == 0x80);
    CHECK(p[2] == 0x00);

    // bgr exchanges the first byte with the third, and nothing else.
    const std::array<uint8_t, 3> q = dcs_rgb666_pack(0x00FF8001u, true);
    CHECK(q[0] == 0x00);
    CHECK(q[1] == 0x80);
    CHECK(q[2] == 0xFC);

    static_assert(dcs_rgb666_unpack(dcs_rgb666_pack(0x00FCFCFCu)) == 0x00FCFCFCu);
    static_assert(dcs_rgb666_unpack(dcs_rgb666_pack(0x00FF8001u)) == 0x00FC8000u);

    uint32_t state = 0x13579BDFu;
    for (uint32_t i = 0; i < 100000u; ++i) {
        const uint32_t colour = next(state) & 0x00FFFFFFu;
        const uint32_t kept = colour & mask666;
        REQUIRE(dcs_rgb666_unpack(dcs_rgb666_pack(colour)) == kept);
        REQUIRE(dcs_rgb666_unpack(dcs_rgb666_pack(colour, true), true) == kept);
        // Packed one way and read the other: the channels come back
        // exchanged, which is exactly what the address mode's BGR bit
        // does to a pixel that crosses the link.
        const uint32_t swapped = ((kept & 0xFFu) << 16) | (kept & 0xFF00u) | ((kept >> 16) & 0xFFu);
        REQUIRE(dcs_rgb666_unpack(dcs_rgb666_pack(colour, true)) == swapped);
    }

    // Every value of every channel, one channel at a time.
    for (uint32_t v = 0; v < 256u; ++v) {
        for (uint8_t shift = 0; shift < 24u; shift += 8u) {
            const uint32_t colour = v << shift;
            REQUIRE(dcs_rgb666_unpack(dcs_rgb666_pack(colour)) == (colour & mask666));
        }
    }
}

TEST_CASE("the 16-bit packer keeps five, six and five, most significant byte first") {
    const std::array<uint8_t, 2> p = dcs_rgb565_pack(0x00FF8001u);
    // 11111 100000 00000
    CHECK(p[0] == 0xFC);
    CHECK(p[1] == 0x00);
    CHECK(dcs_rgb565_pack(0x00000000u)[0] == 0x00);
    CHECK(dcs_rgb565_pack(0x00FFFFFFu)[0] == 0xFF);
    CHECK(dcs_rgb565_pack(0x00FFFFFFu)[1] == 0xFF);

    // Here bgr exchanges the two FIELDS: at this width a channel is not
    // a byte.
    CHECK(dcs_rgb565_pack(0x00FF0000u, true) == std::array<uint8_t, 2>{0x00, 0x1F});

    static_assert(dcs_rgb565_unpack(dcs_rgb565_pack(0x00F8FCF8u)) == 0x00F8FCF8u);

    uint32_t state = 0x2468ACE1u;
    for (uint32_t i = 0; i < 100000u; ++i) {
        const uint32_t colour = next(state) & 0x00FFFFFFu;
        const uint32_t kept = colour & mask565;
        REQUIRE(dcs_rgb565_unpack(dcs_rgb565_pack(colour)) == kept);
        REQUIRE(dcs_rgb565_unpack(dcs_rgb565_pack(colour, true), true) == kept);
    }

    for (uint32_t v = 0; v < 256u; ++v) {
        for (uint8_t shift = 0; shift < 24u; shift += 8u) {
            const uint32_t colour = v << shift;
            REQUIRE(dcs_rgb565_unpack(dcs_rgb565_pack(colour)) == (colour & mask565));
        }
    }
}

TEST_CASE("the 24-bit packer keeps everything") {
    CHECK(dcs_rgb888_pack(0x00123456u) == std::array<uint8_t, 3>{0x12, 0x34, 0x56});
    CHECK(dcs_rgb888_pack(0x00123456u, true) == std::array<uint8_t, 3>{0x56, 0x34, 0x12});

    static_assert(dcs_rgb888_unpack(dcs_rgb888_pack(0x00ABCDEFu)) == 0x00ABCDEFu);

    uint32_t state = 0x0BADF00Du;
    for (uint32_t i = 0; i < 100000u; ++i) {
        const uint32_t colour = next(state) & mask888;
        REQUIRE(dcs_rgb888_unpack(dcs_rgb888_pack(colour)) == colour);
        REQUIRE(dcs_rgb888_unpack(dcs_rgb888_pack(colour, true), true) == colour);
    }
}

TEST_CASE("the three packers agree wherever their formats overlap") {
    // A colour already masked to what the narrower format keeps survives
    // every packer: the formats differ in what they DROP and in nothing
    // else.
    uint32_t state = 0x5EEDFACEu;
    for (uint32_t i = 0; i < 20000u; ++i) {
        const uint32_t colour = next(state) & mask565;
        REQUIRE(dcs_rgb888_unpack(dcs_rgb888_pack(colour)) == colour);
        REQUIRE(dcs_rgb666_unpack(dcs_rgb666_pack(colour)) == colour);
        REQUIRE(dcs_rgb565_unpack(dcs_rgb565_pack(colour)) == colour);
    }
}

TEST_CASE("a read's framing is a named thing and not a number") {
    // The enum exists so that a controller can state its framing and an
    // adapter obey it; nothing here has an opinion about which is which.
    CHECK(static_cast<uint8_t>(DcsReadFraming::undriven) !=
          static_cast<uint8_t>(DcsReadFraming::aligned));
    CHECK(static_cast<uint8_t>(DcsReadFraming::dummy_clock) !=
          static_cast<uint8_t>(DcsReadFraming::dummy_byte));
}
