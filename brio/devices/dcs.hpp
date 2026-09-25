/*
 * dcs.hpp (devices)
 *
 * THE VOCABULARY EVERY COMMAND PANEL SHARES. A display that keeps its
 * own pixels behind a link is addressed in the words of MIPI's Display
 * Command Set: a window, a memory write, a memory read, an address
 * mode, a pixel format. This file is that vocabulary and nothing else -
 * the codes, the bits, the packers - target-free, controller-free and
 * link-free, so that a controller's TRAITS type can say what its
 * silicon makes of these words and a LINK can carry them
 * (docs/design/gfx.md, "The command tier").
 *
 * WHAT IS NOT HERE. Nothing is sent from this file and no state is
 * kept: there is no panel object, no link, no timing and no wait. How
 * the address counter WALKS the frame memory is the controller's fact,
 * because the address mode's bits mean what a given silicon makes them
 * mean, and the rotation map that turns a logical surface into a window
 * and a walk is the driver's, one level above both.
 *
 * THE MIPI DCS SPECIFICATION IS NOT ON THIS DESK. Every code below is
 * the one a controller's own command table lists under the same name,
 * which is what the panel answers to; where two controllers disagree,
 * the traits type is where the disagreement is written and this file
 * stays as it is.
 *
 * A PACKER'S ROUND TRIP IS EXACT ONLY ON THE BITS THE FORMAT KEEPS.
 * Eighteen bits a pixel keep six bits a channel (mask 0xFC on each),
 * sixteen keep five, six and five (0xF8, 0xFC, 0xF8), twenty-four keep
 * everything; unpacking puts the kept bits back in the byte's most
 * significant bits and leaves the rest at zero, so pack then unpack is
 * the identity on a colour already masked and nothing weaker is
 * promised. A colour is the drawing world's: 0x00RRGGBB in a uint32_t.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <span>

namespace brio {

/**
 * The command codes. A scope with no instances rather than an enum:
 * these are BYTES on a link, sent beside a controller's own
 * manufacturer commands, and a type that had to be cast at every use
 * would buy nothing.
 */
struct Dcs {
    Dcs() = delete;

    static constexpr uint8_t nop = 0x00;
    static constexpr uint8_t swreset = 0x01;
    static constexpr uint8_t rddid = 0x04;             ///< read display identification
    static constexpr uint8_t rddst = 0x09;             ///< read display status
    static constexpr uint8_t rddpm = 0x0A;             ///< read display power mode
    static constexpr uint8_t rddmadctl = 0x0B;         ///< read display address mode
    static constexpr uint8_t rddcolmod = 0x0C;         ///< read display pixel format
    static constexpr uint8_t rddim = 0x0D;             ///< read display image mode
    static constexpr uint8_t rddsm = 0x0E;             ///< read display signal mode
    static constexpr uint8_t rddsdr = 0x0F;            ///< read display self-diagnostic result
    static constexpr uint8_t slpin = 0x10;
    static constexpr uint8_t slpout = 0x11;
    static constexpr uint8_t ptlon = 0x12;             ///< partial mode on
    static constexpr uint8_t noron = 0x13;             ///< normal mode on
    static constexpr uint8_t invoff = 0x20;
    static constexpr uint8_t invon = 0x21;
    static constexpr uint8_t gamset = 0x26;            ///< gamma curve select
    static constexpr uint8_t dispoff = 0x28;
    static constexpr uint8_t dispon = 0x29;
    static constexpr uint8_t caset = 0x2A;             ///< column address set
    static constexpr uint8_t paset = 0x2B;             ///< page address set
    static constexpr uint8_t ramwr = 0x2C;             ///< memory write
    static constexpr uint8_t ramrd = 0x2E;             ///< memory read
    static constexpr uint8_t ptlar = 0x30;             ///< partial area
    static constexpr uint8_t teoff = 0x34;             ///< tearing effect line off
    static constexpr uint8_t teon = 0x35;              ///< tearing effect line on
    static constexpr uint8_t madctl = 0x36;            ///< memory access control
    static constexpr uint8_t idmoff = 0x38;            ///< idle mode off
    static constexpr uint8_t idmon = 0x39;             ///< idle mode on
    static constexpr uint8_t colmod = 0x3A;            ///< interface pixel format
    static constexpr uint8_t ramwr_continue = 0x3C;    ///< write memory continue
    static constexpr uint8_t ramrd_continue = 0x3E;    ///< read memory continue
    static constexpr uint8_t wrdisbv = 0x51;           ///< write display brightness
    static constexpr uint8_t rddisbv = 0x52;           ///< read display brightness
    static constexpr uint8_t wrctrld = 0x53;           ///< write control display
    static constexpr uint8_t rdctrld = 0x54;           ///< read control display
};

/**
 * MADCTL's bits, under the standard's names, with the meaning brio
 * takes them in beside each.
 *
 * The three high bits are the WALK: they reorder the way the address
 * counter crosses the window a CASET/PASET pair opened, and on the
 * controller measured here they never move that window's origin. The
 * three low ones are the DISPLAY's - what the panel does with memory it
 * already holds - and a module may leave any of them unconnected, which
 * is a fact of the module and not of the controller.
 */
struct DcsAddressMode {
    DcsAddressMode() = delete;

    static constexpr uint8_t page_order = 0x80;         ///< B7: the walk along the PAGES reversed
    static constexpr uint8_t column_order = 0x40;       ///< B6: the walk along the COLUMNS reversed
    static constexpr uint8_t exchange = 0x20;           ///< B5: the counter steps the PAGES first
    static constexpr uint8_t line_order = 0x10;         ///< B4: line address order, a display bit
    static constexpr uint8_t bgr = 0x08;                ///< B3: a pixel's channels in the other order
    static constexpr uint8_t horizontal_flip = 0x02;    ///< B1: a display bit
    static constexpr uint8_t vertical_flip = 0x01;      ///< B0: a display bit

    /// The three that are the walk, for a reader who wants them at once.
    static constexpr uint8_t walk_bits = page_order | column_order | exchange;
};

/**
 * COLMOD's two three-bit fields: the command interface's pixel format
 * in bits 2:0 (DBI - what a memory write carries) and the display
 * interface's in bits 6:4 (DPI - what a video link carries). The codes
 * are the same in both fields.
 */
struct DcsPixelFormat {
    DcsPixelFormat() = delete;

    static constexpr uint8_t dbi_mask = 0x07;
    static constexpr uint8_t dpi_mask = 0x70;

    static constexpr uint8_t bpp3 = 1;
    static constexpr uint8_t bpp16 = 5;
    static constexpr uint8_t bpp18 = 6;
    static constexpr uint8_t bpp24 = 7;
};

constexpr uint8_t dcs_colmod(uint8_t dbi, uint8_t dpi) {
    return static_cast<uint8_t>((static_cast<uint8_t>(dpi << 4) & DcsPixelFormat::dpi_mask) |
                                (dbi & DcsPixelFormat::dbi_mask));
}

constexpr uint8_t dcs_colmod_dbi(uint8_t code) {
    return static_cast<uint8_t>(code & DcsPixelFormat::dbi_mask);
}

constexpr uint8_t dcs_colmod_dpi(uint8_t code) {
    return static_cast<uint8_t>((code & DcsPixelFormat::dpi_mask) >> 4);
}

/// 0x66: eighteen bits a pixel on both fields, which is the ILI9481's
/// reset value and the only format its serial interface stores.
constexpr uint8_t dcs_colmod_18bpp = dcs_colmod(DcsPixelFormat::bpp18, DcsPixelFormat::bpp18);

/**
 * One edge of a window - the four parameter bytes of CASET or of PASET:
 * the start and the end, each sixteen bits, most significant byte
 * first. THE END IS INCLUSIVE.
 */
constexpr std::array<uint8_t, 4> dcs_window(uint16_t start, uint16_t end) {
    return {static_cast<uint8_t>(start >> 8), static_cast<uint8_t>(start),
            static_cast<uint8_t>(end >> 8), static_cast<uint8_t>(end)};
}

/// A window edge as a pair of addresses.
struct DcsWindowRange {
    uint16_t start;
    uint16_t end;
};

/// The inverse of dcs_window(): the four bytes read back as a range.
constexpr DcsWindowRange dcs_window_range(std::span<const uint8_t, 4> p) {
    return {static_cast<uint16_t>((static_cast<uint32_t>(p[0]) << 8) | p[1]),
            static_cast<uint16_t>((static_cast<uint32_t>(p[2]) << 8) | p[3])};
}

/**
 * Eighteen bits a pixel, three bytes on the link: each channel's six
 * most significant bits in the most significant bits of its byte, the
 * low two at zero (a panel ignores them on a write and returns them
 * zero on a read). `bgr` exchanges the first byte with the third, which
 * is what a host packs when the address mode's BGR bit is set - or,
 * just as often, what a module's own wiring asks for.
 */
constexpr std::array<uint8_t, 3> dcs_rgb666_pack(uint32_t colour, bool bgr = false) {
    const uint8_t r = static_cast<uint8_t>((colour >> 16) & 0xFCu);
    const uint8_t g = static_cast<uint8_t>((colour >> 8) & 0xFCu);
    const uint8_t b = static_cast<uint8_t>(colour & 0xFCu);
    return bgr ? std::array<uint8_t, 3>{b, g, r} : std::array<uint8_t, 3>{r, g, b};
}

constexpr uint32_t dcs_rgb666_unpack(std::span<const uint8_t, 3> p, bool bgr = false) {
    const uint32_t first = p[0] & 0xFCu;
    const uint32_t middle = p[1] & 0xFCu;
    const uint32_t last = p[2] & 0xFCu;
    const uint32_t r = bgr ? last : first;
    const uint32_t b = bgr ? first : last;
    return (r << 16) | (middle << 8) | b;
}

/**
 * Sixteen bits a pixel, two bytes, most significant byte first on the
 * wire. Here `bgr` exchanges the two FIELDS and not two bytes: at this
 * width a channel is not a byte.
 */
constexpr std::array<uint8_t, 2> dcs_rgb565_pack(uint32_t colour, bool bgr = false) {
    const uint32_t r = (colour >> 19) & 0x1Fu;
    const uint32_t g = (colour >> 10) & 0x3Fu;
    const uint32_t b = (colour >> 3) & 0x1Fu;
    const uint32_t word = bgr ? ((b << 11) | (g << 5) | r) : ((r << 11) | (g << 5) | b);
    return {static_cast<uint8_t>(word >> 8), static_cast<uint8_t>(word)};
}

constexpr uint32_t dcs_rgb565_unpack(std::span<const uint8_t, 2> p, bool bgr = false) {
    const uint32_t word = (static_cast<uint32_t>(p[0]) << 8) | p[1];
    const uint32_t high = (word >> 11) & 0x1Fu;
    const uint32_t g = (word >> 5) & 0x3Fu;
    const uint32_t low = word & 0x1Fu;
    const uint32_t r = bgr ? low : high;
    const uint32_t b = bgr ? high : low;
    return (r << 19) | (g << 10) | (b << 3);
}

/// Twenty-four bits a pixel, three bytes, nothing lost.
constexpr std::array<uint8_t, 3> dcs_rgb888_pack(uint32_t colour, bool bgr = false) {
    const uint8_t r = static_cast<uint8_t>(colour >> 16);
    const uint8_t g = static_cast<uint8_t>(colour >> 8);
    const uint8_t b = static_cast<uint8_t>(colour);
    return bgr ? std::array<uint8_t, 3>{b, g, r} : std::array<uint8_t, 3>{r, g, b};
}

constexpr uint32_t dcs_rgb888_unpack(std::span<const uint8_t, 3> p, bool bgr = false) {
    const uint32_t first = p[0];
    const uint32_t middle = p[1];
    const uint32_t last = p[2];
    const uint32_t r = bgr ? last : first;
    const uint32_t b = bgr ? first : last;
    return (r << 16) | (middle << 8) | b;
}

/**
 * How a panel's answer lines up on a link that clocks whole BYTES. A
 * read command's answer is not always byte-aligned with the command
 * that asked for it, and which of these a given command takes is the
 * CONTROLLER's fact - a traits type states it, a framing adapter obeys
 * it, and a panel driver above them never has to know.
 */
enum class DcsReadFraming : uint8_t {
    undriven,       ///< the panel drives nothing: the link's idle level is the answer
    aligned,        ///< the value's first byte is the first byte clocked
    dummy_clock,    ///< one clock before the value's first bit: the whole stream is a bit late
    dummy_byte,     ///< one whole byte before the value
};

}   // namespace brio
