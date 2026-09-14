/*
 * ltdc.hpp
 *
 * The STM32F4's LCD-TFT display controller (RM0090 ch. 16) - the
 * peripheral that turns a rectangle of memory into a parallel RGB panel
 * signal, and the one peripheral of this family whose output is a
 * PICTURE:
 *
 *  LtdcTiming        a panel's synchronous timings in the panel's own
 *                    words (four horizontal numbers, four vertical,
 *                    four polarities), and the accumulated arithmetic
 *                    16.4.1 asks for;
 *  Ltdc              the block: the APB2 gate and the reset that puts
 *                    all three clock domains back, the timing
 *                    registers, the background colour, the dithering,
 *                    the shadow-reload discipline, the four events over
 *                    TWO vectors, and the position counter that is the
 *                    only thing a program can watch a frame with;
 *  LtdcLayer<1|2>    one layer: the window inside the active area, the
 *                    pixel format, the framebuffer address, length and
 *                    pitch, the constant alpha and the two blending
 *                    factors, the colour key, the default colour and
 *                    the CLUT;
 *  LtdcFramebuffer<Pixel>
 *                    a rectangle of memory as a typed surface - the
 *                    pixel type IS the beat, and the CPU writes it and
 *                    does not read it.
 *
 *   using Panel = brio::Ltdc;
 *   Panel::timing(qvga);                       // SSCR/BPCR/AWCR/TWCR
 *   brio::LtdcLayer<1>::configure(layer);      // window, format, buffer
 *   Panel::reload(brio::LtdcReload::immediate);
 *   Panel::enable();
 *
 * THE CLOCK IS A PLL OF ITS OWN. LCD_CLK comes off PLLSAI's R output
 * through RCC_DCKCFGR's PLLSAIDIVR (16.3.2), and nothing else in this
 * family uses it: `Rcc`'s pllsai verbs and `lcd_clock_config_for()`
 * (stm32f4/clock.hpp) are the arithmetic, `Ltdc::pixel_clock()` runs it.
 * The pixel clock is what the panel's data sheet states and the frame
 * rate follows from it and the timings - `ltdc_frame_rate_mhz()` and
 * `ltdc_frame_pixels()` are that division, done once at compile time.
 *
 * THREE CLOCK DOMAINS, AND THE BUS STALLS. 16.3.2: the layer's address
 * and length registers are on HCLK, the interrupt and reload registers
 * on PCLK2, and everything else on LCD_CLK - and an access to a
 * register of the pixel-clock domain stalls the APB2 bus for six or
 * seven PCLK2 periods PLUS five LCD_CLK periods. At a 6 MHz pixel clock
 * and a 90 MHz APB2 that is some 80 core cycles a read, which is why
 * nothing here polls a register of that domain in a loop without saying
 * so, and why `position()` costs what the bench findings say it costs.
 *
 * THE SHADOW REGISTERS ARE THE MODEL. Every layer register but the CLUT
 * is shadowed (16.4.1): a write goes into a shadow that the active
 * register takes either at once (SRCR.IMR) or at the next vertical
 * blanking (SRCR.VBR), and until then a READ RETURNS THE ACTIVE VALUE
 * AND NOT WHAT WAS WRITTEN. So a read-modify-write of a layer register
 * with a reload pending loses the pending write, and `configure()`
 * writes each register whole for that reason. Both reload bits are set
 * by software and cleared by HARDWARE, which is what `reload_pending()`
 * watches - and a read issued straight after the reload STORE races the
 * reload across the clock domains, which is why `reload()` spends one
 * access of the pixel-clock domain on its caller's behalf.
 *
 * WHAT THE PADS ARE IS NOT IN THIS FILE. Twenty-eight signals at most -
 * eight bits per channel plus HSYNC, VSYNC, DE and CLK - and which pad
 * carries which, and how many bits of each channel a BOARD really
 * wires, is a datasheet table and a schematic (DS10693 table 12, where
 * every LTDC signal is AF14 except four pads that carry one on AF9). A
 * panel narrower than 24 bits takes the MOST SIGNIFICANT bits of each
 * channel (16.3.3), so an 18-bit panel is wired to R[7:2], G[7:2] and
 * B[7:2] and the controller is still programmed in 24 bits: the layer's
 * pixel format says what is in MEMORY, never what is on the wire.
 *
 * WHAT THIS FILE DOES NOT DO IS DRIVE A PANEL'S OWN CONTROLLER. A TFT
 * module usually has a controller of its own that must be told to take
 * an RGB interface at all, and that conversation happens over some
 * other bus - SPI or I2C - before a single pixel is fetched. That is
 * the board's business and the application's, not this driver's.
 *
 * CONCURRENCY. Every configuring verb here is a read-modify-write of a
 * register with no set/clear twin and is meant for setup; the layer
 * ones additionally interact with the reload discipline above. The
 * event verbs (`flags`, `clear`, `isr`) are single stores of write-one
 * bits and are legal from a handler. `LtdcFramebuffer`'s writes are
 * plain stores into memory the controller reads: they are safe from any
 * context, and what they cost is the memory's, not this block's.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"

namespace brio {

// =============================================================================
// Pixels
// =============================================================================
//
// The packers for the formats this family's display tier speaks, as
// plain arithmetic with no register in sight: a pixel is a number, and
// which number it is depends only on the format. They live here because
// this is the display chapter, and they compile on every part - the
// accelerator of stm32f4/dma2d.hpp takes its colours in ARGB8888 and
// packs them itself, so a part with the accelerator and no controller
// needs none of them.

/// 0xAARRGGBB, the internal format everything in the tier converts to.
constexpr uint32_t argb8888(uint8_t a, uint8_t r, uint8_t g, uint8_t b) {
    return (static_cast<uint32_t>(a) << 24) | (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) | b;
}

/// 0xRRGGBB in the low three bytes: the 24-bit format, which occupies
/// THREE bytes in memory and not four (16.4.2, table 90).
constexpr uint32_t rgb888(uint8_t r, uint8_t g, uint8_t b) {
    return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | b;
}

/// 5-6-5 in sixteen bits, the top bits of each channel kept.
constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((static_cast<uint16_t>(r) & 0xF8u) << 8) |
                                 ((static_cast<uint16_t>(g) & 0xFCu) << 3) | (b >> 3));
}

/// 1-5-5-5: one bit of alpha, five of each channel.
constexpr uint16_t argb1555(bool opaque, uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>((opaque ? 0x8000u : 0u) |
                                 ((static_cast<uint16_t>(r) & 0xF8u) << 7) |
                                 ((static_cast<uint16_t>(g) & 0xF8u) << 2) | (b >> 3));
}

/// 4-4-4-4: the top nibble of each of the four channels.
constexpr uint16_t argb4444(uint8_t a, uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((static_cast<uint16_t>(a) & 0xF0u) << 8) |
                                 ((static_cast<uint16_t>(r) & 0xF0u) << 4) |
                                 (static_cast<uint16_t>(g) & 0xF0u) | (b >> 4));
}

// =============================================================================
// The layer's vocabulary (16.4.2, 16.7.18)
// =============================================================================

/// LTDC_LxPFCR.PF - what is in the FRAME BUFFER. The controller expands
/// every one of them to its internal ARGB8888 by replicating the most
/// significant bits into the least (16.4.2), so a 5-bit channel of
/// 0b11111 really is full scale and not 248/255.
enum class LtdcPixelFormat : uint8_t {
    argb8888 = 0,
    rgb888 = 1,
    rgb565 = 2,
    argb1555 = 3,
    argb4444 = 4,
    l8 = 5,    ///< an index into the layer's CLUT
    al44 = 6,  ///< 4 bits of alpha, 4 of index
    al88 = 7,  ///< 8 bits of alpha, 8 of index
};

/// How many bytes one pixel of a format occupies in memory (table 90).
/// RGB888 is the awkward one: three bytes, so a line of it is not
/// word-aligned unless the width is a multiple of four.
constexpr uint8_t ltdc_bytes_per_pixel(LtdcPixelFormat f) {
    switch (f) {
        case LtdcPixelFormat::argb8888: return 4;
        case LtdcPixelFormat::rgb888: return 3;
        case LtdcPixelFormat::l8:
        case LtdcPixelFormat::al44: return 1;
        default: return 2;
    }
}

/// Whether a format is an INDEX and therefore wants a CLUT loaded
/// (16.4.2). AL44's sixteen entries sit at the replicated addresses
/// `ltdc_al44_clut_address()` gives.
constexpr bool ltdc_format_indexed(LtdcPixelFormat f) {
    return f == LtdcPixelFormat::l8 || f == LtdcPixelFormat::al44 ||
           f == LtdcPixelFormat::al88;
}

/// Where entry `index` of an AL44 layer's CLUT must be written: the
/// 4-bit luminance replicated to eight bits, so entry 1 is at 0x11 and
/// entry 15 at 0xFF (16.4.2). L8 and AL88 use the index itself.
constexpr uint8_t ltdc_al44_clut_address(uint8_t index) {
    return static_cast<uint8_t>((index & 0x0Fu) * 0x11u);
}

/// LTDC_LxBFCR.BF1 - what the CURRENT layer's colour is multiplied by.
/// Only two of the eight codes are legal (16.7.21).
enum class LtdcBlend1 : uint8_t {
    constant_alpha = 4,
    pixel_alpha_x_constant = 6,
};

/// LTDC_LxBFCR.BF2 - what the layers UNDER it are multiplied by. The
/// two codes are the complements of BF1's two, and the reset value is
/// the second (0x0607: BF1 = 6, BF2 = 7).
enum class LtdcBlend2 : uint8_t {
    one_minus_constant_alpha = 5,
    one_minus_pixel_alpha_x_constant = 7,
};

/// The blending 16.7.21 performs, in the units the registers hold:
/// BC = BF1 x C + BF2 x Cs, with the constant alpha divided by 255 by
/// the hardware and the division rounded DOWN. This is the model a test
/// judges the silicon against, and the arithmetic an application does
/// when it wants to know what a colour will look like.
constexpr uint8_t ltdc_blend_channel(LtdcBlend1 bf1, LtdcBlend2 bf2, uint8_t constant_alpha,
                                     uint8_t pixel_alpha, uint8_t layer, uint8_t below) {
    const uint32_t f1 = (bf1 == LtdcBlend1::constant_alpha)
                            ? constant_alpha
                            : (static_cast<uint32_t>(pixel_alpha) * constant_alpha) / 255u;
    const uint32_t f2 = (bf2 == LtdcBlend2::one_minus_constant_alpha)
                            ? (255u - constant_alpha)
                            : (255u - (static_cast<uint32_t>(pixel_alpha) * constant_alpha) / 255u);
    return static_cast<uint8_t>((f1 * layer + f2 * below) / 255u);
}

/// A rectangle inside the ACTIVE display area, in pixels from its top
/// left corner. The back porches the register wants added are the
/// block's own and this struct does not carry them.
struct LtdcWindow {
    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t width = 0;
    uint16_t height = 0;
};

/// Everything one layer needs to show a rectangle of memory.
struct LtdcLayerConfig {
    LtdcWindow window{};
    LtdcPixelFormat format = LtdcPixelFormat::rgb565;
    /// The address of the pixel at the window's top left corner.
    uint32_t framebuffer = 0;
    /// The distance in BYTES from the start of one line of the frame
    /// buffer to the start of the next. Zero means "the window's own
    /// width", which is the usual case; a layer that shows part of a
    /// wider image states the wider pitch.
    uint16_t pitch = 0;
    /// LxCACR, divided by 255 by the hardware. 255 is opaque.
    uint8_t constant_alpha = 255;
    LtdcBlend1 blend_source = LtdcBlend1::pixel_alpha_x_constant;
    LtdcBlend2 blend_below = LtdcBlend2::one_minus_pixel_alpha_x_constant;
    /// LxDCCR, shown outside the window and while the layer is disabled.
    /// Transparent black by default, which is what keeps a disabled
    /// layer invisible (16.4.2).
    uint32_t default_colour = 0;
    /// LxCKCR + LxCR.COLKEN: a pixel matching this RGB after the format
    /// conversion becomes fully transparent.
    bool colour_key = false;
    uint32_t colour_key_rgb = 0;
    /// LxCR.CLUTEN, meaningful for the three indexed formats alone.
    bool clut = false;
};

// =============================================================================
// The panel's timings (16.4.1)
// =============================================================================

/// LTDC_GCR's four polarity bits. The reset value of all four is
/// "active low" for the three synchronisation signals and "not
/// inverted" for the clock.
enum class LtdcPolarity : uint8_t { active_low = 0, active_high = 1 };

/// PCPOL: whether the panel latches on the pixel clock as the
/// controller makes it, or on its inverse.
enum class LtdcClockEdge : uint8_t { direct = 0, inverted = 1 };

/// A panel's timings AS THE PANEL'S DATA SHEET STATES THEM - four
/// horizontal numbers and four vertical ones, each a count of pixels or
/// of lines, none of them accumulated and none of them off by one. The
/// accumulation and the minus ones that SSCR, BPCR, AWCR and TWCR want
/// are this file's arithmetic and not the caller's.
struct LtdcTiming {
    uint16_t hsync = 0;          ///< the synchronisation pulse, in pixel clocks
    uint16_t hbp = 0;            ///< the back porch after it
    uint16_t width = 0;          ///< the active area
    uint16_t hfp = 0;            ///< the front porch before the next pulse
    uint16_t vsync = 0;          ///< the same four, in lines
    uint16_t vbp = 0;
    uint16_t height = 0;
    uint16_t vfp = 0;
    LtdcPolarity hsync_polarity = LtdcPolarity::active_low;
    LtdcPolarity vsync_polarity = LtdcPolarity::active_low;
    LtdcPolarity de_polarity = LtdcPolarity::active_low;
    LtdcClockEdge clock_edge = LtdcClockEdge::direct;
};

/// The four accumulated numbers the registers really hold, before the
/// minus one each field carries.
constexpr uint16_t ltdc_total_width(const LtdcTiming& t) {
    return static_cast<uint16_t>(t.hsync + t.hbp + t.width + t.hfp);
}
constexpr uint16_t ltdc_total_height(const LtdcTiming& t) {
    return static_cast<uint16_t>(t.vsync + t.vbp + t.height + t.vfp);
}

/// How many pixel clocks one whole frame costs - the active area plus
/// every porch and both synchronisation pulses. This, and not the
/// visible size, is what divides the pixel clock into a frame rate.
constexpr uint32_t ltdc_frame_pixels(const LtdcTiming& t) {
    return static_cast<uint32_t>(ltdc_total_width(t)) * ltdc_total_height(t);
}

/// The frame rate in MILLIHERTZ, so that 65.331 Hz is an integer and
/// not a float this framework does not use. Zero when the timing is
/// empty.
constexpr uint32_t ltdc_frame_rate_mhz(uint32_t pixel_hz, const LtdcTiming& t) {
    const uint32_t pixels = ltdc_frame_pixels(t);
    if (pixels == 0u) {
        return 0u;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(pixel_hz) * 1000u) / pixels);
}

/// How many bytes a second one layer of `format` at this timing FETCHES
/// out of whatever memory holds it - the number that decides whether a
/// framebuffer can live where it is put. The active area alone is
/// fetched; the porches cost the panel time and the memory nothing.
constexpr uint32_t ltdc_fetch_bytes_per_second(uint32_t pixel_hz, const LtdcTiming& t,
                                               LtdcPixelFormat format) {
    const uint32_t per_frame =
        static_cast<uint32_t>(t.width) * t.height * ltdc_bytes_per_pixel(format);
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(per_frame) * ltdc_frame_rate_mhz(pixel_hz, t)) / 1000u);
}

/// Every field of the four timing registers has a width and every one
/// of them holds one less than the count, so a zero is not a legal
/// count anywhere: SSCR, BPCR, AWCR and TWCR are 12 bits horizontal and
/// 11 bits vertical, and 16.4.1 caps the active area at 1024x768.
constexpr bool ltdc_timing_valid(const LtdcTiming& t) {
    if (t.hsync == 0u || t.vsync == 0u || t.width == 0u || t.height == 0u) {
        return false;
    }
    if (t.width > 1024u || t.height > 768u) {
        return false;   // 16.4.1: "only up to 1024x768 is supported"
    }
    if (ltdc_total_width(t) > 4096u || ltdc_total_height(t) > 2048u) {
        return false;   // the accumulated fields are 12 and 11 bits
    }
    return true;
}

/// Whether a window fits the active area a timing describes. The
/// registers take an accumulated position and would happily hold one
/// past the last visible pixel, where nothing is displayed and the
/// FIFO is fetched all the same.
constexpr bool ltdc_window_valid(const LtdcWindow& w, const LtdcTiming& t) {
    if (w.width == 0u || w.height == 0u) {
        return false;
    }
    return (static_cast<uint32_t>(w.x) + w.width <= t.width) &&
           (static_cast<uint32_t>(w.y) + w.height <= t.height);
}

/// LTDC_LxCFBLR.CFBLL: the line length in bytes PLUS THREE (16.7.23),
/// which is the chapter's own formula and not a rounding of ours.
constexpr uint16_t ltdc_line_length(uint16_t pixels, LtdcPixelFormat format) {
    return static_cast<uint16_t>(pixels * ltdc_bytes_per_pixel(format) + 3u);
}

/// What a layer configuration must satisfy before it reaches a
/// register: a window inside the active area, a pitch at least as wide
/// as the window, both length fields inside their 13 bits, the line
/// count inside its 11, and a CLUT asked for only where the format is
/// an index.
constexpr bool ltdc_layer_config_valid(const LtdcLayerConfig& c, const LtdcTiming& t) {
    if (!ltdc_window_valid(c.window, t)) {
        return false;
    }
    const uint8_t bpp = ltdc_bytes_per_pixel(c.format);
    const uint32_t line = static_cast<uint32_t>(c.window.width) * bpp;
    const uint32_t pitch = (c.pitch != 0u) ? c.pitch : line;
    if (pitch < line || pitch > 0x1FFFu || line + 3u > 0x1FFFu) {
        return false;
    }
    if (c.window.height > 0x7FFu) {
        return false;
    }
    if (c.clut && !ltdc_format_indexed(c.format)) {
        return false;
    }
    return true;
}

// =============================================================================
// A framebuffer as a type
// =============================================================================

/// A rectangle of memory seen as pixels of one type: the base address,
/// the width and the height, and the pitch when the surface is a window
/// on something wider. `Pixel` is the ELEMENT the memory holds -
/// uint16_t for the 16-bit formats, uint32_t for ARGB8888, uint8_t for
/// the indexed ones - and it is the beat every access is made in, which
/// on an external memory is the difference between a fast surface and a
/// slow one.
///
/// EVERY ACCESS HERE IS A WRITE. On this family a frame buffer usually
/// lives in the external memory, and ES0206 2.3.5 says a CPU READ of
/// FMC-held data with interrupts live may be corrupted or fault while
/// reads by other masters are not: the readers of a frame buffer are
/// the display controller and the accelerator, both of them masters of
/// their own. `line()` hands out a pointer for the caller who knows
/// what it is doing; nothing here reads a pixel back.
template <typename Pixel>
struct LtdcFramebuffer {
    volatile Pixel* base = nullptr;
    uint16_t width = 0;
    uint16_t height = 0;
    /// Pixels from the start of one line to the start of the next; zero
    /// means the surface is exactly `width` wide.
    uint16_t pitch = 0;

    constexpr uint16_t stride() const { return pitch != 0u ? pitch : width; }
    constexpr uint32_t pitch_bytes() const {
        return static_cast<uint32_t>(stride()) * sizeof(Pixel);
    }
    constexpr uint32_t line_bytes() const {
        return static_cast<uint32_t>(width) * sizeof(Pixel);
    }
    constexpr uint32_t bytes() const { return pitch_bytes() * height; }
    /// Where the surface starts, as the number a layer's LxCFBAR takes.
    uint32_t address() const { return reinterpret_cast<uint32_t>(base); }

    volatile Pixel* line(uint16_t y) const { return base + static_cast<uint32_t>(y) * stride(); }

    /// One pixel, silently dropped when it falls outside the surface -
    /// a drawing routine clips at the edges rather than scribbling into
    /// the next line, which on a shared memory is somebody else's data.
    void write(uint16_t x, uint16_t y, Pixel value) const {
        if (x >= width || y >= height) {
            return;
        }
        line(y)[x] = value;
    }

    void fill(Pixel value) const {
        for (uint16_t y = 0; y < height; ++y) {
            volatile Pixel* row = line(y);
            for (uint16_t x = 0; x < width; ++x) {
                row[x] = value;
            }
        }
    }

    /// A rectangle, clipped to the surface.
    void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, Pixel value) const {
        if (x >= width || y >= height) {
            return;
        }
        const uint16_t x_end = (static_cast<uint32_t>(x) + w > width) ? width
                                                                     : static_cast<uint16_t>(x + w);
        const uint16_t y_end = (static_cast<uint32_t>(y) + h > height)
                                   ? height
                                   : static_cast<uint16_t>(y + h);
        for (uint16_t row_y = y; row_y < y_end; ++row_y) {
            volatile Pixel* row = line(row_y);
            for (uint16_t col = x; col < x_end; ++col) {
                row[col] = value;
            }
        }
    }
};

// The register-facing half exists only where the device header declares
// the controller. A part with the accelerator and no display compiles
// everything above and nothing below.
#if defined(LTDC_BASE)

// =============================================================================
// The block
// =============================================================================

/// The four events of 16.5, as the BIT NUMBER they share in all three
/// of LTDC_IER, LTDC_ISR and LTDC_ICR - the enable, the flag and the
/// write-one clear of one event sit at the same position in the three
/// registers, which is why one enumeration serves all of them.
enum class LtdcEvent : uint8_t {
    line = 0,              ///< the programmed line was reached
    /// A pixel was asked of an empty layer FIFO - PER PIXEL, not per
    /// line and not per frame (measured: 33194 interrupts in three
    /// frames over a starved layer; docs/stm32f4/ltdc.md). Arm its
    /// interrupt over a picture that is known good; watch its FLAG over
    /// one that is not.
    fifo_underrun = 1,
    transfer_error = 2,    ///< an AHB error while fetching
    register_reload = 3,   ///< the shadow registers were taken
};

constexpr uint32_t ltdc_event_mask(LtdcEvent e) { return 1UL << static_cast<uint8_t>(e); }

/// WHICH OF THE FOUR EACH VECTOR CARRIES (figure 85). The four flags
/// live in ONE register, so a handler that cleared everything standing
/// would clear the other vector's event too - and the other vector may
/// not even be enabled, in which case the flag is what a program reads
/// instead of taking. These two masks are what a handler passes to
/// `Ltdc::isr()` so that it takes its own pair and leaves the rest.
inline constexpr uint32_t ltdc_global_events =
    ltdc_event_mask(LtdcEvent::line) | ltdc_event_mask(LtdcEvent::register_reload);
inline constexpr uint32_t ltdc_error_events =
    ltdc_event_mask(LtdcEvent::fifo_underrun) | ltdc_event_mask(LtdcEvent::transfer_error);

/// SRCR's two bits (16.7.6). Both are set by software and cleared by
/// HARDWARE when the reload happens, and there is no way to withdraw
/// one.
enum class LtdcReload : uint8_t {
    immediate = 0,          ///< SRCR.IMR
    vertical_blanking = 1,  ///< SRCR.VBR, at the first line after the active area
};

/// Where the timing generator is now, out of LTDC_CPSR. The counters
/// span the WHOLE frame, porches included, so x runs to the total width
/// and y to the total height - not to the visible size.
struct LtdcPosition {
    uint16_t x = 0;
    uint16_t y = 0;
};

/// LTDC_CDSR, the four signals as the controller is driving them THIS
/// instant. Each bit reads 1 when its signal is in its ACTIVE state,
/// whatever polarity GCR gives it, which is why the reset value is
/// 0x0F with nothing enabled.
struct LtdcDisplayStatus {
    bool hsync = false;
    bool vsync = false;
    bool horizontal_data_enable = false;
    bool vertical_data_enable = false;
};

class Ltdc {
public:
    static_assert(ltdc_present(),
                  "brio Ltdc: this device has no LCD-TFT controller (it is on the parts with a "
                  "display interface alone; stm32f4/device_tables.hpp's ltdc_present() is what a "
                  "program asks)");

    Ltdc() = delete;

    /// TWO vectors (16.5, figure 85): the line and the register reload
    /// come out of the global one, the FIFO underrun and the transfer
    /// error out of the error one. A program that wants all four binds
    /// both and calls the same ISR body from each.
    static constexpr IRQn_Type irq_line = ltdc_irq();
    static constexpr IRQn_Type error_irq_line = ltdc_error_irq();

    static constexpr uint8_t layers = ltdc_layers();

    static LTDC_TypeDef& regs() { return *reinterpret_cast<LTDC_TypeDef*>(ltdc_base()); }

    // ---- the gate and the reset (6.3.16, 16.3.2) -----------------------------------

    /// RCC_APB2ENR.LTDCEN, clear at reset. With it clear every register
    /// of the block reads zero and swallows writes; every configuring
    /// verb here opens it first.
    static void clock(bool on) { Rcc::apb2_clock(ltdc_clock_mask(), on); }
    static bool clock() { return Rcc::apb2_clock(ltdc_clock_mask()); }

    /// RCC_APB2RSTR.LTDCRST: every register back to its reset value,
    /// and 16.3.2 says it resets ALL THREE clock domains and not just
    /// the one the APB reaches.
    static void reset() {
        clock(true);
        Rcc::apb2_reset(ltdc_reset_mask());
    }

    static void enable_interrupt() { Nvic::enable(irq_line); }
    static void disable_interrupt() { Nvic::disable(irq_line); }
    static void enable_error_interrupt() { Nvic::enable(error_irq_line); }
    static void disable_error_interrupt() { Nvic::disable(error_irq_line); }

    // ---- the pixel clock (6.3.24, 6.3.25) ------------------------------------------

    /// Bring LCD_CLK up at `lcd_hz` off the root the MAIN PLL is using,
    /// through PLLSAI's R output and RCC_DCKCFGR's divider: the PLL
    /// stopped, the triple written, the PLL started and waited for.
    /// False - with the PLL left off - when the rate is not exactly
    /// reachable, when the part's PLL has no R output, or when the PLL
    /// does not lock.
    ///
    /// `root_hz` is HSE's rate for a system clock off HSE and the HSI's
    /// 16 MHz otherwise: this PLL shares PLLCFGR.PLLSRC and PLLM with
    /// the main one, so it is the SYSTEM's choice of root and divider
    /// that fixes the VCO input here.
    static bool pixel_clock(uint32_t root_hz, uint32_t lcd_hz) {
        const LcdClockConfig c = lcd_clock_config_for(root_hz, lcd_hz, Rcc::pll_m());
        if (c.pll.n == 0u) {
            return false;
        }
        return pixel_clock(c);
    }

    /// The same from a triple solved at compile time, which is what an
    /// application with one panel does: `lcd_clock_config_for()` is
    /// constexpr and its result can be a static_assert away from being
    /// wrong.
    static bool pixel_clock(const LcdClockConfig& c) {
        if (c.pll.n == 0u || !pllsai_has_r() || !pllsai_has_lcd_divider()) {
            return false;
        }
        Rcc::pllsai_enable(false);
        if (!Rcc::pllsai_wait(false)) {
            return false;
        }
        if (!Rcc::lcd_clock_divider(c.divider) || !Rcc::pllsai_configure(c.pll)) {
            return false;
        }
        Rcc::pllsai_enable(true);
        return Rcc::pllsai_wait(true);
    }

    /// LCD_CLK as the registers now hold it, given the root the PLLs
    /// share. Zero when the PLL is off or the part has no R output.
    static uint32_t pixel_clock(uint32_t root_hz) {
        if (!Rcc::pllsai_ready()) {
            return 0u;
        }
        return lcd_clock_hz(pllsai_r_hz(root_hz, Rcc::pllsai_config(), Rcc::pll_m()),
                            Rcc::lcd_clock_divider());
    }

    // ---- the timings (16.4.1, 16.7.1 .. 16.7.5) ------------------------------------

    /// SSCR, BPCR, AWCR and TWCR from a panel's own numbers, plus GCR's
    /// four polarity bits. The accumulation and the minus one each
    /// field carries are done here; the caller states the panel's data
    /// sheet. Refuses an illegal timing and writes nothing when it
    /// does.
    ///
    /// 16.4.1: with the controller disabled the generator is held at
    /// the last pixel before the vertical synchronisation and the FIFOs
    /// are flushed, so a timing changed with LTDCEN clear costs no
    /// half-drawn frame.
    static bool timing(const LtdcTiming& t) {
        if (!ltdc_timing_valid(t)) {
            return false;
        }
        clock(true);
        LTDC_TypeDef& r = regs();
        r.SSCR = (static_cast<uint32_t>(t.hsync - 1u) << LTDC_SSCR_HSW_Pos) |
                 (static_cast<uint32_t>(t.vsync - 1u) << LTDC_SSCR_VSH_Pos);
        r.BPCR = (static_cast<uint32_t>(t.hsync + t.hbp - 1u) << LTDC_BPCR_AHBP_Pos) |
                 (static_cast<uint32_t>(t.vsync + t.vbp - 1u) << LTDC_BPCR_AVBP_Pos);
        r.AWCR = (static_cast<uint32_t>(t.hsync + t.hbp + t.width - 1u) << LTDC_AWCR_AAW_Pos) |
                 (static_cast<uint32_t>(t.vsync + t.vbp + t.height - 1u) << LTDC_AWCR_AAH_Pos);
        r.TWCR = (static_cast<uint32_t>(ltdc_total_width(t) - 1u) << LTDC_TWCR_TOTALW_Pos) |
                 (static_cast<uint32_t>(ltdc_total_height(t) - 1u) << LTDC_TWCR_TOTALH_Pos);
        const uint32_t polarity =
            ((t.hsync_polarity == LtdcPolarity::active_high) ? LTDC_GCR_HSPOL : 0u) |
            ((t.vsync_polarity == LtdcPolarity::active_high) ? LTDC_GCR_VSPOL : 0u) |
            ((t.de_polarity == LtdcPolarity::active_high) ? LTDC_GCR_DEPOL : 0u) |
            ((t.clock_edge == LtdcClockEdge::inverted) ? LTDC_GCR_PCPOL : 0u);
        r.GCR = (r.GCR & ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL)) |
                polarity;
        return true;
    }

    /// The four registers back as the panel's own numbers - the
    /// accumulation undone, so what comes out is what went in.
    static LtdcTiming timing() {
        clock(true);
        const LTDC_TypeDef& r = regs();
        const uint16_t hsync =
            static_cast<uint16_t>(((r.SSCR & LTDC_SSCR_HSW) >> LTDC_SSCR_HSW_Pos) + 1u);
        const uint16_t vsync =
            static_cast<uint16_t>(((r.SSCR & LTDC_SSCR_VSH) >> LTDC_SSCR_VSH_Pos) + 1u);
        const uint16_t ahbp =
            static_cast<uint16_t>(((r.BPCR & LTDC_BPCR_AHBP) >> LTDC_BPCR_AHBP_Pos) + 1u);
        const uint16_t avbp =
            static_cast<uint16_t>(((r.BPCR & LTDC_BPCR_AVBP) >> LTDC_BPCR_AVBP_Pos) + 1u);
        const uint16_t aaw =
            static_cast<uint16_t>(((r.AWCR & LTDC_AWCR_AAW) >> LTDC_AWCR_AAW_Pos) + 1u);
        const uint16_t aah =
            static_cast<uint16_t>(((r.AWCR & LTDC_AWCR_AAH) >> LTDC_AWCR_AAH_Pos) + 1u);
        const uint16_t total_w =
            static_cast<uint16_t>(((r.TWCR & LTDC_TWCR_TOTALW) >> LTDC_TWCR_TOTALW_Pos) + 1u);
        const uint16_t total_h =
            static_cast<uint16_t>(((r.TWCR & LTDC_TWCR_TOTALH) >> LTDC_TWCR_TOTALH_Pos) + 1u);
        const uint32_t g = r.GCR;
        return LtdcTiming{
            hsync,
            static_cast<uint16_t>(ahbp - hsync),
            static_cast<uint16_t>(aaw - ahbp),
            static_cast<uint16_t>(total_w - aaw),
            vsync,
            static_cast<uint16_t>(avbp - vsync),
            static_cast<uint16_t>(aah - avbp),
            static_cast<uint16_t>(total_h - aah),
            (g & LTDC_GCR_HSPOL) != 0u ? LtdcPolarity::active_high : LtdcPolarity::active_low,
            (g & LTDC_GCR_VSPOL) != 0u ? LtdcPolarity::active_high : LtdcPolarity::active_low,
            (g & LTDC_GCR_DEPOL) != 0u ? LtdcPolarity::active_high : LtdcPolarity::active_low,
            (g & LTDC_GCR_PCPOL) != 0u ? LtdcClockEdge::inverted : LtdcClockEdge::direct};
    }

    /// The accumulated back porches the LAYER registers are counted
    /// from (16.7.15, 16.7.16): the first visible pixel of a line is
    /// AHBP + 1 and the first visible line is AVBP + 1.
    static uint16_t horizontal_back_porch() {
        clock(true);
        return static_cast<uint16_t>((regs().BPCR & LTDC_BPCR_AHBP) >> LTDC_BPCR_AHBP_Pos);
    }
    static uint16_t vertical_back_porch() {
        clock(true);
        return static_cast<uint16_t>((regs().BPCR & LTDC_BPCR_AVBP) >> LTDC_BPCR_AVBP_Pos);
    }

    // ---- the global control register (16.7.5) --------------------------------------

    /// GCR.LTDCEN. Everything else is configured first; this is what
    /// starts the timing generator.
    static void enable() {
        clock(true);
        regs().GCR |= LTDC_GCR_LTDCEN;
    }
    static void disable() {
        clock(true);
        regs().GCR &= ~LTDC_GCR_LTDCEN;
    }
    static bool enabled() {
        clock(true);
        return (regs().GCR & LTDC_GCR_LTDCEN) != 0u;
    }

    /// GCR.BCCR's colour, shown where no layer covers the active area.
    static void background(uint8_t r, uint8_t g, uint8_t b) {
        clock(true);
        regs().BCCR = (static_cast<uint32_t>(r) << LTDC_BCCR_BCRED_Pos) |
                      (static_cast<uint32_t>(g) << LTDC_BCCR_BCGREEN_Pos) |
                      (static_cast<uint32_t>(b) << LTDC_BCCR_BCBLUE_Pos);
    }
    static uint32_t background() {
        clock(true);
        return regs().BCCR & 0x00FF'FFFFu;
    }

    /// GCR.DEN: the pseudo-random dithering of 16.4.1, which rounds a
    /// 24-bit colour onto a narrower panel with a different error every
    /// frame. It may be switched on and off while the controller runs.
    static void dither(bool on) {
        clock(true);
        if (on) {
            regs().GCR |= LTDC_GCR_DEN;
        } else {
            regs().GCR &= ~LTDC_GCR_DEN;
        }
    }
    static bool dither() {
        clock(true);
        return (regs().GCR & LTDC_GCR_DEN) != 0u;
    }

    /// GCR's DRW/DGW/DBW - how many bits of pseudo-random value the
    /// silicon adds per channel. THEY ARE READ-ONLY (16.7.5: "these
    /// bits return the dither red bits"), so there is no setter: the
    /// width is a property of the block and the chapter's two bits per
    /// channel is what it returns.
    static uint8_t dither_red_width() {
        clock(true);
        return static_cast<uint8_t>((regs().GCR & LTDC_GCR_DRW) >> LTDC_GCR_DRW_Pos);
    }
    static uint8_t dither_green_width() {
        clock(true);
        return static_cast<uint8_t>((regs().GCR & LTDC_GCR_DGW) >> LTDC_GCR_DGW_Pos);
    }
    static uint8_t dither_blue_width() {
        clock(true);
        return static_cast<uint8_t>((regs().GCR & LTDC_GCR_DBW) >> LTDC_GCR_DBW_Pos);
    }

    // ---- the shadow registers (16.4.1, 16.7.6) -------------------------------------

    /// Ask for the layer registers written since the last reload to be
    /// taken - at once, or at the beginning of the first line after the
    /// active display area. The bit is cleared by hardware when the
    /// reload happens and CANNOT be withdrawn, so a program that asks
    /// for a vertical-blanking reload has committed to it.
    ///
    /// A READ ISSUED STRAIGHT AFTER THIS STORE RACES THE RELOAD, so
    /// this verb spends one access of the pixel-clock domain to cover
    /// it. The reload crosses into that domain and a read already on
    /// its way there answers the PRE-RELOAD value (measured, eight
    /// times out of eight; docs/stm32f4/ltdc.md), while one access of
    /// any register of it - five LCD_CLK periods of stall by 16.3.2 -
    /// is longer than the crossing. So the read of GCR below is what
    /// makes a caller's own read back the reloaded value; it costs
    /// that stall, once per picture change.
    static void reload(LtdcReload when) {
        clock(true);
        regs().SRCR = (when == LtdcReload::immediate) ? LTDC_SRCR_IMR : LTDC_SRCR_VBR;
        (void)regs().GCR;
    }

    /// Whether a reload asked for has not happened yet.
    static bool reload_pending() {
        clock(true);
        return (regs().SRCR & (LTDC_SRCR_IMR | LTDC_SRCR_VBR)) != 0u;
    }

    /// Spin until no reload is pending. False on a time-out, which with
    /// a vertical-blanking reload means the controller is not running -
    /// a disabled controller never reaches a vertical blanking and the
    /// bit would stand forever.
    static bool wait_reload(uint32_t spins = 2'000'000u) {
        clock(true);
        for (uint32_t i = 0; i < spins; ++i) {
            if (!reload_pending()) {
                (void)regs().GCR;   // the late slot, as reload() spends it
                return true;
            }
        }
        return false;
    }

    // ---- the events (16.5, 16.7.8 .. 16.7.11) --------------------------------------

    /// LTDC_LIPCR: the line whose start raises the line event, counted
    /// in the TOTAL frame and not in the active area - so a program
    /// that wants the event at the end of the visible area programs
    /// AVBP + height, and one that wants it at the start of a frame
    /// programs 0.
    static bool line_interrupt(uint16_t line) {
        if (line > 0x7FFu) {
            return false;
        }
        clock(true);
        regs().LIPCR = static_cast<uint32_t>(line) << LTDC_LIPCR_LIPOS_Pos;
        return true;
    }
    static uint16_t line_interrupt() {
        clock(true);
        return static_cast<uint16_t>((regs().LIPCR & LTDC_LIPCR_LIPOS) >> LTDC_LIPCR_LIPOS_Pos);
    }

    static void interrupt(LtdcEvent e, bool on) {
        clock(true);
        if (on) {
            regs().IER |= ltdc_event_mask(e);
        } else {
            regs().IER &= ~ltdc_event_mask(e);
        }
    }
    static bool interrupt(LtdcEvent e) {
        clock(true);
        return (regs().IER & ltdc_event_mask(e)) != 0u;
    }

    /// AN EVENT'S FLAG EXISTS ONLY WHILE ITS ENABLE IS SET (measured;
    /// docs/stm32f4/ltdc.md). 16.7.9 describes ISR as four status bits
    /// and says nothing about IER, but on this silicon a FIFO underrun
    /// that really happened leaves ISR clear when FUIE is clear - the
    /// same rule this family's EXTI has for its pending bit. So a
    /// program that WATCHES a status instead of taking an interrupt
    /// still has to set the enable; only the vector is optional.
    static bool flag(LtdcEvent e) {
        clock(true);
        return (regs().ISR & ltdc_event_mask(e)) != 0u;
    }

    /// Every flag at once, as a mask of `ltdc_event_mask()` bits - one
    /// read of a PCLK2-domain register instead of four. The same rule
    /// as `flag()`: only an armed event has one.
    static uint32_t flags() {
        clock(true);
        return regs().ISR & 0x0Fu;
    }

    static void clear(LtdcEvent e) {
        clock(true);
        regs().ICR = ltdc_event_mask(e);
    }
    static void clear(uint32_t mask) {
        clock(true);
        regs().ICR = mask & 0x0Fu;
    }

    /// The ISR body for either vector: of the flags standing, the ones
    /// in `handled` are cleared and handed back. `handled` is what makes
    /// one body serve both vectors - `ltdc_global_events` on one,
    /// `ltdc_error_events` on the other - and it matters because THE
    /// FOUR FLAGS SHARE ONE REGISTER: a body that took all four would
    /// clear the other vector's event, and would clear a FIFO underrun
    /// out from under a program that watches the flag instead of arming
    /// its interrupt.
    [[gnu::always_inline]] static uint32_t isr(uint32_t handled = 0x0Fu) {
        const uint32_t standing = regs().ISR & handled & 0x0Fu;
        if (standing != 0u) {
            regs().ICR = standing;
        }
        return standing;
    }

    // ---- what the generator is doing (16.7.12, 16.7.13) ----------------------------

    /// LTDC_CPSR, the pixel and line the generator is on. It is a
    /// register of the PIXEL CLOCK domain, so a read of it stalls the
    /// APB for the seven PCLK2 plus five LCD_CLK periods of 16.3.2 -
    /// which at a slow pixel clock is most of the cost of a polling
    /// loop.
    static LtdcPosition position() {
        clock(true);
        const uint32_t v = regs().CPSR;
        return LtdcPosition{static_cast<uint16_t>((v & LTDC_CPSR_CXPOS) >> LTDC_CPSR_CXPOS_Pos),
                            static_cast<uint16_t>((v & LTDC_CPSR_CYPOS) >> LTDC_CPSR_CYPOS_Pos)};
    }

    /// LTDC_CDSR: each of the four signals as it stands NOW, 1 meaning
    /// active whatever polarity GCR gives it.
    static LtdcDisplayStatus display_status() {
        clock(true);
        const uint32_t v = regs().CDSR;
        return LtdcDisplayStatus{(v & LTDC_CDSR_HSYNCS) != 0u, (v & LTDC_CDSR_VSYNCS) != 0u,
                                 (v & LTDC_CDSR_HDES) != 0u, (v & LTDC_CDSR_VDES) != 0u};
    }
};

// =============================================================================
// One layer (16.4.2, 16.7.14 .. 16.7.25)
// =============================================================================

/// Layer 1 is the bottom one and layer 2 the top: the order is fixed by
/// the silicon and blending always happens, even for a layer that is
/// disabled - which is why a disabled layer's blending factors are left
/// at their reset value and its default colour at transparent black
/// (16.4.2).
template <uint8_t layer>
class LtdcLayer {
public:
    static_assert(layer == 1u || layer == 2u,
                  "brio LtdcLayer: this controller has two layers, 1 (the bottom) and 2 (the top)");
    static_assert(ltdc_present(), "brio LtdcLayer: this device has no LCD-TFT controller");

    LtdcLayer() = delete;

    static LTDC_Layer_TypeDef& regs() {
        return *reinterpret_cast<LTDC_Layer_TypeDef*>(ltdc_layer_base(layer));
    }

    // ---- the window (16.7.15, 16.7.16) ---------------------------------------------

    /// The layer's rectangle inside the active area. The registers hold
    /// ACCUMULATED positions - the first visible pixel of a line is
    /// AHBP + 1 - and this reads the block's own back porches to do
    /// that arithmetic, so a window is stated in the picture's
    /// coordinates and never in the timing generator's.
    static void window(const LtdcWindow& w) {
        Ltdc::clock(true);
        const uint16_t h0 = static_cast<uint16_t>(Ltdc::horizontal_back_porch() + 1u + w.x);
        const uint16_t v0 = static_cast<uint16_t>(Ltdc::vertical_back_porch() + 1u + w.y);
        regs().WHPCR = (static_cast<uint32_t>(h0 + w.width - 1u) << LTDC_LxWHPCR_WHSPPOS_Pos) |
                       (static_cast<uint32_t>(h0) << LTDC_LxWHPCR_WHSTPOS_Pos);
        regs().WVPCR = (static_cast<uint32_t>(v0 + w.height - 1u) << LTDC_LxWVPCR_WVSPPOS_Pos) |
                       (static_cast<uint32_t>(v0) << LTDC_LxWVPCR_WVSTPOS_Pos);
    }

    /// The window back in the picture's coordinates, the back porches
    /// subtracted again.
    static LtdcWindow window() {
        Ltdc::clock(true);
        const uint16_t hbp = static_cast<uint16_t>(Ltdc::horizontal_back_porch() + 1u);
        const uint16_t vbp = static_cast<uint16_t>(Ltdc::vertical_back_porch() + 1u);
        const uint32_t h = regs().WHPCR;
        const uint32_t v = regs().WVPCR;
        const uint16_t h0 =
            static_cast<uint16_t>((h & LTDC_LxWHPCR_WHSTPOS) >> LTDC_LxWHPCR_WHSTPOS_Pos);
        const uint16_t h1 =
            static_cast<uint16_t>((h & LTDC_LxWHPCR_WHSPPOS) >> LTDC_LxWHPCR_WHSPPOS_Pos);
        const uint16_t v0 =
            static_cast<uint16_t>((v & LTDC_LxWVPCR_WVSTPOS) >> LTDC_LxWVPCR_WVSTPOS_Pos);
        const uint16_t v1 =
            static_cast<uint16_t>((v & LTDC_LxWVPCR_WVSPPOS) >> LTDC_LxWVPCR_WVSPPOS_Pos);
        return LtdcWindow{static_cast<uint16_t>(h0 - hbp), static_cast<uint16_t>(v0 - vbp),
                          static_cast<uint16_t>(h1 + 1u - h0),
                          static_cast<uint16_t>(v1 + 1u - v0)};
    }

    // ---- the frame buffer (16.7.18, 16.7.22 .. 16.7.24) ----------------------------

    static void pixel_format(LtdcPixelFormat f) {
        Ltdc::clock(true);
        regs().PFCR = static_cast<uint32_t>(f);
    }
    static LtdcPixelFormat pixel_format() {
        Ltdc::clock(true);
        return static_cast<LtdcPixelFormat>(regs().PFCR & LTDC_LxPFCR_PF);
    }

    /// LxCFBAR: the address of the pixel at the window's top left.
    static void framebuffer(uint32_t address) {
        Ltdc::clock(true);
        regs().CFBAR = address;
    }
    static uint32_t framebuffer() {
        Ltdc::clock(true);
        return regs().CFBAR;
    }

    /// LxCFBLR's two fields: how many bytes of each line are fetched
    /// (the chapter's own "+ 3") and how many bytes it is from the
    /// start of one line to the start of the next. Both are 13 bits and
    /// a value past that is refused.
    static bool buffer_line(uint16_t line_bytes, uint16_t pitch_bytes) {
        if (line_bytes + 3u > 0x1FFFu || pitch_bytes > 0x1FFFu || pitch_bytes < line_bytes) {
            return false;
        }
        Ltdc::clock(true);
        regs().CFBLR = (static_cast<uint32_t>(pitch_bytes) << LTDC_LxCFBLR_CFBP_Pos) |
                       (static_cast<uint32_t>(line_bytes + 3u) << LTDC_LxCFBLR_CFBLL_Pos);
        return true;
    }

    /// LxCFBLNR: how many lines are fetched. Fewer than the window's
    /// height raises the FIFO underrun; more is fetched and discarded
    /// (16.4.2).
    static bool buffer_lines(uint16_t lines) {
        if (lines > 0x7FFu) {
            return false;
        }
        Ltdc::clock(true);
        regs().CFBLNR = static_cast<uint32_t>(lines) << LTDC_LxCFBLNR_CFBLNBR_Pos;
        return true;
    }
    static uint16_t buffer_lines() {
        Ltdc::clock(true);
        return static_cast<uint16_t>((regs().CFBLNR & LTDC_LxCFBLNR_CFBLNBR) >>
                                     LTDC_LxCFBLNR_CFBLNBR_Pos);
    }

    // ---- blending, keying, the default colour (16.7.17, 16.7.19 .. 16.7.21) --------

    /// LxCACR, divided by 255 by the hardware.
    static void constant_alpha(uint8_t alpha) {
        Ltdc::clock(true);
        regs().CACR = alpha;
    }
    static uint8_t constant_alpha() {
        Ltdc::clock(true);
        return static_cast<uint8_t>(regs().CACR & LTDC_LxCACR_CONSTA);
    }

    /// LxBFCR's two factors. Only two codes of eight are legal in each
    /// field and the enumerations spell exactly those, so there is
    /// nothing here to refuse.
    static void blending(LtdcBlend1 source, LtdcBlend2 below) {
        Ltdc::clock(true);
        regs().BFCR = (static_cast<uint32_t>(source) << LTDC_LxBFCR_BF1_Pos) |
                      (static_cast<uint32_t>(below) << LTDC_LxBFCR_BF2_Pos);
    }

    /// LxDCCR, an ARGB8888 word: what is shown outside the window and
    /// while the layer is disabled.
    static void default_colour(uint32_t argb) {
        Ltdc::clock(true);
        regs().DCCR = argb;
    }
    static uint32_t default_colour() {
        Ltdc::clock(true);
        return regs().DCCR;
    }

    /// LxCKCR + LxCR.COLKEN: a pixel whose RGB matches becomes fully
    /// transparent, alpha included, after the format conversion and
    /// before the blending.
    static void colour_key(uint32_t rgb) {
        Ltdc::clock(true);
        regs().CKCR = rgb & 0x00FF'FFFFu;
    }
    static uint32_t colour_key() {
        Ltdc::clock(true);
        return regs().CKCR & 0x00FF'FFFFu;
    }
    static void colour_keying(bool on) {
        Ltdc::clock(true);
        if (on) {
            regs().CR |= LTDC_LxCR_COLKEN;
        } else {
            regs().CR &= ~LTDC_LxCR_COLKEN;
        }
    }

    // ---- the CLUT (16.4.2, 16.7.25) ------------------------------------------------

    /// One entry of the layer's colour table. LxCLUTWR is WRITE-ONLY
    /// and it is the ONE layer register that is not shadowed (16.4.1),
    /// so an entry takes effect the moment it is written and no reload
    /// carries it - which is why 16.6 says the CLUT is the one thing
    /// that may not be changed on the fly, and why a program loads it
    /// before it enables the layer.
    ///
    /// For an AL44 layer the address is the 4-bit index replicated to
    /// eight bits: `ltdc_al44_clut_address()`.
    static void clut_entry(uint8_t address, uint8_t r, uint8_t g, uint8_t b) {
        Ltdc::clock(true);
        regs().CLUTWR = (static_cast<uint32_t>(address) << LTDC_LxCLUTWR_CLUTADD_Pos) |
                        (static_cast<uint32_t>(r) << LTDC_LxCLUTWR_RED_Pos) |
                        (static_cast<uint32_t>(g) << LTDC_LxCLUTWR_GREEN_Pos) |
                        (static_cast<uint32_t>(b) << LTDC_LxCLUTWR_BLUE_Pos);
    }

    static void clut(bool on) {
        Ltdc::clock(true);
        if (on) {
            regs().CR |= LTDC_LxCR_CLUTEN;
        } else {
            regs().CR &= ~LTDC_LxCR_CLUTEN;
        }
    }
    static bool clut() {
        Ltdc::clock(true);
        return (regs().CR & LTDC_LxCR_CLUTEN) != 0u;
    }

    // ---- the enable (16.7.14) ------------------------------------------------------

    static void enable() {
        Ltdc::clock(true);
        regs().CR |= LTDC_LxCR_LEN;
    }
    static void disable() {
        Ltdc::clock(true);
        regs().CR &= ~LTDC_LxCR_LEN;
    }
    static bool enabled() {
        Ltdc::clock(true);
        return (regs().CR & LTDC_LxCR_LEN) != 0u;
    }

    // ---- the whole layer at once ---------------------------------------------------

    /// Every register of the layer from one description, in 16.6's
    /// order, WITH THE LAYER LEFT DISABLED and no reload asked for: the
    /// caller enables it and reloads when the whole picture is ready,
    /// which is what makes a two-layer change atomic.
    ///
    /// The timing is taken as an argument because the window's legality
    /// is a question about the ACTIVE AREA, and reading it back out of
    /// the registers would make a refusal depend on what a previous
    /// stage left there. Refuses and writes nothing.
    static bool configure(const LtdcLayerConfig& c, const LtdcTiming& t) {
        if (!ltdc_layer_config_valid(c, t)) {
            return false;
        }
        Ltdc::clock(true);
        const uint8_t bpp = ltdc_bytes_per_pixel(c.format);
        const uint16_t line = static_cast<uint16_t>(c.window.width * bpp);
        const uint16_t pitch = (c.pitch != 0u) ? c.pitch : line;
        window(c.window);
        pixel_format(c.format);
        framebuffer(c.framebuffer);
        if (!buffer_line(line, pitch) || !buffer_lines(c.window.height)) {
            return false;
        }
        constant_alpha(c.constant_alpha);
        blending(c.blend_source, c.blend_below);
        default_colour(c.default_colour);
        colour_key(c.colour_key_rgb);
        regs().CR = (c.colour_key ? LTDC_LxCR_COLKEN : 0u) | (c.clut ? LTDC_LxCR_CLUTEN : 0u);
        return true;
    }
};

#endif   // LTDC_BASE

} // namespace brio
