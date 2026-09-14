/*
 * dma2d.hpp
 *
 * The STM32F4's Chrom-Art accelerator (RM0090 ch. 11) - a DMA that
 * knows what a PIXEL is: it fills, copies, converts between eleven
 * colour formats and blends two sources, all in rectangles rather than
 * in runs, and it is a bus master of its own.
 *
 *  Dma2dSource       one input: the address, the line offset, the
 *                    format, what to do with its alpha, and the CLUT
 *                    an indexed format needs;
 *  Dma2dOutput       the destination: the address, the line offset and
 *                    one of the five output formats;
 *  Dma2dArea         the rectangle, in pixels per line and lines;
 *  Dma2d             the block - the four modes as four verbs (fill,
 *                    copy, convert, blend), the CLUT loaded by the
 *                    engine itself, the six events, the suspend and
 *                    the abort, and the AHB dead time that is the one
 *                    knob a program has for sharing the bus with a
 *                    display controller's fetch.
 *
 *   brio::Dma2d::init();
 *   brio::Dma2d::fill({.address = frame, .format = brio::Dma2dOutputColor::rgb565},
 *                     brio::argb8888(255, 0, 0, 255), {.pixels = 64, .lines = 32});
 *   (void)brio::Dma2d::wait();
 *
 * THE FOUR MODES ARE ONE ENGINE WITH PARTS SWITCHED OFF (11.3.11).
 * Register-to-memory writes one colour and fetches nothing;
 * memory-to-memory fetches through the foreground FIFO as a plain
 * buffer with NO conversion, so the format field says only how wide a
 * pixel is; memory-to-memory with the pixel format converter runs the
 * foreground through its converter; and the blending mode fetches both
 * sources, converts each, and mixes them with 11.3.11's formula. So
 * `copy()` and `convert()` differ in one bit and in what they promise,
 * and they are two verbs because the promise is the point.
 *
 * THE OFFSETS ARE IN PIXELS AND THE ADDRESSES ARE BYTES. FGOR, BGOR and
 * OOR count PIXELS of their own format (11.5.5), added at the end of
 * every line, while FGMAR, BGMAR and OMAR are byte addresses that must
 * be aligned to the format's own width. A rectangle inside a wider
 * image is therefore "the image's width minus the rectangle's" as an
 * offset, and the address of its top left corner.
 *
 * THE SILICON CHECKS THE CONFIGURATION AND SAYS SO IN A FLAG, NOT IN A
 * REFUSAL. 11.3.11 lists a dozen ways to be wrong - an address not
 * aligned to its format, a colour mode that is not one of the codes, an
 * odd pixel count with a 4-bit format - and the engine answers all of
 * them by raising CEIF at the START and transferring nothing. The
 * driver checks what it can before the store (`dma2d_source_valid()`,
 * `dma2d_output_valid()`, `dma2d_area_valid()`) so that a caller gets a
 * false instead of a flag; what is left to the flag is what only the
 * silicon knows.
 *
 * ONE TRANSFER AT A TIME, AND A CLUT LOAD IS A TRANSFER. CR.START is
 * cleared by hardware at the end, on an abort, on an error and on a
 * configuration error; FGPFCCR.START and BGPFCCR.START are the same bit
 * for the two automatic CLUT loads, and 11.3.11 says a CLUT load can
 * not run beside a data transfer. `busy()` is the one question and
 * `wait()` the one wait.
 *
 * IT IS A MASTER AND IT COMPETES. The AHB dead time (11.3.15) inserts a
 * guaranteed minimum number of cycles between two of its accesses,
 * which is how a program stops the accelerator from starving a display
 * controller reading the same external memory. It is the only
 * bandwidth knob in the chapter, and the bench findings are what say
 * whether it is needed.
 *
 * CONCURRENCY. Every configuring verb writes registers the silicon
 * makes read-only while a transfer runs, so they are setup verbs and
 * the four mode verbs refuse while `busy()`. `flags()`, `clear()` and
 * `isr()` are single stores of write-one bits and are legal from a
 * handler; `abort()` and `suspend()` are too.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"

namespace brio {

// =============================================================================
// The vocabulary (11.3.4, 11.3.8, 11.5)
// =============================================================================

/// FGPFCCR/BGPFCCR.CM - what an INPUT holds (table 53). Eleven formats:
/// the display controller's eight, plus a 4-bit index and the two alpha
/// masks that carry no colour of their own.
enum class Dma2dColor : uint8_t {
    argb8888 = 0,
    rgb888 = 1,
    rgb565 = 2,
    argb1555 = 3,
    argb4444 = 4,
    l8 = 5,
    al44 = 6,
    al88 = 7,
    l4 = 8,   ///< a 4-bit index: two pixels a byte
    a8 = 9,   ///< alpha alone, the colour taken from FGCOLR/BGCOLR
    a4 = 10,  ///< the same in four bits
};

/// OPFCCR.CM - what the OUTPUT holds (11.5.14). Five formats, all of
/// them direct: there is no CLUT generator, so an indexed output does
/// not exist.
enum class Dma2dOutputColor : uint8_t {
    argb8888 = 0,
    rgb888 = 1,
    rgb565 = 2,
    argb1555 = 3,
    argb4444 = 4,
};

/// FGPFCCR/BGPFCCR.AM - what happens to a source's alpha channel
/// (11.5.8). Code 3 is meaningless and has no spelling here.
enum class Dma2dAlpha : uint8_t {
    keep = 0,      ///< the source's own, or 0xFF where the format has none
    replace = 1,   ///< the ALPHA field instead
    multiply = 2,  ///< the source's own times the ALPHA field, over 255
};

/// FGPFCCR/BGPFCCR.CCM - the format of the CLUT the engine loads.
enum class Dma2dClutColor : uint8_t { argb8888 = 0, rgb888 = 1 };

/// CR.MODE (11.5.1). The codes are the register's, so the order is the
/// silicon's and not a reading order.
enum class Dma2dMode : uint8_t {
    memory_to_memory = 0,       ///< the foreground FIFO as a plain buffer
    memory_to_memory_pfc = 1,   ///< with the foreground's converter
    memory_to_memory_blend = 2, ///< both sources, both converters, the blender
    register_to_memory = 3,     ///< one colour, no fetch
};

/// The six events (11.4), as the BIT NUMBER they share in CR's enables,
/// ISR's flags and IFCR's write-one clears - the three registers put
/// each event at the same position, offset by CR's own eight.
enum class Dma2dEvent : uint8_t {
    transfer_error = 0,
    transfer_complete = 1,
    watermark = 2,
    clut_access_error = 3,
    clut_transfer_complete = 4,
    configuration_error = 5,
};

constexpr uint32_t dma2d_event_mask(Dma2dEvent e) { return 1UL << static_cast<uint8_t>(e); }

/// How many BITS one pixel of an input format occupies. The two 4-bit
/// formats are why this is bits and not bytes, and why 11.3.11 makes an
/// odd pixel count and an odd line offset a configuration error for
/// them.
constexpr uint8_t dma2d_bits_per_pixel(Dma2dColor c) {
    switch (c) {
        case Dma2dColor::argb8888: return 32;
        case Dma2dColor::rgb888: return 24;
        case Dma2dColor::rgb565:
        case Dma2dColor::argb1555:
        case Dma2dColor::argb4444:
        case Dma2dColor::al88: return 16;
        case Dma2dColor::l8:
        case Dma2dColor::al44:
        case Dma2dColor::a8: return 8;
        default: return 4;
    }
}

constexpr uint8_t dma2d_bits_per_pixel(Dma2dOutputColor c) {
    switch (c) {
        case Dma2dOutputColor::argb8888: return 32;
        case Dma2dOutputColor::rgb888: return 24;
        default: return 16;
    }
}

/// Whether a format is an INDEX and therefore needs a CLUT.
constexpr bool dma2d_format_indexed(Dma2dColor c) {
    return c == Dma2dColor::l8 || c == Dma2dColor::al44 || c == Dma2dColor::al88 ||
           c == Dma2dColor::l4;
}

/// Whether a format carries no colour at all, so that FGCOLR/BGCOLR is
/// where the colour comes from (11.3.4).
constexpr bool dma2d_format_alpha_only(Dma2dColor c) {
    return c == Dma2dColor::a8 || c == Dma2dColor::a4;
}

/// One input image.
struct Dma2dSource {
    /// The address of its top left pixel, aligned to the format's own
    /// width (11.5.4).
    uint32_t address = 0;
    /// FGOR/BGOR, in PIXELS: what is added at the end of each line to
    /// reach the next one. Zero for an image the rectangle fills.
    uint16_t line_offset = 0;
    Dma2dColor format = Dma2dColor::argb8888;
    Dma2dAlpha alpha_mode = Dma2dAlpha::keep;
    /// The ALPHA field the two modes above use.
    uint8_t alpha = 255;
    /// FGCOLR/BGCOLR, the 0xRRGGBB an A4 or A8 source is painted in.
    uint32_t colour = 0;
    /// The CLUT an indexed format needs: where it is, how many entries
    /// (the register holds one less) and in which of the two formats.
    /// `clut_entries` of zero means "already loaded", which is what a
    /// program that filled the table through the CPU says.
    uint32_t clut_address = 0;
    uint16_t clut_entries = 0;
    Dma2dClutColor clut_format = Dma2dClutColor::argb8888;
};

/// The destination.
struct Dma2dOutput {
    uint32_t address = 0;
    /// OOR, in pixels of the OUTPUT's format.
    uint16_t line_offset = 0;
    Dma2dOutputColor format = Dma2dOutputColor::argb8888;
};

/// The rectangle, in the units NLR holds: PL is 14 bits and NL is 16.
struct Dma2dArea {
    uint16_t pixels = 0;
    uint16_t lines = 0;
};

/// What OCOLR must hold for a fill of `format` with an ARGB8888 colour
/// (11.5.15): the register is one word whose meaning is the output
/// format's, so the packing is the chapter's own and not a caller's.
constexpr uint32_t dma2d_output_colour(Dma2dOutputColor format, uint32_t argb) {
    const uint8_t a = static_cast<uint8_t>(argb >> 24);
    const uint8_t r = static_cast<uint8_t>(argb >> 16);
    const uint8_t g = static_cast<uint8_t>(argb >> 8);
    const uint8_t b = static_cast<uint8_t>(argb);
    switch (format) {
        case Dma2dOutputColor::argb8888: return argb;
        case Dma2dOutputColor::rgb888: return argb & 0x00FF'FFFFu;
        case Dma2dOutputColor::rgb565:
            return (static_cast<uint32_t>(r & 0xF8u) << 8) |
                   (static_cast<uint32_t>(g & 0xFCu) << 3) | (b >> 3);
        case Dma2dOutputColor::argb1555:
            return ((a & 0x80u) != 0u ? 0x8000u : 0u) | (static_cast<uint32_t>(r & 0xF8u) << 7) |
                   (static_cast<uint32_t>(g & 0xF8u) << 2) | (b >> 3);
        default:
            return (static_cast<uint32_t>(a & 0xF0u) << 8) |
                   (static_cast<uint32_t>(r & 0xF0u) << 4) | (g & 0xF0u) | (b >> 4);
    }
}

/// 11.3.11's blend, one channel of it, in the integer arithmetic the
/// silicon does: the divisions are "rounded to the nearest lower
/// integer". This is the model a test judges the block against.
constexpr uint8_t dma2d_blend_alpha(uint8_t fg_alpha, uint8_t bg_alpha) {
    const uint32_t mult = (static_cast<uint32_t>(fg_alpha) * bg_alpha) / 255u;
    return static_cast<uint8_t>(fg_alpha + bg_alpha - mult);
}

constexpr uint8_t dma2d_blend_channel(uint8_t fg_alpha, uint8_t bg_alpha, uint8_t fg, uint8_t bg) {
    const uint32_t mult = (static_cast<uint32_t>(fg_alpha) * bg_alpha) / 255u;
    const uint32_t out_alpha = static_cast<uint32_t>(fg_alpha) + bg_alpha - mult;
    if (out_alpha == 0u) {
        return 0;
    }
    const uint32_t numerator = static_cast<uint32_t>(fg) * fg_alpha +
                               static_cast<uint32_t>(bg) * bg_alpha -
                               static_cast<uint32_t>(bg) * mult;
    return static_cast<uint8_t>(numerator / out_alpha);
}

/// What a source's alpha becomes after its converter (11.3.11).
constexpr uint8_t dma2d_source_alpha(Dma2dAlpha mode, uint8_t original, uint8_t field) {
    switch (mode) {
        case Dma2dAlpha::replace: return field;
        case Dma2dAlpha::multiply:
            return static_cast<uint8_t>((static_cast<uint32_t>(original) * field) / 255u);
        default: return original;
    }
}

// ---- the refusals -------------------------------------------------------------------

/// The alignment 11.5.4 asks of a source or destination address: "a
/// 32-bit per pixel format must be 32-bit aligned, a 16-bit per pixel
/// format must be 16-bit aligned and a 4-bit per pixel format must be
/// 8-bit aligned". The 24-bit format is not in that sentence and three
/// bytes have no wider alignment than one, so it takes the byte's.
constexpr uint32_t dma2d_alignment(uint8_t bits) {
    if (bits == 32u) {
        return 4u;
    }
    if (bits == 16u) {
        return 2u;
    }
    return 1u;
}

constexpr bool dma2d_area_valid(const Dma2dArea& a) {
    return a.pixels != 0u && a.pixels <= 0x3FFFu && a.lines != 0u;
}

constexpr bool dma2d_source_valid(const Dma2dSource& s, const Dma2dArea& a) {
    if (static_cast<uint8_t>(s.format) > 10u) {
        return false;
    }
    if (static_cast<uint8_t>(s.alpha_mode) > 2u) {
        return false;   // AM = 11 is "meaningless"
    }
    if (s.line_offset > 0x3FFFu) {
        return false;
    }
    const uint8_t bits = dma2d_bits_per_pixel(s.format);
    if ((s.address % dma2d_alignment(bits)) != 0u) {
        return false;
    }
    if (bits == 4u && ((a.pixels & 1u) != 0u || (s.line_offset & 1u) != 0u)) {
        return false;   // 11.3.11: both must be even for L4 and A4
    }
    if (s.clut_entries > 256u) {
        return false;   // CS holds the count minus one, in eight bits
    }
    if (s.clut_entries != 0u && s.clut_format == Dma2dClutColor::argb8888 &&
        (s.clut_address % 4u) != 0u) {
        return false;   // 11.3.11: the CLUT address must match CCM
    }
    return true;
}

constexpr bool dma2d_output_valid(const Dma2dOutput& o) {
    if (static_cast<uint8_t>(o.format) > 4u) {
        return false;
    }
    if (o.line_offset > 0x3FFFu) {
        return false;
    }
    return (o.address % dma2d_alignment(dma2d_bits_per_pixel(o.format))) == 0u;
}

// The register-facing half is compiled only where the device header
// declares the accelerator.
#if defined(DMA2D_BASE)

// =============================================================================
// The block
// =============================================================================

class Dma2d {
public:
    static_assert(dma2d_present(),
                  "brio Dma2d: this device has no Chrom-Art accelerator (it is on the parts of "
                  "the class that carries one, with or without a display controller; "
                  "stm32f4/device_tables.hpp's dma2d_present() is what a program asks)");

    Dma2d() = delete;

    static constexpr IRQn_Type irq_line = dma2d_irq();

    static DMA2D_TypeDef& regs() { return *reinterpret_cast<DMA2D_TypeDef*>(dma2d_base()); }

    /// How long `wait()` spins before calling the block stopped. A
    /// full-screen QVGA fill at one pixel a cycle is under a hundred
    /// thousand cycles, so this is an order of magnitude of slack over
    /// anything a rectangle can cost.
    static constexpr uint32_t busy_spins = 20'000'000u;

    // ---- the gate and the reset (6.3.12) -------------------------------------------

    /// RCC_AHB1ENR.DMA2DEN, clear at reset.
    static void clock(bool on) { Rcc::ahb1_clock(dma2d_clock_mask(), on); }
    static bool clock() { return Rcc::ahb1_clock(dma2d_clock_mask()); }

    static void reset() {
        clock(true);
        Rcc::ahb1_reset(dma2d_reset_mask());
    }

    /// The gate opened and every flag cleared - what a program calls
    /// once before its first transfer.
    static void init() {
        clock(true);
        regs().IFCR = 0x3Fu;
    }

    static void enable_interrupt() { Nvic::enable(irq_line); }
    static void disable_interrupt() { Nvic::disable(irq_line); }

    // ---- the state of the engine (11.3.12) -----------------------------------------

    /// CR.START, which the hardware clears at the end of a transfer, on
    /// an abort, on an error and on a configuration error.
    static bool busy() {
        clock(true);
        return (regs().CR & DMA2D_CR_START) != 0u;
    }

    /// Spin until the engine is idle. False on a time-out, which is a
    /// stopped block and not a slow one.
    static bool wait(uint32_t spins = busy_spins) {
        clock(true);
        for (uint32_t i = 0; i < spins; ++i) {
            if ((regs().CR & DMA2D_CR_START) == 0u) {
                return true;
            }
        }
        return false;
    }

    /// CR.ABORT: the transfer stops and TCIF is NOT raised (11.3.12).
    /// The bit is cleared by hardware with START.
    static void abort() {
        clock(true);
        regs().CR |= DMA2D_CR_ABORT;
    }

    /// CR.SUSP: the transfer stops where it is and resumes when the bit
    /// is cleared, or is abandoned with `abort()`.
    static void suspend(bool on) {
        clock(true);
        if (on) {
            regs().CR |= DMA2D_CR_SUSP;
        } else {
            regs().CR &= ~DMA2D_CR_SUSP;
        }
    }
    static bool suspended() {
        clock(true);
        return (regs().CR & DMA2D_CR_SUSP) != 0u;
    }

    // ---- the events (11.4, 11.5.1 .. 11.5.3) ---------------------------------------

    static void interrupt(Dma2dEvent e, bool on) {
        clock(true);
        const uint32_t mask = dma2d_event_mask(e) << DMA2D_CR_TEIE_Pos;
        if (on) {
            regs().CR |= mask;
        } else {
            regs().CR &= ~mask;
        }
    }
    static bool interrupt(Dma2dEvent e) {
        clock(true);
        return (regs().CR & (dma2d_event_mask(e) << DMA2D_CR_TEIE_Pos)) != 0u;
    }

    static bool flag(Dma2dEvent e) {
        clock(true);
        return (regs().ISR & dma2d_event_mask(e)) != 0u;
    }
    static uint32_t flags() {
        clock(true);
        return regs().ISR & 0x3Fu;
    }
    static void clear(Dma2dEvent e) {
        clock(true);
        regs().IFCR = dma2d_event_mask(e);
    }
    static void clear(uint32_t mask) {
        clock(true);
        regs().IFCR = mask & 0x3Fu;
    }

    /// The ISR body: the standing flags read once, cleared, and handed
    /// back as a mask of `dma2d_event_mask()` bits.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t standing = regs().ISR & 0x3Fu;
        if (standing != 0u) {
            regs().IFCR = standing;
        }
        return standing;
    }

    /// LWR: the line whose last pixel raises the watermark event.
    static void watermark(uint16_t line) {
        clock(true);
        regs().LWR = line;
    }
    static uint16_t watermark() {
        clock(true);
        return static_cast<uint16_t>(regs().LWR & DMA2D_LWR_LW);
    }

    // ---- the bandwidth knob (11.3.15) ----------------------------------------------

    /// AMTCR: a guaranteed minimum number of AHB cycles between two of
    /// this master's accesses, which is how a program keeps it from
    /// starving another master on the same memory. A change while the
    /// engine runs is taken at its next access.
    static void dead_time(uint8_t cycles, bool on) {
        clock(true);
        regs().AMTCR = (static_cast<uint32_t>(cycles) << DMA2D_AMTCR_DT_Pos) |
                       (on ? DMA2D_AMTCR_EN : 0u);
    }
    static uint8_t dead_time() {
        clock(true);
        return static_cast<uint8_t>((regs().AMTCR & DMA2D_AMTCR_DT) >> DMA2D_AMTCR_DT_Pos);
    }
    static bool dead_time_enabled() {
        clock(true);
        return (regs().AMTCR & DMA2D_AMTCR_EN) != 0u;
    }

    // ---- the four modes (11.3.11) --------------------------------------------------

    /// Register to memory: the rectangle painted in one colour, no
    /// source fetched at all. The colour is given in ARGB8888 and
    /// packed into the output's own format here.
    static bool fill(const Dma2dOutput& out, uint32_t argb, const Dma2dArea& area) {
        if (!dma2d_output_valid(out) || !dma2d_area_valid(area) || busy()) {
            return false;
        }
        write_output(out);
        regs().OCOLR = dma2d_output_colour(out.format, argb);
        return start(Dma2dMode::register_to_memory, area);
    }

    /// Memory to memory: the bytes moved, no conversion. The
    /// foreground's format says only how WIDE a pixel is, and the
    /// output's format is not consulted by the silicon in this mode -
    /// which is why the two must agree in width and this refuses when
    /// they do not, rather than writing a rectangle of nonsense.
    static bool copy(const Dma2dSource& source, const Dma2dOutput& out, const Dma2dArea& area) {
        if (dma2d_bits_per_pixel(source.format) != dma2d_bits_per_pixel(out.format)) {
            return false;
        }
        if (!ready(source, out, area)) {
            return false;
        }
        write_source(source, true);
        write_output(out);
        return start(Dma2dMode::memory_to_memory, area);
    }

    /// Memory to memory with the pixel format converter: the source
    /// expanded to ARGB8888 and re-encoded into the output's format,
    /// with the source's alpha modified as its `alpha_mode` says.
    static bool convert(const Dma2dSource& source, const Dma2dOutput& out, const Dma2dArea& area) {
        if (!ready(source, out, area)) {
            return false;
        }
        write_source(source, true);
        write_output(out);
        return start(Dma2dMode::memory_to_memory_pfc, area);
    }

    /// Both sources fetched, both converted and mixed by 11.3.11's
    /// formula, the result written in the output's format. The
    /// FOREGROUND is the one on top.
    static bool blend(const Dma2dSource& foreground, const Dma2dSource& background,
                      const Dma2dOutput& out, const Dma2dArea& area) {
        if (!ready(foreground, out, area) || !dma2d_source_valid(background, area)) {
            return false;
        }
        write_source(foreground, true);
        write_source(background, false);
        write_output(out);
        return start(Dma2dMode::memory_to_memory_blend, area);
    }

    /// The engine started in a mode whose registers the caller has
    /// written itself - the escape for a shape the four verbs above do
    /// not have.
    static bool start(Dma2dMode mode, const Dma2dArea& area) {
        if (!dma2d_area_valid(area) || busy()) {
            return false;
        }
        clock(true);
        regs().NLR = (static_cast<uint32_t>(area.pixels) << DMA2D_NLR_PL_Pos) |
                     (static_cast<uint32_t>(area.lines) << DMA2D_NLR_NL_Pos);
        regs().CR = (regs().CR & ~(DMA2D_CR_MODE | DMA2D_CR_SUSP)) |
                    (static_cast<uint32_t>(mode) << DMA2D_CR_MODE_Pos);
        regs().CR |= DMA2D_CR_START;
        return true;
    }

    // ---- the CLUTs (11.3.11) -------------------------------------------------------

    /// The engine's own load of one converter's colour table: the
    /// address, the size and the format written, then FGPFCCR.START (or
    /// BGPFCCR's), which the hardware clears when the table is in.
    /// 11.3.11 forbids it beside a data transfer, so this refuses while
    /// the engine is busy.
    static bool load_clut(const Dma2dSource& source, bool foreground = true) {
        if (source.clut_entries == 0u || source.clut_entries > 256u || busy()) {
            return false;
        }
        clock(true);
        volatile uint32_t& pfccr = foreground ? regs().FGPFCCR : regs().BGPFCCR;
        volatile uint32_t& cmar = foreground ? regs().FGCMAR : regs().BGCMAR;
        cmar = source.clut_address;
        pfccr = (pfccr & ~(DMA2D_FGPFCCR_CS | DMA2D_FGPFCCR_CCM)) |
                (static_cast<uint32_t>(source.clut_entries - 1u) << DMA2D_FGPFCCR_CS_Pos) |
                ((source.clut_format == Dma2dClutColor::rgb888) ? DMA2D_FGPFCCR_CCM : 0u);
        pfccr |= DMA2D_FGPFCCR_START;
        return true;
    }

    /// Whether an automatic CLUT load is still running.
    static bool clut_busy(bool foreground = true) {
        clock(true);
        const uint32_t v = foreground ? regs().FGPFCCR : regs().BGPFCCR;
        return (v & DMA2D_FGPFCCR_START) != 0u;
    }

    static bool wait_clut(bool foreground = true, uint32_t spins = busy_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (!clut_busy(foreground)) {
                return true;
            }
        }
        return false;
    }

    /// One entry written by the CPU instead, into the converter's own
    /// 256-word memory. 11.3.14: an access to a CLUT while the engine
    /// is using it raises CAEIF, so this is for a table filled before a
    /// transfer.
    static void clut_entry(uint8_t index, uint32_t argb, bool foreground = true) {
        clock(true);
        if (foreground) {
            regs().FGCLUT[index] = argb;
        } else {
            regs().BGCLUT[index] = argb;
        }
    }

    /// One entry read back out of that memory. It is not an overload of
    /// the setter: `clut_entry(0, 0)` would be ambiguous between "write
    /// black" and "read entry 0 of the background", and a colour is too
    /// easy a thing to get wrong silently.
    static uint32_t clut_word(uint8_t index, bool foreground = true) {
        clock(true);
        return foreground ? regs().FGCLUT[index] : regs().BGCLUT[index];
    }

private:
    static bool ready(const Dma2dSource& s, const Dma2dOutput& o, const Dma2dArea& a) {
        return dma2d_area_valid(a) && dma2d_source_valid(s, a) && dma2d_output_valid(o) && !busy();
    }

    static void write_source(const Dma2dSource& s, bool foreground) {
        clock(true);
        volatile uint32_t& mar = foreground ? regs().FGMAR : regs().BGMAR;
        volatile uint32_t& lor = foreground ? regs().FGOR : regs().BGOR;
        volatile uint32_t& pfccr = foreground ? regs().FGPFCCR : regs().BGPFCCR;
        volatile uint32_t& colr = foreground ? regs().FGCOLR : regs().BGCOLR;
        mar = s.address;
        lor = s.line_offset;
        colr = s.colour & 0x00FF'FFFFu;
        pfccr = (static_cast<uint32_t>(s.alpha) << DMA2D_FGPFCCR_ALPHA_Pos) |
                (static_cast<uint32_t>(s.alpha_mode) << DMA2D_FGPFCCR_AM_Pos) |
                (static_cast<uint32_t>(s.clut_entries != 0u ? s.clut_entries - 1u : 0u)
                 << DMA2D_FGPFCCR_CS_Pos) |
                ((s.clut_format == Dma2dClutColor::rgb888) ? DMA2D_FGPFCCR_CCM : 0u) |
                static_cast<uint32_t>(s.format);
        if (s.clut_entries != 0u) {
            volatile uint32_t& cmar = foreground ? regs().FGCMAR : regs().BGCMAR;
            cmar = s.clut_address;
        }
    }

    static void write_output(const Dma2dOutput& o) {
        clock(true);
        regs().OMAR = o.address;
        regs().OOR = o.line_offset;
        regs().OPFCCR = static_cast<uint32_t>(o.format);
    }
};

#endif   // DMA2D_BASE

} // namespace brio
