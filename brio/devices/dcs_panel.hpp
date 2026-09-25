/*
 * dcs_panel.hpp (devices)
 *
 * THE PANEL DRIVER OF THE COMMAND TIER: the vocabulary
 * (devices/dcs.hpp) over a controller's traits (devices/ili9481.hpp and
 * its siblings) over a link (devices/dcs_link.hpp), and the fourth and
 * last of the tier's layers (docs/design/gfx.md, "The command tier").
 * What it adds to the three under it is the whole of what a drawing
 * needs and none of them has: a LOGICAL SURFACE over the glass.
 *
 * THE DIRECT SHAPE, AND ONLY IT. Every verb here is a synchronous link
 * transaction, complete on return, which is the shape of a bring-up, of
 * a suite and of a program that owns its bus. The TILED shape - the
 * primitives drawing into a rectangle of RAM that a display active
 * object streams - is a layer above this one and is not here; nothing
 * in this file knows the kernel, and a program that draws through it
 * spends its own time on the bus.
 *
 * WHAT A ROTATION IS HERE, AND WHY IT IS NOT A TRANSFORM. The address
 * mode's three walk bits reorder the counter INSIDE the window a
 * CASET/PASET pair opened and never move that window's origin
 * (docs/devices/ili9481.md). So a rotation is not applied to a picture:
 * it is a WINDOW in the glass's own frame plus the walk bits that make
 * the counter cross that window in the order a logical row runs. The
 * one rule that decides every bit: A RUN OF THE LOGICAL SURFACE - one
 * row, x increasing - MUST BE ONE CONTIGUOUS WALK OF THE COUNTER. Each
 * rotation therefore states the bit ALONG its runs and leaves the bits
 * ACROSS them clear, and the module's mirror flips the column direction,
 * which touches the rotations whose runs lie along the columns and not
 * the ones whose runs lie along the pages.
 *
 * WHY THE BITS ACROSS A RUN MAY BE LEFT CLEAR. Because the two Surface
 * verbs never send a multi-row block that is not UNIFORM: a fill is one
 * colour and a run is one row. The order the counter takes ACROSS the
 * rows of a filled rectangle therefore never shows - every cell of it
 * gets the same three bytes - and a run never has a second row to get
 * wrong. That is what lets one MADCTL code serve both verbs; a verb
 * that streamed a two-dimensional image would have to state those bits
 * too, and there is no such verb.
 *
 * THE ORACLE VERB IS THE OBSERVER'S AND NEVER A PRIMITIVE'S. read_run()
 * and get_pixel() read the frame memory back in LOGICAL coordinates,
 * through the same window and the same walk a write used, which is what
 * makes the bench plane of the three planes of truth cheap: a drawing
 * is judged pixel for pixel with no eye on the glass, and the same test
 * is written once for every orientation. get_pixel() exists so that a
 * DcsPanel is a `ReadableSurface` and the reference renderer's
 * compare() judges it directly - AND IT COSTS A WINDOW AND A READ PER
 * PIXEL on the wire. No primitive of gfx/ may use it, and the base
 * concept having no such verb is the enforcement (gfx/surface.hpp).
 *
 * THE MODULE IS NOT THE CONTROLLER. Which way a glass is mounted on the
 * die, whether its colour channels are crossed and whether it wants the
 * inversion are facts of a BOARD, handed to the driver the way the pins
 * are. The colour order in particular is answered in the byte order the
 * driver packs and never in MADCTL's BGR bit, because that bit acts on
 * writes alone (measured) and would leave every read-back crossed -
 * which would cost the oracle.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include <array>
#include <optional>
#include <span>

#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "gfx/surface.hpp"

namespace brio {

/**
 * The logical surface's orientation over the glass, in DISPLAY
 * coordinates - d left to right, line top to bottom, the glass in its
 * portrait.
 *
 *  - `r0`   the glass's own portrait;
 *  - `r90`  the picture turned CLOCKWISE: logical x runs down the
 *           glass and logical y leftward;
 *  - `r180` upside down;
 *  - `r270` counter-clockwise: logical x runs up the glass and logical
 *           y rightward.
 *
 * These are the four a bench judges by eye against the module's own
 * marking, which is why they are named for the picture's turn and not
 * for a register's bits.
 */
enum class DcsRotation : uint8_t { r0, r90, r180, r270 };

/// True where the rotation exchanges the surface's axes with the
/// glass's - the two whose logical rows run along the PAGES.
constexpr bool dcs_rotation_exchanges(DcsRotation r) {
    return r == DcsRotation::r90 || r == DcsRotation::r270;
}

/**
 * THE MODULE'S FACTS, which are a board's and not a controller's, and
 * the mirror image of the simulator's own glass on the world's side.
 */
struct DcsModule {
    /// The glass shows memory column `columns - 1` at its LEFT, so a
    /// picture written straight is mirrored. It is answered in the
    /// rotation map below, because MADCTL's horizontal flip is a
    /// DISPLAY bit a module may leave unconnected.
    bool column_mirror = false;
    /// The module's channels are crossed: the glass lights a cell's
    /// first byte as blue. The driver therefore writes B, G, R so that
    /// the eye sees its own colour, and reads them back in its own
    /// order - never through MADCTL's BGR bit, which acts on writes
    /// alone and would leave the read-back crossed.
    bool bgr = false;
    /// The panel shows the stored colours only with the inversion on.
    bool wants_inversion = false;
};

/**
 * The surface format of a command panel: twenty-four bits of colour in
 * a uint32_t as 0x00RRGGBB, which is the drawing world's own colour.
 * What a panel KEEPS of it is the controller's business - eighteen bits
 * on the one this file has - and the packers of devices/dcs.hpp are
 * where that is decided.
 *
 * It lives here and not in gfx/surface.hpp because the formats there
 * are the ones a Framebuffer stores, each with a stride and a packing;
 * this one has neither - no pixel of it is ever in the program's memory
 * as a pixel.
 */
struct DcsRgb888 {
    using Color = uint32_t;
    static constexpr uint8_t bits = 24;
    static constexpr Color max_color = 0x00FFFFFFu;
};

/**
 * THE ROTATION MAP, as a function of nothing but the rotation and the
 * module's mirror - so that it can be read, checked and tabulated
 * without a panel, a link or a bus.
 *
 * Each rotation states the walk bit ALONG its runs and leaves the ones
 * across them clear:
 *
 *  - under r0 and r180 a logical row runs along the COLUMNS, so B5 is
 *    clear and B6 says which way: memory columns ascend under r0 on a
 *    straight module and under r180 on a mirrored one, descend
 *    otherwise. That is the whole of the mirror's effect;
 *  - under r90 and r270 a logical row runs along the PAGES, so B5 is
 *    set (the counter steps the pages first) and B7 says which way -
 *    clear under r90, where the lines descend with x, set under r270,
 *    where they ascend. The mirror moves the WINDOW here and not the
 *    walk, so it touches neither code.
 *
 * No BGR bit and no display bit: the first would cost the read-back its
 * order and the second is a wire a module may not have.
 */
constexpr uint8_t dcs_rotation_madctl(DcsRotation r, bool column_mirror) {
    switch (r) {
        case DcsRotation::r0:
            return column_mirror ? DcsAddressMode::column_order : 0u;
        case DcsRotation::r90:
            return DcsAddressMode::exchange;
        case DcsRotation::r180:
            return column_mirror ? 0u : DcsAddressMode::column_order;
        case DcsRotation::r270:
            return static_cast<uint8_t>(DcsAddressMode::exchange | DcsAddressMode::page_order);
    }
    return 0u;
}

/**
 * A COMMAND PANEL AS A SURFACE. `Traits` is the controller
 * (devices/ili9481.hpp), `Link` any `DcsLink`; both are held by
 * reference to things the caller owns - the link, and the module's
 * facts by value.
 *
 * THE TWO BUFFERS ARE MEMBERS AND SIZED BY THE GLASS'S LONG SIDE,
 * because a run of the ROTATED surface is as wide as the glass is tall.
 * A buffer sized by the short side is correct until the first r90 and
 * then writes past itself, which is a bug that costs an afternoon to
 * find; and a buffer on the stack is one a run can overflow on a part
 * whose stack is two kilobytes.
 *
 * EIGHTEEN BITS A PIXEL. This driver packs with `dcs_rgb666_pack`,
 * which is what the one controller it has states as its only format on
 * a serial link; a controller with another width is a gap in
 * docs/devices/dcs_panel.md and not a branch here.
 */
template <typename Traits, DcsLink Link>
class DcsPanel {
public:
    using Controller = Traits;
    using Format = DcsRgb888;
    using Color = typename DcsRgb888::Color;

    static constexpr uint16_t columns = Traits::columns;
    static constexpr uint16_t pages = Traits::pages;
    static constexpr uint8_t bytes_per_pixel = Traits::bytes_per_pixel;

    static_assert(bytes_per_pixel == 3,
                  "this driver packs eighteen bits a pixel, which is three bytes");
    static_assert(dcs_colmod_dbi(Traits::reset_colmod) == DcsPixelFormat::bpp18,
                  "this driver packs eighteen bits a pixel and states that in COLMOD");
    static_assert(columns <= coord_max && pages <= coord_max,
                  "a surface larger than the coordinate domain");

    /// The longest logical row any rotation can ask for, and the row
    /// buffer that holds it.
    static constexpr uint16_t long_side = columns > pages ? columns : pages;
    static constexpr uint16_t row_bytes =
        static_cast<uint16_t>(static_cast<uint32_t>(long_side) * bytes_per_pixel);

    DcsPanel(Link& link, DcsModule module) : link_(link), module_(module) {}

    // ---- the wake -----------------------------------------------------------

    /**
     * THE WAKE, from the reset line up. `reset` is a Pin-like with
     * `set()` and `clear()`, the line being active LOW - every stratum's
     * `PinRef` is one, which is how a board file names a pad at run
     * time; `delay_ms` is any callable `void(uint16_t)`.
     *
     * The traits own the numbers, this driver owns the sequence and the
     * application owns the clock - which is what lets a host test COUNT
     * the milliseconds instead of spending them, and a program on
     * silicon spend them however it spends time. False when a link verb
     * refused; every refusal is counted whether or not the caller reads
     * the answer.
     */
    template <typename Reset, typename Delay>
    bool reset_and_wake(Reset& reset, Delay&& delay_ms) {
        reset.clear();
        delay_ms(Traits::reset_low_ms);
        reset.set();
        delay_ms(Traits::reset_settle_ms);
        return wake(delay_ms);
    }

    /**
     * The same sequence from SLPOUT on, for a panel whose reset line
     * the board does not give the program: SLPOUT, the wait, DISPON,
     * the wait, the inversion the module asks for, COLMOD as a
     * STATEMENT and not a guess, and MADCTL for the rotation in force.
     */
    template <typename Delay>
    bool wake(Delay&& delay_ms) {
        bool ok = send(Dcs::slpout, {});
        delay_ms(Traits::sleep_out_ms);
        ok = send(Dcs::dispon, {}) && ok;
        delay_ms(Traits::display_on_ms);
        ok = send(module_.wants_inversion ? Dcs::invon : Dcs::invoff, {}) && ok;
        const uint8_t colmod[1] = {Traits::reset_colmod};
        ok = send(Dcs::colmod, colmod) && ok;
        return rotation(rotation_) && ok;
    }

    // ---- the orientation ----------------------------------------------------

    /**
     * Put the logical surface in `r`: MADCTL takes the map's code and
     * width()/height() follow. The geometry follows the INTENT, so a
     * caller that ignores a false draws into a panel that is not in the
     * orientation it thinks - which is why the answer exists.
     */
    bool rotation(DcsRotation r) {
        rotation_ = r;
        const uint8_t code[1] = {madctl()};
        return send(Dcs::madctl, code);
    }

    DcsRotation rotation() const { return rotation_; }

    /// The code the map produces for the rotation in force on this
    /// module - the byte MADCTL holds, and nothing else.
    uint8_t madctl() const { return dcs_rotation_madctl(rotation_, module_.column_mirror); }

    const DcsModule& module() const { return module_; }

    // ---- the surface --------------------------------------------------------

    Extent width() const {
        return dcs_rotation_exchanges(rotation_) ? pages : columns;
    }

    Extent height() const {
        return dcs_rotation_exchanges(rotation_) ? columns : pages;
    }

    /**
     * A rectangle of one colour: ONE window, one row of packed pixels
     * built once, then RAMWR for the first row and write memory
     * continue for each of the others - so a `w` x `h` fill is h + 2
     * link transactions whatever w is. A rectangle wholly outside the
     * surface touches the link not at all.
     */
    void fill_rect(Coord x, Coord y, Extent w, Extent h, Color c) {
        const std::optional<Rect> r = clip(Rect{x, y, w, h}, width(), height());
        if (!r) {
            return;
        }
        if (!window_logical(r->x, r->y, r->w, r->h)) {
            return;
        }
        const std::array<uint8_t, 3> pixel = dcs_rgb666_pack(c, module_.bgr);
        for (Extent i = 0; i < r->w; ++i) {
            const uint16_t at = static_cast<uint16_t>(i * bytes_per_pixel);
            row_[at] = pixel[0];
            row_[at + 1u] = pixel[1];
            row_[at + 2u] = pixel[2];
        }
        const std::span<const uint8_t> bytes(
            row_, static_cast<size_t>(r->w) * bytes_per_pixel);
        for (Extent line = 0; line < r->h; ++line) {
            if (!send_pixels(line == 0 ? Dcs::ramwr : Dcs::ramwr_continue, bytes)) {
                return;
            }
        }
    }

    /**
     * One row of caller-given pixels: the head the clip cut is SKIPPED
     * in the caller's run, not dropped from its tail, then one window
     * one row high and one RAMWR - three link transactions.
     */
    void write_run(Coord x, Coord y, std::span<const Color> run) {
        const std::optional<Rect> r = clip(Rect{x, y, run_extent(run), 1}, width(), height());
        if (!r) {
            return;
        }
        const uint_fast16_t skip = static_cast<uint_fast16_t>(
            static_cast<int_fast32_t>(r->x) - static_cast<int_fast32_t>(x));
        if (!window_logical(r->x, r->y, r->w, 1)) {
            return;
        }
        for (Extent i = 0; i < r->w; ++i) {
            const std::array<uint8_t, 3> pixel =
                dcs_rgb666_pack(run[static_cast<size_t>(skip) + i], module_.bgr);
            const uint16_t at = static_cast<uint16_t>(i * bytes_per_pixel);
            row_[at] = pixel[0];
            row_[at + 1u] = pixel[1];
            row_[at + 2u] = pixel[2];
        }
        (void)send_pixels(Dcs::ramwr,
                          std::span<const uint8_t>(
                              row_, static_cast<size_t>(r->w) * bytes_per_pixel));
    }

    // ---- the oracle ---------------------------------------------------------

    /**
     * THE ORACLE VERB: the logical run at (x, y) read back in LOGICAL
     * order, through the same window and the same walk a write of it
     * would use - one window, one RAMRD, three link transactions.
     *
     * The bytes come back as the LINK clocked them, which is what the
     * link promises, and the controller's own framing for a memory read
     * is applied here: the dummy byte or the dummy clock the traits
     * name, then three bytes a pixel, unpacked in the module's colour
     * order. A run partly outside the surface is clipped like a write
     * and the pixels outside come back 0; one wholly outside touches
     * the link not at all and answers true, nothing having been
     * refused. False when a link verb refused.
     */
    bool read_run(Coord x, Coord y, std::span<Color> out) const {
        for (Color& c : out) {
            c = 0;
        }
        if constexpr (read_framing == DcsReadFraming::undriven) {
            // A controller that drives nothing on a memory read has no
            // oracle: the verb says so rather than handing back a
            // pull-up dressed as a colour.
            return false;
        } else {
            const std::optional<Rect> r =
                clip(Rect{x, y, run_extent(out), 1}, width(), height());
            if (!r) {
                return true;
            }
            const uint_fast16_t skip = static_cast<uint_fast16_t>(
                static_cast<int_fast32_t>(r->x) - static_cast<int_fast32_t>(x));
            if (!window_logical(r->x, r->y, r->w, 1)) {
                return false;
            }
            const uint16_t wanted = static_cast<uint16_t>(r->w * bytes_per_pixel);
            const uint16_t clocked = static_cast<uint16_t>(wanted + read_extra);
            for (uint16_t i = 0; i < clocked; ++i) {
                raw_[i] = 0;
            }
            if (!read_back(Dcs::ramrd, std::span<uint8_t>(raw_, clocked))) {
                return false;
            }
            for (Extent i = 0; i < r->w; ++i) {
                uint8_t pixel[3];
                for (uint8_t b = 0; b < bytes_per_pixel; ++b) {
                    pixel[b] = realigned(static_cast<uint16_t>(i * bytes_per_pixel + b));
                }
                out[static_cast<size_t>(skip) + i] =
                    dcs_rgb666_unpack(std::span<const uint8_t, 3>(pixel), module_.bgr);
            }
            return true;
        }
    }

    /**
     * One pixel read back, so that a DcsPanel is a `ReadableSurface`
     * and the reference renderer judges it with no adapter. ON SILICON
     * IT IS A WINDOW AND A READ PER PIXEL, which is a whole screen's
     * worth of transactions for a whole screen's worth of pixels: it is
     * the OBSERVER'S verb - a test, a golden, an eye - and never a
     * primitive's. Out of bounds, and where the link refused, it
     * answers 0.
     */
    Color get_pixel(Coord x, Coord y) const {
        Color one = 0;
        if (!read_run(x, y, std::span<Color>(&one, 1))) {
            return 0;
        }
        return one;
    }

    // ---- what the link refused ----------------------------------------------

    /**
     * Link transactions this panel asked for and did not get. The two
     * Surface verbs are `void` by contract, so a refusal has nowhere
     * else to go and counting is what keeps it from being a silent
     * wrong picture. A verb STOPS at its first refusal - a window half
     * written is not a window - so this counts one per verb on a link
     * that refuses everything.
     *
     * The driver counts nothing else. What an update costs in
     * rectangles, runs and pixels is a different question, and
     * gfx/counting.hpp is where it is asked.
     */
    uint32_t link_failures() const { return link_failures_; }

    void reset_counters() { link_failures_ = 0; }

private:
    /// How a memory read comes back on this link, and what it costs in
    /// bytes clocked before the first one that carries a colour.
    static constexpr DcsReadFraming read_framing = Traits::read_framing(Dcs::ramrd);
    static constexpr uint16_t read_offset = read_framing == DcsReadFraming::dummy_byte ? 1u : 0u;
    static constexpr uint8_t read_shift = read_framing == DcsReadFraming::dummy_clock ? 1u : 0u;
    static constexpr uint16_t read_extra =
        static_cast<uint16_t>(read_offset + (read_shift != 0u ? 1u : 0u));

    /// A run's length as an extent. Clamped rather than cast, so that a
    /// run longer than anything a coordinate can reach is CLIPPED like
    /// any other and never silently truncated to its own low bits.
    template <typename T>
    static Extent run_extent(std::span<T> run) {
        return run.size() > 0xFFFFu ? static_cast<Extent>(0xFFFFu)
                                    : static_cast<Extent>(run.size());
    }

    /// One byte of the value behind the framing: the raw stream from
    /// `read_offset`, and a whole stream one bit late where the
    /// controller answers a memory read a dummy CLOCK behind the
    /// command.
    uint8_t realigned(uint16_t i) const {
        const uint8_t b = raw_[read_offset + i];
        // `if constexpr`, because the other branch reads one byte
        // further than the buffer is sized for where there is no shift.
        if constexpr (read_shift == 0u) {
            return b;
        } else {
            return static_cast<uint8_t>((b << read_shift) |
                                        (raw_[read_offset + i + 1u] >> (8u - read_shift)));
        }
    }

    /**
     * THE LOGICAL WINDOW: a rectangle of the logical surface, already
     * clipped, turned into the glass's own column and page ranges and
     * handed to CASET and PASET - in the order the walk expects, CASET
     * taking the PAGES once B5 has exchanged the axes (measured;
     * docs/devices/ili9481.md's "Where a window lands").
     *
     * The display rectangle first - d is the glass's own left-to-right
     * column and `line` its top-to-bottom row - then the module's
     * mirror, which is the only place a memory column and a display
     * column differ.
     */
    bool window_logical(Coord x, Coord y, Extent w, Extent h) const {
        const uint_fast16_t lx = static_cast<uint_fast16_t>(x);
        const uint_fast16_t ly = static_cast<uint_fast16_t>(y);
        uint_fast16_t d0 = 0, d1 = 0, p0 = 0, p1 = 0;
        switch (rotation_) {
            case DcsRotation::r0:
                d0 = lx;
                d1 = lx + w - 1u;
                p0 = ly;
                p1 = ly + h - 1u;
                break;
            case DcsRotation::r90:
                d0 = columns - ly - h;
                d1 = columns - 1u - ly;
                p0 = lx;
                p1 = lx + w - 1u;
                break;
            case DcsRotation::r180:
                d0 = columns - lx - w;
                d1 = columns - 1u - lx;
                p0 = pages - ly - h;
                p1 = pages - 1u - ly;
                break;
            case DcsRotation::r270:
                d0 = ly;
                d1 = ly + h - 1u;
                p0 = pages - lx - w;
                p1 = pages - 1u - lx;
                break;
        }
        uint_fast16_t c0 = d0;
        uint_fast16_t c1 = d1;
        if (module_.column_mirror) {
            c0 = columns - 1u - d1;
            c1 = columns - 1u - d0;
        }
        const bool exchanged = dcs_rotation_exchanges(rotation_);
        const std::array<uint8_t, 4> caset =
            dcs_window(static_cast<uint16_t>(exchanged ? p0 : c0),
                       static_cast<uint16_t>(exchanged ? p1 : c1));
        const std::array<uint8_t, 4> paset =
            dcs_window(static_cast<uint16_t>(exchanged ? c0 : p0),
                       static_cast<uint16_t>(exchanged ? c1 : p1));
        return send(Dcs::caset, caset) && send(Dcs::paset, paset);
    }

    /// A command, its refusal counted. `link_` is a reference, so its
    /// verbs stay reachable from the const oracle - which is right: a
    /// panel that is asked a question is not a panel that has changed.
    bool send(uint8_t command, std::span<const uint8_t> parameters) const {
        if (link_.command(command, parameters)) {
            return true;
        }
        ++link_failures_;
        return false;
    }

    /// A read, its refusal counted. Named apart from `send()` because
    /// an array of bytes converts to either span and a reader should
    /// not have to work out which verb a call means.
    bool read_back(uint8_t command, std::span<uint8_t> in) const {
        if (link_.read(command, in)) {
            return true;
        }
        ++link_failures_;
        return false;
    }

    /// A memory write, which is the link's OTHER verb: on a serial link
    /// the same tenure, on a parallel one a store of a different width.
    bool send_pixels(uint8_t command, std::span<const uint8_t> bytes) {
        if (link_.write(command, bytes)) {
            return true;
        }
        ++link_failures_;
        return false;
    }

    Link& link_;
    DcsModule module_;
    DcsRotation rotation_ = DcsRotation::r0;
    mutable uint32_t link_failures_ = 0;

    uint8_t row_[row_bytes] = {};
    mutable uint8_t raw_[row_bytes + read_extra] = {};
};

}   // namespace brio
