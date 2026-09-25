/*
 * ili9481.hpp (devices)
 *
 * WHAT THE ILI9481 MAKES OF THE DCS WORDS: the controller as a TRAITS
 * type, one constexpr fact at a time, each with the section or the
 * measurement that settles it. There is no verb here and no state - a
 * panel driver is the vocabulary (devices/dcs.hpp) over a traits type
 * over a link, and this is the middle one (docs/design/gfx.md, "The
 * command tier").
 *
 * DOCUMENTS OF RECORD: the ILI9481 data sheet version 0.27 - chapter 7
 * for the interfaces, chapter 8 for the commands, 13.3.2 for the serial
 * clock - and THE BENCH, which wins wherever the two differ. A bench
 * finding corrects this file; this file never corrects the bench.
 *
 * WHAT A MODULE ADDS, AND WHY IT IS STILL HERE. Which way a glass is
 * mounted, what its colour order is and whether it wants the inversion
 * are the MODULE's facts and belong to a board file, not to a
 * controller. Two of them are stated below all the same, marked as the
 * module's: a simulator has to answer with the same bytes the bench
 * saw, and "this module drives nothing for 04h and 09h" is a fact about
 * the answer, not about the glass.
 *
 * THE MEMORY'S SEMANTICS, STATED ONCE. A driver and a panel simulator
 * written from two readings of a command table agree on the same
 * mistake, so the rules are written here, every one of them measured,
 * and read from here by both:
 *
 *   - B5, B6 and B7 of MADCTL reorder the address counter's walk INSIDE
 *     the window and never move its origin.
 *   - Under B5 (exchange) CASET addresses the PAGES (0..479) and PASET
 *     the COLUMNS (0..319), and the counter steps the pages first.
 *   - B6 reverses the walk along the columns and B7 along the pages -
 *     each of them whichever axis it has become, the fast one or the
 *     slow one.
 *   - The registers keep the bytes they were given and the axes are
 *     resolved when the counter walks: a MADCTL written AFTER the window
 *     decides which axis is which.
 *   - The BGR bit acts on WRITES alone: a pixel written under it is
 *     stored with its first and third byte exchanged, and read back as
 *     stored.
 *   - The address counter is an ADDRESS, not an index into the walk:
 *     RAMWR puts the write address at the walk's first pixel and RAMRD
 *     the read address, and nothing else moves either - a window
 *     re-issued, with the same values or others, leaves both where they
 *     are, and the two are independent of each other.
 *   - A write steps along the walk and after the window's last pixel
 *     WRAPS to its first, the first pixels overwritten; a read wraps the
 *     same way. A write whose address lies outside the window (the
 *     window moved away from under it) lands nowhere.
 *   - The read pointer counts the BYTES CLOCKED: a following 3Eh starts
 *     at the first pixel not fully clocked, from that pixel's first
 *     byte.
 *   - A pixel cut short at the close of a write transaction is DROPPED;
 *     the next 2Ch or 3Ch starts a fresh pixel and the address has not
 *     moved for the partial one.
 *   - A CASET or PASET whose START is beyond the axis it addresses is
 *     IGNORED, the register unchanged. One whose END is beyond the axis,
 *     or that runs backwards, is TAKEN and makes the window invalid: a
 *     write lands nowhere and a read answers `invalid_window_read_byte`
 *     on every clock.
 *   - A hardware reset leaves the frame memory in alternate pages of
 *     0x54 and 0xA8, the even pages 0x54.
 *   - INVON changes nothing in the memory.
 */

#pragma once

#include <stdint.h>

#include <array>

#include "devices/dcs.hpp"

namespace brio {

/**
 * The ILI9481, as a traits type. Every member is a fact a driver or a
 * simulator needs; nothing is here for symmetry.
 */
struct Ili9481 {
    Ili9481() = delete;

    // ---- the frame memory ---------------------------------------------------

    /// The native array: 320 columns by 480 pages (8.2.20 and 7.2's
    /// figures). A rotation is a window and a walk, never a resize.
    static constexpr uint16_t columns = 320;
    static constexpr uint16_t pages = 480;

    /// The four-wire serial interface stores EIGHTEEN BITS A PIXEL and
    /// nothing else (7.2): three bytes, the six most significant bits of
    /// each carrying the channel, the low two ignored on a write and
    /// zero on a read.
    static constexpr uint8_t bytes_per_pixel = 3;

    /// COLMOD at reset, and the only code this link can use.
    static constexpr uint8_t reset_colmod = dcs_colmod_18bpp;

    /// MADCTL at reset - measured: RDDMADCTL answers 0x00 on a module
    /// that has only been woken.
    static constexpr uint8_t reset_madctl = 0x00;

    /// What a HARDWARE reset leaves in the frame memory: alternate pages
    /// of two values, the EVEN pages the first of them (measured).
    static constexpr uint8_t reset_fill_even_page = 0x54;
    static constexpr uint8_t reset_fill_odd_page = 0xA8;

    /// What a read from a window the axes cannot hold answers on every
    /// byte clocked (measured: CASET 210..205 taken, then RAMRD). It is
    /// an odd page's reset fill, as if the address had left the array;
    /// the value is recorded, the reason is not known.
    static constexpr uint8_t invalid_window_read_byte = 0xA8;

    // ---- how a read comes back on the four-wire serial link -----------------

    /// 7.2.2 gives every read command a "dummy read" as its first
    /// parameter and its figures do not say whether that is a BYTE or a
    /// CLOCK. The bench answers, and the answer is not one answer:
    ///
    ///  - a MULTI-PARAMETER register read comes one dummy CLOCK late -
    ///    the whole stream shifted right by one bit. Measured on the
    ///    device code (BFh): the raw bytes 01 02 4A 40 FF for the value
    ///    02 04 94 81 FF, the leading bit driven low.
    ///  - a SINGLE-parameter read (RDDPM 0Ah, RDDMADCTL 0Bh, RDDCOLMOD
    ///    0Ch) has NO dummy at all: the value is the first byte clocked.
    ///  - RAMRD (2Eh) and read memory continue (3Eh) begin with ONE
    ///    DUMMY BYTE, which reads 0x80 on the bench, and then three
    ///    bytes a pixel.
    ///
    /// RDDID (04h) and RDDST (09h) are multi-parameter reads of the
    /// device code's shape, but THE BENCH MODULE DRIVES NEITHER: both
    /// read all ones, which is the link's pull-up and not a value. They
    /// are classified `undriven` below, because a shift applied to an
    /// undriven line would invent a 0x7F that the bench never saw.
    static constexpr uint8_t memory_read_dummy_byte = 0x80;

    /// The module's own silence on the two identification registers.
    static constexpr bool module_rddid_all_ones = true;
    static constexpr bool module_rddst_all_ones = true;

    /// The manufacturer command that answers with the device code; not a
    /// DCS command, so it is named here and not in the vocabulary.
    static constexpr uint8_t device_code_command = 0xBF;

    /// The four bytes behind the dummy clock (measured).
    static constexpr std::array<uint8_t, 4> device_code{0x02, 0x04, 0x94, 0x81};

    static constexpr DcsReadFraming read_framing(uint8_t command) {
        switch (command) {
            case Dcs::ramrd:
            case Dcs::ramrd_continue:
                return DcsReadFraming::dummy_byte;
            case Dcs::rddpm:
            case Dcs::rddmadctl:
            case Dcs::rddcolmod:
                return DcsReadFraming::aligned;
            case device_code_command:
                return DcsReadFraming::dummy_clock;
            default:
                // RDDID and RDDST among them: the module drives nothing,
                // and so does the controller for every write command.
                return DcsReadFraming::undriven;
        }
    }

    // ---- the serial clock ---------------------------------------------------

    /// 13.3.2: 40 ns high and 40 ns low for a write, 120 + 120 for a
    /// read. These are the interface's ceilings and the rates a driver
    /// stays inside by default.
    static constexpr uint32_t write_clock_max_hz = 12'500'000;
    static constexpr uint32_t read_clock_max_hz = 4'166'666;

    /// WHAT ONE MODULE ON ONE DESK DID, which is a different statement
    /// and is kept apart from the one above: a write reads back exact at
    /// the first of these and garbage at the second, and a READ is exact
    /// at three times the data sheet's ceiling and wrong above it.
    static constexpr uint32_t measured_write_exact_hz = 12'000'000;
    static constexpr uint32_t measured_write_wrong_hz = 24'000'000;
    static constexpr uint32_t measured_read_exact_hz = 12'000'000;
    static constexpr uint32_t measured_read_wrong_hz = 24'000'000;

    // ---- the wake -----------------------------------------------------------

    /// The sequence the bench proved, as times and not as code: the
    /// reset line low for `reset_low_ms`, `reset_settle_ms` after it
    /// rises, SLPOUT, `sleep_out_ms`, DISPON, `display_on_ms`. A driver
    /// owns the waiting; this file owns the numbers.
    static constexpr uint16_t reset_low_ms = 10;
    static constexpr uint16_t reset_settle_ms = 150;
    static constexpr uint16_t sleep_out_ms = 150;
    static constexpr uint16_t display_on_ms = 25;
};

}   // namespace brio
